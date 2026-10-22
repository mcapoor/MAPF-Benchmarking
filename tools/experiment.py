"""Benchmark the solvers on the MovingAI scenario files of benchmarks/ as one experiment, experiments/<name>/. The
problem with n agents of a scenario file is its first n agents. The runs of each (scenario file, solver) go up in agent
count, and after the first failure the larger counts are counted as failures without running them (more agents of the
same file only make it harder), so a solver that cannot keep up costs one time limit per scenario file.

    python tools/experiment.py plan NAME --timeout 300 --agents 10 20 50 100 200
    python tools/experiment.py run NAME --timeout 300 --agents 10 20 50 100 200 --shard 0 --shards 20 --jobs 12
    python tools/experiment.py merge NAME --timeout 300 --agents 10 20 50 100 200

--maps limits it to some maps (default: all, or none with --reuse). --reuse EXPERIMENT starts from an earlier
experiment's results.csv: its solved runs are kept (a run solved within its limit takes the same time with a longer
one), its unsolved runs are rerun, and its scenario files and agent counts join the plan. benchmark.sh submits all of
it on Slurm.

`run` does every shards-th group, starting at shard, and appends to experiments/<name>/shards/shard<i>.csv as each group
ends: its runs, a "#failed" row (instance, solver, reason, wall seconds) for its failure, and a "#done" row, so that a
resubmitted shard skips the groups it finished. With --retry it also redoes the finished groups whose failure says
nothing about the instance: a run killed (usually by the out-of-memory killer) or crashed, or one whose reason was not
recorded. Later rows win.

`merge` writes experiments/<name>/results.csv and the MAPFAST dataset, {yaml,agent,map}_details.json; it stops if a group
is unfinished.
"""
import argparse
import collections
import concurrent.futures
import csv
import glob
import json
import os
import re
import sys
import threading

# allow running as a script (python tools/experiment.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from benchmark import BENCHMARK_DIR, RESULTS_FILE, SCEN_TYPES, experiment_dir, load_instance, scenario_files, \
    solver_outcome
from tools import reloc_wrappers, utils

HEADER = ["solver", "n_agents", "map", "seconds", "cost", "makespan", "solver_cost"]
INSTANCE = re.compile(r"^((.+)-(?:random|even)-\d+)_agents(\d+)\.yaml$")
# the solvers of the standard MAPF problem (bcp2 needs a Gurobi license on the node)
SOLVERS = ["cbs", "ecbs", "mapf_prioritized_sipp", "cbsh2_rtc", "cbsh2_rtc_ch", "cbsh2_rtc_bp", "cbsh2_rtc_chbp",
           "reloc", "bcp2"]
# failures that say something about the instance (see benchmark.FAILURES); --retry redoes the others
FINAL_FAILURES = ("timeout", "no solution")
MAPFAST_DETAILS = ("yaml_details", "agent_details", "map_details")


def read_runs(path):
    """{(instance name, solver): CSV row} of a results CSV."""
    with open(path, newline="") as results_file:
        reader = csv.reader(results_file)
        if next(reader, None) != HEADER:
            raise SystemExit(f"{path} does not have the columns {HEADER}")
        return {(row[2], row[0]): row for row in reader}


def scenario_file(instance):
    """(map file, scenario file, agent count) of an instance name, e.g. Berlin_1_256-random-1_agents10.yaml"""
    match = INSTANCE.match(instance)
    directory = os.path.join(BENCHMARK_DIR, match.group(2))
    return (os.path.join(directory, match.group(2) + ".map"), os.path.join(directory, match.group(1) + ".scen"),
            int(match.group(3)))


def plan(reused, agents, solvers, maps):
    """The work, as {(scenario file, solver): (map file, [agent count, ...])}: the counts of `agents` that the
    scenario file has, and those of its reused runs, in increasing order; `run_group` decides which of them to run."""
    scens, counts = {}, collections.defaultdict(set)
    for name in maps:
        for scen_type in SCEN_TYPES:
            map_file, files = scenario_files(os.path.join(BENCHMARK_DIR, name), scen_type)
            scens.update(dict.fromkeys(files, map_file))
    for instance, solver in reused:
        map_file, scen, n_agents = scenario_file(instance)
        scens[scen] = map_file
        counts[scen, solver].add(n_agents)
    groups = {}
    for scen, map_file in scens.items():
        available = len(load_instance(map_file, scen)[3])
        for solver in solvers:
            groups[scen, solver] = (map_file, sorted(counts[scen, solver] | {n for n in agents if n <= available}))
    return dict(sorted(groups.items()))


def solver_input(scen, n_agents, solver, map_file):
    """The problem in the format `solver` reads, converted if it is not cached yet (as benchmark.py does)"""
    if solver in utils.YAML_SOLVERS:
        return utils.libmrp_problem(scen, n_agents, map_file)
    if solver == "reloc":
        reloc_wrappers.convert(scen, map_file, n_agents, reloc_wrappers.cpf_file(scen, n_agents))
    return (scen, n_agents)


def run_group(scen, solver, map_file, counts, runs, timeout):
    """The CSV rows of one group: a run for each count not solved in `runs`, until one fails, then the larger counts
    as failures without running them, then a "#failed" row for the failure. Returns (rows, runs done, runs skipped)"""
    rows, done, skipped, failure = [], 0, 0, None
    for n_agents in counts:
        instance = utils.instance_name(scen, n_agents)
        old = runs.get((instance, solver))
        if old is not None and old[3] != "":
            continue
        if failure:
            rows.append([solver, n_agents, instance, "", "", "", ""])
            skipped += 1
            continue
        run, reason, seconds = solver_outcome(solver_input(scen, n_agents, solver, map_file), solver, timeout)
        done += 1
        if run is None:
            failure = ["#failed", instance, solver, reason, f"{seconds:.3f}"]
            rows.append([solver, n_agents, instance, "", "", "", ""])
        else:
            rows.append([solver, n_agents, instance, f"{run.seconds:.6f}", run.cost, run.makespan,
                         "" if run.solver_cost is None else run.solver_cost])
    return rows + ([failure] if failure else []), done, skipped


def read_shards(paths):
    """(runs, finished groups, failure reasons) of shard files, read in order so that later rows win:
    {(instance, solver): CSV row}, {(scenario file, solver)}, {(instance, solver): reason}"""
    rows, finished, reasons = {}, set(), {}
    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path, newline="") as shard_file:
            for row in csv.reader(shard_file):
                if not row:
                    continue
                if row[0] == "#done":
                    finished.add(tuple(row[1].split("|", 1)))
                elif row[0] == "#failed":
                    reasons[row[1], row[2]] = row[3]
                else:
                    rows[row[2], row[0]] = row
    return rows, finished, reasons


def failure(scen, solver, counts, runs):
    """(instance, solver) of a group's failed run (its smallest unsolved count: the larger ones were skipped after it),
    or None"""
    for n_agents in counts:
        key = (utils.instance_name(scen, n_agents), solver)
        if key in runs and runs[key][3] == "":
            return key
    return None


def to_retry(groups, keys, runs, reasons):
    """{group: reason} of the groups among `keys` whose failure is not one of FINAL_FAILURES"""
    retry = {}
    for key in keys:
        failed = failure(*key, groups[key][1], runs)
        if failed and reasons.get(failed, "unknown") not in FINAL_FAILURES:
            retry[key] = reasons.get(failed, "unknown")
    return retry


def describe(counter):
    """e.g. "12 timeout, 3 killed" of a Counter"""
    return ", ".join(f"{n} {reason}" for reason, n in counter.most_common()) or "none"


def shard_paths(name):
    return sorted(glob.glob(os.path.join(glob.escape(experiment_dir(name)), "shards", "shard*.csv")))


def run_shard(args, runs, groups):
    keys = list(groups)[args.shard::args.shards]
    path = os.path.join(experiment_dir(args.name), "shards", f"shard{args.shard}.csv")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rows, finished, reasons = read_shards([path])
    runs = {**runs, **rows}
    retry = to_retry(groups, [key for key in keys if key in finished], runs, reasons) if args.retry else {}
    todo = [key for key in keys if key not in finished or key in retry]
    print(f"shard {args.shard}/{args.shards}: {len(keys)} groups, {len(keys) - len(todo) + len(retry)} finished before"
          + (f", {len(retry)} of them to retry ({describe(collections.Counter(retry.values()))})" if args.retry else ""),
          flush=True)
    lock = threading.Lock()
    totals = collections.Counter()

    def work(key):
        map_file, counts = groups[key]
        new, done, skipped = run_group(*key, map_file, counts, runs, args.timeout)
        with lock:
            with open(path, "a", newline="") as shard_file:
                csv.writer(shard_file).writerows(new + [["#done", "|".join(key)]])
            totals.update(groups=1, done=done, skipped=skipped)
            print(f"{totals['groups']}/{len(todo)} groups | {os.path.basename(key[0])} {key[1]}: {done} runs, "
                  f"{skipped} skipped after a failure", flush=True)

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for future in [pool.submit(work, key) for key in todo]:
            future.result()
    print(f"shard {args.shard} done: {totals['done']} runs, {totals['skipped']} skipped", flush=True)


def mapfast_records(map_file, scen_file, n_agents, seconds):
    """The MAPFAST records of an instance, given each solver's `seconds` (None if unsolved), or None if no solver
    solved it: MAPFAST labels an instance with its fastest solver"""
    solved = {solver: t for solver, t in seconds.items() if t is not None}
    if not solved:
        return None
    width, height, obstacles, agents = load_instance(map_file, scen_file)
    agents = agents[:n_agents]
    return {
        "yaml_details": {"SOLVER": min(solved, key=solved.get),
                         **{solver: -1 if t is None else round(t, 6) for solver, t in seconds.items()}},
        "agent_details": {"starts": [list(start) for start, _ in agents], "goals": [list(goal) for _, goal in agents]},
        "map_details": {"no_agents": n_agents, "mp_dim": [width, height], "no_obs": len(obstacles)},
    }


def write_atomically(path, write):
    with open(path + ".tmp", "w", newline="") as out:
        write(out)
    os.replace(path + ".tmp", path)


def merge(args, runs, groups):
    new, finished, reasons = read_shards(shard_paths(args.name))
    missing = [key for key in groups if key not in finished]
    if missing:
        sys.exit(f"{len(missing)} of {len(groups)} groups were not finished by any shard (e.g. "
                 f"{os.path.basename(missing[0][0])} {missing[0][1]}): rerun those shards")
    merged = {**runs, **new}
    failures = collections.Counter(reasons.get(key, "unknown") for key in
                                   filter(None, (failure(*key, counts, merged) for key, (_, counts) in groups.items())))
    print(f"failed runs (one per group, the larger counts were skipped after them): {describe(failures)}")
    if any(reason not in FINAL_FAILURES for reason in failures):
        print("warning: some failures say nothing about the instance; run the shards again with --retry", file=sys.stderr)

    out = experiment_dir(args.name)
    write_atomically(os.path.join(out, RESULTS_FILE),
                     lambda f: csv.writer(f).writerows([HEADER] + [merged[key] for key in sorted(merged)]))
    solvers = sorted({solver for _, solver in merged})
    seconds = {}
    for (instance, solver), row in merged.items():
        seconds.setdefault(instance, dict.fromkeys(solvers))[solver] = float(row[3]) if row[3] else None
    details = {name: {} for name in MAPFAST_DETAILS}
    for instance, times in seconds.items():
        for name, record in (mapfast_records(*scenario_file(instance), times) or {}).items():
            details[name][instance] = record
    for name, records in details.items():
        write_atomically(os.path.join(out, f"{name}.json"), lambda f: json.dump(records, f))
    print(f"wrote {out}/: {len(merged)} runs, {len(details['yaml_details'])} MAPFAST problems")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["plan", "run", "merge"])
    parser.add_argument("name", help="the experiment, a name in experiments/ or a folder")
    parser.add_argument("--timeout", type=float, required=True, help="time limit of a run, in seconds")
    parser.add_argument("--agents", type=int, nargs="*", default=[], help="agent counts to run")
    parser.add_argument("--solvers", nargs="+", default=SOLVERS)
    parser.add_argument("--maps", nargs="*", default=None, help="maps of benchmarks/ (default: all, none with --reuse)")
    parser.add_argument("--reuse", default=None, help="earlier experiment whose solved runs to keep")
    parser.add_argument("--shard", type=int, default=0)
    parser.add_argument("--shards", type=int, default=1)
    parser.add_argument("--jobs", type=int, default=1, help="runs at the same time (at most the number of cores)")
    parser.add_argument("--retry", action="store_true", help="also redo the finished groups whose run was killed or "
                                                             "crashed (run, plan)")
    args = parser.parse_args()

    runs = read_runs(os.path.join(experiment_dir(args.reuse), RESULTS_FILE)) if args.reuse else {}
    runs = {key: row for key, row in runs.items() if key[1] in args.solvers}
    maps = args.maps if args.maps is not None else [] if args.reuse else sorted(
        name for name in os.listdir(BENCHMARK_DIR) if glob.glob(os.path.join(BENCHMARK_DIR, name, "*.map")))
    groups = plan(runs, args.agents, args.solvers, maps)
    if args.command == "plan":
        rows, finished, reasons = read_shards(shard_paths(args.name))
        known = {**runs, **rows}
        todo = [key for key in groups if key not in finished]
        if args.retry:
            todo += to_retry(groups, [key for key in groups if key in finished], known, reasons)
        left = sum(1 for key in todo for n in groups[key][1]
                   if known.get((utils.instance_name(key[0], n), key[1]), [""] * 4)[3] == "")
        print(f"{len(groups)} (scenario file, solver) groups, {len(finished)} finished, {len(todo)} to do: {left} runs "
              f"before the skipping, at most {len(todo) * args.timeout / 3600:.0f} CPU-hours of timeouts with it "
              f"(plus the solved runs)")
    else:
        (run_shard if args.command == "run" else merge)(args, runs, groups)
