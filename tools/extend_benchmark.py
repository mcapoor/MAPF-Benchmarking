"""Extend a finished experiment (experiments/<experiment>/results.csv, merged by tools/aggregate.py) into a new one,
without rerunning what it already has:

    --timeout T     rerun every run it did not solve, now with a time limit of T seconds
    --agents N ...  also run these agent counts, on every scenario file with at least that many agents

Runs that were solved are kept as they are: a run that finished within the old limit takes the same time with a longer
one. The runs of each (scenario file, solver) go in increasing agent count, and once one fails within T seconds the
larger counts of that scenario file are counted as failures without running them (more agents of the same scenario
file only make the problem harder), which bounds the cost of a solver that cannot keep up to one timeout per scenario
file.

    python tools/extend_benchmark.py plan movingai_10-200-agents_60s --timeout 300 --agents 250 300 350 400 450 500
    python tools/extend_benchmark.py run movingai_10-200-agents_60s --timeout 300 --agents 250 300 350 400 450 500 \
        --name movingai_10-500-agents_300s --shard 0 --shards 20 --jobs 12
    python tools/extend_benchmark.py merge movingai_10-200-agents_60s --timeout 300 --agents 250 300 350 400 450 500 \
        --name movingai_10-500-agents_300s

`run` does one shard of the (scenario file, solver) groups and writes experiments/<name>/shards/shard<i>.csv as each
group ends, so a job that is cut off keeps the groups it finished (and a resubmitted shard skips them). `merge` (with
the same --timeout and --agents) writes experiments/<name>/results.csv, the original runs with the new ones in place,
and the MAPFAST files experiments/<name>/{yaml,agent,map}_details.json, as aggregate.py would; it warns about groups
no shard finished, whose runs keep their old results. MAPFAST's scripts/generate_benchmark.sh submits all of it on
Slurm. Experiments are names in experiments/ or folders.
"""
import argparse
import concurrent.futures
import csv
import glob
import os
import re
import sys
import threading

# allow running as a script (python tools/extend_benchmark.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from benchmark import BENCHMARK_DIR, MAPFAST_DETAILS, RESULTS_FILE, experiment_dir, load_instance, mapfast_records, time_solver, \
    write_mapfast
from tools import reloc_wrappers, utils

HEADER = ["solver", "n_agents", "map", "seconds", "cost", "makespan", "solver_cost"]
INSTANCE = re.compile(r"^((.+)-(?:random|even)-\d+)_agents(\d+)\.yaml$")


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
    scen, map_name, n_agents = match.group(1), match.group(2), int(match.group(3))
    directory = os.path.join(BENCHMARK_DIR, map_name)
    return os.path.join(directory, map_name + ".map"), os.path.join(directory, scen + ".scen"), n_agents


def plan(runs, timeout, agents, solvers):
    """The work, as {(scenario file, solver): (map file, [agent count, ...])}: per group, every agent count it has and
    the new ones, in increasing order; `run_group` decides which of them to run."""
    groups = {}
    scens = {}
    for instance, solver in runs:
        map_file, scen, n_agents = scenario_file(instance)
        scens[scen] = map_file
        if solver in solvers:
            groups.setdefault((scen, solver), (map_file, set()))[1].add(n_agents)
    for scen, map_file in scens.items():
        available = len(load_instance(map_file, scen)[3])
        for solver in solvers:
            counts = groups.setdefault((scen, solver), (map_file, set()))[1]
            counts.update(n for n in agents if n <= available)
    return {key: (map_file, sorted(counts)) for key, (map_file, counts) in sorted(groups.items())}


def solver_input(scen, n_agents, solver, map_file):
    """The problem in the format `solver` reads, converted if it is not cached yet (as benchmark.py does)"""
    if solver in utils.YAML_SOLVERS:
        return utils.libmrp_problem(scen, n_agents, map_file)
    if solver == "reloc":
        reloc_wrappers.convert(scen, map_file, n_agents, reloc_wrappers.cpf_file(scen, n_agents))
    return (scen, n_agents)


def run_group(scen, solver, map_file, counts, runs, timeout):
    """The new CSV rows of one (scenario file, solver) group: a run for each count that is new or was not solved,
    until one fails; the counts after that are written as failures without running them.

    Returns (rows, runs done, runs skipped)"""
    rows, done, skipped = [], 0, 0
    failed = False
    for n_agents in counts:
        instance = utils.instance_name(scen, n_agents)
        old = runs.get((instance, solver))
        if old is not None and old[3] != "":
            continue
        if failed:
            rows.append([solver, n_agents, instance, "", "", "", ""])
            skipped += 1
            continue
        run = time_solver(solver_input(scen, n_agents, solver, map_file), solver, timeout)
        done += 1
        if run is None:
            failed = True
            rows.append([solver, n_agents, instance, "", "", "", ""])
        else:
            rows.append([solver, n_agents, instance, f"{run.seconds:.6f}", run.cost, run.makespan,
                         "" if run.solver_cost is None else run.solver_cost])
    return rows, done, skipped


def shard_path(name, shard):
    return os.path.join(experiment_dir(name), "shards", f"shard{shard}.csv")


def run_shard(args, runs, groups):
    keys = list(groups)[args.shard::args.shards]
    path = shard_path(args.name, args.shard)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    finished = set()
    if os.path.exists(path):
        # a resubmitted shard skips the groups its earlier job finished
        with open(path, newline="") as shard_file:
            finished = {tuple(row[1].split("|", 1)) for row in csv.reader(shard_file) if row and row[0] == "#done"}
    lock = threading.Lock()
    totals = {"groups": 0, "done": 0, "skipped": 0}

    def work(key):
        scen, solver = key
        map_file, counts = groups[key]
        rows, done, skipped = run_group(scen, solver, map_file, counts, runs, args.timeout)
        with lock:
            with open(path, "a", newline="") as shard_file:
                writer = csv.writer(shard_file)
                writer.writerows(rows)
                # marks the group finished, so that merge knows its runs are final
                writer.writerow(["#done", f"{scen}|{solver}"])
            totals["groups"] += 1
            totals["done"] += done
            totals["skipped"] += skipped
            print(f"{totals['groups']}/{len(todo)} groups | {os.path.basename(scen)} {solver}: {done} runs, "
                  f"{skipped} skipped after a failure", flush=True)

    todo = [key for key in keys if key not in finished]
    print(f"shard {args.shard}/{args.shards}: {len(keys)} groups, {len(keys) - len(todo)} finished before", flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for future in [pool.submit(work, key) for key in todo]:
            future.result()
    print(f"shard {args.shard} done: {totals['done']} runs, {totals['skipped']} skipped", flush=True)


def merge(args, runs, groups):
    new, finished = {}, set()
    for path in sorted(glob.glob(os.path.join(glob.escape(experiment_dir(args.name)), "shards", "shard*.csv"))):
        with open(path, newline="") as shard_file:
            for row in csv.reader(shard_file):
                if row and row[0] == "#done":
                    finished.add(tuple(row[1].split("|", 1)))
                elif row:
                    new[(row[2], row[0])] = row
    missing = [key for key in groups if key not in finished]
    if missing:
        print(f"warning: {len(missing)} of {len(groups)} groups were not finished by any shard (e.g. "
              f"{os.path.basename(missing[0][0])} {missing[0][1]}); their runs keep the old results, so rerun "
              f"those shards", file=sys.stderr)
    merged = {**runs, **new}

    out = experiment_dir(args.name)
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, RESULTS_FILE)
    with open(path + ".tmp", "w", newline="") as results_file:
        writer = csv.writer(results_file)
        writer.writerow(HEADER)
        writer.writerows(merged[key] for key in sorted(merged))
    os.replace(path + ".tmp", path)

    solvers = sorted({solver for _, solver in merged})
    seconds = {}
    for (instance, solver), row in merged.items():
        seconds.setdefault(instance, {s: None for s in solvers})[solver] = float(row[3]) if row[3] else None
    details = {name: {} for name in MAPFAST_DETAILS}
    for instance, times in seconds.items():
        map_file, scen, n_agents = scenario_file(instance)
        for name, record in (mapfast_records(map_file, scen, n_agents, times) or {}).items():
            details[name][instance] = record
    write_mapfast(out, details)
    replaced = sum(1 for key in new if key in runs)
    print(f"wrote {out}/: {len(merged)} runs ({replaced} rerun, {len(new) - replaced} new), "
          f"{len(details['yaml_details'])} MAPFAST problems")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["plan", "run", "merge"])
    parser.add_argument("source", help="experiment to extend, e.g. movingai_10-200-agents_60s")
    parser.add_argument("--timeout", type=float, required=True, help="time limit of the new runs, in seconds")
    parser.add_argument("--agents", type=int, nargs="*", default=[], help="agent counts to add")
    parser.add_argument("--solvers", nargs="+", default=None, help="solvers to extend (default: every solver of the benchmark)")
    parser.add_argument("--name", default=None, help="the new experiment, e.g. movingai_10-500-agents_300s (run, merge)")
    parser.add_argument("--shard", type=int, default=0)
    parser.add_argument("--shards", type=int, default=1)
    parser.add_argument("--jobs", type=int, default=1, help="runs at the same time (run; at most the number of cores)")
    args = parser.parse_args()

    runs = read_runs(os.path.join(experiment_dir(args.source), RESULTS_FILE))
    solvers = args.solvers or sorted({solver for _, solver in runs})
    groups = plan(runs, args.timeout, args.agents, solvers)
    if args.command == "plan":
        reruns = sum(1 for key, row in runs.items() if key[1] in solvers and row[3] == "")
        new = sum(1 for (scen, solver), (_, counts) in groups.items() for n in counts
                  if (utils.instance_name(scen, n), solver) not in runs)
        print(f"{len(runs)} runs of {len(solvers)} solvers; {reruns} unsolved to rerun and {new} new runs, "
              f"in {len(groups)} (scenario file, solver) groups")
        print(f"at most {(reruns + new) * args.timeout / 3600:.0f} CPU-hours without the skipping, "
              f"and at most {len(groups) * args.timeout / 3600:.0f} CPU-hours of timeouts with it (plus the solved runs)")
    else:
        if not args.name:
            sys.exit("give --name")
        (run_shard if args.command == "run" else merge)(args, runs, groups)
