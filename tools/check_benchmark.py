"""Checks that the benchmark stack works, run by test_cluster.sh before the
full ./benchmark.sh run spends the compute quota.

    python tools/check_benchmark.py preflight
    python tools/check_benchmark.py results all_1234 --maps empty-8-8 den312d \\
        --agents 2 4 --instances 2 --strict empty-8-8

preflight (on the login node, in seconds) checks that every solver of
benchmark.sh is built and finds its shared libraries, that every map of
benchmarks/ has its map and scenario files and loads, and that Slurm and the
output directories are there.

results checks the merged output of a small ./benchmark.sh run (see
tools/aggregate.py): every map has a complete set of runs (so no job crashed
or ran out of time), every solver solved something, every solver solved every
problem of the --strict maps (small problems that each solver must solve)
with the optimal solvers agreeing on the sum of costs, and the MAPFAST files
hold exactly the problems some solver solved.

Both print what they found and exit non-zero on any failure.
"""
import argparse
import collections
import csv
import json
import os
import re
import shutil
import subprocess
import sys

# allow running as a script (python tools/check_benchmark.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from benchmark import (BENCHMARK_DIR, MAPFAST_DETAILS, OUTPUT_DIR, ROOT_DIR, SCEN_TYPES, SOLVERS, load_instance,
                       scenario_files)
from tools.solve_and_visualize import solver_binary

BENCHMARK_SH = os.path.join(ROOT_DIR, "benchmark.sh")
# the scenario files each map has of each type (see benchmarks/README.md)
SCENS_PER_TYPE = 25
# the solvers that return a solution of minimum sum of costs, so they must
# agree on the cost of every problem they solve
OPTIMAL_SOLVERS = {"cbs", "cbsh2_rtc", "cbsh2_rtc_ch", "cbsh2_rtc_bp", "cbsh2_rtc_chbp", "bcp2"}
# the map and agent count of an instance name, e.g. empty-8-8 and 10 for
# empty-8-8-random-1_agents10.yaml (see utils.instance_name)
INSTANCE_NAME = re.compile(r"^(.*)-(?:%s)-\d+_agents(\d+)\.yaml$" % "|".join(SCEN_TYPES))


def benchmark_sh_solvers():
    """The SOLVERS that benchmark.sh runs, read from the script so that the
    two cannot drift apart."""
    with open(BENCHMARK_SH) as script:
        match = re.search(r"^SOLVERS=\((.*?)\)", script.read(), re.MULTILINE | re.DOTALL)
    return [word for line in match.group(1).splitlines() for word in line.split("#")[0].split()]


def benchmark_maps():
    """The map directories of BENCHMARK_DIR."""
    return sorted(name for name in os.listdir(BENCHMARK_DIR) if os.path.isdir(os.path.join(BENCHMARK_DIR, name)))


def preflight():
    """The problems found before submitting anything (see the module doc)."""
    problems = []
    for solver in benchmark_sh_solvers():
        if solver not in SOLVERS:
            problems.append(f"benchmark.sh runs {solver}, which benchmark.py does not know")
            continue
        binary = solver_binary(solver)
        if not os.access(binary, os.X_OK):
            problems.append(f"{solver}: {binary} is not built (run ./build.sh)")
            continue
        # a library of a module that is not loaded (e.g. boost, yaml-cpp, gurobi)
        ldd = subprocess.run(["ldd", binary], capture_output=True, text=True)
        missing = [line.split()[0] for line in ldd.stdout.splitlines() if "not found" in line]
        if missing:
            problems.append(f"{solver}: {binary} cannot find {', '.join(missing)} (module load ...?)")
    print(f"checked the binaries of {len(benchmark_sh_solvers())} solvers", flush=True)

    maps = benchmark_maps()
    for name in maps:
        for scen_type in SCEN_TYPES:
            try:
                map_file, scens = scenario_files(os.path.join(BENCHMARK_DIR, name), scen_type)
                if len(scens) != SCENS_PER_TYPE:
                    problems.append(f"{name}: {len(scens)} {scen_type} scenario files, not {SCENS_PER_TYPE}")
                if not load_instance(map_file, scens[0])[3]:
                    problems.append(f"{scens[0]} has no agents")
            except (ValueError, OSError) as error:
                problems.append(f"{name}: {error}")
    print(f"checked the files of {len(maps)} maps", flush=True)

    if not shutil.which("sbatch"):
        problems.append("sbatch is not on the PATH (run this on a login node of the cluster)")
    for directory in (OUTPUT_DIR, os.path.join(ROOT_DIR, "logs"), os.path.join(ROOT_DIR, "cache")):
        os.makedirs(directory, exist_ok=True)
        if not os.access(directory, os.W_OK):
            problems.append(f"{directory} is not writable")
    return problems


def check_results(prefix, maps, agents, instances, strict):
    """The problems of the merged output/<prefix>_* files of a run of
    `instances` scenario files per map with the agent counts `agents`, and
    the warnings (see the module doc). Also prints the problems each solver
    solved on each map."""
    problems, warnings = [], []
    solvers = benchmark_sh_solvers()
    results = os.path.join(OUTPUT_DIR, f"{prefix}_results.csv")
    if not os.path.exists(results):
        return [f"{results} is missing: the aggregate job failed (see logs/mapf_benchmark_aggregate_*.err)"], []
    with open(results, newline="") as results_file:
        rows = list(csv.DictReader(results_file))

    # (map, solver) -> {(instance, n_agents): row}
    runs = collections.defaultdict(dict)
    for row in rows:
        match = INSTANCE_NAME.match(row["map"])
        if not match:
            problems.append(f"unexpected instance name {row['map']}")
            continue
        runs[match.group(1), row["solver"]][row["map"], int(row["n_agents"])] = row

    print(f"{'':>22} " + " ".join(f"{name[:12]:>12}" for name in maps))
    expected = len(agents) * instances
    for solver in solvers:
        solved_counts = []
        for name in maps:
            map_runs = runs.get((name, solver), {})
            solved = [row for row in map_runs.values() if row["seconds"]]
            solved_counts.append(f"{len(solved)}/{len(map_runs)}")
            if len(map_runs) != expected or {n for _, n in map_runs} != set(agents):
                problems.append(f"{name}, {solver}: {len(map_runs)} of {expected} runs, so its job did not finish "
                                f"(see logs/mapf_benchmark_{name}_*.err)")
            elif len(solved) < len(map_runs):
                (problems if name in strict else warnings).append(
                    f"{name}, {solver}: solved {len(solved)} of {len(map_runs)}")
        print(f"{solver:>22} " + " ".join(f"{count:>12}" for count in solved_counts))
        if not any(row["seconds"] for name in maps for row in runs.get((name, solver), {}).values()):
            problems.append(f"{solver} solved nothing, so it does not run on the compute nodes "
                            "(bcp2: is there a Gurobi license on the node?)")

    # the optimal solvers must agree on the sum of costs of each problem
    for name in strict:
        costs = collections.defaultdict(dict)
        for solver in OPTIMAL_SOLVERS & set(solvers):
            for problem, row in runs.get((name, solver), {}).items():
                if row["cost"]:
                    costs[problem][solver] = int(row["cost"])
        for (instance, n_agents), by_solver in sorted(costs.items()):
            if len(set(by_solver.values())) > 1:
                problems.append(f"{instance}: the optimal solvers disagree on the sum of costs: {by_solver}")

    # the MAPFAST files hold the problems some solver solved
    solved_problems = {row["map"] for row in rows if row["seconds"]}
    for detail in MAPFAST_DETAILS:
        path = os.path.join(OUTPUT_DIR, f"{prefix}_{detail}.json")
        if not os.path.exists(path):
            problems.append(f"{path} is missing")
            continue
        with open(path) as details_file:
            records = json.load(details_file)
        if set(records) != solved_problems:
            problems.append(f"{path} has {len(records)} problems, not the {len(solved_problems)} solved ones")
    return problems, warnings


def report(problems, warnings=()):
    """Print the warnings and problems and exit with PASS or FAIL."""
    for warning in warnings:
        print(f"warning: {warning}")
    for problem in problems:
        print(f"FAIL: {problem}")
    print("FAIL" if problems else "PASS")
    sys.exit(1 if problems else 0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("preflight", help="check the setup before submitting anything")
    results_parser = commands.add_parser("results", help="check the merged output of a small benchmark run")
    results_parser.add_argument("prefix", help="prefix of the merged files in output/, e.g. all_1234")
    results_parser.add_argument("--maps", nargs="+", required=True, help="the maps the run covered")
    results_parser.add_argument("--agents", type=int, nargs="+", required=True, help="the agent counts of the run")
    results_parser.add_argument("--instances", type=int, required=True, help="the scenario files per map of the run")
    results_parser.add_argument("--strict", nargs="*", default=[],
                                help="maps whose problems every solver must solve")
    args = parser.parse_args()

    if args.command == "preflight":
        report(preflight())
    report(*check_results(args.prefix, args.maps, args.agents, args.instances, args.strict))
