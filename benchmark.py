import argparse
import collections
import concurrent.futures
import csv
import json
import os
import re
import subprocess
import sys

import matplotlib.pyplot as plt
import yaml

# allow running as a script (python benchmark.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tools import solver_wrappers
from tools.solve_and_visualize import GRID_SOLVERS, solve

INSTANCE = re.compile(r"agents(\d+)_ex(\d+)\.yaml$")

# the plot, the raw results and the MAPFAST files of a run are written here
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")

# the json files MAPFAST (the algorithm selector) reads, see its README
MAPFAST_DETAILS = ("yaml_details", "agent_details", "map_details")

# mcts_nonoverlap looks for vertex-disjoint paths, so most instances of the
# MAPF scenarios have no solution for it: it is only run when asked for
DEFAULT_SOLVERS = [solver for solver in GRID_SOLVERS if solver != "mcts_nonoverlap"]


def find_instances(scenario):
    """Group the instance files of a benchmark scenario directory by agent count.

    Returns {n_agents: [file, ...]}, e.g. for benchmark/8x8_obst12 the m files
    map_8by8_obst12_agents<n>_ex<0..m-1>.yaml of each batch of n agents. Other
    files in the directory are ignored.
    """
    batches = collections.defaultdict(list)
    for name in sorted(os.listdir(scenario)):
        match = INSTANCE.search(name)
        if match:
            batches[int(match.group(1))].append(os.path.join(scenario, name))
    return dict(sorted(batches.items()))


def time_solver(instance, solver, timeout):
    """Seconds `solver` takes to solve `instance`, or None if it fails, crashes
    or exceeds `timeout` seconds."""
    if solver.endswith("_ta"):
        # already converted by benchmark(), so this only looks up the cached file
        instance = solver_wrappers.ensure_potential_goals(instance)
    try:
        result, seconds = solve(instance, solver, timeout=timeout, quiet=True, report=False)
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        return None
    return seconds if result is not None else None


def benchmark(scenario, solvers, timeout=10, max_instances=None, jobs=1, on_batch=None):
    """For every batch of n agents, run each solver on the (up to
    `max_instances`) example maps of the batch, `jobs` runs at a time.

    Runs share the machine, so with jobs > 1 the times include some contention
    (and more runs hit the timeout) compared to running them one by one but the 
    total benchmark runtime is significantly lower.

    `on_batch(n_agents, files, times)` is called after each batch finishes, with
    the results of all batches so far to log them as they come in.

    Returns {solver: {n_agents: [seconds or None per map]}}.
    """
    batches = {n: files[:max_instances] for n, files in find_instances(scenario).items()}
    if not batches:
        raise ValueError(f"no *_agents<n>_ex<k>.yaml instances in {scenario}")

    if any(solver.endswith("_ta") for solver in solvers):
        # convert up front and serially: both task assignment solvers share the
        # converted file, and this keeps the conversion out of the timed runs
        for files in batches.values():
            for file in files:
                solver_wrappers.ensure_potential_goals(file)

    times = {solver: {} for solver in solvers}
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        for n_agents, files in batches.items():
            # threads are enough: each run just waits on its solver subprocess
            runs = {solver: [pool.submit(time_solver, file, solver, timeout) for file in files]
                    for solver in solvers}
            for solver in solvers:
                times[solver][n_agents] = [run.result() for run in runs[solver]]
                solved = [t for t in times[solver][n_agents] if t is not None]
                mean = f", mean {sum(solved) / len(solved):.3f} s" if solved else ""
                print(f"{solver:>22} {n_agents:>4} agents: {len(solved)}/{len(files)} solved{mean}",
                    flush=True)
            if on_batch:
                on_batch(n_agents, files, times)
    return times


def average(times):
    """Mean time over the solved maps of each batch, plus the fraction solved.

    Returns ({solver: {n: mean_seconds}}, {solver: {n: fraction_solved}});
    batches where a solver solved nothing have no mean.
    """
    means, success = {}, {}
    for solver, batches in times.items():
        means[solver], success[solver] = {}, {}
        for n_agents, runs in batches.items():
            solved = [t for t in runs if t is not None]
            success[solver][n_agents] = len(solved) / len(runs)
            if solved:
                means[solver][n_agents] = sum(solved) / len(solved)
    return means, success


def plot(times, title, output=None, show=True):
    """Plot mean time vs. number of agents for each solver (top) and the
    fraction of maps it solved within the timeout (bottom)."""
    means, success = average(times)
    fig, (time_ax, success_ax) = plt.subplots(2, 1, sharex=True, figsize=(7, 8))
    for solver in times:
        time_ax.plot(list(means[solver]), list(means[solver].values()), marker="o", label=solver)
        success_ax.plot(list(success[solver]), list(success[solver].values()), marker="o", label=solver)
    time_ax.set_yscale("log")
    time_ax.set_ylabel("mean time over solved maps (s)")
    time_ax.set_title(title)
    time_ax.legend()
    time_ax.grid(True, alpha=0.3)
    success_ax.set_ylabel("fraction of maps solved")
    success_ax.set_xlabel("number of agents")
    success_ax.set_ylim(-0.05, 1.05)
    success_ax.grid(True, alpha=0.3)
    fig.tight_layout()
    if output:
        fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def append_results(path, n_agents, files, times):
    """Append the raw runs of one batch to the CSV `path` (seconds is empty for
    a run that was not solved), writing the header if the file is new."""
    new_file = not os.path.exists(path)
    with open(path, "a", newline="") as results_file:
        writer = csv.writer(results_file)
        if new_file:
            writer.writerow(["solver", "n_agents", "map", "seconds"])
        for solver, batches in times.items():
            for file, seconds in zip(files, batches[n_agents]):
                writer.writerow([solver, n_agents, os.path.basename(file),
                                 "" if seconds is None else f"{seconds:.6f}"])


def mapfast_records(file, seconds):
    """The MAPFAST records of the instance `file`, given the `seconds` (or None
    for a run that was not solved) that each solver took on it.

    Returns {name: record} for the names in MAPFAST_DETAILS, or None if no
    solver solved the instance: MAPFAST labels an instance with its fastest
    solver, so it has nothing to learn from one that nobody solved.
    """
    solved = {solver: t for solver, t in seconds.items() if t is not None}
    if not solved:
        return None
    with open(file) as stream:
        problem = yaml.load(stream, Loader=solver_wrappers.YAML_LOADER)
    agents = problem["agents"]
    return {
        "yaml_details": {"SOLVER": min(solved, key=solved.get),
                         **{solver: -1 if t is None else round(t, 6) for solver, t in seconds.items()}},
        "agent_details": {"starts": [agent["start"] for agent in agents],
                          "goals": [agent["goal"] for agent in agents]},
        "map_details": {"no_agents": len(agents),
                        "mp_dim": problem["map"]["dimensions"],
                        "no_obs": len(problem["map"].get("obstacles") or [])},
    }


def add_mapfast_batch(details, n_agents, files, times):
    """Add the maps of one batch that some solver solved to `details`, a
    {name: {map: record}} dict with an entry for each of MAPFAST_DETAILS."""
    for i, file in enumerate(files):
        records = mapfast_records(file, {solver: batches[n_agents][i] for solver, batches in times.items()})
        for name, record in (records or {}).items():
            details[name][os.path.basename(file)] = record


def write_mapfast(directory, prefix, details):
    """Write each of the MAPFAST `details` to <directory>/<prefix>_<name>.json.

    A file is replaced in one step, so a run that is killed while writing still
    leaves the complete files of the previous batch."""
    os.makedirs(directory, exist_ok=True)
    for name, records in details.items():
        path = os.path.join(directory, f"{prefix}_{name}.json")
        with open(path + ".tmp", "w") as details_file:
            json.dump(records, details_file)
        os.replace(path + ".tmp", path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('scenario', help='benchmark scenario directory, e.g. benchmark/8x8_obst12')
    parser.add_argument('--solvers', nargs='+', choices=list(GRID_SOLVERS), default=DEFAULT_SOLVERS,
                        help=f"solvers to compare (default: {' '.join(DEFAULT_SOLVERS)}; mcts_nonoverlap only if listed)")
    parser.add_argument('--timeout', type=float, default=10.0, help='seconds allowed per solver run')
    parser.add_argument('--jobs', type=int, default=1, help='solver runs to execute at the same time (more than the number of physical cores distorts the timings)')
    parser.add_argument('--max-instances', type=int, default=None, help='only use the first m maps of every batch')
    parser.add_argument('--name', default=None,
                        help='prefix of the files written to output/ (default: the scenario directory name): '
                             '<name>_times.png, <name>_results.csv and the MAPFAST dataset files <name>_yaml_details.json, '
                             '<name>_agent_details.json and <name>_map_details.json, all updated after every batch')
    parser.add_argument('--output', default=None, help='plot file (default: output/<name>_times.png)')
    parser.add_argument('--results', default=None, help='CSV file of the raw times, started afresh by every run (default: output/<name>_results.csv)')
    parser.add_argument('--no-show', action='store_true', help='do not open the plot window')
    args = parser.parse_args()

    scenario = os.path.normpath(args.scenario)
    title = os.path.basename(scenario)
    prefix = args.name or title
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output = args.output or os.path.join(OUTPUT_DIR, prefix + "_times.png")
    results = args.results or os.path.join(OUTPUT_DIR, prefix + "_results.csv")
    if os.path.exists(results):
        os.remove(results)  # append_results adds to it batch by batch
    mapfast = {name: {} for name in MAPFAST_DETAILS}

    def on_batch(n_agents, files, times):
        # keep the plot, the raw results and the MAPFAST files up to date, so
        # that a run that is killed part way (e.g. by a cluster time limit)
        # still leaves all of them
        append_results(results, n_agents, files, times)
        add_mapfast_batch(mapfast, n_agents, files, times)
        write_mapfast(OUTPUT_DIR, prefix, mapfast)
        plot(times, title, output, show=False)

    times = benchmark(scenario, args.solvers, args.timeout, args.max_instances, args.jobs, on_batch)
    maps = sum(len(runs) for runs in next(iter(times.values())).values())
    print(f"wrote {len(mapfast['yaml_details'])} of {maps} maps to the MAPFAST files in {OUTPUT_DIR} "
          "(the others were solved by no solver)")
    plot(times, title, output, show=not args.no_show)
