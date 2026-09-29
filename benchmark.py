import argparse
import collections
import concurrent.futures
import csv
import functools
import json
import os
import re
import subprocess
import sys

import matplotlib.pyplot as plt

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
# allow running as a script (python benchmark.py), not only as a module
sys.path.insert(0, ROOT_DIR)

from tools import reloc_wrappers, standard_benchmark_converter, utils
from tools.solve_and_visualize import GRID_SOLVERS, MOVINGAI_SOLVERS, solve, solver_binary

# the MovingAI benchmarks: one directory per map, with the map and its random
# and even scenario files
BENCHMARK_DIR = os.path.join(ROOT_DIR, "benchmarks")
# the kinds of MovingAI scenario files, e.g. empty-8-8-random-3.scen
SCEN_TYPES = ("random", "even")
# the type and number of a MovingAI scenario file, e.g. random and 3 for
# empty-8-8-random-3.scen
SCEN_NAME = re.compile(r"-(\w+)-(\d+)\.scen$")

# the plot, the raw results and the MAPFAST files of a run are written here
OUTPUT_DIR = os.path.join(ROOT_DIR, "output")

# a solved run: the wall time in seconds, the sum of costs and makespan of the
# solution (see utils.solution_metrics) and the cost the solver reported
# itself (None if it reports none)
Run = collections.namedtuple("Run", "seconds cost makespan solver_cost")

# the json files MAPFAST (the algorithm selector) reads, see its README
MAPFAST_DETAILS = ("yaml_details", "agent_details", "map_details")

# every solver that runs on the MovingAI problems
SOLVERS = [*GRID_SOLVERS, *MOVINGAI_SOLVERS]
# mcts_nonoverlap looks for vertex-disjoint paths, so most instances of the
# MAPF scenarios have no solution for it: it is only run when asked for
DEFAULT_SOLVERS = [solver for solver in SOLVERS if solver != "mcts_nonoverlap"]


def scenario_dir(scenario):
    """The scenario directory `scenario` names: a directory, or the name of a
    map in BENCHMARK_DIR (e.g. empty-8-8)."""
    if not os.path.isdir(scenario) and os.path.isdir(os.path.join(BENCHMARK_DIR, scenario)):
        return os.path.join(BENCHMARK_DIR, scenario)
    return scenario


def scenario_files(scenario, scen_type="random"):
    """The map and the scenario files of type `scen_type` (one of SCEN_TYPES)
    of a MovingAI scenario directory.

    A scenario directory holds one map and its scenario files, e.g.
    empty-8-8/ holds empty-8-8.map, empty-8-8-random-<k>.scen and
    empty-8-8-even-<k>.scen for k = 1, ..., 25. Returns (map_file,
    [scen_file, ...]), sorted by k.
    """
    names = os.listdir(scenario)
    maps = [name for name in names if name.endswith(".map")]
    if len(maps) != 1:
        raise ValueError(f"{scenario} must contain exactly one .map file, not {len(maps)}")
    scens = [(int(match.group(2)), name) for name in names
             for match in [SCEN_NAME.search(name)] if match and match.group(1) == scen_type]
    if not scens:
        raise ValueError(f"no {scen_type} scenario files (<map>-{scen_type}-<k>.scen) in {scenario}")
    return os.path.join(scenario, maps[0]), [os.path.join(scenario, name) for _, name in sorted(scens)]


@functools.cache
def load_instance(map_file, scen_file):
    """(width, height, obstacles, [(start, goal) of every agent]) of a MovingAI
    map and scenario file, with the agents in file order."""
    width, height, obstacles = standard_benchmark_converter.load_map_file(map_file)
    agents = standard_benchmark_converter.load_scenario_file(scen_file, obstacles, width, height)
    return width, height, obstacles, agents


def agent_counts(scenario, scen_type="random"):
    """The default agent counts of a scenario directory: 10, 20, 30, ... up to
    the number of agents of its smallest scenario file of type `scen_type`."""
    map_file, scens = scenario_files(scenario, scen_type)
    most = min(len(load_instance(map_file, scen)[3]) for scen in scens)
    return list(range(10, most + 1, 10))


def find_instances(scenario, solver, agents=None, max_instances=None, scen_type="random"):
    """The input of `solver` for each problem of a MovingAI scenario directory,
    grouped by agent count.

    The problem with n agents of a scenario file is its first n agents. A batch
    of n agents has that problem for each of the first `max_instances` (by
    default all) scenario files of type `scen_type`, for every n in `agents`
    (by default agent_counts(scenario, scen_type)).

    A solver that reads libMultiRobotPlanning's format (utils.YAML_SOLVERS,
    which includes mcts_nonoverlap) gets the YAML file of each problem, converted
    with tools/standard_benchmark_converter.py into cache/libMultiRobotPlanning/
    <map>/ unless it is there already. Any other solver gets (scen_file,
    n_agents), to read the MovingAI files itself.

    Returns {n_agents: [input, ...]}, with the scenario files in the same order
    for every solver.
    """
    map_file, scens = scenario_files(scenario, scen_type)
    scens = scens[:max_instances]
    agents = agents or agent_counts(scenario, scen_type)
    for scen in scens:
        available = len(load_instance(map_file, scen)[3])
        if max(agents) > available:
            raise ValueError(f"{scen} has {available} agents, fewer than {max(agents)}")
    if solver not in utils.YAML_SOLVERS:
        return {n: [(scen, n) for scen in scens] for n in agents}
    return {n: [utils.libmrp_problem(scen, n, map_file) for scen in scens] for n in agents}


def time_solver(instance, solver, timeout):
    """The Run of `solver` on `instance`, or None if it fails, crashes or
    exceeds `timeout` seconds.

    The solvers that stop by themselves at the timeout measure it by their own
    clock (e.g. excluding reading the instance), so a solution that arrives
    after more than `timeout` seconds of wall time also counts as unsolved."""
    if solver.endswith("_ta"):
        # already converted by benchmark(), so this only looks up the cached file
        instance = utils.ensure_potential_goals(instance)
    try:
        result, seconds = solve(instance, solver, timeout=timeout, quiet=True, report=False)
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        return None
    if result is None or seconds > timeout:
        return None
    cost, makespan = utils.solution_metrics(result["schedule"])
    return Run(seconds, cost, makespan, (result.get("statistics") or {}).get("cost"))


def benchmark(scenario, solvers, timeout=10, agents=None, max_instances=None, jobs=1, on_batch=None,
              scen_type="random"):
    """For every batch of n agents (see find_instances), run each solver on the
    problems of the batch, `jobs` runs at a time.

    Runs share the machine, so with jobs > 1 the times include some contention
    (and more runs hit the timeout) compared to running them one by one but the 
    total benchmark runtime is significantly lower.

    `on_batch(n_agents, scens, times)` is called after each batch finishes, with
    the scenario files of the batch and the results of all batches so far to
    log them as they come in.

    Returns {solver: {n_agents: [Run or None per scenario file]}}.
    """
    missing = [solver for solver in solvers if not os.access(solver_binary(solver), os.X_OK)]
    if missing:
        raise ValueError(f"not built: {', '.join(missing)} (run ./build.sh)")

    # convert up front and serially, which keeps the conversions out of the
    # timed runs (and the task assignment solvers share their converted file)
    map_file = scenario_files(scenario, scen_type)[0]
    inputs = {solver: find_instances(scenario, solver, agents, max_instances, scen_type) for solver in solvers}
    for solver in solvers:
        for instances in inputs[solver].values():
            for instance in instances:
                if solver.endswith("_ta"):
                    utils.ensure_potential_goals(instance)
                elif solver == "reloc":
                    scen, n_agents = instance
                    reloc_wrappers.convert(scen, map_file, n_agents, reloc_wrappers.cpf_file(scen, n_agents))
    scens = scenario_files(scenario, scen_type)[1][:max_instances]

    times = {solver: {} for solver in solvers}
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        for n_agents in inputs[solvers[0]]:
            # threads are enough: each run just waits on its solver subprocess
            runs = {solver: [pool.submit(time_solver, instance, solver, timeout)
                             for instance in inputs[solver][n_agents]]
                    for solver in solvers}
            for solver in solvers:
                times[solver][n_agents] = [run.result() for run in runs[solver]]
                solved = [run.seconds for run in times[solver][n_agents] if run is not None]
                mean = f", mean {sum(solved) / len(solved):.3f} s" if solved else ""
                print(f"{solver:>22} {n_agents:>4} agents: {len(solved)}/{len(scens)} solved{mean}",
                    flush=True)
            if on_batch:
                on_batch(n_agents, scens, times)
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
            solved = [run.seconds for run in runs if run is not None]
            success[solver][n_agents] = len(solved) / len(runs)
            if solved:
                means[solver][n_agents] = sum(solved) / len(solved)
    return means, success


def average_quality(times):
    """Mean sum of costs and makespan of each solver in each batch, over the
    problems that every solver solved.

    Averaging each solver over only the problems it solved would favor one
    that gives up on the hard ones, so the solvers are compared on the same
    problems. A solver that solves few problems (e.g. mcts_nonoverlap) thus
    leaves few, or none, to compare on.

    Returns ({solver: {n: mean_cost}}, {solver: {n: mean_makespan}}); batches
    without a problem that every solver solved have no mean.
    """
    costs = {solver: {} for solver in times}
    makespans = {solver: {} for solver in times}
    for n_agents in next(iter(times.values()), {}):
        common = [i for i in range(len(next(iter(times.values()))[n_agents]))
                  if all(batches[n_agents][i] is not None for batches in times.values())]
        if not common:
            continue
        for solver, batches in times.items():
            costs[solver][n_agents] = sum(batches[n_agents][i].cost for i in common) / len(common)
            makespans[solver][n_agents] = sum(batches[n_agents][i].makespan for i in common) / len(common)
    return costs, makespans


def plot(times, title, output=None, show=True):
    """Plot, against the number of agents, each solver's mean time (top left)
    and the fraction of maps it solved within the timeout (bottom left), and
    the mean sum of costs (top right) and makespan (bottom right) over the
    maps every solver solved (see average_quality)."""
    means, success = average(times)
    costs, makespans = average_quality(times)
    fig, ((time_ax, cost_ax), (success_ax, makespan_ax)) = plt.subplots(2, 2, sharex=True, figsize=(13, 8))
    # the default color cycle has 10 colors, fewer than there are solvers
    colors = plt.get_cmap("tab20").colors if len(times) > 10 else [None] * len(times)
    for solver, color in zip(times, colors):
        for ax, values in ((time_ax, means), (success_ax, success), (cost_ax, costs), (makespan_ax, makespans)):
            ax.plot(list(values[solver]), list(values[solver].values()), marker="o", label=solver, color=color)
    time_ax.set_yscale("log")
    time_ax.set_ylabel("mean time over solved maps (s)")
    time_ax.legend()
    success_ax.set_ylabel("fraction of maps solved")
    success_ax.set_ylim(-0.05, 1.05)
    cost_ax.set_ylabel("mean sum of costs over maps all solved")
    makespan_ax.set_ylabel("mean makespan over maps all solved")
    for ax in (time_ax, success_ax, cost_ax, makespan_ax):
        ax.grid(True, alpha=0.3)
    for ax in (success_ax, makespan_ax):
        ax.set_xlabel("number of agents")
    fig.suptitle(title)
    fig.tight_layout()
    if output:
        fig.savefig(output, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def append_results(path, n_agents, scens, times):
    """Append the raw runs of one batch to the CSV `path`, writing the header
    if the file is new. The map column is the utils.instance_name() of each
    problem; seconds (wall time), cost (sum of costs), makespan and
    solver_cost (the cost the solver reported itself) are empty for a run
    that was not solved, and solver_cost also for a solver that reports none."""
    new_file = not os.path.exists(path)
    with open(path, "a", newline="") as results_file:
        writer = csv.writer(results_file)
        if new_file:
            writer.writerow(["solver", "n_agents", "map", "seconds", "cost", "makespan", "solver_cost"])
        for solver, batches in times.items():
            for scen, run in zip(scens, batches[n_agents]):
                writer.writerow([solver, n_agents, utils.instance_name(scen, n_agents),
                                 *(["", "", "", ""] if run is None else
                                   [f"{run.seconds:.6f}", run.cost, run.makespan,
                                    "" if run.solver_cost is None else run.solver_cost])])


def mapfast_records(map_file, scen_file, n_agents, seconds):
    """The MAPFAST records of the problem of the first `n_agents` agents of
    `scen_file`, given the `seconds` (or None for a run that was not solved)
    that each solver took on it.

    Returns {name: record} for the names in MAPFAST_DETAILS, or None if no
    solver solved the problem: MAPFAST labels a problem with its fastest
    solver, so it has nothing to learn from one that nobody solved.
    """
    solved = {solver: t for solver, t in seconds.items() if t is not None}
    if not solved:
        return None
    width, height, obstacles, agents = load_instance(map_file, scen_file)
    agents = agents[:n_agents]
    return {
        "yaml_details": {"SOLVER": min(solved, key=solved.get),
                         **{solver: -1 if t is None else round(t, 6) for solver, t in seconds.items()}},
        "agent_details": {"starts": [list(start) for start, _ in agents],
                          "goals": [list(goal) for _, goal in agents]},
        "map_details": {"no_agents": n_agents,
                        "mp_dim": [width, height],
                        "no_obs": len(obstacles)},
    }


def add_mapfast_batch(details, map_file, n_agents, scens, times):
    """Add the problems of one batch that some solver solved to `details`, a
    {name: {instance name: record}} dict with an entry for each of
    MAPFAST_DETAILS."""
    for i, scen in enumerate(scens):
        records = mapfast_records(map_file, scen, n_agents,
                                  {solver: None if batches[n_agents][i] is None else batches[n_agents][i].seconds
                                   for solver, batches in times.items()})
        for name, record in (records or {}).items():
            details[name][utils.instance_name(scen, n_agents)] = record


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
    parser.add_argument('scenario', help='MovingAI scenario directory (one .map and its .scen files), or the name of a map '
                                         'in benchmarks/, e.g. empty-8-8')
    parser.add_argument('--scen-type', choices=SCEN_TYPES, default='random',
                        help='which scenario files of the map to use (default: random)')
    parser.add_argument('--solvers', nargs='+', choices=SOLVERS, default=DEFAULT_SOLVERS,
                        help=f"solvers to compare (default: {' '.join(DEFAULT_SOLVERS)}; mcts_nonoverlap only if listed)")
    parser.add_argument('--timeout', type=float, default=10.0, help='seconds allowed per solver run')
    parser.add_argument('--jobs', type=int, default=1, help='solver runs to execute at the same time (more than the number of physical cores distorts the timings)')
    parser.add_argument('--agents', type=int, nargs='+', default=None,
                        help='agent counts to run, each the first n agents of every scenario file '
                             '(default: 10, 20, 30, ... up to the agents of the smallest scenario file)')
    parser.add_argument('--max-instances', type=int, default=None, help='only use the first m scenario files of the map')
    parser.add_argument('--name', default=None,
                        help='prefix of the files written to output/ (default: the scenario directory name, plus -even for --scen-type even): '
                             '<name>_times.png, <name>_results.csv and the MAPFAST dataset files <name>_yaml_details.json, '
                             '<name>_agent_details.json and <name>_map_details.json, all updated after every batch')
    parser.add_argument('--output', default=None, help='plot file (default: output/<name>_times.png)')
    parser.add_argument('--results', default=None, help='CSV file of the raw times, started afresh by every run (default: output/<name>_results.csv)')
    parser.add_argument('--no-show', action='store_true', help='do not open the plot window')
    args = parser.parse_args()

    scenario = os.path.normpath(scenario_dir(args.scenario))
    title = os.path.basename(scenario) + ("" if args.scen_type == "random" else f" ({args.scen_type} scenarios)")
    prefix = args.name or os.path.basename(scenario) + ("" if args.scen_type == "random" else f"-{args.scen_type}")
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output = args.output or os.path.join(OUTPUT_DIR, prefix + "_times.png")
    results = args.results or os.path.join(OUTPUT_DIR, prefix + "_results.csv")
    if os.path.exists(results):
        os.remove(results)  # append_results adds to it batch by batch
    mapfast = {name: {} for name in MAPFAST_DETAILS}
    try:
        map_file = scenario_files(scenario, args.scen_type)[0]
    except (ValueError, FileNotFoundError) as error:
        sys.exit(str(error))

    def on_batch(n_agents, scens, times):
        # keep the plot, the raw results and the MAPFAST files up to date, so
        # that a run that is killed part way (e.g. by a cluster time limit)
        # still leaves all of them
        append_results(results, n_agents, scens, times)
        add_mapfast_batch(mapfast, map_file, n_agents, scens, times)
        write_mapfast(OUTPUT_DIR, prefix, mapfast)
        plot(times, title, output, show=False)

    try:
        times = benchmark(scenario, args.solvers, args.timeout, args.agents, args.max_instances, args.jobs, on_batch,
                          args.scen_type)
    except ValueError as error:
        sys.exit(str(error))
    maps = sum(len(runs) for runs in next(iter(times.values())).values())
    print(f"wrote {len(mapfast['yaml_details'])} of {maps} problems to the MAPFAST files in {OUTPUT_DIR} "
          "(the others were solved by no solver)")
    plot(times, title, output, show=not args.no_show)
