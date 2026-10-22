"""Time MAPF solvers on the MovingAI scenarios of one map (a directory with one .map and its <map>-<random|even>-<k>.scen
files, or the name of a map in benchmarks/). The problem with n agents of a scenario file is its first n agents, so
every solver gets the same problem.

    python benchmark.py empty-8-8 --timeout 5 --jobs 4
    python benchmark.py room-32-32-4 --scen-type even --agents 10 20 50 --experiment room-test

The results go to experiments/<experiment>/maps/<map>/ (see experiment_dir and the options), rewritten after every
agent count so that an interrupted run keeps them: results.csv (solver, n_agents, instance, wall time, sum of costs,
makespan and the cost the solver reported; empty if unsolved) and times.png (time, fraction solved, and sum of costs
and makespan on the instances every solver solved). tools/experiment.py benchmarks every map (benchmark.sh on Slurm).

Comparing the numbers: the sum of costs and makespan are computed from each solver's paths the same way, but are only
comparable between solvers of the same problem. ecbs (bounded-suboptimal) and mapf_prioritized_sipp (neither optimal
nor complete) are not optimal; the task assignment solvers (cbs_ta, ecbs_ta) also choose the goals; reloc forbids
following an agent into the cell it leaves; mcts_nonoverlap looks for vertex-disjoint paths, so it is only run when
listed in --solvers. With --jobs above 1 the runs share the machine, so the times include some contention (more jobs
than physical cores distorts them); converting problems to a library's format happens before the timing.
"""
import argparse
import collections
import concurrent.futures
import csv
import functools
import os
import re
import subprocess
import sys
import time

import matplotlib.pyplot as plt

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT_DIR)

from tools import reloc_wrappers, standard_benchmark_converter, utils
from tools.solve_and_visualize import GRID_SOLVERS, MOVINGAI_SOLVERS, solve, solver_binary


BENCHMARK_DIR = os.path.join(ROOT_DIR, "benchmarks")
SCEN_TYPES = ("random", "even")

SCEN_NAME = re.compile(r"-(\w+)-(\d+)\.scen$")

# every benchmark experiment gets a folder here (see experiment_dir): experiments/<experiment>/maps/<map>/ holds a
# benchmark.py run of one map, experiments/<experiment>/ the results of tools/experiment.py and its Slurm logs
EXPERIMENTS_DIR = os.path.join(ROOT_DIR, "experiments")
# the files of a benchmark: the raw runs and their plot
RESULTS_FILE = "results.csv"
PLOT_FILE = "times.png"

Run = collections.namedtuple("Run", "seconds cost makespan solver_cost")

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
def load_map(map_file):
    """(width, height, obstacles) of a MovingAI map, read once per map so that
    its scenario files share one obstacle set."""
    return standard_benchmark_converter.load_map_file(map_file)


@functools.cache
def load_instance(map_file, scen_file):
    """(width, height, obstacles, [(start, goal) of every agent]) of a MovingAI
    map and scenario file, with the agents in file order. The obstacles are
    load_map's set, shared with the map's other scenario files: do not change it."""
    width, height, obstacles = load_map(map_file)
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
    exceeds `timeout` seconds (see solver_outcome)."""
    return solver_outcome(instance, solver, timeout)[0]


# why a run is unsolved (solver_outcome): it ran out of time, gave up earlier without a solution, was killed by a
# signal (on Slurm usually the out-of-memory killer, when the runs of a job together use more than its memory) or
# exited with an error. Only "timeout" and "no solution" say something about the instance; the others are retried
# by tools/experiment.py.
FAILURES = ("timeout", "no solution", "killed", "crashed")
# a solver that stops by itself at its time limit ends a little before `timeout` seconds of wall time
TIMEOUT_MARGIN = 0.95


def solver_outcome(instance, solver, timeout):
    """(Run or None, reason, wall seconds) of `solver` on `instance`, where
    reason is "solved" or one of FAILURES.

    The solvers that stop by themselves at the timeout measure it by their own
    clock (e.g. excluding reading the instance), so a solution that arrives
    after more than `timeout` seconds of wall time also counts as unsolved."""
    if solver.endswith("_ta"):
        # already converted by benchmark(), so this only looks up the cached file
        instance = utils.ensure_potential_goals(instance)
    start = time.perf_counter()
    try:
        result, seconds = solve(instance, solver, timeout=timeout, quiet=True, report=False)
    except subprocess.TimeoutExpired:
        return None, "timeout", time.perf_counter() - start
    except subprocess.CalledProcessError as error:
        return None, "killed" if error.returncode < 0 else "crashed", time.perf_counter() - start
    if seconds > timeout or (result is None and seconds >= TIMEOUT_MARGIN * timeout):
        return None, "timeout", seconds
    if result is None:
        return None, "no solution", seconds
    cost, makespan = utils.solution_metrics(result["schedule"])
    return Run(seconds, cost, makespan, (result.get("statistics") or {}).get("cost")), "solved", seconds


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


def experiment_dir(experiment):
    """The folder of an experiment: a path to it, or its name in EXPERIMENTS_DIR."""
    if os.path.isabs(experiment) or os.sep in experiment.rstrip(os.sep):
        return os.path.normpath(experiment)
    return os.path.join(EXPERIMENTS_DIR, experiment)


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
    parser.add_argument('--experiment', default=None,
                        help='experiment to add this map to: a name in experiments/ or a folder (default: '
                             '<date>_<time>_<name>, a new one)')
    parser.add_argument('--name', default=None,
                        help='folder of this run in the experiment, experiments/<experiment>/maps/<name>/ (default: the '
                             'scenario directory name, plus -even for --scen-type even), which gets times.png and '
                             'results.csv, both updated after every batch')
    parser.add_argument('--output', default=None, help='plot file (default: <run folder>/times.png)')
    parser.add_argument('--results', default=None, help='CSV file of the raw times, started afresh by every run (default: <run folder>/results.csv)')
    parser.add_argument('--no-show', action='store_true', help='do not open the plot window')
    args = parser.parse_args()

    scenario = os.path.normpath(scenario_dir(args.scenario))
    title = os.path.basename(scenario) + ("" if args.scen_type == "random" else f" ({args.scen_type} scenarios)")
    prefix = args.name or os.path.basename(scenario) + ("" if args.scen_type == "random" else f"-{args.scen_type}")
    experiment = experiment_dir(args.experiment or time.strftime("%Y-%m-%d_%H%M_") + prefix)
    run_dir = os.path.join(experiment, "maps", prefix)
    os.makedirs(run_dir, exist_ok=True)
    output = args.output or os.path.join(run_dir, PLOT_FILE)
    results = args.results or os.path.join(run_dir, RESULTS_FILE)
    if os.path.exists(results):
        os.remove(results)  # append_results adds to it batch by batch

    def on_batch(n_agents, scens, times):
        # keep the plot and the raw results up to date, so that a run that is
        # killed part way (e.g. by a cluster time limit) still leaves both
        append_results(results, n_agents, scens, times)
        plot(times, title, output, show=False)

    try:
        times = benchmark(scenario, args.solvers, args.timeout, args.agents, args.max_instances, args.jobs, on_batch,
                          args.scen_type)
    except (ValueError, FileNotFoundError) as error:
        sys.exit(str(error))
    plot(times, title, output, show=not args.no_show)
