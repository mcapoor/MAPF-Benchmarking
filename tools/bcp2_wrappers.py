import re

from tools.utils import *

# Lam's branch-and-cut-and-price MAPF solver (git submodule libs/bcp2-mapf,
# built into its build/ directory; needs a Gurobi license to run)
BCP2_DIR = os.path.join(LIBS_DIR, "bcp2-mapf")
BCP2_BINARY = os.path.join(BCP2_DIR, "build", "bcp2-mapf")

# extra seconds the process gets beyond its own time limit before it is
# killed, for reading the instance and printing the solution
KILL_GRACE = 5

# a waypoint of the solution: (x,y)
WAYPOINT = re.compile(r"\((-?\d+),(-?\d+)\)")


def _read_solution(stdout):
    """The (status, cost, runtime, schedule) of the solver's console output.

    status is Optimal, Feasible (a solution that is not proven optimal),
    Unknown (none found, e.g. on timeout) or Infeasible. The schedule is None
    unless a solution was printed: after "Solution:" and a line of time step
    numbers, line a is `Agent a: (x,y) (x,y) ...`, one waypoint per time step,
    ending when the agent reaches its goal.
    """
    status = re.search(r"^Status:\s*(\w+)", stdout, re.MULTILINE)
    cost = re.search(r"^Upper bound:\s*(\S+)", stdout, re.MULTILINE)
    runtime = re.search(r"^Solve time:\s*([\d.]+)", stdout, re.MULTILINE)
    schedule = None
    if "\nSolution:" in stdout:
        schedule = {}
        for line in stdout.split("\nSolution:", 1)[1].splitlines():
            agent = re.match(r"\s*Agent\s+(\d+):", line)
            if agent:
                schedule[f"agent{agent.group(1)}"] = [
                    {"x": int(x), "y": int(y), "t": t}
                    for t, (x, y) in enumerate(WAYPOINT.findall(line))]
    return (status.group(1) if status else None,
            cost.group(1) if cost else None,
            float(runtime.group(1)) if runtime else None,
            schedule)


def bcp2(scen_file, n_agents, output=None, timeout=None, additional_args=(),
         quiet=False):
    """Run BCP2-MAPF on the first `n_agents` agents of `scen_file` (sum of
    costs optimal). The solver reads the map named in the scenario file from
    the scenario file's directory.

    `output` keeps the solver's console output, which holds the solution.
    Arguments in `additional_args` go on the command line as they are.

    `timeout` (seconds) is the solver's own time limit, which ends the run
    without a proven optimal solution. As a backstop the process is killed
    KILL_GRACE seconds later, raising subprocess.TimeoutExpired. Raises
    subprocess.CalledProcessError on a non-zero exit status (e.g. without a
    Gurobi license). With `quiet` the solver's console output is not printed.

    Returns a dict like that of the libMultiRobotPlanning wrappers, with
    "statistics" (cost, makespan, runtime) and "schedule" (agent name to a
    list of {x, y, t} waypoints), or None unless the solver proved its
    solution optimal: a solution that is only feasible when the time limit
    runs out counts as unsolved, as for the other optimal solvers.
    """
    completed = subprocess.run(
        [BCP2_BINARY,
         f"--agent-limit={n_agents}",
         *([f"--time-limit={timeout}"] if timeout is not None else []),
         *[str(a) for a in additional_args],
         scen_file],
        check=True,
        timeout=None if timeout is None else timeout + KILL_GRACE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL if quiet else None,
        text=True)
    if output:
        with open(output, "w") as output_file:
            output_file.write(completed.stdout)
    if not quiet:
        print(completed.stdout, end="")

    status, cost, runtime, schedule = _read_solution(completed.stdout)
    if status != "Optimal" or not schedule:
        return None
    return {
        "statistics": {
            "cost": int(cost),
            "makespan": max(len(path) for path in schedule.values()) - 1,
            "runtime": runtime,
        },
        "schedule": schedule,
    }
