import glob
import re
import shutil

from tools.utils import *

# Lam et al.'s branch-and-cut-and-price MAPF solver, the predecessor of bcp2
# (git submodule libs/bcp1-mapf, built into its build/ directory with SCIP;
# needs a Gurobi license to run)
BCP1_DIR = os.path.join(LIBS_DIR, "bcp1-mapf")
BCP1_BINARY = os.path.join(BCP1_DIR, "build", "bcp-mapf")

# extra seconds the process gets beyond its own time limit before it is
# killed, for reading the instance and writing the solution
KILL_GRACE = 5

# a waypoint of the solution: (x,y)
WAYPOINT = re.compile(r"\((-?\d+),(-?\d+)\)")


def _read_solution(sol_file):
    """(cost, schedule) of the solution file the solver writes, or None if it
    has none. Its first line is the sum of costs (or "-"), then line a is
    `Agent a, cost c, path (x,y),(x,y),...`, one waypoint per time step."""
    with open(sol_file) as sol:
        cost = sol.readline().strip()
        if cost == "-":
            return None
        schedule = {}
        for line in sol:
            agent = re.match(r"Agent (\d+),", line)
            if agent:
                schedule[f"agent{agent.group(1)}"] = [
                    {"x": int(x), "y": int(y), "t": t}
                    for t, (x, y) in enumerate(WAYPOINT.findall(line.split("path", 1)[1]))]
    return int(cost), schedule


def bcp1(scen_file, n_agents, output=None, timeout=None, additional_args=(),
         quiet=False):
    """Run BCP-MAPF on the first `n_agents` agents of `scen_file` (sum of
    costs optimal). The solver reads the map named in the scenario file from
    the scenario file's directory, and writes its solution to
    outputs/<scenario>-<n>agents.sol in its working directory, a temporary
    one here.

    `output` keeps the solution file. Arguments in `additional_args` go on
    the command line as they are.

    `timeout` (seconds) is the solver's own time limit, which ends the run
    without a proven optimal solution. As a backstop the process is killed
    KILL_GRACE seconds later, raising subprocess.TimeoutExpired. Raises
    subprocess.CalledProcessError on a non-zero exit status (e.g. without a
    Gurobi license). With `quiet` the solver's console output is discarded.

    Returns a dict like that of the libMultiRobotPlanning wrappers, with
    "statistics" (cost, makespan, runtime) and "schedule" (agent name to a
    list of {x, y, t} waypoints), or None unless the solver proved its
    solution optimal, as for bcp2.
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        completed = subprocess.run(
            [BCP1_BINARY,
             f"--agent-limit={n_agents}",
             *([f"--time-limit={timeout}"] if timeout is not None else []),
             *[str(a) for a in additional_args],
             os.path.abspath(scen_file)],
            check=True,
            cwd=tmp_dir,
            timeout=None if timeout is None else timeout + KILL_GRACE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL if quiet else None,
            text=True)
        if not quiet:
            print(completed.stdout, end="")
        sol_file, = glob.glob(os.path.join(tmp_dir, "outputs", "*.sol"))
        if output:
            shutil.copyfile(sol_file, output)
        solution = _read_solution(sol_file)

    status = re.search(r"^SCIP Status\s*:.*\[(.*)\]", completed.stdout, re.MULTILINE)
    runtime = re.search(r"^Solving Time \(sec\)\s*:\s*([\d.]+)", completed.stdout, re.MULTILINE)
    if not solution or not status or status.group(1) != "optimal solution found":
        return None
    cost, schedule = solution
    return {
        "statistics": {
            "cost": cost,
            "makespan": max(len(path) for path in schedule.values()) - 1,
            "runtime": float(runtime.group(1)) if runtime else None,
        },
        "schedule": schedule,
    }
