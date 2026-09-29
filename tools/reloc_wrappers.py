import math
import re

from tools import standard_benchmark_converter
from tools.utils import *

# Surynek's SAT-based MAPF solver with Glucose compiled in (git submodule
# libs/reLOC, built into its build/ directory)
RELOC_DIR = os.path.join(LIBS_DIR, "reLOC")
RELOC_BINARY = os.path.join(RELOC_DIR, "build", "insolver_reLOC")

# reLOC versions (.cpf) of the MovingAI problems, converted once and reused
RELOC_CACHE_DIR = os.path.join(ROOT_DIR, "cache", "reLOC")

# the solver's own limit (--total-timeout, whole seconds) when no timeout is
# given, as its default of 600 s would silently limit the run
NO_CUTOFF = 10 ** 7
# extra seconds the process gets beyond its own limit before it is killed,
# for reading the instance and printing the solution
KILL_GRACE = 5

# a move of the solution: <agent>#<from vertex>-><to vertex>
MOVE = re.compile(r"(\d+)#(\d+)->(\d+)")


def grid_vertices(width, height, obstacles):
    """The free cells (x, y) of a grid in row-major order, which are the
    vertices 0, 1, ... of its reLOC graph."""
    return [(x, y) for y in range(height) for x in range(width) if (x, y) not in obstacles]


def cpf_file(scen_file, n_agents):
    """The cached reLOC problem of the first `n_agents` agents of `scen_file`,
    cache/reLOC/<map>/<scen>_agents<n>.cpf."""
    map_name = os.path.splitext(os.path.basename(scenario_map(scen_file)))[0]
    scen_name = os.path.splitext(os.path.basename(scen_file))[0]
    return os.path.join(RELOC_CACHE_DIR, map_name, f"{scen_name}_agents{n_agents}.cpf")


def convert(scen_file, map_file, n_agents, output_file):
    """Write the problem of the first `n_agents` agents of `scen_file` on
    `map_file` to the reLOC problem file `output_file` (see the .cpf format in
    libs/reLOC/README.md), unless it already exists and is newer than both.

    The vertices are the free cells in the order of grid_vertices(), joined
    to their free 4-neighbours; agent i of the scenario file is reLOC agent
    i + 1. Written to a temporary file first, so a converted file is always
    complete.
    """
    if (os.path.exists(output_file)
            and os.path.getmtime(output_file) >= max(os.path.getmtime(scen_file), os.path.getmtime(map_file))):
        return output_file
    width, height, obstacles = standard_benchmark_converter.load_map_file(map_file)
    agents = standard_benchmark_converter.load_scenario_file(scen_file, obstacles, width, height)
    if n_agents > len(agents):
        raise ValueError(f"{scen_file} has {len(agents)} agents, fewer than {n_agents}")
    cells = grid_vertices(width, height, obstacles)
    vertex = {cell: v for v, cell in enumerate(cells)}
    starts, goals = {}, {}
    for i, (start, goal) in enumerate(agents[:n_agents]):
        starts[vertex[start]] = i + 1
        goals[vertex[goal]] = i + 1

    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
    with open(output_file + ".tmp", "w") as cpf:
        cpf.write("V =\n")
        for v in range(len(cells)):
            goal = goals.get(v, 0)
            cpf.write(f"({v}:-1)[{starts.get(v, 0)}:{goal}:{goal}]\n")
        cpf.write("E =\n")
        for v, (x, y) in enumerate(cells):
            for neighbour in ((x + 1, y), (x, y + 1)):
                if neighbour in vertex:
                    cpf.write(f"{{{v},{vertex[neighbour]}}} (-1)\n")
    os.replace(output_file + ".tmp", output_file)
    return output_file


def _read_solution(stdout, cells, starts):
    """The (cost, makespan, runtime, schedule) of the solver's console output,
    or None if it found no solution.

    The moves of each time step follow "Optimal solution:"; an agent that does
    not move waits. Each agent's waypoints ({x, y, t}, agents named agent0,
    agent1, ...) end when it makes its last move, as in libMultiRobotPlanning.
    """
    cost = re.search(r"Computed sum of costs:(-?\d+)", stdout)
    if not cost or int(cost.group(1)) < 0:
        return None
    runtime = re.search(r"Wall clock TIME \(seconds\)\s*=\s*([\d.]+)", stdout)
    steps = []
    in_solution = False
    for line in stdout.splitlines():
        if line.startswith("Optimal solution:"):
            in_solution = True
        elif in_solution and line.strip() == "]":
            break
        elif in_solution and line.strip().startswith("Step"):
            steps.append([tuple(map(int, move)) for move in MOVE.findall(line)])

    positions = list(starts)  # vertex of each agent
    paths = [[vertex] for vertex in positions]
    arrival = [0] * len(starts)
    for t, moves in enumerate(steps, start=1):
        for agent, source, target in moves:
            if positions[agent - 1] != source:
                raise ValueError(f"reLOC moved agent {agent} from {source}, but it is at {positions[agent - 1]}")
            positions[agent - 1] = target
            arrival[agent - 1] = t
        for i, vertex in enumerate(positions):
            paths[i].append(vertex)
    schedule = {f"agent{i}": [{"x": cells[vertex][0], "y": cells[vertex][1], "t": t}
                              for t, vertex in enumerate(path[:arrival[i] + 1])]
                for i, path in enumerate(paths)}
    return int(cost.group(1)), len(steps), float(runtime.group(1)) if runtime else None, schedule


def reloc(scen_file, n_agents, map_file=None, encoding="mdd", output=None,
          timeout=None, additional_args=(), quiet=False, brief=False):
    """Run reLOC's SAT-based MAPF solver on the first `n_agents` agents of
    `scen_file` (sum-of-costs optimal).

    The problem is converted to cache/reLOC/ first (see convert and
    cpf_file), unless it is there already. `map_file` is by default the map
    named in the scenario file (see utils.scenario_map). `encoding` is the
    solver's --encoding (mdd, or a variant such as mddx++). `output` keeps the
    solver's console output, which holds the solution. Arguments in
    `additional_args` go on the command line as they are.

    `timeout` (seconds, rounded up) is the solver's own limit, which ends the
    run without a solution. As a backstop the process is killed KILL_GRACE
    seconds later, raising subprocess.TimeoutExpired. Raises
    subprocess.CalledProcessError on a non-zero exit status. With `quiet` the
    solver's console output is not printed. With `brief` only a one-line
    summary of it is, like the other solvers print: the outcome, cost,
    makespan and the solver's own runtime.

    Returns a dict like that of the libMultiRobotPlanning wrappers, with
    "statistics" (cost, makespan, runtime) and "schedule" (agent name to a
    list of {x, y, t} waypoints), or None if no solution was found.
    """
    map_file = map_file or scenario_map(scen_file)
    problem = convert(scen_file, map_file, n_agents, cpf_file(scen_file, n_agents))
    limit = NO_CUTOFF if timeout is None else max(1, math.ceil(timeout))
    with tempfile.TemporaryDirectory() as tmp_dir:
        completed = subprocess.run(
            [RELOC_BINARY,
             f"--input-file={problem}",
             f"--encoding={encoding}",
             f"--total-timeout={limit}",
             f"--minisat-timeout={limit}",
             # required for it to solve, although the MDD encodings do not write it
             f"--output-file={os.path.join(tmp_dir, 'solution.txt')}",
             *[str(a) for a in additional_args]],
            check=True,
            timeout=None if timeout is None else timeout + KILL_GRACE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL if quiet else None,
            text=True)
    if output:
        with open(output, "w") as output_file:
            output_file.write(completed.stdout)
    if not quiet and not brief:
        print(completed.stdout, end="")

    width, height, obstacles = standard_benchmark_converter.load_map_file(map_file)
    cells = grid_vertices(width, height, obstacles)
    vertex = {cell: v for v, cell in enumerate(cells)}
    agents = standard_benchmark_converter.load_scenario_file(scen_file, obstacles, width, height)
    solution = _read_solution(completed.stdout, cells, [vertex[start] for start, _ in agents[:n_agents]])
    if not quiet and brief:
        runtime = re.search(r"Wall clock TIME \(seconds\)\s*=\s*([\d.]+)", completed.stdout)
        runtime = f"{runtime.group(1)} s" if runtime else "unknown time"
        if solution is None:
            print(f"reLOC {encoding}: no solution, {runtime}", flush=True)
        else:
            print(f"reLOC {encoding}: Optimal, cost {solution[0]}, makespan {solution[1]}, {runtime}", flush=True)
    if solution is None:
        return None
    cost, makespan, runtime, schedule = solution
    return {
        "statistics": {"cost": cost, "makespan": makespan, "runtime": runtime},
        "schedule": schedule,
    }
