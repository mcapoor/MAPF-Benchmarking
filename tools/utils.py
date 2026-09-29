import copy
import importlib.util
import os
import subprocess
import sys
import tempfile

import yaml

from tools import standard_benchmark_converter

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# solver libraries are git submodules in libs/ (git submodule update --init)
LIBS_DIR = os.path.join(ROOT_DIR, "libs")
LIBMRP_DIR = os.path.join(LIBS_DIR, "libMultiRobotPlanning")
BUILD_DIR = os.path.join(LIBMRP_DIR, "build")
# grid animation, used for the solutions of every library
VISUALIZE = os.path.join(ROOT_DIR, "tools", "visualize.py")
VISUALIZE_ROADMAP = os.path.join(LIBMRP_DIR, "tools", "visualize_roadmap.py")
ANNOTATE_ROADMAP = os.path.join(LIBMRP_DIR, "tools", "annotate_roadmap.py")

# the wrappers in libMRP_wrappers.py, which each run the libMultiRobotPlanning
# binary of the same name
LIBMRP_SOLVERS = {
    "a_star", "a_star_epsilon", "assignment", "cbs", "cbs_roadmap", "cbs_ta",
    "ecbs", "ecbs_ta", "mapf_prioritized_sipp", "next_best_assignment", "sipp",
}
# the solvers that read libMultiRobotPlanning's YAML problem format (the
# others read the MovingAI files themselves)
YAML_SOLVERS = LIBMRP_SOLVERS | {"mcts_nonoverlap"}
# libMultiRobotPlanning versions (YAML) of MovingAI problems, and MovingAI
# versions of YAML problems, converted once and reused
LIBMRP_CACHE_DIR = os.path.join(ROOT_DIR, "cache", "libMultiRobotPlanning")
MOVINGAI_CACHE_DIR = os.path.join(ROOT_DIR, "cache", "movingai")
# libyaml's loader parses the schedule of a large instance far faster than the
# pure Python one (a solver run is timed including this), if PyYAML has it
YAML_LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


def load_module(path, name):
    """Import the Python file `path` as module `name`.

    Library scripts are loaded by path so that each library's tools/ does not
    clash with ours (or another library's) as a package named "tools".
    """
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scenario_map(scen_file):
    """The .map file that a MovingAI scenario file names (on every agent line),
    looked up in the directory of the scenario file."""
    with open(scen_file) as scen:
        next(scen)  # "version 1"
        for line in scen:
            if line.strip():
                return os.path.join(os.path.dirname(scen_file), line.split("\t")[1])
    raise ValueError(f"{scen_file} has no agents")


def instance_name(scen_file, n_agents):
    """Name of the problem of the first `n_agents` agents of `scen_file`, e.g.
    empty-8-8-random-1_agents10.yaml, which is also the file name of its
    libMultiRobotPlanning version."""
    return f"{os.path.splitext(os.path.basename(scen_file))[0]}_agents{n_agents}.yaml"


def libmrp_problem(scen_file, n_agents, map_file=None):
    """The libMultiRobotPlanning (YAML) version of the problem of the first
    `n_agents` agents of `scen_file`, converted into
    cache/libMultiRobotPlanning/<map>/<instance_name> unless it is there
    already. `map_file` is by default the map the scenario file names."""
    map_file = map_file or scenario_map(scen_file)
    cache_dir = os.path.join(LIBMRP_CACHE_DIR, os.path.splitext(os.path.basename(map_file))[0])
    return standard_benchmark_converter.convert(scen_file, map_file, n_agents,
                                                os.path.join(cache_dir, instance_name(scen_file, n_agents)))


def movingai_problem(yaml_file):
    """The MovingAI version of a libMultiRobotPlanning grid problem, for the
    solvers that read MovingAI files: (scen_file, n_agents), with its map
    and scenario file written to cache/movingai/<name>/<name>.map and .scen
    (again if `yaml_file` is newer). The scenario lists the agents in the
    order of the YAML file."""
    with open(yaml_file) as stream:
        problem = yaml.load(stream, Loader=YAML_LOADER)
    agents = problem.get("agents") or []
    if "dimensions" not in problem.get("map", {}) or not all("start" in a and "goal" in a for a in agents):
        raise ValueError(f"{yaml_file} is not a grid problem with a start and goal for every agent")
    name = os.path.splitext(os.path.basename(yaml_file))[0]
    cache_dir = os.path.join(MOVINGAI_CACHE_DIR, name)
    map_file = os.path.join(cache_dir, name + ".map")
    scen_file = os.path.join(cache_dir, name + ".scen")
    if not os.path.exists(scen_file) or os.path.getmtime(scen_file) < os.path.getmtime(yaml_file):
        os.makedirs(cache_dir, exist_ok=True)
        width, height = problem["map"]["dimensions"]
        standard_benchmark_converter.write_movingai(
            width, height, {tuple(o) for o in problem["map"].get("obstacles") or []},
            [(tuple(a["start"]), tuple(a["goal"])) for a in agents], map_file, scen_file)
    return scen_file, len(agents)


def solution_metrics(schedule):
    """(sum of costs, makespan) of a solution's `schedule` ({agent: [waypoint,
    ...]}, each waypoint with a time step `t`), the same for every solver.

    An agent's cost is its arrival time, the `t` of its last waypoint (not the
    number of waypoints, since some solvers leave out steps where an agent
    waits); the sum of costs adds them up and the makespan is the largest."""
    arrivals = [path[-1]["t"] if path else 0 for path in schedule.values()]
    return sum(arrivals), max(arrivals, default=0)


def stdio(quiet):
    """subprocess.run keyword arguments that discard console output if `quiet`."""
    return {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL} if quiet else {}


def run(binary, args, output=None, timeout=None, quiet=False, build_dir=BUILD_DIR):
    """Run the solver `binary` from `build_dir` (by default libMultiRobotPlanning's
    build/) as `<binary> <args> -o output` and return its parsed YAML output.

    If `output` is None a temporary file is used, so nothing is left behind.
    Raises subprocess.CalledProcessError on a non-zero exit status and
    subprocess.TimeoutExpired if `timeout` (seconds) is exceeded. The result is
    None if the solver wrote an empty output file (e.g. a* found no path) or no
    output file at all (e.g. the task assignment solvers when planning fails).
    With `quiet` the solver's console output (stdout and stderr) is discarded.
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        output_path = output or os.path.join(tmp_dir, "output.yaml")
        subprocess.run(
            [os.path.join(build_dir, binary)] + [str(a) for a in args] +
            ["-o", output_path],
            check=True,
            timeout=timeout,
            **stdio(quiet))
        if not os.path.exists(output_path):
            return None
        with open(output_path) as output_file:
            return yaml.load(output_file, Loader=YAML_LOADER)


def _visualize(input_file, output_file, video, suffix, script=VISUALIZE,
               extra_args=(), quiet=False):
    subprocess.run(
        [sys.executable, script,
         input_file,
         output_file,
         "--video", video or os.path.splitext(os.path.basename(input_file))[0] + suffix,
         *[str(a) for a in extra_args]],
        check=True,
        **stdio(quiet))


def _fill_empty_schedules(result, input_file):
    """Give agents the solver left without a path a single waypoint at their
    start, so that the result can be handed to the Animation classes in
    tools/visualize.py and tools/visualize_roadmap.py (which index the last
    waypoint of every agent). Grid problems use {x, y, t} waypoints, roadmap
    problems (agents starting at a named vertex) use {v, t}."""
    if not result or "schedule" not in result:
        return result
    with open(input_file) as input_stream:
        agents = (yaml.load(input_stream, Loader=YAML_LOADER) or {}).get("agents", [])
    for agent in agents:
        start = agent.get("start")
        if result["schedule"].get(agent["name"]) == []:
            result["schedule"][agent["name"]] = [
                {"x": start[0], "y": start[1], "t": 0} if isinstance(start, list)
                else {"v": start, "t": 0}]
    return result


def ensure_potential_goals(input_file):
    """Return a task assignment version of `input_file`, creating it if needed.

    The task assignment solvers (cbs_ta, ecbs_ta) read a `potentialGoals` list
    per agent instead of a single `goal`. If `input_file` has none, a file of
    the same name is written (unless one already exists) to the directory
    `<input dir>_potentialGoals` next to the input's directory, so the original
    files are left alone. In it every agent may take any agent's goal, i.e. the
    goals become tasks to assign, as in test/mapfta_simple1_a1.yaml.
    """
    with open(input_file) as input_stream:
        problem = yaml.load(input_stream, Loader=YAML_LOADER)
    if all("potentialGoals" in agent for agent in problem["agents"]):
        return input_file

    input_dir, name = os.path.split(os.path.abspath(input_file))
    ta_dir = input_dir + "_potentialGoals"
    ta_file = os.path.join(ta_dir, name)
    if not os.path.exists(ta_file):
        goals = [agent["goal"] for agent in problem["agents"] if "goal" in agent]
        for agent in problem["agents"]:
            agent.pop("goal", None)
            agent["potentialGoals"] = copy.deepcopy(goals)  # no yaml anchors
        os.makedirs(ta_dir, exist_ok=True)
        with open(ta_file, "w") as ta_stream:
            yaml.dump(problem, ta_stream, sort_keys=False)
    return ta_file


def run_mapf(binary, input_file, args=(), output=None, timeout=None,
              additional_args=(), create_video=False, video=None,
              visualize_script=VISUALIZE, visualize_args=(), quiet=False,
              build_dir=BUILD_DIR):
    """Shared runner for the solvers of the form `<binary> -i input -o output`.

    The returned dict has the form the Animation classes expect: a "schedule"
    entry mapping each agent name to a list of {x, y, t} waypoints ({v, t} for
    roadmaps). If the solver found no solution the dict has no "schedule" entry
    (see statistics["success"]).
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        output_path = output or os.path.join(tmp_dir, "output.yaml")
        result = _fill_empty_schedules(
            run(binary, ["-i", input_file, *args, *additional_args],
                 output=output_path, timeout=timeout, quiet=quiet,
                 build_dir=build_dir),
            input_file)
        if create_video:
            _visualize(input_file, output_path, video, "_" + binary + ".mp4",
                       visualize_script, visualize_args, quiet)
        return result


def _write_mapping(mapping, path):
    """Write a {(agent, task): cost} dict in the `agent->task:cost` line format."""
    with open(path, "w") as mapping_file:
        for pair, value in mapping.items():
            mapping_file.write(pair[0] + "->" + pair[1] + ":" + str(value) + "\n")


def run_assignment(binary, mapping, output=None, timeout=None, quiet=False):
    with tempfile.TemporaryDirectory() as tmp_dir:
        input_path = os.path.join(tmp_dir, "input.txt")
        _write_mapping(mapping, input_path)
        return run(binary, ["-i", input_path], output=output, timeout=timeout,
                    quiet=quiet)