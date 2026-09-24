import copy
import importlib.util
import os
import subprocess
import sys
import tempfile

import yaml

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# solver libraries are git submodules in libs/ (git submodule update --init)
LIBS_DIR = os.path.join(ROOT_DIR, "libs")
LIBMRP_DIR = os.path.join(LIBS_DIR, "libMultiRobotPlanning")
BUILD_DIR = os.path.join(LIBMRP_DIR, "build")
VISUALIZE = os.path.join(LIBMRP_DIR, "tools", "visualize.py")
VISUALIZE_ROADMAP = os.path.join(LIBMRP_DIR, "tools", "visualize_roadmap.py")
ANNOTATE_ROADMAP = os.path.join(LIBMRP_DIR, "tools", "annotate_roadmap.py")
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


def _stdio(quiet):
    """subprocess.run keyword arguments that discard console output if `quiet`."""
    return {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL} if quiet else {}


def _run(binary, args, output=None, timeout=None, quiet=False):
    """Run a libMultiRobotPlanning solver binary from its build/ and return its parsed YAML output.

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
            [os.path.join(BUILD_DIR, binary)] + [str(a) for a in args] +
            ["-o", output_path],
            check=True,
            timeout=timeout,
            **_stdio(quiet))
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
        **_stdio(quiet))


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


def _run_mapf(binary, input_file, args=(), output=None, timeout=None,
              additional_args=(), create_video=False, video=None,
              visualize_script=VISUALIZE, visualize_args=(), quiet=False):
    """Shared runner for the solvers of the form `<binary> -i input -o output`.

    The returned dict has the form the Animation classes expect: a "schedule"
    entry mapping each agent name to a list of {x, y, t} waypoints ({v, t} for
    roadmaps). If the solver found no solution the dict has no "schedule" entry
    (see statistics["success"]).
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        output_path = output or os.path.join(tmp_dir, "output.yaml")
        result = _fill_empty_schedules(
            _run(binary, ["-i", input_file, *args, *additional_args],
                 output=output_path, timeout=timeout, quiet=quiet),
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


def _run_assignment(binary, mapping, output=None, timeout=None, quiet=False):
    with tempfile.TemporaryDirectory() as tmp_dir:
        input_path = os.path.join(tmp_dir, "input.txt")
        _write_mapping(mapping, input_path)
        return _run(binary, ["-i", input_path], output=output, timeout=timeout,
                    quiet=quiet)


def a_star(start, goal, map_file, output=None, timeout=None, quiet=False):
    """Single-agent A* on a txt map. `start` and `goal` are (x, y) pairs."""
    return _run("a_star",
                ["--startX", start[0], "--startY", start[1],
                 "--goalX", goal[0], "--goalY", goal[1],
                 "-m", map_file],
                output=output, timeout=timeout, quiet=quiet)


def a_star_epsilon(start, goal, map_file, w, output=None, timeout=None,
                   quiet=False):
    """Single-agent bounded-suboptimal A* (suboptimality factor `w`)."""
    return _run("a_star_epsilon",
                ["--startX", start[0], "--startY", start[1],
                 "--goalX", goal[0], "--goalY", goal[1],
                 "-m", map_file,
                 "-w", w],
                output=output, timeout=timeout, quiet=quiet)


def assignment(mapping, output=None, timeout=None, quiet=False):
    """Optimal assignment for a {(agent, task): cost} dict."""
    return _run_assignment("assignment", mapping, output=output, timeout=timeout,
                           quiet=quiet)


def cbs(input_file, output=None, timeout=None, additional_args=(),
        create_video=False, video=None, quiet=False):
    return _run_mapf("cbs", input_file, output=output, timeout=timeout,
                     additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def cbs_roadmap(input_file, output=None, timeout=None, additional_args=(),
                create_video=False, video=None, annotate=False,
                annotate_radius=0.3, quiet=False):
    """CBS on a roadmap. `annotate_radius` is the robot radius, used to annotate
    the roadmap with collision information (if `annotate`) and to draw it (if
    `create_video`, using tools/visualize_roadmap.py)."""
    original_file = input_file
    with tempfile.TemporaryDirectory() as tmp_dir:
        if annotate:
            annotated_file = os.path.join(tmp_dir, "annotated_input.yaml")
            subprocess.run(
                [sys.executable, ANNOTATE_ROADMAP,
                 input_file,
                 annotated_file,
                 str(annotate_radius)],
                check=True,
                **_stdio(quiet))
            input_file = annotated_file
        if create_video and not video:
            video = os.path.splitext(os.path.basename(original_file))[0] + "_cbs_roadmap.mp4"
        return _run_mapf("cbs_roadmap", input_file, output=output,
                         timeout=timeout, additional_args=additional_args,
                         create_video=create_video, video=video,
                         visualize_script=VISUALIZE_ROADMAP,
                         visualize_args=["--radius", annotate_radius],
                         quiet=quiet)


def cbs_ta(input_file, output=None, timeout=None, additional_args=(),
           create_video=False, video=None, quiet=False):
    """CBS with task assignment. An input without `potentialGoals` is converted
    first (see ensure_potential_goals)."""
    return _run_mapf("cbs_ta", ensure_potential_goals(input_file), output=output, timeout=timeout,
                     additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def ecbs(input_file, w=1, output=None, timeout=None, additional_args=(),
         create_video=False, video=None, quiet=False):
    """Enhanced CBS with suboptimality factor `w`."""
    return _run_mapf("ecbs", input_file, args=["-w", w], output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def ecbs_ta(input_file, w=1, output=None, timeout=None, additional_args=(),
            create_video=False, video=None, quiet=False):
    """Enhanced CBS with task assignment and suboptimality factor `w`. An input
    without `potentialGoals` is converted first (see ensure_potential_goals)."""
    return _run_mapf("ecbs_ta", ensure_potential_goals(input_file), args=["-w", w], output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def mapf_prioritized_sipp(input_file, output=None, timeout=None,
                          additional_args=(), create_video=False, video=None, quiet=False):
    return _run_mapf("mapf_prioritized_sipp", input_file, output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def mcts_nonoverlap(input_file, iterations=None, exploration=None,
                    attempts=None, seed=None, output=None, timeout=None,
                    additional_args=(), create_video=False, video=None,
                    quiet=False):
    """Monte-Carlo tree search for vertex-disjoint, time-free paths (agents'
    start and goal are the two ends of one path). `iterations`, `exploration`,
    `attempts` and `seed` are only passed on if given. Waypoint `t` is the step
    index; there is no schedule entry if no solution was found."""
    args = []
    for flag, value in (("--iterations", iterations),
                        ("--exploration", exploration),
                        ("--attempts", attempts),
                        ("--seed", seed)):
        if value is not None:
            args += [flag, value]
    return _run_mapf("mcts_nonoverlap", input_file, args=args, output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def next_best_assignment(mapping, output=None, timeout=None, quiet=False):
    """All assignments for a {(agent, task): cost} dict, best first."""
    return _run_assignment("next_best_assignment", mapping, output=output,
                           timeout=timeout, quiet=quiet)


def sipp(input_file, output=None, timeout=None, quiet=False):
    """Single-agent safe interval path planning."""
    return _run_mapf("sipp", input_file, output=output, timeout=timeout,
                     quiet=quiet)
