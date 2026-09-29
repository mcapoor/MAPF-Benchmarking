from tools.utils import *

def a_star(start, goal, map_file, output=None, timeout=None, quiet=False):
    """Single-agent A* on a txt map. `start` and `goal` are (x, y) pairs."""
    return run("a_star",
                ["--startX", start[0], "--startY", start[1],
                 "--goalX", goal[0], "--goalY", goal[1],
                 "-m", map_file],
                output=output, timeout=timeout, quiet=quiet)


def a_star_epsilon(start, goal, map_file, w, output=None, timeout=None,
                   quiet=False):
    """Single-agent bounded-suboptimal A* (suboptimality factor `w`)."""
    return run("a_star_epsilon",
                ["--startX", start[0], "--startY", start[1],
                 "--goalX", goal[0], "--goalY", goal[1],
                 "-m", map_file,
                 "-w", w],
                output=output, timeout=timeout, quiet=quiet)


def assignment(mapping, output=None, timeout=None, quiet=False):
    """Optimal assignment for a {(agent, task): cost} dict."""
    return run_assignment("assignment", mapping, output=output, timeout=timeout,
                           quiet=quiet)


def cbs(input_file, output=None, timeout=None, additional_args=(),
        create_video=False, video=None, quiet=False):
    return run_mapf("cbs", input_file, output=output, timeout=timeout,
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
                **stdio(quiet))
            input_file = annotated_file
        if create_video and not video:
            video = os.path.splitext(os.path.basename(original_file))[0] + "_cbs_roadmap.mp4"
        return run_mapf("cbs_roadmap", input_file, output=output,
                         timeout=timeout, additional_args=additional_args,
                         create_video=create_video, video=video,
                         visualize_script=VISUALIZE_ROADMAP,
                         visualize_args=["--radius", annotate_radius],
                         quiet=quiet)


def cbs_ta(input_file, output=None, timeout=None, additional_args=(),
           create_video=False, video=None, quiet=False):
    """CBS with task assignment. An input without `potentialGoals` is converted
    first (see ensure_potential_goals)."""
    return run_mapf("cbs_ta", ensure_potential_goals(input_file), output=output, timeout=timeout,
                     additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def ecbs(input_file, w=1, output=None, timeout=None, additional_args=(),
         create_video=False, video=None, quiet=False):
    """Enhanced CBS with suboptimality factor `w`."""
    return run_mapf("ecbs", input_file, args=["-w", w], output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def ecbs_ta(input_file, w=1, output=None, timeout=None, additional_args=(),
            create_video=False, video=None, quiet=False):
    """Enhanced CBS with task assignment and suboptimality factor `w`. An input
    without `potentialGoals` is converted first (see ensure_potential_goals)."""
    return run_mapf("ecbs_ta", ensure_potential_goals(input_file), args=["-w", w], output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)


def mapf_prioritized_sipp(input_file, output=None, timeout=None,
                          additional_args=(), create_video=False, video=None, quiet=False):
    return run_mapf("mapf_prioritized_sipp", input_file, output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet)

def next_best_assignment(mapping, output=None, timeout=None, quiet=False):
    """All assignments for a {(agent, task): cost} dict, best first."""
    return run_assignment("next_best_assignment", mapping, output=output,
                           timeout=timeout, quiet=quiet)


def sipp(input_file, output=None, timeout=None, quiet=False):
    """Single-agent safe interval path planning."""
    return run_mapf("sipp", input_file, output=output, timeout=timeout,
                     quiet=quiet)
