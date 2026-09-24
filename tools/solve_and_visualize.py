import argparse
import os
import sys
import time

import yaml

# allow running as a script (python tools/solve_and_visualize.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import solver_wrappers

Animation = solver_wrappers.load_module(solver_wrappers.VISUALIZE, "libmrp_visualize").Animation
RoadmapAnimation = solver_wrappers.load_module(solver_wrappers.VISUALIZE_ROADMAP, "libmrp_visualize_roadmap").Animation

# Grid solvers take a MAPF yaml (map + agents with [x, y] start/goal)
GRID_SOLVERS = {
    "cbs": lambda file, w, radius, annotate, timeout, quiet: solver_wrappers.cbs(file, timeout=timeout, quiet=quiet),
    "cbs_ta": lambda file, w, radius, annotate, timeout, quiet: solver_wrappers.cbs_ta(file, timeout=timeout, quiet=quiet),
    "ecbs": lambda file, w, radius, annotate, timeout, quiet: solver_wrappers.ecbs(file, w, timeout=timeout, quiet=quiet),
    "ecbs_ta": lambda file, w, radius, annotate, timeout, quiet: solver_wrappers.ecbs_ta(file, w, timeout=timeout, quiet=quiet),
    "mapf_prioritized_sipp": lambda file, w, radius, annotate, timeout, quiet: solver_wrappers.mapf_prioritized_sipp(file, timeout=timeout, quiet=quiet),
    "mcts_nonoverlap": lambda file, w, radius, annotate, timeout, quiet: solver_wrappers.mcts_nonoverlap(file, timeout=timeout, quiet=quiet),
}
# Roadmap solvers a roadmap yaml (agents with vertex names).
ROADMAP_SOLVERS = {
    "cbs_roadmap": lambda file, w, radius, annotate, timeout, quiet: solver_wrappers.cbs_roadmap(
        file, annotate=annotate, annotate_radius=radius, timeout=timeout,
        quiet=quiet),
}


def solve(map_file, solver, w=1.0, radius=0.3, annotate=False, timeout=None,
          quiet=False, report=True):
    """Run `solver` on the MAPF problem in `map_file`, time it and report it.

    `w` is the suboptimality factor (ecbs, ecbs_ta only); `radius` and `annotate`
    are the robot radius and whether to annotate the roadmap with collisions
    before solving (cbs_roadmap only). `timeout` is in seconds and raises
    subprocess.TimeoutExpired. With `quiet` all of the solver's own console
    output is discarded, leaving only the success and timing message printed
    here, which `report=False` suppresses too.

    Returns (result, seconds), where result is None if there was no solution.
    """
    start_time = time.perf_counter()
    result = {**GRID_SOLVERS, **ROADMAP_SOLVERS}[solver](map_file, w, radius, annotate, timeout, quiet)
    elapsed = time.perf_counter() - start_time
    if not result or "schedule" not in result:
        if report:
            print(f"{solver} found no solution for {map_file} after {elapsed:.3f} s", file=sys.stderr)
        return None, elapsed
    if report:
        print(f"{solver} solved {map_file} in {elapsed:.3f} s")
    return result, elapsed


def solve_and_visualize(map_file, solver, w=1.0, radius=0.3, annotate=False,
                        visualize=True, video=None, speed=1, quiet=False):
    """Solve the MAPF problem in `map_file` with `solver` and animate the result.

    See `solve` for the solver arguments. The animation is saved to `video` if
    given, else shown on screen if `visualize`.

    Returns the solver's result, or None if it found no solution.
    """
    with open(map_file) as stream:
        map = yaml.load(stream, Loader=solver_wrappers.YAML_LOADER)

    is_roadmap = solver in ROADMAP_SOLVERS
    if is_roadmap and "vertices" not in map.get("roadmap", {}):
        raise ValueError(f"{map_file} has no roadmap vertex positions, which are needed to draw it")

    schedule, _ = solve(map_file, solver, w, radius, annotate, quiet=quiet)
    if schedule is None:
        return None

    if video or visualize:
        animation = RoadmapAnimation(map, schedule, radius) if is_roadmap else Animation(map, schedule)
        if video:
            animation.save(video, speed)
        else:
            animation.show()
    return schedule


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # required arguments for all instances
    parser.add_argument('map', help='MAPF problem (yaml)')
    parser.add_argument('solver', choices=[*GRID_SOLVERS, *ROADMAP_SOLVERS], help='Solver to run')

    # solver-specific arguments
    parser.add_argument('-w', type=float, default=1.0, help='suboptimality factor (ecbs, ecbs_ta only)')
    parser.add_argument('--quiet', action='store_true', help='discard the solver\'s own console output, keeping only the success and timing message')
    parser.add_argument('--radius', type=float, default=0.3, help='radius of robot (cbs_roadmap only)')
    parser.add_argument('--annotate', action='store_true', help='annotate the roadmap with collisions before solving (cbs_roadmap only)')

    # video settings
    parser.add_argument('--visualize', action=argparse.BooleanOptionalAction, default=True, help='whether to display matplot animation (--no-visualize to skip)')
    parser.add_argument('--video', default=None, help='output video file (or leave empty to show on screen)')
    parser.add_argument('--speed', type=int, default=1, help='video speedup-factor')
    args = parser.parse_args()

    try:
        result = solve_and_visualize(args.map, args.solver, w=args.w, radius=args.radius,
                                     annotate=args.annotate, visualize=args.visualize,
                                     video=args.video, speed=args.speed, quiet=args.quiet)
    except ValueError as error:
        sys.exit(str(error))
    if result is None:
        sys.exit(1)
