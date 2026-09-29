import argparse
import os
import subprocess
import sys
import time

import yaml

# allow running as a script (python tools/solve_and_visualize.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import bcp2_wrappers, chbp_wrappers, libMRP_wrappers, mcts_wrappers, reloc_wrappers, utils
from tools.visualize import Animation

RoadmapAnimation = utils.load_module(utils.VISUALIZE_ROADMAP, "libmrp_visualize_roadmap").Animation

# Grid solvers take a MAPF yaml (map + agents with [x, y] start/goal)
GRID_SOLVERS = {
    "cbs": lambda file, w, radius, annotate, timeout, quiet, brief: libMRP_wrappers.cbs(file, timeout=timeout, quiet=quiet),
    "cbs_ta": lambda file, w, radius, annotate, timeout, quiet, brief: libMRP_wrappers.cbs_ta(file, timeout=timeout, quiet=quiet),
    "ecbs": lambda file, w, radius, annotate, timeout, quiet, brief: libMRP_wrappers.ecbs(file, w, timeout=timeout, quiet=quiet),
    "ecbs_ta": lambda file, w, radius, annotate, timeout, quiet, brief: libMRP_wrappers.ecbs_ta(file, w, timeout=timeout, quiet=quiet),
    "mapf_prioritized_sipp": lambda file, w, radius, annotate, timeout, quiet, brief: libMRP_wrappers.mapf_prioritized_sipp(file, timeout=timeout, quiet=quiet),
    "mcts_nonoverlap": lambda file, w, radius, annotate, timeout, quiet, brief: mcts_wrappers.mcts_nonoverlap(file, timeout=timeout, quiet=quiet),
}
# MovingAI solvers read the MovingAI map and scenario files themselves, given
# (scen_file, n_agents): the problem of the first n_agents agents of scen_file
MOVINGAI_SOLVERS = {
    "cbsh2_rtc": lambda instance, w, radius, annotate, timeout, quiet, brief: chbp_wrappers.cbsh2_rtc(*instance, timeout=timeout, quiet=quiet),
    "cbsh2_rtc_ch": lambda instance, w, radius, annotate, timeout, quiet, brief: chbp_wrappers.cbsh2_rtc_ch(*instance, timeout=timeout, quiet=quiet),
    "cbsh2_rtc_bp": lambda instance, w, radius, annotate, timeout, quiet, brief: chbp_wrappers.cbsh2_rtc_bp(*instance, timeout=timeout, quiet=quiet),
    "cbsh2_rtc_chbp": lambda instance, w, radius, annotate, timeout, quiet, brief: chbp_wrappers.cbsh2_rtc_chbp(*instance, timeout=timeout, quiet=quiet),
    "reloc": lambda instance, w, radius, annotate, timeout, quiet, brief: reloc_wrappers.reloc(*instance, timeout=timeout, quiet=quiet, brief=brief),
    "bcp2": lambda instance, w, radius, annotate, timeout, quiet, brief: bcp2_wrappers.bcp2(*instance, timeout=timeout, quiet=quiet),
}
# Roadmap solvers a roadmap yaml (agents with vertex names).
ROADMAP_SOLVERS = {
    "cbs_roadmap": lambda file, w, radius, annotate, timeout, quiet, brief: libMRP_wrappers.cbs_roadmap(
        file, annotate=annotate, annotate_radius=radius, timeout=timeout,
        quiet=quiet),
}


def solver_binary(solver):
    """The executable that `solver` runs, which must have been built first
    (see build.sh)."""
    if solver in utils.LIBMRP_SOLVERS:
        return os.path.join(utils.BUILD_DIR, solver)
    return {
        "mcts_nonoverlap": os.path.join(mcts_wrappers.MCTS_BUILD_DIR, "mcts_nonoverlap"),
        "cbsh2_rtc": chbp_wrappers.CHBP_BINARY,
        "cbsh2_rtc_ch": chbp_wrappers.CHBP_BINARY,
        "cbsh2_rtc_bp": chbp_wrappers.CHBP_BINARY,
        "cbsh2_rtc_chbp": chbp_wrappers.CHBP_BINARY,
        "reloc": reloc_wrappers.RELOC_BINARY,
        "bcp2": bcp2_wrappers.BCP2_BINARY,
    }[solver]


def solve(map_file, solver, w=1.0, radius=0.3, annotate=False, timeout=None,
          quiet=False, brief=False, report=True):
    """Run `solver` on the MAPF problem in `map_file`, time it and report it.

    `map_file` is a YAML problem for GRID_SOLVERS and ROADMAP_SOLVERS, and a
    (scen_file, n_agents) pair for MOVINGAI_SOLVERS (see `solver_input` to
    get either from any problem file).

    `w` is the suboptimality factor (ecbs, ecbs_ta only); `radius` and `annotate`
    are the robot radius and whether to annotate the roadmap with collisions
    before solving (cbs_roadmap only). `timeout` is in seconds: the
    libMultiRobotPlanning solvers raise subprocess.TimeoutExpired when it runs
    out, and the MOVINGAI_SOLVERS stop by themselves without a solution. With `quiet` all of the solver's own console
    output is discarded, leaving only the success and timing message printed
    here, which `report=False` suppresses too. With `brief` a solver that is
    verbose by default (reloc) prints only a one-line summary, like the others.

    Returns (result, seconds), where result is None if there was no solution.
    """
    start_time = time.perf_counter()
    result = {**GRID_SOLVERS, **MOVINGAI_SOLVERS, **ROADMAP_SOLVERS}[solver](map_file, w, radius, annotate, timeout, quiet, brief)
    elapsed = time.perf_counter() - start_time
    if isinstance(map_file, tuple):
        map_file = f"the first {map_file[1]} agents of {map_file[0]}"
    if not result or "schedule" not in result:
        if report:
            print(f"{solver} found no solution for {map_file} after {elapsed:.3f} s", file=sys.stderr)
        return None, elapsed
    if report:
        print(f"{solver} solved {map_file} in {elapsed:.3f} s ({describe_solution(result)})")
    return result, elapsed


def describe_solution(result):
    """The sum of costs and makespan of a solver's result (see
    utils.solution_metrics), e.g. "cost 55, makespan 8"."""
    cost, makespan = utils.solution_metrics(result["schedule"])
    return f"cost {cost}, makespan {makespan}"


def solver_input(problem_file, solver, n_agents=None):
    """The input of `solver` for a problem file, and the YAML problem to draw
    its solution on.

    `problem_file` is a libMultiRobotPlanning YAML problem, or a MovingAI
    scenario file (.scen) whose first `n_agents` agents are the problem.
    Either kind is converted to what `solver` reads, into cache/ (see
    utils.libmrp_problem and utils.movingai_problem). Roadmap solvers only
    take YAML roadmaps.

    Returns (input, yaml_file): input as `solve` takes it.
    """
    if problem_file.endswith(".scen"):
        if solver in ROADMAP_SOLVERS:
            raise ValueError(f"{solver} solves roadmap problems, not MovingAI scenarios")
        if not n_agents:
            raise ValueError("a MovingAI scenario needs the number of agents to solve (--agents)")
        yaml_file = utils.libmrp_problem(problem_file, n_agents)
        return ((problem_file, n_agents) if solver in MOVINGAI_SOLVERS else yaml_file), yaml_file
    if solver in MOVINGAI_SOLVERS:
        return utils.movingai_problem(problem_file), problem_file
    return problem_file, problem_file


def solve_and_visualize(problem_file, solver, n_agents=None, w=1.0, radius=0.3,
                        annotate=False, timeout=None, visualize=True, video=None,
                        speed=1, quiet=False, brief=False):
    """Solve a MAPF problem with `solver` and animate the result.

    `problem_file` and `n_agents` are as in `solver_input`, so any solver runs
    on a YAML problem or a MovingAI scenario. See `solve` for the solver
    arguments. The animation is saved to `video` if given, else shown on
    screen if `visualize`.

    Returns the solver's result, or None if it found no solution.
    """
    solver_problem, yaml_file = solver_input(problem_file, solver, n_agents)
    with open(yaml_file) as stream:
        map = yaml.load(stream, Loader=utils.YAML_LOADER)

    is_roadmap = solver in ROADMAP_SOLVERS
    if is_roadmap and "vertices" not in map.get("roadmap", {}):
        raise ValueError(f"{yaml_file} has no roadmap vertex positions, which are needed to draw it")

    schedule, elapsed = solve(solver_problem, solver, w, radius, annotate, timeout=timeout,
                              quiet=quiet, brief=brief, report=False)
    problem = problem_file if yaml_file == problem_file else f"the first {n_agents} agents of {problem_file}"
    if schedule is None:
        print(f"{solver} found no solution for {problem} after {elapsed:.3f} s", file=sys.stderr)
        return None
    print(f"{solver} solved {problem} in {elapsed:.3f} s ({describe_solution(schedule)})")
    if solver in MOVINGAI_SOLVERS:
        # these name the agents agent0, agent1, ... in the order of the problem
        names = [agent["name"] for agent in map["agents"]]
        schedule["schedule"] = {names[int(name[len("agent"):])]: path
                                for name, path in schedule["schedule"].items()}

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
    parser.add_argument('problem', help='MAPF problem: a libMultiRobotPlanning YAML file, or a MovingAI .scen file (with --agents)')
    parser.add_argument('solver', choices=[*GRID_SOLVERS, *MOVINGAI_SOLVERS, *ROADMAP_SOLVERS], help='Solver to run')
    parser.add_argument('--agents', type=int, default=None, help='number of agents of a .scen problem: its first n agents')
    parser.add_argument('--timeout', type=float, default=None, help='seconds allowed for the solver (default: no limit)')

    # solver-specific arguments
    parser.add_argument('-w', type=float, default=1.0, help='suboptimality factor (ecbs, ecbs_ta only)')
    parser.add_argument('--quiet', action='store_true', help='discard the solver\'s own console output, keeping only the success and timing message')
    parser.add_argument('--brief', action='store_true', help='print only a one-line summary of the solver\'s output, like the other solvers print (reloc only)')
    parser.add_argument('--radius', type=float, default=0.3, help='radius of robot (cbs_roadmap only)')
    parser.add_argument('--annotate', action='store_true', help='annotate the roadmap with collisions before solving (cbs_roadmap only)')

    # video settings
    parser.add_argument('--visualize', action=argparse.BooleanOptionalAction, default=True, help='whether to display matplot animation (--no-visualize to skip)')
    parser.add_argument('--video', default=None, help='output video file (or leave empty to show on screen)')
    parser.add_argument('--speed', type=int, default=1, help='video speedup-factor')
    args = parser.parse_args()

    try:
        result = solve_and_visualize(args.problem, args.solver, n_agents=args.agents, w=args.w,
                                     radius=args.radius, annotate=args.annotate, timeout=args.timeout,
                                     visualize=args.visualize, video=args.video, speed=args.speed,
                                     quiet=args.quiet, brief=args.brief)
    except (ValueError, FileNotFoundError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        sys.exit(str(error))
    if result is None:
        sys.exit(1)
