from tools.utils import *

# vertex-disjoint path search by Monte-Carlo tree search (libs/mcts_nonoverlap,
# built into its build/ directory), which reads libMultiRobotPlanning's YAML
# problem format
MCTS_BUILD_DIR = os.path.join(LIBS_DIR, "mcts_nonoverlap", "build")


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
    return run_mapf("mcts_nonoverlap", input_file, args=args, output=output,
                     timeout=timeout, additional_args=additional_args,
                     create_video=create_video, video=video, quiet=quiet,
                     build_dir=MCTS_BUILD_DIR)
