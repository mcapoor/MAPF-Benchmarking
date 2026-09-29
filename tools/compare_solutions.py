"""Solve one MAPF problem with several solvers and draw their solutions side
by side, one panel per solver: a still image of the paths, or an animation of
all panels on one clock (--animate).

    python tools/compare_solutions.py benchmarks/room-32-32-4/room-32-32-4-random-1.scen \\
        --agents 15 --solvers cbs ecbs mapf_prioritized_sipp reloc bcp2
"""
import argparse
import math
import os
import subprocess
import sys

import matplotlib.pyplot as plt
import yaml
from matplotlib import animation
from matplotlib.patches import Circle, Rectangle

# allow running as a script (python tools/compare_solutions.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import utils
from tools.solve_and_visualize import GRID_SOLVERS, MOVINGAI_SOLVERS, solve, solver_input

SOLVERS = [*GRID_SOLVERS, *MOVINGAI_SOLVERS]

# agent i has color i (mod 20) in every panel
AGENT_COLORS = plt.get_cmap("tab20").colors
# solvers of a different problem, whose costs are not comparable with the
# others': the _ta solvers also choose each agent's goal, and mcts_nonoverlap
# has no time (its cost is the total path length)
DIFFERENT_PROBLEM = {solver for solver in SOLVERS if solver.endswith("_ta")} | {"mcts_nonoverlap"}
# how strongly the steps an agent takes as in the best solution are drawn
SAME_AS_BEST_ALPHA = 0.12
# animation frames per time step
FRAMES_PER_STEP = 10


class Solution:
    """One solver's outcome on the problem: `paths` maps each agent name to
    its cell (x, y) at every time step 0, 1, ..., its arrival time (None if
    the solver found no solution, with the reason in `failure`)."""

    def __init__(self, solver, paths=None, seconds=None, failure=None):
        self.solver = solver
        self.paths = paths
        self.seconds = seconds
        self.failure = failure

    def arrival(self, agent):
        return len(self.paths[agent]) - 1

    def cost(self):
        return sum(self.arrival(agent) for agent in self.paths)

    def makespan(self):
        return max((self.arrival(agent) for agent in self.paths), default=0)

    def cell(self, agent, t):
        """The cell of `agent` at time step `t`; it stays at its goal after
        arriving."""
        path = self.paths[agent]
        return path[min(t, len(path) - 1)]

    def step_matches(self, other, agent, t):
        """Whether `agent` moves (or waits) from time step t to t + 1 as it
        does in the `other` solution."""
        return (other is not None and other.paths is not None and agent in other.paths
                and other.cell(agent, t) == self.cell(agent, t)
                and other.cell(agent, t + 1) == self.cell(agent, t + 1))


def dense_path(waypoints):
    """The cell of a path at every time step from 0 to its last waypoint's.

    Some solvers (e.g. mapf_prioritized_sipp) leave out the time steps where
    an agent waits: an agent is at a waypoint's cell from that waypoint's time
    step until the next waypoint's."""
    cells = []
    for waypoint, following in zip(waypoints, waypoints[1:] + [None]):
        until = following["t"] if following else waypoint["t"] + 1
        cells += [(waypoint["x"], waypoint["y"])] * (until - waypoint["t"])
    return cells


def run_solver(problem_file, solver, n_agents, agent_names, w, timeout):
    """Solve the problem with `solver` (quietly) and return its Solution, with
    the agents named as in the YAML problem."""
    try:
        solver_problem, _ = solver_input(problem_file, solver, n_agents)
        result, seconds = solve(solver_problem, solver, w=w, timeout=timeout, quiet=True, report=False)
    except subprocess.TimeoutExpired:
        return Solution(solver, failure="timed out")
    except (subprocess.CalledProcessError, FileNotFoundError) as error:
        return Solution(solver, failure=f"failed: {error}")
    if result is None:
        return Solution(solver, seconds=seconds, failure="no solution")
    schedule = result["schedule"]
    if solver in MOVINGAI_SOLVERS:
        # these name the agents agent0, agent1, ... in the order of the problem
        schedule = {agent_names[int(name[len("agent"):])]: path for name, path in schedule.items()}
    return Solution(solver, {name: dense_path(path) for name, path in schedule.items()}, seconds)


def draw_map(ax, problem):
    """Draw the grid's boundary and obstacles (dark grey, rather than the red
    of tools/visualize.py, so the colored paths stand out)
    and set up `ax` for it."""
    width, height = problem["map"]["dimensions"]
    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(-0.5, height - 0.5)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.add_patch(Rectangle((-0.5, -0.5), width, height, facecolor="none", edgecolor="black"))
    for x, y in problem["map"].get("obstacles") or []:
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor="0.3", edgecolor="0.3"))


def best_solution(solutions):
    """The solution with the lowest sum of costs, the fastest one among
    equals; a solver of a different problem (DIFFERENT_PROBLEM) is only best
    if no other solver found a solution. None if no solver found one."""
    solved = [s for s in solutions if s.paths is not None]
    comparable = [s for s in solved if s.solver not in DIFFERENT_PROBLEM] or solved
    return min(comparable, key=lambda s: (s.cost(), s.seconds), default=None)


def panel_title(solution, best):
    """The solver and its metrics, plus its cost and time difference from the
    best solution."""
    if solution.paths is None:
        return f"{solution.solver}\n{solution.failure}"
    title = (f"{solution.solver}\ncost {solution.cost()}, makespan {solution.makespan()}, "
             f"{solution.seconds:.3f} s")
    if best is solution:
        title += "\nbest (lowest cost)"
    elif best is not None:
        title += (f"\nvs best ({best.solver}): cost {solution.cost() - best.cost():+d}, "
                  f"time {solution.seconds - best.seconds:+.3f} s")
    return title


def draw_paths(ax, solution, best, agent_names, starts=True):
    """Draw each agent's path as a line from its start (circle, unless not
    `starts`) to its goal (square), with a dot on each cell where it waits,
    larger for longer waits. The steps it takes as in the best solution
    are faded."""
    for i, agent in enumerate(agent_names):
        if agent not in solution.paths:
            continue
        color = AGENT_COLORS[i % len(AGENT_COLORS)]
        waits = {}
        for t in range(solution.arrival(agent)):
            (x0, y0), (x1, y1) = solution.cell(agent, t), solution.cell(agent, t + 1)
            alpha = SAME_AS_BEST_ALPHA if best is not solution and solution.step_matches(best, agent, t) else 0.9
            if (x0, y0) == (x1, y1):
                waits.setdefault((x0, y0), []).append(alpha)
            else:
                ax.plot([x0, x1], [y0, y1], color=color, alpha=alpha, linewidth=2.5, solid_capstyle="round")
        for (x, y), alphas in waits.items():
            ax.scatter([x], [y], s=12 * (1 + len(alphas)) ** 1.5, color=color, alpha=min(alphas), zorder=3)
        (sx, sy), (gx, gy) = solution.cell(agent, 0), solution.paths[agent][-1]
        if starts:
            ax.add_patch(Circle((sx, sy), 0.3, facecolor=color, edgecolor="black", zorder=4))
        ax.add_patch(Rectangle((gx - 0.25, gy - 0.25), 0.5, 0.5, facecolor=color, edgecolor="black", zorder=4))


def make_figure(problem, solutions, best):
    """A figure with one panel per solution, in a near-square grid."""
    width, height = problem["map"]["dimensions"]
    columns = math.ceil(math.sqrt(len(solutions)))
    rows = math.ceil(len(solutions) / columns)
    panel = 4.5
    fig, axes = plt.subplots(rows, columns, squeeze=False,
                             figsize=(columns * panel * max(width / height, 0.6), rows * (panel + 0.8)))
    for ax in axes.flat[len(solutions):]:
        ax.set_visible(False)
    panels = list(axes.flat[:len(solutions)])
    for ax, solution in zip(panels, solutions):
        draw_map(ax, problem)
        ax.set_title(panel_title(solution, best), fontsize=9)
        if solution.paths is None:
            ax.text((width - 1) / 2, (height - 1) / 2, solution.failure, ha="center", va="center",
                    fontsize=12, bbox={"facecolor": "white", "alpha": 0.8})
    fig.tight_layout()
    return fig, panels


def draw_still(problem, solutions, best, agent_names):
    fig, panels = make_figure(problem, solutions, best)
    for ax, solution in zip(panels, solutions):
        if solution.paths is not None:
            draw_paths(ax, solution, best, agent_names)
    return fig, None


def draw_animation(problem, solutions, best, agent_names):
    """All panels move on one clock: frame f shows time step f / FRAMES_PER_STEP
    in every panel. The paths are drawn faintly underneath, as in the still
    image, and a panel's agents stop once its makespan is reached."""
    fig, panels = make_figure(problem, solutions, best)
    agents = []  # (solution, agent, circle, label) of every drawn agent
    for ax, solution in zip(panels, solutions):
        if solution.paths is None:
            continue
        # without the start circles, which would look like agents standing still
        draw_paths(ax, solution, best, agent_names, starts=False)
        for line in ax.lines:
            line.set_alpha(line.get_alpha() * 0.4)
        for i, agent in enumerate(agent_names):
            if agent in solution.paths:
                x, y = solution.cell(agent, 0)
                circle = Circle((x, y), 0.35, facecolor=AGENT_COLORS[i % len(AGENT_COLORS)],
                                edgecolor="black", linewidth=1.5, zorder=5)
                ax.add_patch(circle)
                label = ax.text(x, y, agent.replace("agent", ""), ha="center", va="center", fontsize=7, zorder=6)
                agents.append((solution, agent, circle, label))
    clock = fig.text(0.01, 0.99, "", va="top", fontsize=11)
    last_step = max((s.makespan() for s in solutions if s.paths is not None), default=0)

    def update(frame):
        time = frame / FRAMES_PER_STEP
        step, fraction = int(time), time - int(time)
        for solution, agent, circle, label in agents:
            (x0, y0), (x1, y1) = solution.cell(agent, step), solution.cell(agent, step + 1)
            position = (x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction)
            circle.center = position
            label.set_position(position)
        clock.set_text(f"t = {time:.1f}")
        return [artist for _, _, circle, label in agents for artist in (circle, label)] + [clock]

    return fig, animation.FuncAnimation(fig, update, frames=last_step * FRAMES_PER_STEP + 1,
                                        interval=1000 / FRAMES_PER_STEP, blit=False)


def compare(problem_file, solvers, n_agents=None, w=1.0, timeout=60,
            animate=False, output=None, speed=1, show=True):
    """Solve the problem with each of `solvers` and draw the solutions.

    `problem_file` and `n_agents` are as in solve_and_visualize.solver_input.
    Each solution is compared with the best one (see best_solution): its
    steps that match the best solution's are faded, and its title gives its
    cost and time difference from it. The figure (or with `animate` the animation) is saved to `output`
    if given, else shown if `show`. Returns the Solution of each solver."""
    _, yaml_file = solver_input(problem_file, solvers[0], n_agents)
    with open(yaml_file) as stream:
        problem = yaml.load(stream, Loader=utils.YAML_LOADER)
    agent_names = [agent["name"] for agent in problem["agents"]]

    solutions = []
    for solver in solvers:
        solution = run_solver(problem_file, solver, n_agents, agent_names, w, timeout)
        summary = (panel_title(solution, None).replace("\n", ": ", 1) if solution.paths is not None
                   else f"{solver}: {solution.failure}")
        print(summary, flush=True)
        solutions.append(solution)
    best = best_solution(solutions)
    if best is not None:
        print(f"best: {best.solver} (lowest cost)", flush=True)

    draw = draw_animation if animate else draw_still
    fig, anim = draw(problem, solutions, best, agent_names)
    if output and anim is not None:
        anim.save(output, "ffmpeg" if not output.endswith(".gif") else "pillow",
                  fps=FRAMES_PER_STEP * speed, dpi=150)
    elif output:
        fig.savefig(output, dpi=150)
    elif show:
        plt.show()
    plt.close(fig)
    return solutions


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Solve one MAPF problem with several solvers and draw their paths side by side.")
    parser.add_argument("problem", help="MAPF problem: a libMultiRobotPlanning YAML file, or a MovingAI .scen file (with --agents)")
    parser.add_argument("--solvers", nargs="+", choices=SOLVERS, required=True, help="solvers to compare, one panel each")
    parser.add_argument("--agents", type=int, default=None, help="number of agents of a .scen problem: its first n agents")
    parser.add_argument("--timeout", type=float, default=60.0, help="seconds allowed per solver (default 60)")
    parser.add_argument("-w", type=float, default=1.0, help="suboptimality factor (ecbs, ecbs_ta only)")
    parser.add_argument("--animate", action="store_true", help="animate all panels on one clock instead of drawing the paths")
    parser.add_argument("--output", default=None, help="save to this file (an image, or .mp4/.gif with --animate) instead of showing it")
    parser.add_argument("--speed", type=int, default=1, help="animation speed-up factor")
    args = parser.parse_args()

    try:
        compare(args.problem, args.solvers, n_agents=args.agents, w=args.w,
                timeout=args.timeout, animate=args.animate, output=args.output, speed=args.speed)
    except (ValueError, FileNotFoundError) as error:
        sys.exit(str(error))
