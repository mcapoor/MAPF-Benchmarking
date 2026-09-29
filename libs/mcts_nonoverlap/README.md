# mcts_nonoverlap

Monte-Carlo tree search for **vertex-disjoint, static** multi-agent paths (no time dimension), following Kiarostami et al., *Multi-Agent non-Overlapping Pathfinding with Monte-Carlo Tree Search* (IEEE, 2019). Each agent's start and goal are the two ends of one path, and no two paths may share a cell. This is a different problem from the one CBS/ECBS solve: an instance in which two agents share a terminal, or in which paths must cross, has no solution here.

It was first written as part of the [libMultiRobotPlanning fork](https://github.com/mcapoor/libMultiRobotPlanning) and moved here because it is not an algorithm of that library.

| File | Contents |
| --- | --- |
| `mcts.hpp` | A header-only `MCTS<State, Action, Environment>` template (UCT selection, uniform random rollouts, exact dead-end detection, restarts), in the style of libMultiRobotPlanning's algorithms |
| `mcts_nonoverlap.cpp` | The grid instantiation and command-line program |
| `timer.hpp` | Copied from libMultiRobotPlanning (MIT License) |

## Building

Needs CMake, Boost and yaml-cpp. From the benchmarking project, `./build.sh mcts` builds it into `build/`; or by hand:

```sh
cmake -S libs/mcts_nonoverlap -B libs/mcts_nonoverlap/build
cmake --build libs/mcts_nonoverlap/build -j
```

## Usage

```sh
./build/mcts_nonoverlap -i problem.yaml -o output.yaml [--iterations 100] [--exploration 1.414] [--attempts 10] [--seed 0]
```

It reads and writes libMultiRobotPlanning's YAML formats (a `map` and `agents` with `start`/`goal`; a `statistics` and `schedule` output). Waypoint `t` is the step index along the path. If no solution is found, the output has `statistics` but no `schedule`.
