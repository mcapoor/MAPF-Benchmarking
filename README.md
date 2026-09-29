# MAPF Benchmarking

A harness for running and comparing multi-agent path finding (MAPF) solvers from different libraries on the same benchmark scenarios. Each solver library is a git submodule in `libs/` and stays as close to its own upstream as possible. The scripts here translate between the libraries and one common problem/result format, run the solvers, and collect the timings.

## Layout

| Path | Contents |
| --- | --- |
| `libs/libMultiRobotPlanning/` | Submodule: [a fork](https://github.com/mcapoor/libMultiRobotPlanning) of Wolfgang Hönig's [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) (CBS, ECBS, CBS-TA, ECBS-TA, prioritized SIPP). See its README for the algorithms and for what the fork changes. |
| `libs/CBSH2-RTC-CHBP/` | Submodule: [a fork](https://github.com/mcapoor/CBSH2-RTC-CHBP) of Bojie Shen et al.'s [CBSH2-RTC-CHBP](https://github.com/bshen95/CBSH2-RTC-CHBP) (GPL-3.0; run as a separate program): the optimal CBS solver CBSH2-RTC with the cluster heuristic and bypass (CHBP) improvements. Wrapped in `tools/chbp_wrappers.py`. |
| `libs/mcts_nonoverlap/` | Monte-Carlo tree search for vertex-disjoint paths (not a submodule; moved here from the libMultiRobotPlanning fork). See its README. |
| `libs/bcp2-mapf/` | Submodule: [a fork](https://github.com/mcapoor/bcp2-mapf) of Edward Lam's [BCP2-MAPF](https://github.com/ed-lam/bcp2-mapf) (PolyForm Noncommercial license; branch-and-cut-and-price, needs Gurobi) |
| `libs/reLOC/` | Submodule: [a fork](https://github.com/mcapoor/reLOC) of Pavel Surynek's [reLOC](https://github.com/surynek/reLOC), cut down to its SAT-based MAPF solver `insolver_reLOC` (the unmodified code is on its `upstream` branch). See its README. |
| `tools/utils.py` | Shared helpers of the wrappers: library paths, running a solver binary, reading its output |
| `tools/<library>_wrappers.py` | One Python function per solver binary of a library (`libMRP_wrappers.py`, `mcts_wrappers.py`, `chbp_wrappers.py`, `reloc_wrappers.py`). This is where library-specific paths, arguments and output formats are handled. |
| `tools/solve_and_visualize.py` | Solve one problem and animate the result |
| `tools/compare_solutions.py` | Solve one problem with several solvers and draw their paths side by side |
| `tools/visualize.py` | Grid animation of a solution, used for every solver (moved here from libMultiRobotPlanning, MIT License) |
| `tools/standard_benchmark_converter.py` | Converts MovingAI problems to libMultiRobotPlanning's YAML format |
| `benchmark.py` | Benchmark runner: time every solver on a scenario directory |
| `build.sh` | Builds the solver libraries (see [Setup](#setup)) |
| `benchmark.sh` | Slurm launcher for `benchmark.py` |
| `benchmarks/` | The [Moving AI MAPF benchmarks](https://movingai.com/benchmarks/mapf/index.html), one directory per map with its random and even scenarios (see [Problems](#problems)) |
| `cache/` | Problems converted for a library's own format, reused between runs (not tracked by git) |
| `output/` | Results of `benchmark.py` runs (not tracked by git) |

## Setup

Clone with the submodules, then build every library with `build.sh`:

```sh
git clone --recurse-submodules <this repo>
./build.sh                  # all libraries, each into libs/<library>/build/
./build.sh cbsh2 bcp2       # only some of them (libmrp, mcts, cbsh2, bcp2, reloc)
./build.sh --clean libmrp   # delete the build/ directory first
```

`build.sh` initializes a submodule that has not been cloned yet, but leaves the ones already checked out alone. It tries every library even if one fails, and lists the failures at the end. `JOBS=4 ./build.sh` limits the parallel compile jobs (bcp2-mapf needs a lot of memory with many), and `CC`/`CXX` choose the compiler.

What each library needs:

- libMultiRobotPlanning and mcts_nonoverlap: CMake, Boost and yaml-cpp.
- CBSH2-RTC-CHBP: CMake and Boost.
- reLOC: `make` and a C++11 compiler.
- bcp2-mapf: CMake 3.21+, a C++23 compiler and [Gurobi](https://www.gurobi.com/) (with a license, free for academics). The script finds Gurobi through `$GUROBI_DIR`, `$GUROBI_HOME` or the `gurobi_cl` on your `PATH`, and works around two problems of the library's build: some of its downloaded dependencies are pinned in a way that git cannot check out (the script clones them itself into `build/_deps-full/`), and GCC 16 cannot compile it (Clang is used instead if installed).

On the Oscar cluster, run `module load boost yaml-cpp gurobi` first. The solvers must be compiled on the machine that runs them, so build them on the cluster too.

Python 3.9+ with the libraries' requirements:

```sh
pip install -r libs/libMultiRobotPlanning/requirements.txt
```

Saving an animation to a video (`--video`) also needs `ffmpeg` on your `PATH`.

Run everything from this directory, e.g. `python tools/solve_and_visualize.py ...`.

## Problems

### MovingAI scenarios (the default)

Benchmarks use the [Moving AI MAPF benchmarks](https://movingai.com/benchmarks/mapf/index.html), in `benchmarks/`. It has one directory per map, holding the `.map` file and its 25 random and 25 even scenario files:

```text
benchmarks/empty-8-8/
    empty-8-8.map
    empty-8-8-random-1.scen ... empty-8-8-random-25.scen
    empty-8-8-even-1.scen   ... empty-8-8-even-25.scen
```

Random scenarios place starts and goals at random; even scenarios have the same number of agents (10) for each range of optimal single-agent path length, so long paths are as common as short ones. `benchmark.py --scen-type` chooses between them (random by default). There are 33 maps: empty, random, room, maze, warehouse, city (`Berlin_1_256`, `Boston_0_256`, `Paris_1_256`) and game maps (`den312d`, `ht_chantry`, `lak303d`, ...). A `.scen` file lists up to 1000 agents (fewer on small maps; 32 on `empty-8-8`). **The problem with n agents of a scenario file is its first n agents**, in file order, which is also what `--agent-limit` of bcp2-mapf does, so every solver gets the same problem.

To add a map, make a directory under `benchmarks/` (or anywhere else, then pass its path to `benchmark.py`) with its `.map` and its `<map>-random-<k>.scen` and/or `<map>-even-<k>.scen` files. The map must be in the same directory as its scenarios, because the solvers that read MovingAI files look for it there.

### libMultiRobotPlanning's YAML format

The libMultiRobotPlanning solvers read a YAML file with a `map` and a list of `agents`:

```yaml
agents:
-   name: agent0
    start: [1, 7]
    goal: [6, 3]
map:
    dimensions: [8, 8]
    obstacles:
    - [4, 5]
```

`benchmark.py` writes these for you with `tools/standard_benchmark_converter.py`, into `cache/libMultiRobotPlanning/<map>/<scen>_agents<n>.yaml`, e.g. `cache/libMultiRobotPlanning/empty-8-8/empty-8-8-random-1_agents10.yaml`. A file that already exists is reused. To convert a problem by hand:

```python
from tools import standard_benchmark_converter
standard_benchmark_converter.convert("benchmarks/empty-8-8/empty-8-8-random-1.scen",
                                     "benchmarks/empty-8-8/empty-8-8.map", 10, "empty-8-8-random-1_agents10.yaml")
```

Run as a script, `python tools/standard_benchmark_converter.py <file.scen> <file.map> <output_prefix>` writes `<output_prefix>_<n>_agents.yaml` for n = 10, 20, 30, ...

The task-assignment solvers (`cbs_ta`, `ecbs_ta`) read a `potentialGoals` list per agent instead. If a file has none, the wrappers convert it automatically: a copy is written to `<input dir>_potentialGoals/` (e.g. `cache/libMultiRobotPlanning/empty-8-8_potentialGoals/`) in which every agent may take any agent's goal. The original files are not modified, and an existing converted file is reused.

Roadmap problems (for `cbs_roadmap`) have agents that start and end at named vertices, and a `roadmap` section.

## Solving one problem: `tools/solve_and_visualize.py`

```sh
python tools/solve_and_visualize.py <problem.yaml> <solver> [options]
python tools/solve_and_visualize.py <scenario.scen> <solver> --agents N [options]

python tools/solve_and_visualize.py benchmarks/empty-8-8/empty-8-8-random-1.scen bcp2 --agents 10
python tools/solve_and_visualize.py benchmarks/empty-8-8/empty-8-8-random-1.scen ecbs --agents 10 -w 1.5 --video out.mp4 --speed 2
python tools/solve_and_visualize.py cache/libMultiRobotPlanning/empty-8-8/empty-8-8-random-1_agents10.yaml cbsh2_rtc_chbp
```

Every solver runs on either kind of problem: a libMultiRobotPlanning YAML file, or the first N agents of a MovingAI scenario file. The problem is converted to the format the solver reads first, into `cache/`:

- A scenario becomes a YAML file in `cache/libMultiRobotPlanning/<map>/`, the same file `benchmark.py` uses. It is also what the animation draws.
- A YAML problem becomes a map and scenario file in `cache/movingai/<name>/` for the solvers that read MovingAI files (written again when the YAML file is newer), and their paths are shown under the YAML file's agent names.

Solvers: `cbs`, `cbs_ta`, `ecbs`, `ecbs_ta`, `mapf_prioritized_sipp`, `mcts_nonoverlap`, `cbsh2_rtc`, `cbsh2_rtc_ch`, `cbsh2_rtc_bp`, `cbsh2_rtc_chbp`, `reloc`, `bcp2` (grid) and `cbs_roadmap` (roadmap, YAML only).

| Option | Meaning |
| --- | --- |
| `--agents N` | Solve the first N agents of a `.scen` problem (required for one) |
| `--timeout T` | Seconds allowed for the solver (default: no limit) |
| `-w W` | Suboptimality factor (`ecbs`, `ecbs_ta` only; default 1.0) |
| `--radius R` | Robot radius (`cbs_roadmap` only; default 0.3) |
| `--annotate` | Annotate the roadmap with collisions before solving (`cbs_roadmap` only; needs `cvxpy`) |
| `--video FILE` | Save the animation instead of showing it |
| `--speed N` | Video speed-up factor (default 1) |
| `--no-visualize` | Solve and report the time only |
| `--quiet` | Hide the solver's own output, keeping the success and timing message |
| `--brief` | Print only a one-line summary of the solver's output (outcome, cost, makespan, time), like the other solvers print, instead of reLOC's full log (`reloc` only) |

The success message gives the wall time, the sum of costs and the makespan of the solution (computed as in `benchmark.py`). The exit status is 1 if the solver found no solution. The same functionality is available from Python as `solve_and_visualize(...)`, and as `solve(...)` (returns `(result, seconds)`) with the input from `solver_input(problem_file, solver, n_agents)`.

## Comparing solvers' paths: `tools/compare_solutions.py`

```sh
python tools/compare_solutions.py <problem.yaml | scenario.scen> --solvers S [S ...] [options]

python tools/compare_solutions.py benchmarks/room-32-32-4/room-32-32-4-random-1.scen --agents 15 \
    --solvers cbs ecbs mapf_prioritized_sipp reloc bcp2
python tools/compare_solutions.py benchmarks/empty-8-8/empty-8-8-random-1.scen --agents 10 \
    --solvers cbs reloc --animate --output compare.gif
```

Solves one problem (as for `solve_and_visualize.py`, a YAML file or the first N agents of a scenario) with each solver and draws one panel per solver, titled with its sum of costs, makespan and wall time. Every agent has the same color in every panel. A solver that fails, finds no solution or times out gets a panel saying so.

- The still image (default) draws each agent's path from its start (circle) to its goal (square), with a dot on each cell where it waits (larger for longer waits).
- `--animate` moves the agents of all panels on one clock, over their paths drawn faintly.
- Every solution is compared with the best one: the lowest sum of costs, the fastest among equals. Its panel says "best (lowest cost)", and every other panel gives its cost and time difference from it (e.g. `vs best (bcp2): cost +2, time +0.018 s`). The steps an agent takes as in the best solution are faded, so the differences stand out. The `_ta` solvers and `mcts_nonoverlap` solve a different problem, so they are only the best if no other solver found a solution.

| Option | Meaning |
| --- | --- |
| `--solvers S ...` | Solvers to compare (any grid solver; required) |
| `--agents N` | Solve the first N agents of a `.scen` problem (required for one) |
| `--timeout T` | Seconds allowed per solver (default 60) |
| `-w W` | Suboptimality factor (`ecbs`, `ecbs_ta` only; default 1.0) |
| `--animate` | Animate instead of drawing a still image |
| `--output FILE` | Save to FILE (an image, or `.mp4`/`.gif` with `--animate`) instead of showing it |
| `--speed N` | Animation speed-up factor (default 1) |

## Calling solvers from Python: `tools/*_wrappers.py`

`tools/libMRP_wrappers.py` has one function per libMultiRobotPlanning binary: `cbs`, `cbs_ta`, `ecbs`, `ecbs_ta`, `cbs_roadmap`, `mapf_prioritized_sipp`, `sipp`, `a_star`, `a_star_epsilon`, `assignment` and `next_best_assignment`. `tools/mcts_wrappers.py` has `mcts_nonoverlap`, which reads the same YAML format. Each runs the binary and returns its parsed YAML output, or `None` if there was no solution. Common keyword arguments are `timeout` (seconds; raises `subprocess.TimeoutExpired`), `output` (keep the raw output file), `quiet`, and for the MAPF solvers `create_video` / `video`.

`mcts_nonoverlap` looks for vertex-disjoint paths (each agent's start and goal are the two ends of one path), so most of the benchmark instances have no solution for it; the result then has no `schedule` entry instead of being `None`. It also takes `iterations`, `exploration`, `attempts` and `seed`, which are only passed on if given.

```python
from tools import libMRP_wrappers

result = libMRP_wrappers.ecbs("cache/libMultiRobotPlanning/empty-8-8/empty-8-8-random-1_agents10.yaml", w=1.5, timeout=10)
print(result["statistics"], result["schedule"])
```

The solvers that read the MovingAI files take `(scen_file, n_agents)` and return the same kind of dict (with `statistics` and `schedule`, or `None`):

- `tools/chbp_wrappers.py`: `cbsh2_rtc`, `cbsh2_rtc_ch`, `cbsh2_rtc_bp` and `cbsh2_rtc_chbp`, the variants of CBSH2-RTC-CHBP.
- `tools/bcp2_wrappers.py`: `bcp2`, BCP2-MAPF's sum-of-costs optimal branch-and-cut-and-price solver. It reads the solution from the solver's console output, and counts a solution that is not proven optimal when the time limit runs out as unsolved (`None`). Running it needs a Gurobi license.
- `tools/reloc_wrappers.py`: `reloc`, reLOC's sum-of-costs optimal SAT solver (`encoding="mdd"` by default). It converts the problem into `cache/reLOC/<map>/<scen>_agents<n>.cpf` first (reused if it exists), with the free cells in row-major order as the vertices, and reads the solution from the solver's console output. reLOC does not let an agent move into a cell that another agent leaves in the same step, which the other solvers allow, so its optimal costs can be higher (e.g. 56 instead of 55 on the first 10 agents of `empty-8-8-random-1`).

Library scripts are imported by file path with `utils.load_module(path, name)` rather than as packages, so that each library's `tools/` folder does not clash with this one.

## Benchmarking: `benchmark.py`

`benchmark.py` measures how the solvers scale with the number of agents on one map of `benchmarks/`: the libMultiRobotPlanning grid solvers (`cbs`, `cbs_ta`, `ecbs`, `ecbs_ta`, `mapf_prioritized_sipp`), `mcts_nonoverlap`, the CBSH2-RTC-CHBP variants (`cbsh2_rtc`, `cbsh2_rtc_ch`, `cbsh2_rtc_bp`, `cbsh2_rtc_chbp`), `reloc` and `bcp2`. Every solver it runs must be built (`./build.sh`); it stops with an error naming the ones that are not.

The scenario is the name of a map in `benchmarks/` (e.g. `empty-8-8`) or the path of any directory with one `.map` and its scenario files.

```sh
python benchmark.py empty-8-8 --timeout 5 --jobs 4
python benchmark.py random-32-32-10 --agents 10 20 50 100 --max-instances 5 --timeout 30
python benchmark.py room-32-32-4 --scen-type even --timeout 30
```

For every agent count n, a batch runs each solver on the first n agents of each scenario file. Before any timing starts, `find_instances()` prepares each solver's input: solvers that read libMultiRobotPlanning's format (`utils.YAML_SOLVERS`, including `mcts_nonoverlap`) get YAML files, converted into `cache/` as described under [Problems](#problems); any other solver gets the `.scen` file and n (`MOVINGAI_SOLVERS` in `tools/solve_and_visualize.py`), and for `reloc` its `.cpf` file is written into `cache/reLOC/` as well.

| Option | Meaning |
| --- | --- |
| `--solvers S ...` | Solvers to compare (default: all but `mcts_nonoverlap`) |
| `--timeout T` | Seconds allowed per run (default 10); a run that exceeds it counts as unsolved. The CBSH2-RTC-CHBP, reLOC and BCP2-MAPF solvers get it as their own time limit, and a solution that arrives after more than T seconds of wall time still counts as unsolved |
| `--scen-type T` | Scenario files to use: `random` (default) or `even` |
| `--agents N ...` | Agent counts to run (default: 10, 20, 30, ... up to the number of agents of the smallest scenario file) |
| `--max-instances M` | Use only the first M scenario files (default: all 25) |
| `--jobs J` | Runs to execute in parallel (default 1) |
| `--name NAME` | Prefix of the files written to `output/` (default: the map directory name, e.g. `empty-8-8`, plus `-even` with `--scen-type even`) |
| `--output FILE` | Plot file (default `output/<name>_times.png`) |
| `--results FILE` | CSV of the raw results (default `output/<name>_results.csv`) |
| `--no-show` | Do not open the plot window |

Every run writes these files to `output/`, updating them after every agent count so that an interrupted run still leaves partial results:

| File | Contents |
| --- | --- |
| `<name>_times.png` | Plot against the number of agents: mean time over solved instances per solver on a log scale (top left), the fraction of instances solved within the timeout (bottom left), and the mean sum of costs (top right) and makespan (bottom right) over the instances that every solver in the run solved |
| `<name>_results.csv` | Raw results, with the columns `solver,n_agents,map,seconds,cost,makespan,solver_cost`: the wall time, the sum of costs and makespan of the solution, and the cost the solver reported itself (all empty for an unsolved run; `solver_cost` also for a solver that reports none) |
| `<name>_yaml_details.json`, `<name>_agent_details.json`, `<name>_map_details.json` | The MAPFAST dataset (see below) |

A new run with the same name replaces the previous run's files; pass `--name` to keep them apart.

The sum of costs and makespan are computed the same way for every solver, from its paths: an agent's cost is its arrival time (the time step of its last waypoint), the sum of costs adds them up and the makespan is the largest. They are only comparable between solvers that solve the same problem: the `_ta` solvers also choose which agent goes to which goal, so their costs are lower; `ecbs` and `mapf_prioritized_sipp` are not optimal; `mcts_nonoverlap` has no time, so its cost is the total path length. The plot compares the solvers on the instances they all solved, since averaging each over only the ones it solved would favor a solver that gives up on the hard ones; a solver that solves few instances (such as `mcts_nonoverlap`) leaves few or none to compare on.

`reloc` solves a slightly stricter problem than the other solvers (an agent may not follow another into the cell it leaves; see [its wrapper](#calling-solvers-from-python-tools_wrapperspy)), so its times are not a like-for-like comparison.

`mcts_nonoverlap` is not in the default set, because it solves a different problem (vertex-disjoint paths, see above): an instance in which two agents share a terminal, or in which the paths are forced to cross, counts as unsolved for it, so its success fraction falls quickly with the number of agents. Add it with e.g. `--solvers cbs ecbs mcts_nonoverlap`. It runs with its default options (`--iterations 100 --attempts 10 --seed 0`).

Running with `--jobs` above 1 makes the runs share the machine, so the timings include some contention, and using more jobs than physical cores distorts them. The `_ta` solvers' and `reloc`'s converted files are all created before timing starts, so the conversion is not part of the measured times.

### MAPFAST dataset

The three JSON files are the dataset the MAPFAST algorithm selector trains on. As in MAPFAST's own `solved_*.json`, each is keyed by problem name (`<scen>_agents<n>.yaml`, the name of its converted YAML file): `yaml_details` holds the time of every solver (-1 if it did not solve the instance) and the fastest one as `SOLVER`, `agent_details` the `starts` and `goals`, and `map_details` the number of agents (`no_agents`), the map dimensions (`mp_dim`) and the number of obstacles (`no_obs`). Instances that no solver solved are left out, because MAPFAST needs a fastest solver for each.

The solvers you run are MAPFAST's portfolio: list them as the `mapping` in its `config.json`, e.g. `{"cbs": 0, "ecbs": 1}` for `--solvers cbs ecbs`. The `_ta` solvers solve the relaxed problem in which any agent may take any goal, so leave them out of `--solvers` if they should not compete. MAPFAST also needs an input image (or `.npz`) per instance, named like the instance file, which `benchmark.py` does not create.

### On a Slurm cluster: `benchmark.sh`

```sh
./benchmark.sh [scenario] [extra benchmark.py arguments]
./benchmark.sh empty-8-8 --agents 10 20 30 --timeout 5
```

The script submits itself with `sbatch` (1 node, 12 CPUs, 16 GB, 1 hour; edit the `#SBATCH` lines to change this) and runs `benchmark.py` with `--jobs` set to the allocated CPUs and `--name <scenario>_<job id>`, so jobs never overwrite each other's results:

- `logs/mapf_benchmark_<id>.out` / `.err`: progress, one line per solver and agent count
- `output/<scenario>_<id>_*`: the plot, the CSV and the MAPFAST files described above

If your cluster needs it, uncomment the `module load python` line in `benchmark.sh`.

## Adding a solver library

1. `git submodule add <url> libs/<name>`, preferably your own fork of it, and keep changes to the library itself small.
2. Add wrapper functions to `tools/` that build the library's command line and return results in the same shape as the existing wrappers (a dict with `statistics` and `schedule`, or `None` if unsolved). A solver that reads MovingAI files gets `(scen_file, n_agents)` from `find_instances()`. If the library needs another format, convert into `cache/<library>/` in `find_instances()`, as is done for libMultiRobotPlanning.
3. Register the new solvers in `GRID_SOLVERS` in `tools/solve_and_visualize.py`, which `benchmark.py` uses.
4. Credit the library and state its license in the Layout table above. Code under a copyleft license such as the GPL is fine to call as a separate program, but should not be copied into an MIT-licensed library.
