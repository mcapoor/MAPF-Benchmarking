# MAPF Benchmarking

A harness for running and comparing multi-agent path finding (MAPF) solvers from different libraries on the same benchmark scenarios. Each solver library is a git submodule in `libs/` and stays as close to its own upstream as possible. The scripts here translate between the libraries and one common problem/result format, run the solvers, and collect the timings.

## Layout

| Path | Contents |
| --- | --- |
| `libs/libMultiRobotPlanning/` | Submodule: [a fork](https://github.com/mcapoor/libMultiRobotPlanning) of Wolfgang Hönig's [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) (CBS, ECBS, CBS-TA, ECBS-TA, prioritized SIPP, MCTS). See its README for the algorithms and for what the fork changes. |
| `tools/solver_wrappers.py` | One Python function per solver binary. This is where library-specific paths, arguments and output formats are handled. |
| `tools/solve_and_visualize.py` | Solve one problem and animate the result |
| `benchmark.py` | Benchmark runner: time every solver on a scenario directory |
| `benchmark.sh` | Slurm launcher for `benchmark.py` |
| `benchmark/` | Benchmark scenarios (see [Scenarios](#scenarios)) |
| `output/` | Results of `benchmark.py` runs (not tracked by git) |

## Setup

Clone with the submodules, then build each library:

```sh
git clone --recurse-submodules <this repo>
# or, in an existing clone:
git submodule update --init --recursive

cmake -S libs/libMultiRobotPlanning -B libs/libMultiRobotPlanning/build
cmake --build libs/libMultiRobotPlanning/build -j
```

libMultiRobotPlanning needs CMake, Boost and yaml-cpp. On the Oscar cluster, run `module load boost` and `module load yaml-cpp` first. The solvers must be compiled on the machine that runs them, so build them on the cluster too.

Python 3.9+ with the libraries' requirements:

```sh
pip install -r libs/libMultiRobotPlanning/requirements.txt
```

Saving an animation to a video (`--video`) also needs `ffmpeg` on your `PATH`.

Run everything from this directory, e.g. `python tools/solve_and_visualize.py ...`.

## Problem files

Problems are libMultiRobotPlanning's YAML format: a `map` and a list of `agents`:

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

The task-assignment solvers (`cbs_ta`, `ecbs_ta`) read a `potentialGoals` list per agent instead. If a file has none, the wrappers convert it automatically: a copy is written to `<input dir>_potentialGoals/` (e.g. `benchmark/8x8_obst12_potentialGoals/`) in which every agent may take any agent's goal. The original files are not modified, and an existing converted file is reused.

Roadmap problems (for `cbs_roadmap`) have agents that start and end at named vertices, and a `roadmap` section.

### Scenarios

A scenario directory contains files named `*_agents<n>_ex<k>.yaml`: for each agent count n there are several random instances k. The scenarios come from the [Moving AI lab](https://movingai.com/benchmarks/mapf/index.html):

* `benchmark/8x8_obst12`: 100 instances for each of n = 1, 2, ..., 20
* `benchmark/32x32_obst204`: 100 instances for each of n = 10, 20, ..., 100
* `benchmark/9x9_obst0`: a single obstacle-free 9x9 instance with 8 agents, for quick checks

More Moving AI scenarios can be converted with `libs/libMultiRobotPlanning/tools/standard_benchmark_converter.py`.

## Solving one problem: `tools/solve_and_visualize.py`

```sh
python tools/solve_and_visualize.py <problem.yaml> <solver> [options]

python tools/solve_and_visualize.py benchmark/8x8_obst12/map_8by8_obst12_agents10_ex0.yaml cbs
python tools/solve_and_visualize.py benchmark/8x8_obst12/map_8by8_obst12_agents10_ex0.yaml ecbs -w 1.5 --video out.mp4 --speed 2
```

Solvers: `cbs`, `cbs_ta`, `ecbs`, `ecbs_ta`, `mapf_prioritized_sipp`, `mcts_nonoverlap` (grid) and `cbs_roadmap` (roadmap).

| Option | Meaning |
| --- | --- |
| `-w W` | Suboptimality factor (`ecbs`, `ecbs_ta` only; default 1.0) |
| `--radius R` | Robot radius (`cbs_roadmap` only; default 0.3) |
| `--annotate` | Annotate the roadmap with collisions before solving (`cbs_roadmap` only; needs `cvxpy`) |
| `--video FILE` | Save the animation instead of showing it |
| `--speed N` | Video speed-up factor (default 1) |
| `--no-visualize` | Solve and report the time only |
| `--quiet` | Hide the solver's own output, keeping the success and timing message |

The exit status is 1 if the solver found no solution. The same functionality is available from Python as `solve(...)` (returns `(result, seconds)`) and `solve_and_visualize(...)`.

## Calling solvers from Python: `tools/solver_wrappers.py`

One function per libMultiRobotPlanning binary: `cbs`, `cbs_ta`, `ecbs`, `ecbs_ta`, `cbs_roadmap`, `mapf_prioritized_sipp`, `mcts_nonoverlap`, `sipp`, `a_star`, `a_star_epsilon`, `assignment` and `next_best_assignment`. Each runs the binary and returns its parsed YAML output, or `None` if there was no solution. Common keyword arguments are `timeout` (seconds; raises `subprocess.TimeoutExpired`), `output` (keep the raw output file), `quiet`, and for the MAPF solvers `create_video` / `video`.

`mcts_nonoverlap` looks for vertex-disjoint paths (each agent's start and goal are the two ends of one path), so most of the benchmark instances have no solution for it; the result then has no `schedule` entry instead of being `None`. It also takes `iterations`, `exploration`, `attempts` and `seed`, which are only passed on if given.

```python
from tools import solver_wrappers

result = solver_wrappers.ecbs("benchmark/8x8_obst12/map_8by8_obst12_agents10_ex0.yaml", w=1.5, timeout=10)
print(result["statistics"], result["schedule"])
```

Library scripts are imported by file path with `solver_wrappers.load_module(path, name)` rather than as packages, so that each library's `tools/` folder does not clash with this one.

## Benchmarking: `benchmark.py`

`benchmark.py` measures how the grid solvers (`cbs`, `cbs_ta`, `ecbs`, `ecbs_ta`, `mapf_prioritized_sipp`, `mcts_nonoverlap`) scale with the number of agents on a scenario directory.

```sh
python benchmark.py benchmark/8x8_obst12 --max-instances 20 --timeout 5 --jobs 4
```

| Option | Meaning |
| --- | --- |
| `--solvers S ...` | Solvers to compare (default: all but `mcts_nonoverlap`) |
| `--timeout T` | Seconds allowed per run (default 10); a run that exceeds it counts as unsolved |
| `--max-instances M` | Use only the first M instances of every agent count |
| `--jobs J` | Runs to execute in parallel (default 1) |
| `--name NAME` | Prefix of the files written to `output/` (default: the scenario directory name, e.g. `8x8_obst12`) |
| `--output FILE` | Plot file (default `output/<name>_times.png`) |
| `--results FILE` | CSV of the raw times (default `output/<name>_results.csv`) |
| `--no-show` | Do not open the plot window |

Every run writes these files to `output/`, updating them after every agent count so that an interrupted run still leaves partial results:

| File | Contents |
| --- | --- |
| `<name>_times.png` | Plot: mean time over solved instances per solver on a log scale (top), and the fraction of instances solved within the timeout (bottom) |
| `<name>_results.csv` | Raw times, with the columns `solver,n_agents,map,seconds` (`seconds` empty for an unsolved run) |
| `<name>_yaml_details.json`, `<name>_agent_details.json`, `<name>_map_details.json` | The MAPFAST dataset (see below) |

A new run with the same name replaces the previous run's files; pass `--name` to keep them apart.

`mcts_nonoverlap` is not in the default set, because it solves a different problem (vertex-disjoint paths, see above): an instance in which two agents share a terminal, or in which the paths are forced to cross, counts as unsolved for it, so its success fraction falls quickly with the number of agents. Add it with e.g. `--solvers cbs ecbs mcts_nonoverlap`. It runs with its default options (`--iterations 100 --attempts 10 --seed 0`).

Running with `--jobs` above 1 makes the runs share the machine, so the timings include some contention, and using more jobs than physical cores distorts them. The `_ta` solvers' converted files are all created before timing starts, so the conversion is not part of the measured times.

### MAPFAST dataset

The three JSON files are the dataset the MAPFAST algorithm selector trains on. As in MAPFAST's own `solved_*.json`, each is keyed by instance file name: `yaml_details` holds the time of every solver (-1 if it did not solve the instance) and the fastest one as `SOLVER`, `agent_details` the `starts` and `goals`, and `map_details` the number of agents (`no_agents`), the map dimensions (`mp_dim`) and the number of obstacles (`no_obs`). Instances that no solver solved are left out, because MAPFAST needs a fastest solver for each.

The solvers you run are MAPFAST's portfolio: list them as the `mapping` in its `config.json`, e.g. `{"cbs": 0, "ecbs": 1}` for `--solvers cbs ecbs`. The `_ta` solvers solve the relaxed problem in which any agent may take any goal, so leave them out of `--solvers` if they should not compete. MAPFAST also needs an input image (or `.npz`) per instance, named like the instance file, which `benchmark.py` does not create.

### On a Slurm cluster: `benchmark.sh`

```sh
./benchmark.sh [scenario] [extra benchmark.py arguments]
./benchmark.sh benchmark/8x8_obst12 --max-instances 20 --timeout 5
```

The script submits itself with `sbatch` (1 node, 12 CPUs, 16 GB, 1 hour; edit the `#SBATCH` lines to change this) and runs `benchmark.py` with `--jobs` set to the allocated CPUs and `--name <scenario>_<job id>`, so jobs never overwrite each other's results:

- `logs/mapf_benchmark_<id>.out` / `.err`: progress, one line per solver and agent count
- `output/<scenario>_<id>_*`: the plot, the CSV and the MAPFAST files described above

If your cluster needs it, uncomment the `module load python` line in `benchmark.sh`.

## Adding a solver library

1. `git submodule add <url> libs/<name>`, preferably your own fork of it, and keep changes to the library itself small.
2. Add wrapper functions to `tools/` that build the library's command line, convert the problem from the YAML format above if the library reads something else, and return results in the same shape as the existing wrappers (a dict with `statistics` and `schedule`, or `None` if unsolved).
3. Register the new solvers in `GRID_SOLVERS` in `tools/solve_and_visualize.py`, which `benchmark.py` uses.
4. Credit the library and state its license in the Layout table above. Code under a copyleft license such as the GPL is fine to call as a separate program, but should not be copied into an MIT-licensed library.
