# MAPF Benchmarking

Runs Multi-Agent Path Finding (MAPF) solvers from several libraries on the same problems and records how long each takes, the cost of its solution, and whether it finishes within a time limit. Each library stays a git submodule, close to its upstream; small Python wrappers translate the common problem format to each library's own. The results also form the training data of the [MAPFAST](https://github.com/USC-ACTLab/MAPFAST) algorithm selector.

## Solvers

| Solvers | Library | License |
| --- | --- | --- |
| `cbs`, `ecbs`, `cbs_ta`, `ecbs_ta`, `mapf_prioritized_sipp`, `cbs_roadmap` | [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) (W. Hönig), [fork](https://github.com/mcapoor/libMultiRobotPlanning) | MIT |
| `cbsh2_rtc`, `cbsh2_rtc_ch`, `cbsh2_rtc_bp`, `cbsh2_rtc_chbp` | [CBSH2-RTC-CHBP](https://github.com/bshen95/CBSH2-RTC-CHBP) (B. Shen et al.), [fork](https://github.com/mcapoor/CBSH2-RTC-CHBP) | GPL-3.0, run as a separate program |
| `bcp2` | [BCP2-MAPF](https://github.com/ed-lam/bcp2-mapf) (E. Lam; needs [Gurobi](https://www.gurobi.com/)), [fork](https://github.com/mcapoor/bcp2-mapf) | PolyForm Noncommercial |
| `reloc` | [reLOC](https://github.com/surynek/reLOC) (P. Surynek), [fork](https://github.com/mcapoor/reLOC) cut down to its SAT-based solver | © Pavel Surynek 2011–2018; no license stated |
| `mcts_nonoverlap` | `libs/mcts_nonoverlap/` | its `timer.hpp` from libMultiRobotPlanning (MIT) |

`tools/visualize.py` is adapted from libMultiRobotPlanning (MIT).

The problems are the [MovingAI MAPF benchmarks](https://movingai.com/benchmarks/mapf/index.html), 33 maps with 25 random and 25 even scenario files each, in `benchmarks/`:

> Roni Stern, Nathan R. Sturtevant, Ariel Felner, Sven Koenig, Hang Ma, Thayne T. Walker, Jiaoyang Li, Dor Atzmon, Liron Cohen, T. K. Satish Kumar, Eli Boyarski and Roman Barták. "Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks." *Symposium on Combinatorial Search (SoCS)*, 2019.

## Layout

| Path | Contents |
| --- | --- |
| `libs/` | The solver libraries (submodules) |
| `benchmarks/` | The MovingAI maps and scenario files, one folder per map |
| `tools/` | One wrapper module per library (`*_wrappers.py`), solving and drawing one problem (`solve_and_visualize.py`, `compare_solutions.py`), merging and extending benchmarks (`aggregate.py`, `extend_benchmark.py`), moving results of the old layout (`temp_organize_results.py`) |
| `benchmark.py`, `benchmark.sh` | Time every solver on one map; run it on every map as Slurm jobs |
| `experiments/` | One folder per benchmark experiment: each map's results, the merged results and the Slurm logs (not tracked) |
| `cache/` | Problems converted to a library's own format (not tracked) |

## Usage

1. **Build** (CMake, Boost, yaml-cpp, a C++23 compiler and Gurobi; on Oscar `module load python boost yaml-cpp gurobi`):

   ```sh
   git clone --recurse-submodules git@github.com:mcapoor/MAPF-Benchmarking.git && cd MAPF-Benchmarking
   python -m venv .venv && source .venv/bin/activate && pip install pyyaml numpy matplotlib
   JOBS=4 ./build.sh
   ```

2. **Try a solver** on the first 10 agents of a scenario, or compare several:

   ```sh
   python tools/solve_and_visualize.py benchmarks/empty-8-8/empty-8-8-random-1.scen bcp2 --agents 10
   python tools/compare_solutions.py benchmarks/room-32-32-4/room-32-32-4-random-1.scen --agents 15 --solvers cbs bcp2 reloc
   ```

3. **Benchmark** one map locally, or every map on a Slurm cluster, into `experiments/<EXPERIMENT>/`:

   ```sh
   python benchmark.py empty-8-8 --timeout 5 --jobs 4
   EXPERIMENT=movingai_60s ./benchmark.sh --timeout 60                     # random scenarios, one job per map
   EXPERIMENT=movingai_60s ./benchmark.sh --scen-type even --timeout 60    # add the even scenarios
   ```

   `MAPS="den312d orz900d"` limits it to some maps (e.g. to rerun failed ones). Results of the old `output/` and `logs/` layout move into `experiments/` with `python tools/temp_organize_results.py` (it prints the plan; `--apply` moves the files).
