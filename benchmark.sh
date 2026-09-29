#!/bin/bash
#SBATCH -J mapf_benchmark
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --cpus-per-task=12
#SBATCH --mem=32G
#SBATCH -t 6:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err
#
# Benchmark the MAPF solvers on every map of benchmarks/, or on one map (or
# any scenario directory, see benchmark.py).
#
#   ./benchmark.sh [scenario] [extra benchmark.py arguments...]
#   ./benchmark.sh                                   every map, one job each
#   ./benchmark.sh --scen-type even                  every map, even scenarios
#   ./benchmark.sh empty-8-8 --agents 10 20 30 --timeout 5
#   ./benchmark.sh room-32-32-4 --scen-type even
#
# Without a scenario (no arguments, or the first one an option), it submits
# one job per map directory of benchmarks/, each with the extra arguments and
# named mapf_benchmark_<map>, plus a job mapf_benchmark_aggregate that starts
# once they have all ended (finished, failed or timed out) and merges their
# results with tools/aggregate.py into
#   output/all_<id>_results.csv                  the results of every map
#   output/all_<id>_{yaml,agent,map}_details.json   the MAPFAST dataset of every map
# where <id> is the aggregate job's id. Set MAPS to a space-separated list of
# maps to run only those, e.g. MAPS="empty-8-8 den312d" ./benchmark.sh.
#
# Run ./test_cluster.sh first: it checks the whole stack on a few small
# problems before this spends the compute quota.
#
# Runs every solver except the task assignment solvers (cbs_ta, ecbs_ta) and
# mcts_nonoverlap, which solve different problems (see SOLVERS below); pass
# --solvers to run others instead, as it overrides this list.
#
# Submits itself to Slurm (creating logs/ first, which Slurm does not do for
# the -o/-e paths above). The progress log goes to logs/ and the results to
# output/, all named after the job id:
#   logs/<job name>_<id>.out/.err                progress: one line per solver and batch
#   output/<scenario>_<id>_results.csv           time, cost and makespan of every run, appended per batch
#   output/<scenario>_<id>_times.png             time / success / cost / makespan plot, rewritten per batch
#   output/<scenario>_<id>_{yaml,agent,map}_details.json   MAPFAST dataset, rewritten per batch
# The results are written as the run goes, so a job cut off by the time limit
# still leaves its partial results.

if [ -z "$SLURM_JOB_ID" ]; then
    SCRIPT=$(readlink -f "$0")
    cd "$(dirname "$SCRIPT")" || exit 1
    mkdir -p logs
    if [ -n "$1" ] && [[ "$1" != -* ]]; then
        exec sbatch "$SCRIPT" "$@"
    fi
    # no scenario given: one job per map (of $MAPS if set, e.g. by test_cluster.sh)
    if [ -n "$MAPS" ]; then
        map_dirs=()
        for map in $MAPS; do map_dirs+=("benchmarks/$map/"); done
    else
        map_dirs=(benchmarks/*/)
    fi
    ids=()
    for map_dir in "${map_dirs[@]}"; do
        if ! compgen -G "$map_dir*.map" > /dev/null; then
            [ -z "$MAPS" ] || echo "no map in $map_dir, skipped" >&2
            continue
        fi
        map=$(basename "$map_dir")
        id=$(sbatch --parsable -J "mapf_benchmark_$map" "$SCRIPT" "$map" "$@") || continue
        id=${id%%;*}   # --parsable prints <id> or <id>;<cluster>
        echo "Submitted batch job $id ($map)"
        ids+=("$id")
    done
    [ ${#ids[@]} -gt 0 ] || exit 1
    # then merge their results, whether they finished or not (afterany)
    sbatch -J mapf_benchmark_aggregate --dependency="afterany:$(IFS=:; echo "${ids[*]}")" \
        -N 1 -n 1 --cpus-per-task=1 --mem=16G -t 1:00:00 \
        -o "logs/%x_%j.out" -e "logs/%x_%j.err" \
        --wrap "MPLBACKEND=Agg python tools/aggregate.py ${ids[*]} --name all_\$SLURM_JOB_ID"
    exit
fi

cd "$SLURM_SUBMIT_DIR" || exit 1
mkdir -p logs

SCENARIO=$1
shift
NAME=$(basename "$SCENARIO")_${SLURM_JOB_ID}

export PYTHONUNBUFFERED=1   # write progress lines to the log as they happen
export MPLBACKEND=Agg       # no display on compute nodes

module load boost yaml-cpp gurobi

# the solvers of the standard MAPF problem (bcp2 needs a Gurobi license on the node)
SOLVERS=(
    cbs ecbs mapf_prioritized_sipp
    cbsh2_rtc cbsh2_rtc_ch cbsh2_rtc_bp cbsh2_rtc_chbp
    reloc bcp2
)

python benchmark.py "$SCENARIO" \
    --solvers "${SOLVERS[@]}" \
    --agents 10 20 50 100 200 \
    --jobs 12 \
    --timeout 60 \
    --no-show \
    --name "$NAME" \
    "$@"
