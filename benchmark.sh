#!/bin/bash
#SBATCH -J mapf_benchmark
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --cpus-per-task=12
#SBATCH --mem=32G
#SBATCH -t 6:00:00
#
# Benchmark the MAPF solvers on every map of benchmarks/, or on one map (or
# any scenario directory, see benchmark.py), as one experiment.
#
#   ./benchmark.sh [scenario] [extra benchmark.py arguments...]
#   EXPERIMENT=movingai_60s ./benchmark.sh               every map, one job each
#   EXPERIMENT=movingai_60s ./benchmark.sh --scen-type even   add the even scenarios to it
#   ./benchmark.sh empty-8-8 --agents 10 20 30 --timeout 5
#
# Everything goes to experiments/<EXPERIMENT>/ (default <date>_<time>_<map or
# all-maps>); give it a name that says what was measured:
#   maps/<map>/                  one map's run (<map>-even for even scenarios):
#                                results.csv (time, cost and makespan of every run), times.png and the MAPFAST
#                                files {yaml,agent,map}_details.json, all rewritten after every agent count
#   logs/<map>.out/.err          that map's progress, one line per solver and agent count (appended if it is rerun)
#   results.csv, {yaml,agent,map}_details.json   every map merged by tools/aggregate.py (logs/merge.out)
#   submitted.txt                what was submitted, when, with which job ids
#
# Without a scenario (no arguments, or the first one an option), it submits
# one job per map directory of benchmarks/, each with the extra arguments and
# named mapf_benchmark_<map>, plus a job mapf_benchmark_merge that starts once
# they have all ended (finished, failed or timed out) and merges every map of
# the experiment (also maps added by earlier submissions). Set MAPS to a
# space-separated list of maps to run only those, e.g. MAPS="empty-8-8
# den312d" ./benchmark.sh, which also reruns failed maps; AGGREGATE=0 leaves
# out the merge job, e.g. to merge once several submissions have ended.
#
# Runs every solver except the task assignment solvers (cbs_ta, ecbs_ta) and
# mcts_nonoverlap, which solve different problems (see SOLVERS below); pass
# --solvers to run others instead, as it overrides this list.
#
# The results are written as the run goes, so a job cut off by the time limit
# still leaves its partial results.

# SLURM_JOB_ID alone does not tell the job apart, as it is also set on an
# interact node (inside salloc), where this is meant to be run from: the jobs
# this submits inherit MAPF_BENCHMARK_JOB from it (sbatch exports the
# environment) and nothing else sets it.
if [ -z "$SLURM_JOB_ID" ] || [ -z "$MAPF_BENCHMARK_JOB" ]; then
    SCRIPT=$(readlink -f "$0")
    cd "$(dirname "$SCRIPT")" || exit 1
    export MAPF_BENCHMARK_JOB=1

    # the run folder of a map: <map>, or <map>-even with --scen-type even (as benchmark.py names it)
    run_name() {
        local name=$(basename "$1") prev=""
        shift
        for arg in "$@"; do
            [ "$prev" = --scen-type ] && [ "$arg" = even ] && name+="-even"
            [ "$arg" = --scen-type=even ] && name+="-even"
            prev=$arg
        done
        echo "$name"
    }

    if [ -n "$1" ] && [[ "$1" != -* ]]; then
        scope=$(basename "$1")
    elif [ -n "$MAPS" ]; then
        scope=maps
    else
        scope=all-maps
    fi
    export MAPF_EXPERIMENT="${EXPERIMENT:-$(date +%Y-%m-%d_%H%M)_$scope}"
    dir="experiments/$MAPF_EXPERIMENT"
    mkdir -p "$dir/logs"
    log_options() { echo -o "$dir/logs/$1.out" -e "$dir/logs/$1.err" --open-mode=append; }

    if [ -n "$1" ] && [[ "$1" != -* ]]; then
        id=$(sbatch --parsable $(log_options "$(run_name "$@")") "$SCRIPT" "$@") || exit 1
        echo "$(date '+%F %T') $(run_name "$@"): job ${id%%;*}, arguments: $*" >> "$dir/submitted.txt"
        echo "Submitted batch job ${id%%;*} ($1), results in $dir/"
        exit
    fi
    # no scenario given: one job per map (of $MAPS if set)
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
        name=$(run_name "$map" "$@")
        id=$(sbatch --parsable -J "mapf_benchmark_$map" $(log_options "$name") "$SCRIPT" "$map" "$@") || continue
        id=${id%%;*}   # --parsable prints <id> or <id>;<cluster>
        echo "Submitted batch job $id ($map)"
        echo "$(date '+%F %T') $name: job $id, arguments: $*" >> "$dir/submitted.txt"
        ids+=("$id")
    done
    [ ${#ids[@]} -gt 0 ] || exit 1
    echo "Experiment $dir/"
    [ "${AGGREGATE:-1}" = 0 ] && exit
    # then merge every map of the experiment, whether its job finished or not (afterany)
    id=$(sbatch --parsable -J mapf_benchmark_merge --dependency="afterany:$(IFS=:; echo "${ids[*]}")" \
        -N 1 -n 1 --cpus-per-task=1 --mem=16G -t 1:00:00 $(log_options merge) \
        --wrap "MPLBACKEND=Agg python tools/aggregate.py $dir") || exit 1
    echo "Merge job ${id%%;*} writes the merged results to $dir/"
    exit
fi

cd "$SLURM_SUBMIT_DIR" || exit 1

SCENARIO=$1
shift

export PYTHONUNBUFFERED=1   # write progress lines to the log as they happen
export MPLBACKEND=Agg       # no display on compute nodes

module load boost yaml-cpp gurobi
source .venv/bin/activate

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
    --experiment "$MAPF_EXPERIMENT" \
    "$@"
