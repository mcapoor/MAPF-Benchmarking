#!/bin/bash
# Run tools/experiment.py on Slurm: a job array of SHARDS jobs (default 20; SHARD_JOBS runs at a time each, default
# 12, in SHARD_MEM, default 96G, for SHARD_TIME, default 24:00:00), then a job that merges them into
# experiments/<EXPERIMENT>/ once they have ended. Every argument goes to experiment.py, e.g.
#
#   EXPERIMENT=movingai_10-200-agents_300s ./benchmark.sh --timeout 300 --agents 10 20 50 100 200
#   EXPERIMENT=... ./benchmark.sh --timeout 300 --agents ... --retry      again: redo the runs killed or crashed
#
# Prints the plan, then "Merge: job <id>". Submitting it again with the same arguments resumes it. Logs in
# experiments/<EXPERIMENT>/logs/.
set -e
cd "$(dirname "$(readlink -f "$0")")"
[ -n "$EXPERIMENT" ] || { echo "set EXPERIMENT" >&2; exit 1; }
SHARDS="${SHARDS:-20}"
SHARD_JOBS="${SHARD_JOBS:-12}"
logs="experiments/$EXPERIMENT/logs"
mkdir -p "$logs"
setup="module load boost yaml-cpp gurobi && source .venv/bin/activate && export MPLBACKEND=Agg"
args="$EXPERIMENT $*"

source .venv/bin/activate
python tools/experiment.py plan $args
array=$(sbatch --parsable -J benchmark --array="0-$((SHARDS - 1))" --nodes=1 --cpus-per-task="$SHARD_JOBS" \
	--mem="${SHARD_MEM:-96G}" --time="${SHARD_TIME:-24:00:00}" -o "$logs/shard%a.out" -e "$logs/shard%a.err" \
	--open-mode=append \
	--wrap "$setup && python -u tools/experiment.py run $args --shard \$SLURM_ARRAY_TASK_ID --shards $SHARDS --jobs $SHARD_JOBS")
# afterany: a shard whose runs were killed for memory ends as OUT_OF_MEMORY although it finished; the merge fails if
# a group is unfinished
merge=$(sbatch --parsable -J benchmark_merge --dependency="afterany:$array" --nodes=1 --cpus-per-task=1 --mem=16G \
	--time=2:00:00 -o "$logs/merge.out" -e "$logs/merge.err" --open-mode=append \
	--wrap "$setup && python -u tools/experiment.py merge $args")
echo "Shards: job array $array, logs $logs/shard<i>.out"
echo "Merge: job $merge, into experiments/$EXPERIMENT/"
