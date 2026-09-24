#!/bin/bash
#SBATCH -J mapf_benchmark
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --cpus-per-task=12
#SBATCH --mem=16G
#SBATCH -t 3:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err
#
# Benchmark the MAPF solvers on one scenario of benchmark/.
#
#   ./benchmark.sh [scenario] [extra benchmark.py arguments...]
#   ./benchmark.sh benchmark/8x8_obst12 --max-instances 20 --timeout 5
#
# Submits itself to Slurm (creating logs/ first, which Slurm does not do for
# the -o/-e paths above). The progress log goes to logs/ and the results to
# output/, all named after the job id:
#   logs/mapf_benchmark_<id>.out/.err            progress: one line per solver and batch
#   output/<scenario>_<id>_results.csv           raw time of every run, appended per batch
#   output/<scenario>_<id>_times.png             time / success plot, rewritten per batch
#   output/<scenario>_<id>_{yaml,agent,map}_details.json   MAPFAST dataset, rewritten per batch
# The results are written as the run goes, so a job cut off by the time limit
# still leaves its partial results.

if [ -z "$SLURM_JOB_ID" ]; then
    mkdir -p "$(dirname "$(readlink -f "$0")")/logs"
    cd "$(dirname "$(readlink -f "$0")")" && exec sbatch "$(readlink -f "$0")" "$@"
fi

cd "$SLURM_SUBMIT_DIR" || exit 1
mkdir -p logs

SCENARIO=${1:-benchmark/8x8_obst12}
shift
NAME=$(basename "$SCENARIO")_${SLURM_JOB_ID}

export PYTHONUNBUFFERED=1   # write progress lines to the log as they happen
export MPLBACKEND=Agg       # no display on compute nodes

python benchmark.py "$SCENARIO" \
    --jobs 12 \
    --timeout 30 \
    --no-show \
    --name "$NAME" \
    "$@"
