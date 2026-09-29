#!/bin/bash
#
# Check the whole benchmark stack on the cluster with a few small problems
# before running ./benchmark.sh on every map, which uses most of the compute
# quota. Run it on a login node, from the Python environment the full run
# uses (the jobs inherit it):
#
#   ./test_cluster.sh
#
# 1. tools/check_benchmark.py preflight, on the login node, in seconds: every
#    solver is built and finds its libraries, every map's files load, sbatch
#    is there. Nothing is submitted if this fails.
# 2. ./benchmark.sh itself on the maps of $MAPS (below: the smallest map and
#    one large map of each family), with 2 and 4 agents on 2 scenario files
#    each and a 15 minute job limit, so it goes through the same script,
#    solvers, Slurm jobs and aggregate job as the full run.
# 3. A job mapf_test_check that starts once the aggregate job has ended and
#    checks the merged results with tools/check_benchmark.py results: every
#    job finished, every solver solves every problem of empty-8-8 (and the
#    optimal ones agree on its costs), every solver solves something on the
#    compute nodes (e.g. bcp2 finds its Gurobi license), the MAPFAST files
#    are complete. Unsolved problems on the large maps are only warnings.
#
# The verdict is the last line of logs/mapf_test_check_<id>.out, PASS or
# FAIL, and that job's state in sacct (COMPLETED or FAILED).

cd "$(dirname "$(readlink -f "$0")")" || exit 1
mkdir -p logs
export MPLBACKEND=Agg

MAPS=${MAPS:-"empty-8-8 random-64-64-20 maze-128-128-10 room-64-64-16 warehouse-20-40-10-2-2 Paris_1_256 orz900d"}
AGENTS="2 4"
INSTANCES=2
TIMEOUT=20

# the modules benchmark.sh loads in its jobs, so that preflight finds the
# solvers' libraries as the jobs will
if type module &> /dev/null; then
    module load boost yaml-cpp gurobi
fi
python tools/check_benchmark.py preflight || exit 1

# SBATCH_TIMELIMIT overrides benchmark.sh's own 6 hour limit
submitted=$(MAPS="$MAPS" SBATCH_TIMELIMIT=0:15:00 ./benchmark.sh \
    --agents $AGENTS --max-instances $INSTANCES --timeout $TIMEOUT)
status=$?
echo "$submitted"
[ $status -eq 0 ] || exit 1
# the last job benchmark.sh submits is the aggregate job
aggregate=$(echo "$submitted" | awk '/^Submitted batch job/ {id = $4} END {print id}')

sbatch -J mapf_test_check --dependency="afterany:$aggregate" \
    -N 1 -n 1 --cpus-per-task=1 --mem=4G -t 0:10:00 \
    -o "logs/%x_%j.out" -e "logs/%x_%j.err" \
    --wrap "MPLBACKEND=Agg python tools/check_benchmark.py results all_$aggregate \
        --maps $MAPS --agents $AGENTS --instances $INSTANCES --strict empty-8-8"
echo "the verdict will be the last line of logs/mapf_test_check_<id>.out"
