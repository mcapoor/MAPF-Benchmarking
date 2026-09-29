"""Merge the output/ files of several benchmark.py runs, e.g. the one-job-per-map
runs of ./benchmark.sh, into one results CSV and one MAPFAST dataset:

    output/<name>_results.csv
    output/<name>_yaml_details.json, <name>_agent_details.json, <name>_map_details.json

    python tools/aggregate.py 1234 1235 1236 --name all_maps
    python tools/aggregate.py empty-8-8_1234 room-32-32-4_1235

Each run is a Slurm job id, standing for every output/*_<id>_results.csv, or
the prefix (benchmark.py --name) of one run. ./benchmark.sh without a map
runs this by itself once all its jobs have ended.
"""
import argparse
import csv
import glob
import json
import os
import sys

# allow running as a script (python tools/aggregate.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from benchmark import MAPFAST_DETAILS, OUTPUT_DIR, write_mapfast

RESULTS_SUFFIX = "_results.csv"


def run_prefixes(runs):
    """The output/ prefix of each run in `runs` (a prefix, or a job id standing
    for every <prefix>_<id> in output/). A run without a results file, e.g. a
    job that ended before its first batch, is left out with a warning."""
    prefixes = []
    for run in runs:
        if os.path.exists(os.path.join(OUTPUT_DIR, run + RESULTS_SUFFIX)):
            prefixes.append(run)
            continue
        matches = sorted(glob.glob(os.path.join(OUTPUT_DIR, f"*_{glob.escape(run)}{RESULTS_SUFFIX}")))
        if not matches:
            print(f"warning: no results of {run} in {OUTPUT_DIR}", file=sys.stderr)
        prefixes += [os.path.basename(match)[:-len(RESULTS_SUFFIX)] for match in matches]
    return prefixes


def merge_results(prefixes, path):
    """Concatenate the results CSVs of the runs `prefixes` into `path`, with
    the header once. Returns the number of rows written."""
    header, rows = None, []
    for prefix in prefixes:
        with open(os.path.join(OUTPUT_DIR, prefix + RESULTS_SUFFIX), newline="") as results_file:
            reader = csv.reader(results_file)
            run_header = next(reader, None)
            if run_header is None:
                continue
            if header is not None and run_header != header:
                raise ValueError(f"{prefix}{RESULTS_SUFFIX} has columns {run_header}, not {header}")
            header = run_header
            rows += list(reader)
    with open(path + ".tmp", "w", newline="") as results_file:
        writer = csv.writer(results_file)
        if header:
            writer.writerow(header)
        writer.writerows(rows)
    os.replace(path + ".tmp", path)
    return len(rows)


def merge_mapfast(prefixes):
    """The union of the MAPFAST details of the runs `prefixes`, as a {name:
    {instance name: record}} dict with an entry for each of MAPFAST_DETAILS.
    Instance names include the map, so the runs of different maps do not
    collide."""
    details = {name: {} for name in MAPFAST_DETAILS}
    for prefix in prefixes:
        for name in MAPFAST_DETAILS:
            path = os.path.join(OUTPUT_DIR, f"{prefix}_{name}.json")
            if os.path.exists(path):
                with open(path) as details_file:
                    details[name].update(json.load(details_file))
    return details


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('runs', nargs='+', help='Slurm job ids or output/ prefixes of the runs to merge')
    parser.add_argument('--name', default='all', help='prefix of the merged files in output/ (default: all)')
    args = parser.parse_args()

    prefixes = run_prefixes(args.runs)
    if not prefixes:
        sys.exit("no results to merge")
    try:
        rows = merge_results(prefixes, os.path.join(OUTPUT_DIR, args.name + RESULTS_SUFFIX))
    except ValueError as error:
        sys.exit(str(error))
    details = merge_mapfast(prefixes)
    write_mapfast(OUTPUT_DIR, args.name, details)
    print(f"merged {len(prefixes)} runs into {OUTPUT_DIR}/{args.name}_*: {rows} result rows, "
          f"{len(details['yaml_details'])} MAPFAST problems")
