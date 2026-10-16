"""Merge the per-map results of an experiment (experiments/<experiment>/maps/<map>/, one benchmark.py run each) into
experiments/<experiment>/:

    results.csv                                          every run of every map
    yaml_details.json, agent_details.json, map_details.json   the MAPFAST dataset of every map

    python tools/aggregate.py movingai_10-200-agents_60s
    python tools/aggregate.py experiments/a experiments/b --out experiments/a_and_b

./benchmark.sh without a map runs this by itself once all its jobs have ended. Several experiments can be merged into
a new one with --out.
"""
import argparse
import csv
import glob
import json
import os
import sys

# allow running as a script (python tools/aggregate.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from benchmark import MAPFAST_DETAILS, RESULTS_FILE, experiment_dir, write_mapfast


def run_dirs(experiments):
    """The run folders (with a results.csv) of the maps of `experiments`. A map without results, e.g. one whose job
    ended before its first batch, is left out with a warning, and so are the superseded runs <map>_earlier-<k> that
    tools/temp_organize_results.py keeps."""
    dirs = []
    for experiment in experiments:
        maps = sorted(glob.glob(os.path.join(experiment, "maps", "*", "")))
        if not maps:
            print(f"warning: no maps/ in {experiment}", file=sys.stderr)
        for run in maps:
            if "_earlier-" in os.path.basename(os.path.normpath(run)):
                continue
            if os.path.exists(os.path.join(run, RESULTS_FILE)):
                dirs.append(run)
            else:
                print(f"warning: no {RESULTS_FILE} in {run}", file=sys.stderr)
    return dirs


def merge_results(dirs, path):
    """Concatenate the results CSVs of the run folders `dirs` into `path`, with the header once. Returns the number of
    rows written."""
    header, rows = None, []
    for directory in dirs:
        with open(os.path.join(directory, RESULTS_FILE), newline="") as results_file:
            reader = csv.reader(results_file)
            run_header = next(reader, None)
            if run_header is None:
                continue
            if header is not None and run_header != header:
                raise ValueError(f"{directory}{RESULTS_FILE} has columns {run_header}, not {header}")
            header = run_header
            rows += list(reader)
    with open(path + ".tmp", "w", newline="") as results_file:
        writer = csv.writer(results_file)
        if header:
            writer.writerow(header)
        writer.writerows(rows)
    os.replace(path + ".tmp", path)
    return len(rows)


def merge_mapfast(dirs):
    """The union of the MAPFAST details of the run folders `dirs`, as a {name: {instance name: record}} dict with an
    entry for each of MAPFAST_DETAILS. Instance names include the map, so the runs of different maps do not collide."""
    details = {name: {} for name in MAPFAST_DETAILS}
    for directory in dirs:
        for name in MAPFAST_DETAILS:
            path = os.path.join(directory, f"{name}.json")
            if os.path.exists(path):
                with open(path) as details_file:
                    details[name].update(json.load(details_file))
    return details


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('experiments', nargs='+', help='experiments to merge: names in experiments/ or folders')
    parser.add_argument('--out', default=None, help='experiment to write the merged files to (default: the first one)')
    args = parser.parse_args()

    experiments = [experiment_dir(e) for e in args.experiments]
    out = experiment_dir(args.out) if args.out else experiments[0]
    dirs = run_dirs(experiments)
    if not dirs:
        sys.exit("no results to merge")
    os.makedirs(out, exist_ok=True)
    try:
        rows = merge_results(dirs, os.path.join(out, RESULTS_FILE))
    except ValueError as error:
        sys.exit(str(error))
    details = merge_mapfast(dirs)
    write_mapfast(out, details)
    print(f"merged {len(dirs)} maps into {out}/: {rows} result rows, {len(details['yaml_details'])} MAPFAST problems")
