"""Move the results of the old layout (output/ and logs/, files named after Slurm job ids) into experiment folders, the
layout benchmark.py, benchmark.sh, tools/aggregate.py and tools/extend_benchmark.py now write:

    experiments/<experiment>/
        results.csv, {yaml,agent,map}_details.json    the merged results
        maps/<map>/                                   one map's run (<map>-even for even scenarios)
        merges/<group>/                               merges of part of the maps, e.g. one scenario type
        shards/shard<i>.csv                           an extended experiment's runs (tools/extend_benchmark.py)
        logs/                                         the Slurm logs
        migrated.txt                                  where each file came from (with the old job ids)

It works out which files belong together from their content, not their names: a merged results file belongs to the
largest merge whose runs include all of its runs (e.g. an aggregate job's output to the merge of all aggregates built
for MAPFAST), and a map's run to the smallest merge that includes it. An extended benchmark (<name>_<T>s, or with shard
files) becomes an experiment of its own. Each experiment is named after what it holds, e.g. movingai_10-200-agents_60s
(the time limit is inferred from the longest solved run); rename the folders afterwards if you like. Logs follow the
job they belong to; whatever cannot be placed goes to experiments/unsorted/.

    python tools/temp_organize_results.py            print the plan, change nothing
    python tools/temp_organize_results.py --apply    move the files, then remove output/ and logs/ if they are empty
    python tools/temp_organize_results.py --apply --rename full_7095349=movingai_10-200-agents_60s
                                                     ... naming the experiment of an old merge (its file prefix)

Nothing is deleted: every file is moved (logs of the same map are appended to one file). Run it when no benchmark job
is running, as a running job would keep writing to the old place.
"""
import argparse
import collections
import csv
import os
import re
import shutil
import subprocess
import sys

# allow running as a script (python tools/temp_organize_results.py), not only as a module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from benchmark import BENCHMARK_DIR, EXPERIMENTS_DIR, MAPFAST_DETAILS, PLOT_FILE, RESULTS_FILE, ROOT_DIR

OLD_OUTPUT = os.path.join(ROOT_DIR, "output")
OLD_LOGS = os.path.join(ROOT_DIR, "logs")
# the files of a benchmark in the old layout: <stem><suffix>, and what they are called in a folder now
SUFFIXES = {"_results.csv": RESULTS_FILE, "_times.png": PLOT_FILE,
            **{f"_{name}.json": f"{name}.json" for name in MAPFAST_DETAILS}}
SHARD = re.compile(r"^(.*)_shard(\d+)\.csv$")
INSTANCE = re.compile(r"^(.*)-(random|even)-\d+_agents(\d+)\.yaml$")
LOG = re.compile(r"^(.*)_(\d+)\.(out|err)$")
JOB_ID = re.compile(r"_(\d+)$")
MERGE_STEM = re.compile(r"^(all|full|pass1)_")
TIME_LIMITS = (5, 10, 30, 60, 120, 300, 600, 1200, 1800, 3600)


class FileSet:
    """The files of one old stem (e.g. Berlin_1_256_7094000, all_7095276, full_7095349) and what its results hold."""

    def __init__(self, stem):
        self.stem = stem
        self.files = {}     # new file name -> old path
        self.shards = {}    # shard number -> old path
        self.keys = set()   # (instance, solver) of every run
        self.runs = set()   # (map, scenario type) it covers
        self.agents = set()
        self.longest = 0.
        match = JOB_ID.search(stem)
        self.job = int(match.group(1)) if match else None

    def read(self):
        path = self.files.get(RESULTS_FILE)
        if not path:
            return
        with open(path, newline="") as results_file:
            for row in csv.DictReader(results_file):
                self.keys.add((row["map"], row["solver"]))
                match = INSTANCE.match(row["map"])
                if match:
                    self.runs.add((match.group(1), match.group(2)))
                    self.agents.add(int(match.group(3)))
                if row.get("seconds"):
                    self.longest = max(self.longest, float(row["seconds"]))

    def is_map_run(self):
        """One map's benchmark.py run, not a merge (the merges of the old layout were all_<id>, full_<id> and pass1_<date>)"""
        return len(self.runs) == 1 and not MERGE_STEM.match(self.stem)

    def describe(self):
        """A name for what it holds, e.g. movingai_10-200-agents_60s or Berlin_1_256-even_10-200-agents_60s"""
        limit = next((t for t in TIME_LIMITS if self.longest <= t), int(self.longest) + 1)
        agents = f"{min(self.agents)}-{max(self.agents)}-agents" if self.agents else "no-runs"
        maps = {m for m, _ in self.runs}
        types = {t for _, t in self.runs}
        if self.is_map_run():
            m, t = next(iter(self.runs))
            scope = m + ("-even" if t == "even" else "")
        else:
            total = len([d for d in os.listdir(BENCHMARK_DIR) if os.path.isdir(os.path.join(BENCHMARK_DIR, d))])
            scope = "movingai" + ("" if len(maps) >= total else f"-{len(maps)}-maps")
            if len(types) == 1:
                scope += f"-{next(iter(types))}-only"
        return f"{scope}_{agents}_{limit}s"

    def run_name(self):
        m, t = next(iter(self.runs))
        return m + ("-even" if t == "even" else "")


def collect():
    """The FileSets of output/, and the files of output/ that are not part of one"""
    sets, other = {}, []
    for name in sorted(os.listdir(OLD_OUTPUT)) if os.path.isdir(OLD_OUTPUT) else []:
        path = os.path.join(OLD_OUTPUT, name)
        shard = SHARD.match(name)
        if shard:
            sets.setdefault(shard.group(1), FileSet(shard.group(1))).shards[int(shard.group(2))] = path
            continue
        suffix = next((s for s in SUFFIXES if name.endswith(s)), None)
        if suffix and os.path.isfile(path):
            stem = name[:-len(suffix)]
            sets.setdefault(stem, FileSet(stem)).files[SUFFIXES[suffix]] = path
        else:
            other.append(path)
    for file_set in sets.values():
        file_set.read()
    return sets, other


def plan(sets, other, renames={}):
    """{old path: new path} and {new folder: [(old path, new path)]}, for every file of output/ and logs/"""
    moves = {}
    # an extended benchmark: <source>_<T>s of tools/extend_benchmark.py, or one with shard files
    extended = {s.stem: s for s in sets.values()
                if s.shards or (re.match(r"^(.*)_\d+s$", s.stem) and re.match(r"^(.*)_\d+s$", s.stem).group(1) in sets)}
    merges = [s for s in sets.values() if s.keys and not s.is_map_run() and s.stem not in extended]
    map_runs = [s for s in sets.values() if s.keys and s.is_map_run() and s.stem not in extended]

    def contains(big, small):
        return big is not small and len(big.keys) >= len(small.keys) and small.keys <= big.keys

    # each merge goes to the largest merge that contains it
    top = {}
    for merge in merges:
        owners = [m for m in merges if contains(m, merge) and len(m.keys) > len(merge.keys)]
        top[merge.stem] = max(owners, key=lambda m: len(m.keys)) if owners else merge
    names, used = {}, set()

    def folder(file_set):
        if file_set.stem not in names:
            name = base = renames.get(file_set.stem) or file_set.describe()
            k = 2
            # a name given with --rename may exist already (a run that stopped part way): its files go into it
            while name in used or (file_set.stem not in renames and os.path.exists(os.path.join(EXPERIMENTS_DIR, name))):
                name, k = f"{base}_{k}", k + 1
            used.add(name)
            names[file_set.stem] = os.path.join(EXPERIMENTS_DIR, name)
        return names[file_set.stem]

    job_places = {}

    def place(file_set, directory):
        for name, path in file_set.files.items():
            moves[path] = os.path.join(directory, name)
        for number, path in file_set.shards.items():
            moves[path] = os.path.join(folder(file_set), "shards", f"shard{number}.csv")

    taken = collections.defaultdict(set)

    def unique(directory, name):
        """name, or <name>_earlier-<k> if the folder has it already (in this plan or on disk)"""
        base, k = name, 1
        while name in taken[directory] or os.path.exists(os.path.join(directory, name)):
            name, k = f"{base}_earlier-{k}", k + 1
        taken[directory].add(name)
        return name

    # the latest merge (and below, map run) of a group gets its name, earlier ones <name>_earlier-<k>
    for merge in sorted(merges, key=lambda s: -(s.job or 0)):
        owner = top[merge.stem]
        if owner is merge:
            place(merge, folder(merge))
            if merge.job:
                job_places[merge.job] = (folder(merge), "merge")
        else:
            group = merge.describe().split("_", 1)[1] if len(merge.runs) else merge.stem
            types = sorted({t for _, t in merge.runs})
            group = unique(os.path.join(folder(owner), "merges"), f"{'+'.join(types)}_{group}")
            place(merge, os.path.join(folder(owner), "merges", group))
            if merge.job:
                job_places[merge.job] = (folder(owner), f"merge_{group}")
    for run in sorted(map_runs, key=lambda s: -(s.job or 0)):
        owners = [m for m in merges if contains(m, run)]
        if owners:
            # the smallest merge that includes the run, and among equals the first job after it
            owner = min(owners, key=lambda m: (len(m.keys), abs((m.job or 0) - (run.job or 0))))
            experiment = folder(top[owner.stem])
        else:
            experiment = folder(run)
        name = unique(os.path.join(experiment, "maps"), run.run_name())
        place(run, os.path.join(experiment, "maps", name))
        if run.job:
            job_places[run.job] = (experiment, name)
    for file_set in extended.values():
        place(file_set, folder(file_set) if file_set.keys else os.path.join(EXPERIMENTS_DIR, "unsorted", file_set.stem))
    unsorted = os.path.join(EXPERIMENTS_DIR, "unsorted")
    for file_set in sets.values():
        if not file_set.keys and file_set.stem not in extended:
            for name, path in file_set.files.items():
                moves[path] = os.path.join(unsorted, os.path.basename(path))
    for path in other:
        moves[path] = os.path.join(unsorted, os.path.basename(path))

    appends = collections.defaultdict(list)
    for name in sorted(os.listdir(OLD_LOGS)) if os.path.isdir(OLD_LOGS) else []:
        path = os.path.join(OLD_LOGS, name)
        match = LOG.match(name)
        place_of = job_places.get(int(match.group(2))) if match else None
        if place_of:
            target = os.path.join(place_of[0], "logs", f"{place_of[1]}.{match.group(3)}")
        else:
            target = os.path.join(unsorted, "logs", name)
        appends[target].append(path)
    for target, paths in appends.items():
        if len(paths) == 1:
            moves[paths[0]] = target
        else:
            for path in paths:
                moves[path] = target + "  (appended)"
    return moves, names


def running_jobs():
    try:
        out = subprocess.run(["squeue", "--me", "-h"], capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [line for line in out.splitlines() if line.strip()]


def apply(moves, names, sets):
    origins = collections.defaultdict(list)
    for old, new in sorted(moves.items(), key=lambda m: m[0]):
        append = new.endswith("  (appended)")
        new = new.replace("  (appended)", "")
        os.makedirs(os.path.dirname(new), exist_ok=True)
        if append:
            with open(new, "a") as target, open(old) as source:
                target.write(f"===== {os.path.relpath(old, ROOT_DIR)} =====\n")
                shutil.copyfileobj(source, target)
            os.remove(old)
        else:
            if os.path.exists(new):
                sys.exit(f"{new} exists, not overwriting it (nothing after this was moved)")
            shutil.move(old, new)
        experiment = os.path.relpath(new, EXPERIMENTS_DIR).split(os.sep)[0]
        origins[experiment].append(f"{os.path.relpath(old, ROOT_DIR)} -> {os.path.relpath(new, os.path.join(EXPERIMENTS_DIR, experiment))}")
    for experiment, lines in origins.items():
        with open(os.path.join(EXPERIMENTS_DIR, experiment, "migrated.txt"), "a") as record:
            record.write("Moved from the old output/ and logs/ by tools/temp_organize_results.py (the numbers are Slurm job ids)\n")
            record.write("\n".join(lines) + "\n")
    for directory in (OLD_OUTPUT, OLD_LOGS):
        if os.path.isdir(directory):
            try:
                os.rmdir(directory)
                print(f"removed the empty {directory}")
            except OSError:
                print(f"{directory} is not empty: {os.listdir(directory)[:5]}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--apply", action="store_true", help="move the files (default: only print the plan)")
    parser.add_argument("--rename", action="append", default=[], metavar="PREFIX=NAME",
                        help="name the experiment of the old files <PREFIX>_* (e.g. full_7095349=movingai_10-200-agents_60s)")
    parser.add_argument("--force", action="store_true", help="move them even though benchmark jobs are running")
    args = parser.parse_args()

    sets, other = collect()
    if not sets and not other and not (os.path.isdir(OLD_LOGS) and os.listdir(OLD_LOGS)):
        # e.g. run a second time: everything is in experiments/ already
        print("nothing to move: no output/ or logs/")
        sys.exit(0)
    renames = dict(r.split("=", 1) for r in args.rename)
    unknown = [stem for stem in renames if stem not in sets]
    if unknown:
        sys.exit(f"--rename: no files of {', '.join(unknown)} in output/ (its prefixes: {', '.join(sorted(sets)[:10])})")
    moves, names = plan(sets, other, renames)
    by_experiment = collections.defaultdict(list)
    for old, new in moves.items():
        by_experiment[os.path.relpath(new.replace("  (appended)", ""), EXPERIMENTS_DIR).split(os.sep)[0]].append((old, new))
    for experiment, items in sorted(by_experiment.items()):
        stems = sorted({s.stem for s in sets.values() if names.get(s.stem, "").endswith(os.sep + experiment)})
        print(f"\nexperiments/{experiment}/" + (f"   (from {', '.join(stems)})" if stems else ""))
        for old, new in sorted(items, key=lambda i: i[1]):
            print(f"    {os.path.relpath(new, os.path.join(EXPERIMENTS_DIR, experiment)):<55} <- {os.path.relpath(old, ROOT_DIR)}")
    print(f"\n{len(moves)} files into {len(by_experiment)} folders of experiments/")
    if not args.apply:
        print("Nothing was moved: check the plan, then run again with --apply")
        sys.exit(0)
    jobs = running_jobs()
    if jobs and not args.force:
        sys.exit(f"{len(jobs)} of your Slurm jobs are queued or running; wait for them (or --force if none writes "
                 "to output/ or logs/)")
    apply(moves, names, sets)
