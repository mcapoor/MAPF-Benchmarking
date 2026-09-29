import csv

from tools.utils import *

# CBSH2-RTC with the cluster heuristic and bypass improvements of Shen et al.
# (git submodule libs/CBSH2-RTC-CHBP, built into its build/ directory)
CHBP_DIR = os.path.join(LIBS_DIR, "CBSH2-RTC-CHBP")
CHBP_BINARY = os.path.join(CHBP_DIR, "build", "cbs")

# the wrappers below, which all read a MovingAI map and scenario file
CHBP_SOLVERS = {"cbsh2_rtc", "cbsh2_rtc_ch", "cbsh2_rtc_bp", "cbsh2_rtc_chbp"}

# the solver's own cutoff (-t, in seconds) when no timeout is given, as its
# default of 60 s would silently limit the run
NO_CUTOFF = 1e9
# extra seconds the process gets beyond its own cutoff before it is killed,
# for loading the instance and writing the results
KILL_GRACE = 5


def _read_paths(paths_file):
    """The schedule of a paths file of the solver, in which line i is
    `Agent i: (row,col)->(row,col)->...` with one location per timestep.

    Waypoints are {x, y, t} with x the column and y the row, as in
    libMultiRobotPlanning, and agents are named agent0, agent1, ...
    """
    schedule = {}
    with open(paths_file) as paths:
        for line in paths:
            if not line.strip():
                continue
            agent, locations = line.split(":", 1)
            locations = [location.strip("()").split(",")
                         for location in locations.strip().split("->") if location]
            schedule["agent" + agent.split()[1]] = [
                {"x": int(col), "y": int(row), "t": t}
                for t, (row, col) in enumerate(locations)]
    return schedule


def run_cbsh2(scen_file, n_agents, cluster_heuristics, map_file=None,
              output=None, stats=None, timeout=None, additional_args=(),
              quiet=False):
    """Run CBSH2-RTC-CHBP on the first `n_agents` agents of `scen_file`.

    `cluster_heuristics` is the solver's --cluster_heuristics variant: N
    (plain CBSH2-RTC), CH (cluster heuristic), BP (bypass) or CHBP (both).
    `map_file` is by default the map named in the scenario file (see
    scenario_map). `output` is the solver's paths file and `stats` its CSV of
    search statistics, to which it appends one row per run; both are temporary
    files if None. Arguments in `additional_args` go on the command line as
    they are (e.g. ["--heuristics", "DG"]).

    `timeout` (seconds) is the solver's own cutoff, which ends the run without
    a solution. As a backstop the process is killed KILL_GRACE seconds later,
    raising subprocess.TimeoutExpired. Raises subprocess.CalledProcessError on
    a non-zero exit status. With `quiet` the solver's console output is
    discarded.

    Returns a dict like that of the libMultiRobotPlanning wrappers, with
    "statistics" (cost, makespan, runtime, highLevelExpanded,
    lowLevelExpanded) and "schedule" (agent name to a list of {x, y, t}
    waypoints), or None if no solution was found.
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        paths_path = output or os.path.join(tmp_dir, "paths.txt")
        stats_path = stats or os.path.join(tmp_dir, "stats.csv")
        if os.path.exists(paths_path):
            os.remove(paths_path)  # only written if there is a solution
        subprocess.run(
            [CHBP_BINARY,
             "-m", map_file or scenario_map(scen_file),
             "-a", scen_file,
             "-k", str(n_agents),
             "-t", str(NO_CUTOFF if timeout is None else timeout),
             "--cluster_heuristics", cluster_heuristics,
             "-o", stats_path,
             "--outputPaths", paths_path,
             *[str(a) for a in additional_args]],
            check=True,
            timeout=None if timeout is None else timeout + KILL_GRACE,
            **stdio(quiet))
        if not os.path.exists(paths_path):
            return None
        schedule = _read_paths(paths_path)
        with open(stats_path) as stats_file:
            row = list(csv.DictReader(stats_file))[-1]
    return {
        "statistics": {
            "cost": int(row["solution cost"]),
            "makespan": max(len(path) for path in schedule.values()) - 1,
            "runtime": float(row["runtime"]),
            "highLevelExpanded": int(row["#high-level expanded"]),
            "lowLevelExpanded": int(row["#low-level expanded"]),
        },
        "schedule": schedule,
    }


def cbsh2_rtc(scen_file, n_agents, map_file=None, output=None, stats=None,
              timeout=None, additional_args=(), quiet=False):
    """CBSH2-RTC (Li et al.) without the CHBP improvements."""
    return run_cbsh2(scen_file, n_agents, "N", map_file=map_file,
                     output=output, stats=stats, timeout=timeout,
                     additional_args=additional_args, quiet=quiet)


def cbsh2_rtc_ch(scen_file, n_agents, map_file=None, output=None, stats=None,
                 timeout=None, additional_args=(), quiet=False):
    """CBSH2-RTC with the cluster heuristic only."""
    return run_cbsh2(scen_file, n_agents, "CH", map_file=map_file,
                     output=output, stats=stats, timeout=timeout,
                     additional_args=additional_args, quiet=quiet)


def cbsh2_rtc_bp(scen_file, n_agents, map_file=None, output=None, stats=None,
                 timeout=None, additional_args=(), quiet=False):
    """CBSH2-RTC with the cluster bypass only."""
    return run_cbsh2(scen_file, n_agents, "BP", map_file=map_file,
                     output=output, stats=stats, timeout=timeout,
                     additional_args=additional_args, quiet=quiet)


def cbsh2_rtc_chbp(scen_file, n_agents, map_file=None, output=None,
                   stats=None, timeout=None, additional_args=(), quiet=False):
    """CBSH2-RTC with the cluster heuristic and bypass (the paper's CHBP)."""
    return run_cbsh2(scen_file, n_agents, "CHBP", map_file=map_file,
                     output=output, stats=stats, timeout=timeout,
                     additional_args=additional_args, quiet=quiet)
