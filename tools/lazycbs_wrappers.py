import shutil

from tools.utils import *

# Gange et al.'s CBS with a lazy clause generation backend (git submodule
# libs/lazycbs, built into its build/ directory), which reads
# libMultiRobotPlanning's YAML problem format
LAZYCBS_BINARY = os.path.join(LIBS_DIR, "lazycbs", "build", "lazycbs_mapf")


def lazycbs(input_file, output=None, timeout=None, quiet=False):
    """Run Lazy CBS on a YAML grid problem (sum of costs optimal). It has no
    time limit of its own, so the process is killed after `timeout` seconds,
    raising subprocess.TimeoutExpired.

    The solver writes its solution, in libMultiRobotPlanning's output format,
    to ../example/output_lazycbs.yaml relative to its working directory, here
    inside a temporary directory; `output` keeps it. Raises
    subprocess.CalledProcessError on a non-zero exit status. With `quiet` the
    solver's console output is discarded.

    Returns the parsed output (statistics with cost and runtime, and the
    schedule), or None if no solution was found.
    """
    with tempfile.TemporaryDirectory() as tmp_dir:
        os.makedirs(os.path.join(tmp_dir, "run"))
        os.makedirs(os.path.join(tmp_dir, "example"))
        output_path = os.path.join(tmp_dir, "example", "output_lazycbs.yaml")
        subprocess.run(
            [LAZYCBS_BINARY, "-i", os.path.abspath(input_file)],
            check=True,
            cwd=os.path.join(tmp_dir, "run"),
            timeout=timeout,
            **stdio(quiet))
        if not os.path.exists(output_path):
            return None
        if output:
            shutil.copyfile(output_path, output)
        with open(output_path) as output_file:
            return yaml.load(output_file, Loader=YAML_LOADER)
