#!/usr/bin/env python3
"""
This script is designed to take standard MAPF benchmark problems
(https://movingai.com/benchmarks/mapf/index.html) and convert them into YAML
files used by the example implementations provided by libMultiRobotPlanning.

It can also be imported: benchmark.py converts the instances it runs on a
libMultiRobotPlanning solver with convert(), and write_movingai() converts
the other way, for the solvers that read MovingAI files.
"""
import os
import argparse


def setup_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", type=str, help=".scen Scenario file")
    parser.add_argument("map", type=str, help=".map Map file")
    parser.add_argument("output_prefix", type=str, help=".yaml Output file prefix")
    return parser.parse_args()


def convert_nums(l):
    for i in range(len(l)):
        try:
            l[i] = int(l[i])
        except ValueError:
            try:
                l[i] = float(l[i])
            except ValueError:
                ""
    return l


def load_map_file(map_file, occupied_char={'@', 'T', 'O'}, valid_chars={'@', '.', 'T', 'G', 'O', 'S', 'W'}):
    if not os.path.isfile(map_file):
        raise FileNotFoundError(f"Map file not found: {map_file}")
    with open(map_file, 'r') as f:
        map_ls = f.readlines()
    height = int(map_ls[1].replace("height ", ""))
    width = int(map_ls[2].replace("width ", ""))
    map_ls = map_ls[4:]
    map_ls = [l.rstrip('\r\n') for l in map_ls]  # also CRLF files
    occupancy_lst = set()
    assert(len(map_ls) == height)
    for y, l in enumerate(map_ls):
        assert(len(l) == width)
        for x, c in enumerate(l):
            assert(c in valid_chars)
            if c in occupied_char:
                occupancy_lst.add((x, y))
    return width, height, occupancy_lst


def load_scenario_file(scen_file,
                       occupancy_list,
                       map_width,
                       map_height):
    """The ((sx, sy), (gx, gy)) of every agent of `scen_file`, in file order.

    The order is kept (not sorted by bucket) because a problem with n agents is
    the first n agents of the file, as for the other solvers (e.g. the
    --agent-limit of bcp2-mapf), so that all solvers get the same problem.
    """
    if not os.path.isfile(scen_file):
        raise FileNotFoundError(f"Scenario file not found: {scen_file}")
    with open(scen_file, 'r') as f:
        ls = f.readlines()
    if "version 1" not in ls[0]:
        raise ValueError(f"{scen_file}: .scen version type does not match!")
    instances = [convert_nums(l.split('\t')) for l in ls[1:] if l.strip()]
    for i in instances:
        assert(i[2] == map_width)
        assert(i[3] == map_height)
    # ((sx, sy), (gx, gy))
    instances = [((i[4], i[5]), (i[6], i[7])) for i in instances]
    for start, goal in instances:
        assert(start not in occupancy_list)
        assert(goal not in occupancy_list)
    return instances


def generate_sliced_problems(instances,
                             map_width,
                             map_height,
                             occupancy_list,
                             file_pattern,
                             min_agents=10,
                             agent_step=10):
    for agent_count in range(min_agents, len(instances) + 1, agent_step):
        file_name = file_pattern.format(agent_count)
        print("Generating", file_name)
        dump_yaml(instances[:agent_count],
                  map_width,
                  map_height,
                  occupancy_list,
                  file_name)


def dump_yaml(instances, map_width, map_height, occupancy_list, filename):
    # obstacles are plain lists (not !!python/tuple) so that yaml.safe_load,
    # used by the Python tools, can read the file as well as the C++ solvers
    with open(filename, 'w') as f:
        f.write("agents:\n")
        for idx, i in enumerate(instances):
            f.write(f"""-   goal: {list(i[1])}
    name: agent{idx}
    start: {list(i[0])}
""")
        f.write("map:\n")
        f.write(f"    dimensions: {[map_width, map_height]}\n")
        # an empty list rather than nothing, which reads as None
        f.write("    obstacles:\n" if occupancy_list else "    obstacles: []\n")
        for o in sorted(occupancy_list):
            f.write(f"    - {list(o)}\n")


def convert(scen_file, map_file, n_agents, output_file):
    """Write the problem of the first `n_agents` agents of `scen_file` on
    `map_file` to the YAML file `output_file`, unless it already exists.

    Written to a temporary file first, so a converted file is always complete.
    """
    if os.path.exists(output_file):
        return output_file
    map_width, map_height, occupancy_list = load_map_file(map_file)
    instances = load_scenario_file(scen_file, occupancy_list, map_width, map_height)
    if n_agents > len(instances):
        raise ValueError(f"{scen_file} has {len(instances)} agents, fewer than {n_agents}")
    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
    dump_yaml(instances[:n_agents], map_width, map_height, occupancy_list, output_file + ".tmp")
    os.replace(output_file + ".tmp", output_file)
    return output_file


def write_movingai(map_width, map_height, occupancy_list, instances, map_file, scen_file):
    """Write a grid and its agents ((start, goal) pairs of (x, y) cells, in
    order) as a MovingAI map and scenario file, the reverse of load_map_file
    and load_scenario_file. The scenario names the map by its file name, so
    both belong in the same directory. The optimal length column is 0 (unknown),
    which the solvers do not use."""
    with open(map_file, 'w') as f:
        f.write(f"type octile\nheight {map_height}\nwidth {map_width}\nmap\n")
        for y in range(map_height):
            f.write("".join("@" if (x, y) in occupancy_list else "." for x in range(map_width)) + "\n")
    with open(scen_file, 'w') as f:
        f.write("version 1\n")
        for (sx, sy), (gx, gy) in instances:
            f.write(f"0\t{os.path.basename(map_file)}\t{map_width}\t{map_height}\t{sx}\t{sy}\t{gx}\t{gy}\t0\n")


if __name__ == "__main__":
    args = setup_args()
    print("Loading map")
    map_width, map_height, occupancy_list = load_map_file(args.map)
    print("Map loaded")
    print("Loading scenario file")
    instances = load_scenario_file(args.scenario,
                                   occupancy_list,
                                   map_width,
                                   map_height)
    print("Scenario loaded")
    generate_sliced_problems(instances,
                             map_width,
                             map_height,
                             occupancy_list,
                             args.output_prefix + "_{}_agents.yaml")
