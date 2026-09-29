#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include "mcts.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::MCTS;

// Besides the agent ids 0, 1, ... a cell of the grid is EMPTY or a WALL
const int EMPTY = -1;
const int WALL = -2;

// The whole map after some of the paths have been grown (the colouring chi of
// the paper). Cell (x, y) is stored at x + dimx * y.
struct State
{
  std::vector<int> grid;  // agent id, EMPTY or WALL
  std::vector<int> head;  // last cell of the path of each agent
  std::vector<char> done; // head is on or next to the goal
  std::vector<int> moves; // EMPTY neighbours of head[i] (unfinished agents)
  std::vector<int> open;  // EMPTY neighbours of goal i (unfinished agents)
  int nDone = 0;
  int active = -1;   // agent extended next (-1: none)
  bool dead = false; // an unfinished agent can no longer be finished
};

///
enum class Action
{
  Up,
  Down,
  Left,
  Right,
};

std::ostream &operator<<(std::ostream &os, const Action &a)
{
  switch (a)
  {
  case Action::Up:
    os << "Up";
    break;
  case Action::Down:
    os << "Down";
    break;
  case Action::Left:
    os << "Left";
    break;
  case Action::Right:
    os << "Right";
    break;
  }
  return os;
}

const Action ACTIONS[] = {Action::Up, Action::Down, Action::Left,
                          Action::Right};

// Manhattan distance of two cells of a map with dimx columns
int manhattan(int dimx, int a, int b)
{
  return std::abs(a % dimx - b % dimx) + std::abs(a / dimx - b / dimx);
}

///
class Environment
{
public:
  Environment(int dimx, int dimy, const std::unordered_set<int> &obstacles,
              std::vector<int> starts, std::vector<int> goals)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_starts(std::move(starts)),
        m_goals(std::move(goals)),
        m_agents(static_cast<int>(m_starts.size())),
        m_baseGrid(static_cast<size_t>(dimx) * static_cast<size_t>(dimy),
                   EMPTY),
        m_goalOwner(m_baseGrid.size(), -1),
        m_expanded(0)
  {
    for (int cell : obstacles)
    {
      m_baseGrid[cell] = WALL;
    }
    for (int i = 0; i < m_agents; ++i)
    {
      m_goalOwner[m_goals[i]] = i;
    }
  }

  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;

  State initialState()
  {
    State s;
    s.grid = m_baseGrid;
    s.head = m_starts;
    s.done.assign(m_starts.size(), 0);
    s.moves.assign(m_starts.size(), 0);
    s.open.assign(m_starts.size(), 0);
    for (int i = 0; i < m_agents; ++i)
    {
      s.grid[m_starts[i]] = i;
      s.grid[m_goals[i]] = i;
    }
    for (int i = 0; i < m_agents; ++i)
    {
      if (m_starts[i] == m_goals[i] || adjacent(m_starts[i], m_goals[i]))
      {
        s.done[i] = 1;
        ++s.nDone;
      }
      s.moves[i] = countEmpty(s, m_starts[i]);
      s.open[i] = countEmpty(s, m_goals[i]);
    }
    selectActive(s);
#ifdef MCTS_CHECK_COUNTERS
    checkCounters(s);
#endif
    return s;
  }

  // The moves of the head of the active agent: at most four (Eq. 5)
  void getActions(const State &s, std::vector<Action> &actions)
  {
    actions.clear();
    if (s.dead || s.active < 0)
    {
      return;
    }
    for (const Action &a : ACTIONS)
    {
      const int v = neighbor(s.head[s.active], a);
      if (v >= 0 && s.grid[v] == EMPTY)
      {
        actions.push_back(a);
      }
    }
  }

  // Paints exactly one cell: the one next to the head of the active agent (Eq.
  // 4)
  void apply(State &s, const Action &a)
  {
    const int c = s.active;
    assert(c >= 0);
    const int v = neighbor(s.head[c], a);
    assert(v >= 0 && s.grid[v] == EMPTY);
    s.grid[v] = c;
    s.head[c] = v;
    // v is not EMPTY any more: update the cells that have it as a neighbour
    for (const Action &b : ACTIONS)
    {
      const int u = neighbor(v, b);
      if (u < 0)
      {
        continue;
      }
      const int j = s.grid[u];
      if (j >= 0 && !s.done[j] && s.head[j] == u)
      {
        --s.moves[j];
      }
      const int g = m_goalOwner[u];
      if (g >= 0 && !s.done[g])
      {
        --s.open[g];
      }
    }
    s.moves[c] = countEmpty(s, v);
    if (adjacent(v, m_goals[c]))
    {
      s.done[c] = 1;
      ++s.nDone;
    }
    selectActive(s);
#ifdef MCTS_CHECK_COUNTERS
    checkCounters(s);
#endif
  }

  bool isSolution(const State &s) { return s.nDone == m_agents; }

  double reward(const State &s)
  {
    return m_agents == 0 ? 1.0 : static_cast<double>(s.nDone) / m_agents;
  }

  void onExpandNode(const State & /*s*/) { m_expanded++; }

  int expanded() const { return m_expanded; }

private:
  // The cell next to `cell` in direction `a`, -1 if that is outside the map
  int neighbor(int cell, Action a) const
  {
    switch (a)
    {
    case Action::Up:
      return cell / m_dimx + 1 < m_dimy ? cell + m_dimx : -1;
    case Action::Down:
      return cell / m_dimx > 0 ? cell - m_dimx : -1;
    case Action::Left:
      return cell % m_dimx > 0 ? cell - 1 : -1;
    case Action::Right:
      return cell % m_dimx + 1 < m_dimx ? cell + 1 : -1;
    }
    return -1;
  }

  bool adjacent(int a, int b) const { return manhattan(m_dimx, a, b) == 1; }

  int countEmpty(const State &s, int cell) const
  {
    int count = 0;
    for (const Action &a : ACTIONS)
    {
      const int u = neighbor(cell, a);
      if (u >= 0 && s.grid[u] == EMPTY)
      {
        ++count;
      }
    }
    return count;
  }

  // Lower branching factor first (Fig. 4 of the paper): the unfinished agent
  // whose head has the fewest EMPTY neighbours is extended next. Ties go to
  // the agent extended last, then to the lowest index. The state is dead if an
  // unfinished agent cannot continue, or if nothing next to its goal is EMPTY.
  void selectActive(State &s) const
  {
    const int previous = s.active;
    s.active = -1;
    s.dead = false;
    int fewest = std::numeric_limits<int>::max();
    for (int i = 0; i < m_agents; ++i)
    {
      if (s.done[i])
      {
        continue;
      }
      if (s.moves[i] == 0 || s.open[i] == 0)
      {
        s.dead = true;
        s.active = -1;
        return;
      }
      if (s.moves[i] < fewest || (s.moves[i] == fewest && i == previous))
      {
        fewest = s.moves[i];
        s.active = i;
      }
    }
  }

#ifdef MCTS_CHECK_COUNTERS
  // The incremental counters must equal a recount from scratch
  void checkCounters(const State &s) const
  {
    for (int i = 0; i < m_agents; ++i)
    {
      if (!s.done[i])
      {
        assert(s.moves[i] == countEmpty(s, s.head[i]));
        assert(s.open[i] == countEmpty(s, m_goals[i]));
      }
    }
  }
#endif

private:
  int m_dimx;
  int m_dimy;
  std::vector<int> m_starts;
  std::vector<int> m_goals;
  int m_agents;
  std::vector<int> m_baseGrid;  // WALL on the obstacles, EMPTY elsewhere
  std::vector<int> m_goalOwner; // agent whose goal the cell is, else -1
  int m_expanded;
};

// Why the terminals cannot be connected by disjoint paths (empty: no reason)
std::string checkTerminals(const std::unordered_set<int> &obstacles,
                           const std::vector<int> &starts,
                           const std::vector<int> &goals)
{
  std::unordered_set<int> taken;
  for (size_t i = 0; i < starts.size(); ++i)
  {
    for (int cell : {starts[i], goals[i]})
    {
      if (cell < 0)
      {
        return "Terminal cell outside the map";
      }
      if (obstacles.count(cell) > 0)
      {
        return "Terminal cell on an obstacle";
      }
      if (taken.count(cell) > 0)
      {
        return "Terminal cells overlap";
      }
    }
    // start == goal of one agent is fine, so both are taken only afterwards
    taken.insert(starts[i]);
    taken.insert(goals[i]);
  }
  return "";
}

void writeFailure(const std::string &outputFile, double runtime,
                  size_t attempts, int expanded)
{
  std::ofstream out(outputFile);
  out << "statistics:" << std::endl;
  out << "  success: " << false << std::endl;
  out << "  runtime: " << runtime << std::endl;
  out << "  attempts: " << attempts << std::endl;
  out << "  nodesExpanded: " << expanded << std::endl;
}

int main(int argc, char *argv[])
{
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  int iterations;
  double exploration;
  int attempts;
  unsigned int seed;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "iterations", po::value<int>(&iterations)->default_value(100),
      "MCTS iterations per committed cell")(
      "exploration", po::value<double>(&exploration)->default_value(1.41421356),
      "UCT exploration constant")(
      "attempts", po::value<int>(&attempts)->default_value(10),
      "maximum number of independent search attempts")(
      "seed", po::value<unsigned int>(&seed)->default_value(0),
      "seed of the random number generator");

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // before notify(), which complains about missing required options
    if (vm.count("help") != 0u)
    {
      std::cout << desc << "\n";
      return 0;
    }
    po::notify(vm);

    if (iterations < 1)
    {
      throw po::error("iterations must be at least 1");
    }
    if (attempts < 1)
    {
      throw po::error("attempts must be at least 1");
    }
    if (!(exploration >= 0) || std::isinf(exploration))
    {
      throw po::error("exploration must be a finite number, at least 0");
    }
  }
  catch (po::error &e)
  {
    std::cerr << e.what() << std::endl
              << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  const auto &dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  // cell x + dimx * y of a YAML node [x, y], -1 if that is outside the map
  const auto cellOf = [dimx, dimy](const YAML::Node &node)
  {
    const int x = node[0].as<int>();
    const int y = node[1].as<int>();
    return (x < 0 || x >= dimx || y < 0 || y >= dimy) ? -1 : x + dimx * y;
  };

  // obstacles outside the map are never reached and can be ignored
  std::unordered_set<int> obstacles;
  for (const auto &node : config["map"]["obstacles"])
  {
    const int cell = cellOf(node);
    if (cell >= 0)
    {
      obstacles.insert(cell);
    }
  }

  std::vector<int> starts;
  std::vector<int> goals;
  for (const auto &node : config["agents"])
  {
    starts.push_back(cellOf(node["start"]));
    goals.push_back(cellOf(node["goal"]));
  }

  // sanity check: terminals on free cells of the map, none of them shared
  const std::string problem = checkTerminals(obstacles, starts, goals);
  if (!problem.empty())
  {
    std::cout << problem << " -> no solution!" << std::endl;
    std::cout << "Planning NOT successful!" << std::endl;
    writeFailure(outputFile, 0.0, 0, 0);
    return 0;
  }

  Environment mapf(dimx, dimy, obstacles, starts, goals);
  MCTS<State, Action, Environment> mcts(mapf, static_cast<size_t>(iterations),
                                        exploration, seed,
                                        static_cast<size_t>(attempts));
  std::vector<Action> actions;

  Timer timer;
  bool success = mcts.search(mapf.initialState(), actions);
  timer.stop();

  if (success)
  {
    std::cout << "Planning successful! " << std::endl;

    // Replay the actions: each one extends the path of the active agent, and a
    // path that is finished continues onto its goal.
    std::vector<std::vector<int>> paths(starts.size());
    State s = mapf.initialState();
    for (size_t i = 0; i < paths.size(); ++i)
    {
      paths[i].push_back(starts[i]);
      if (s.done[i] && goals[i] != starts[i])
      {
        paths[i].push_back(goals[i]);
      }
    }
    for (const Action &a : actions)
    {
      const int c = s.active;
      mapf.apply(s, a);
      paths[c].push_back(s.head[c]);
      if (s.done[c])
      {
        paths[c].push_back(goals[c]);
      }
    }
    assert(mapf.isSolution(s));

    // The paths run from start to goal in unit steps and share no cell
    std::vector<char> used(
        static_cast<size_t>(dimx) * static_cast<size_t>(dimy), 0);
    for (size_t i = 0; i < paths.size(); ++i)
    {
      assert(paths[i].front() == starts[i] && paths[i].back() == goals[i]);
      for (size_t k = 0; k < paths[i].size(); ++k)
      {
        assert(used[paths[i][k]] == 0);
        used[paths[i][k]] = 1;
        assert(k == 0 || manhattan(dimx, paths[i][k - 1], paths[i][k]) == 1);
      }
    }

    int cost = 0;
    int makespan = 0;
    for (const auto &path : paths)
    {
      const int length = static_cast<int>(path.size()) - 1;
      cost += length;
      makespan = std::max<int>(makespan, length);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  success: " << true << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  attempts: " << mcts.attempts() << std::endl;
    out << "  nodesExpanded: " << mapf.expanded() << std::endl;
    out << "schedule:" << (paths.empty() ? " {}" : "") << std::endl;
    for (size_t a = 0; a < paths.size(); ++a)
    {
      out << "  agent" << a << ":" << std::endl;
      for (size_t t = 0; t < paths[a].size(); ++t)
      {
        out << "    - x: " << paths[a][t] % dimx << std::endl
            << "      y: " << paths[a][t] / dimx << std::endl
            << "      t: " << t << std::endl;
      }
    }
  }
  else
  {
    std::cout << "Planning NOT successful!" << std::endl;
    writeFailure(outputFile, timer.elapsedSeconds(), mcts.attempts(),
                 mapf.expanded());
  }

  return 0;
}
