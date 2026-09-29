#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <random>
#include <utility>
#include <vector>

namespace libMultiRobotPlanning {

/*!
  \example mcts_nonoverlap.cpp Example that solves the Multi-Agent
  Path-Finding (MAPF) problem with non-overlapping paths in a 2D grid world
  with up/down/left/right actions
*/

/*! \brief Monte-Carlo Tree Search (MCTS) to solve Multi-Agent Path-Finding
(MAPF) problems with non-overlapping paths

This class implements a Monte-Carlo Tree Search (MCTS) for the Multi-Agent
Path-Finding (MAPF) problem in which the paths of all agents must not share any
cell. The paths are static: there is no time dimension and no objective such as
the path length.
A node of the search tree is a partial solution, and a child differs from its
parent in exactly one cell, which extends the path of one agent by one step.
The Environment decides which agent is extended and which actions are legal.
A search pass repeats selection (UCT), expansion, rollout (uniform random
default policy) and backpropagation for a given number of iterations. Then it
commits the most visited child of the root, discards the tree and continues
with the next pass from that child.
A pass stops as soon as an expansion or a rollout completes all agents. A state
with only one legal action is extended without a search.
Dead ends are detected exactly: a state without legal actions that is not a
solution is dead, and so is a state whose children are all dead. Dead states
are never selected again. If the root of a pass is dead, the attempt has failed
and a new attempt is started from the initial state, up to a given number of
attempts. If this happens before the search has committed an action that was
not forced, the problem is proven to be unsolvable and the search stops.
The search is randomized (but reproducible for a fixed seed) and neither
complete nor optimal.
The constructor takes the number of iterations of a search pass, the
exploration constant C of the UCT formula, the seed of the random number
generator and the maximum number of attempts.
There is no Cost template parameter, because the rewards are doubles in [0, 1].

Details of the algorithm can be found in the following paper:\n
Mohammad Sina Kiarostami, Mohammad Reza Daneshvaramoli, Saleh Khalaj Monfared,
Dara Rahmati, Saeid Gorgin:\n
"Multi-Agent non-Overlapping Pathfinding with Monte-Carlo Tree Search". IEEE
(2019)

The MCTS methods it builds on are surveyed in the following paper:\n
Cameron B. Browne et al.:\n
"A survey of Monte Carlo tree search methods". IEEE Trans. Comput. Intell. AI
Games 4(1): 1-43 (2012)

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able and
default-constructible
\tparam Environment This class needs to provide the custom logic. In
particular, it needs to support the following functions:
  - `void getActions(const State& s, std::vector<Action>& actions)`\n
    Fill the list of legal actions for the given state s. Each action extends
one path by one cell, and every solution that extends s has to start with one
of them (e.g. all moves of the agent that is extended next; at most four). The
list is empty if and only if s is a solution or a dead end.

  - `void apply(State& s, const Action& a)`\n
    Apply the given action to the state s in place. This paints exactly one
cell. Every sequence of actions has to reach a state without legal actions
after a finite number of steps.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a solution, i.e., all agents are done.

  - `double reward(const State& s)`\n
    Return the reward of the given state as a number in [0, 1], which is 1 if
and only if s is a solution. It rates the dead end at which a rollout ended,
e.g. by the fraction of agents that are done.

  - `void onExpandNode(const State& s)`\n
    This function is called on every tree expansion and can be used for
statistical purposes.
*/
template <typename State, typename Action, typename Environment>
class MCTS {
 public:
  MCTS(Environment& environment, size_t iterations = 100,
       double exploration = 1.41421356, unsigned int seed = 0,
       size_t attempts = 10)
      : m_env(environment),
        m_iterations(iterations),
        m_exploration(exploration),
        m_attempts(attempts),
        m_attemptsUsed(0),
        m_rng(seed) {
    assert(iterations > 0 && attempts > 0);
  }

  bool search(const State& startState, std::vector<Action>& solution) {
    solution.clear();
    m_attemptsUsed = 0;
    for (size_t attempt = 1; attempt <= m_attempts; ++attempt) {
      m_attemptsUsed = attempt;
      const Outcome outcome = attemptOnce(startState, solution);
      if (outcome == Outcome::Solved) {
        return true;
      }
      if (outcome == Outcome::Unsolvable) {
        return false;  // proven, a restart cannot help
      }
    }
    return false;
  }

  size_t attempts() const { return m_attemptsUsed; }

 private:
  enum class Outcome { Solved, Unsolvable, DeadEnd };

  enum class PassStatus { Ok, Solved, RootDead };

  struct Node {
    Node(State state, int parent, const Action& action)
        : state(std::move(state)),
          parent(parent),
          action(action),
          visits(0),
          reward(0.0),
          dead(false) {}

    State state;
    int parent;  // index in m_nodes, -1 for the root
    Action action;
    std::vector<Action> untried;
    std::vector<int> children;
    size_t visits;
    double reward;
    bool dead;
  };

  Outcome attemptOnce(const State& startState, std::vector<Action>& solution) {
    State current(startState);
    std::vector<Action> committed;
    std::vector<Action> actions;
    bool exact = true;  // every action committed so far was forced
    while (true) {
      if (m_env.isSolution(current)) {
        solution = committed;
        return Outcome::Solved;
      }
      actions.clear();
      m_env.getActions(current, actions);
      if (actions.empty()) {
        return exact ? Outcome::Unsolvable : Outcome::DeadEnd;
      }
      if (actions.size() == 1) {
        m_env.apply(current, actions[0]);
        committed.push_back(actions[0]);
        continue;
      }
      Action best = Action();
      const PassStatus status = runMCTS(current, committed, solution, best);
      if (status == PassStatus::Solved) {
        return Outcome::Solved;
      }
      if (status == PassStatus::RootDead) {
        return exact ? Outcome::Unsolvable : Outcome::DeadEnd;
      }
      m_env.apply(current, best);
      committed.push_back(best);
      exact = false;
    }
  }

  // One pass of MCTS (Algorithm 1) from rootState, which has at least two legal
  // actions. On PassStatus::Solved, solution is committed + the actions found
  // by the pass. On PassStatus::Ok, bestAction is the action to commit.
  PassStatus runMCTS(const State& rootState,
                     const std::vector<Action>& committed,
                     std::vector<Action>& solution, Action& bestAction) {
    m_nodes.clear();
    m_nodes.reserve(m_iterations + 1);
    m_nodes.emplace_back(rootState, -1, Action());
    m_env.getActions(m_nodes[0].state, m_nodes[0].untried);

    std::vector<Action> tail;
    size_t iteration = 0;
    while (iteration < m_iterations || !hasAliveChild(0)) {
      ++iteration;
      const int leaf = selection(0);
      const int child = expansion(leaf);
      if (m_env.isSolution(m_nodes[child].state)) {
        solution = committed;
        append(solution, actionsFromRoot(child));
        return PassStatus::Solved;
      }
      double reward = 0.0;
      if (rollout(child, reward, tail)) {
        solution = committed;
        append(solution, actionsFromRoot(child));
        append(solution, tail);
        return PassStatus::Solved;
      }
      backpropagation(child, reward);
      if (m_nodes[0].dead) {
        return PassStatus::RootDead;
      }
    }
    bestAction = m_nodes[bestChild(0)].action;
    return PassStatus::Ok;
  }

  // Descends through the alive children with the highest UCT value until it
  // reaches a node that still has untried actions.
  int selection(int x) const {
    while (m_nodes[x].untried.empty()) {
      const Node& parent = m_nodes[x];
      const double logVisits = std::log(static_cast<double>(parent.visits));
      int best = -1;
      double bestValue = 0.0;
      for (int c : parent.children) {
        const Node& child = m_nodes[c];
        if (child.dead) {
          continue;
        }
        const double value =
            mean(child) +
            m_exploration *
                std::sqrt(logVisits / static_cast<double>(child.visits));
        if (best < 0 || value > bestValue) {
          best = c;
          bestValue = value;
        }
      }
      assert(best >= 0);  // a node without alive children is dead
      x = best;
    }
    return x;
  }

  // Adds one child of x for a random untried action and returns its index.
  int expansion(int x) {
    std::vector<Action>& untried = m_nodes[x].untried;
    const size_t k = randomIndex(untried.size());
    const Action action = untried[k];
    untried[k] = untried.back();
    untried.pop_back();

    State state(m_nodes[x].state);
    m_env.apply(state, action);
    const int child = static_cast<int>(m_nodes.size());
    m_nodes.emplace_back(std::move(state), x, action);
    m_nodes[x].children.push_back(child);

    Node& node = m_nodes[child];
    m_env.getActions(node.state, node.untried);
    m_env.onExpandNode(node.state);
    if (node.untried.empty() && !m_env.isSolution(node.state)) {
      markDead(child);
    }
    return child;
  }

  // Default policy: random legal actions, applied in place, until the state is
  // a solution (returns true, tail holds the actions) or a dead end (returns
  // false, reward is the reward of the dead end).
  bool rollout(int x, double& reward, std::vector<Action>& tail) {
    State state(m_nodes[x].state);
    tail.clear();
    while (true) {
      if (m_env.isSolution(state)) {
        reward = 1.0;
        return true;
      }
      m_actions.clear();
      m_env.getActions(state, m_actions);
      if (m_actions.empty()) {
        reward = m_env.reward(state);
        tail.clear();
        return false;
      }
      const Action& action = m_actions[randomIndex(m_actions.size())];
      m_env.apply(state, action);
      tail.push_back(action);
    }
  }

  void backpropagation(int x, double reward) {
    for (int y = x; y >= 0; y = m_nodes[y].parent) {
      ++m_nodes[y].visits;
      m_nodes[y].reward += reward;
    }
  }

  // Alive child of x with the most visits; ties go to the larger mean reward,
  // then to the first child. Returns -1 if there is no alive child.
  int bestChild(int x) const {
    int best = -1;
    for (int c : m_nodes[x].children) {
      const Node& child = m_nodes[c];
      if (child.dead) {
        continue;
      }
      if (best < 0 || child.visits > m_nodes[best].visits ||
          (child.visits == m_nodes[best].visits &&
           mean(child) > mean(m_nodes[best]))) {
        best = c;
      }
    }
    return best;
  }

  // x is dead, and so is every ancestor whose actions are all expanded and
  // whose children are all dead.
  void markDead(int x) {
    while (true) {
      m_nodes[x].dead = true;
      const int p = m_nodes[x].parent;
      if (p < 0 || !m_nodes[p].untried.empty()) {
        return;
      }
      for (int c : m_nodes[p].children) {
        if (!m_nodes[c].dead) {
          return;
        }
      }
      x = p;
    }
  }

  bool hasAliveChild(int x) const {
    for (int c : m_nodes[x].children) {
      if (!m_nodes[c].dead) {
        return true;
      }
    }
    return false;
  }

  std::vector<Action> actionsFromRoot(int x) const {
    std::vector<Action> path;
    for (int y = x; m_nodes[y].parent >= 0; y = m_nodes[y].parent) {
      path.push_back(m_nodes[y].action);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  // Uniformly distributed index in [0, n), n > 0.
  size_t randomIndex(size_t n) {
    return std::uniform_int_distribution<size_t>(0, n - 1)(m_rng);
  }

  static double mean(const Node& node) {
    return node.reward / static_cast<double>(node.visits);
  }

  static void append(std::vector<Action>& to, const std::vector<Action>& from) {
    to.insert(to.end(), from.begin(), from.end());
  }

 private:
  Environment& m_env;
  size_t m_iterations;
  double m_exploration;
  size_t m_attempts;
  size_t m_attemptsUsed;
  std::mt19937 m_rng;
  std::vector<Node> m_nodes;
  std::vector<Action> m_actions;  // legal actions of the current rollout step
};

}  // namespace libMultiRobotPlanning
