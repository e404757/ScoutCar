#pragma once

#include "graph.h"

#include <functional>
#include <vector>

// Admissible heuristic: estimated remaining cost from a node to the goal.
using Heuristic = std::function<double(int node)>;

// A* shortest path on the directed graph `g`.
//
//   Returns the node sequence [start ... goal] with minimum total edge weight,
//   or an empty vector if the goal is unreachable from start.
//
// Only currently-traversable directed edges (see Graph::neighbors / canGo) are
// expanded, so directional obstacles naturally cause detours / replanning.
std::vector<int> astar(const Graph& g, int start, int goal, const Heuristic& h);
