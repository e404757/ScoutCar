#pragma once

#include "graph.h"

#include <unordered_map>
#include <utility>

// Build the 20-node bidirectional graph described in map.txt.
Graph buildDefaultMap();

// Node coordinates (col, row) used only for the A* Euclidean heuristic.
const std::unordered_map<int, std::pair<double, double>>& nodePositions();
