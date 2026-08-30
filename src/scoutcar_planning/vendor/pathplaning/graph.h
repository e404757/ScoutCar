#pragma once

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Directed graph with per-direction edge blocking.
//
// The base map is built bidirectionally (each undirected edge is added as
// from->to AND to->from), but an obstacle may block ONLY one direction:
//   setBlocked(from, to, true)   removes the edge from->to,
//   while to->from stays traversable.
class Graph {
public:
    struct Edge {
        int to;
        double w;
    };

    // Add a single directed edge from -> to (idempotent).
    void addEdge(int from, int to, double w = 1.0);

    // Add an undirected edge: creates both from->to and to->from.
    void addBidirectional(int from, int to, double w = 1.0);

    // Block/unblock ONLY the directed edge from->to. The reverse edge
    // to->from is unaffected. Throws if the edge does not exist.
    void setBlocked(int from, int to, bool blocked);

    // Is directed edge from->to currently traversable?
    bool canGo(int from, int to) const;

    // Is directed edge from->to currently blocked (only that direction)?
    bool isBlocked(int from, int to) const;

    // Currently traversable outgoing edges of `u`.
    const std::vector<Edge>& neighbors(int u) const;

    // All nodes present in the graph, sorted ascending.
    std::vector<int> nodes() const;

private:
    std::unordered_map<int, std::vector<Edge>> adj_;        // live adjacency
    std::unordered_map<long long, double> edgeWeight_;      // original weight per directed edge
    std::unordered_set<long long> blocked_;                 // currently blocked directed edges

    static long long key(int from, int to) {
        return (static_cast<long long>(from) << 32) | static_cast<unsigned>(to);
    }
};
