#include "astar.h"

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace {

struct Open {
    double f;   // g + h
    int node;
};

struct OpenGreater {
    bool operator()(const Open& a, const Open& b) const {
        if (a.f != b.f) return a.f > b.f;
        return a.node > b.node;  // deterministic tie-break
    }
};

}  // namespace

std::vector<int> astar(const Graph& g, int start, int goal, const Heuristic& h) {
    if (start == goal) return {start};

    std::unordered_map<int, double> gScore;
    std::unordered_map<int, int> cameFrom;
    std::unordered_set<int> closed;
    std::priority_queue<Open, std::vector<Open>, OpenGreater> open;

    gScore[start] = 0.0;
    open.push({h(start), start});

    while (!open.empty()) {
        const Open cur = open.top();
        open.pop();
        const int u = cur.node;
        if (u == goal) break;
        if (!closed.insert(u).second) continue;  // stale entry from an earlier g-score

        for (const Graph::Edge& e : g.neighbors(u)) {
            const int v = e.to;
            const double tentative = gScore[u] + e.w;
            const auto it = gScore.find(v);
            if (it != gScore.end() && tentative >= it->second) continue;
            gScore[v] = tentative;
            cameFrom[v] = u;
            open.push({tentative + h(v), v});
        }
    }

    if (gScore.find(goal) == gScore.end()) return {};  // unreachable

    std::vector<int> path;
    for (int n = goal;; n = cameFrom.at(n)) {
        path.push_back(n);
        if (n == start) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}
