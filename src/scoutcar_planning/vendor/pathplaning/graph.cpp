#include "graph.h"

#include <algorithm>
#include <stdexcept>

void Graph::addEdge(int from, int to, double w) {
    auto& list = adj_[from];
    list.erase(std::remove_if(list.begin(), list.end(),
                              [to](const Edge& e) { return e.to == to; }),
               list.end());
    list.push_back({to, w});
    edgeWeight_[key(from, to)] = w;
    blocked_.erase(key(from, to));  // a freshly added edge is unblocked
    adj_[to];                       // ensure `to` is a known node even if only inbound
}

void Graph::addBidirectional(int from, int to, double w) {
    addEdge(from, to, w);
    addEdge(to, from, w);
}

void Graph::setBlocked(int from, int to, bool blocked) {
    const long long k = key(from, to);
    if (edgeWeight_.find(k) == edgeWeight_.end()) {
        throw std::invalid_argument("graph.cpp: edge " + std::to_string(from) +
                                    "->" + std::to_string(to) + " does not exist");
    }
    auto& list = adj_[from];
    if (blocked) {
        if (blocked_.insert(k).second) {
            list.erase(std::remove_if(list.begin(), list.end(),
                                      [to](const Edge& e) { return e.to == to; }),
                       list.end());
        }
    } else {
        if (blocked_.erase(k) > 0) {
            list.push_back({to, edgeWeight_[k]});
        }
    }
}

bool Graph::canGo(int from, int to) const {
    const auto it = adj_.find(from);
    if (it == adj_.end()) return false;
    for (const Edge& e : it->second) {
        if (e.to == to) return true;
    }
    return false;
}

bool Graph::isBlocked(int from, int to) const {
    return blocked_.count(key(from, to)) > 0;
}

const std::vector<Graph::Edge>& Graph::neighbors(int u) const {
    static const std::vector<Edge> kEmpty;
    const auto it = adj_.find(u);
    return it == adj_.end() ? kEmpty : it->second;
}

std::vector<int> Graph::nodes() const {
    std::vector<int> result;
    result.reserve(adj_.size());
    for (const auto& [n, _] : adj_) result.push_back(n);
    std::sort(result.begin(), result.end());
    return result;
}
