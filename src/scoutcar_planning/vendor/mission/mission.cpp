#include "mission.h"

#include "navigation.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <cstdio>

namespace mission {

namespace {

constexpr int N = 20;   // 地图 20 节点
constexpr int NH = 4;   // 四种朝向(东/南/西/北,见 pathplan::Heading)
constexpr int S = N * NH;  // 扩展状态数 = 80
const double INF = std::numeric_limits<double>::infinity();

// 扩展状态下标:(node, heading) → [0,80)
int sid(int node, int h) { return (node - 1) * NH + h; }

bool isTunnel(const Edge& e, const std::vector<Edge>& tunnels) {
    return std::find(tunnels.begin(), tunnels.end(), e) != tunnels.end();
}

// 转向代价:直行 0、90° 转弯 turn_penalty、180° 掉头 u_turn_penalty(允许掉头)
double turnCost(pathplan::Heading from, pathplan::Heading to,
                const CostConfig& cfg) {
    switch (pathplan::turnAction(from, to)) {
        case pathplan::TurnAction::STRAIGHT: return 0.0;
        case pathplan::TurnAction::LEFT:
        case pathplan::TurnAction::RIGHT:    return cfg.turn_penalty;
        case pathplan::TurnAction::UTURN:    return cfg.u_turn_penalty;
        default:                             return 0.0;
    }
}

// ── 转弯感知全源最短路(80 状态)+ 派生查表 ──────────────────────────
struct Distances {
    std::vector<std::vector<double>> d80;    // [80][80] 状态间最短路
    std::vector<std::vector<int>>    nxt80;  // [80][80] 状态级 next(-1 不可达)
    std::vector<std::vector<double>> D;      // [N+1][N+1] 节点级最优(任意起止朝向)
    std::vector<std::vector<int>>    hFrom;  // D[u][v] 的最优起始朝向
    std::vector<std::vector<int>>    hTo;    // D[u][v] 的最优到达朝向
    std::vector<std::vector<std::vector<double>>> goCost;  // [N+1][NH][N+1] 从 (p,h) 到 n 的最优代价
    std::vector<std::vector<std::vector<int>>>    goH;     // [N+1][NH][N+1] 对应最优到达朝向
};

// 在含障碍的图上构建转弯感知全源最短路(障碍通过 g.neighbors() 已反映)。
Distances buildDistances(const Graph& g, const std::vector<Edge>& tunnels,
                         const CostConfig& cfg) {
    Distances ds;
    ds.d80.assign(S, std::vector<double>(S, INF));
    ds.nxt80.assign(S, std::vector<int>(S, -1));
    for (int i = 0; i < S; ++i) {
        ds.d80[i][i] = 0.0;
        ds.nxt80[i][i] = i;
    }

    for (int u = 1; u <= N; ++u) {
        for (const auto& e : g.neighbors(u)) {
            const int v = e.to;
            double w = cfg.base;
            if (isTunnel(normEdge(u, v), tunnels)) w += cfg.tunnel_risk;
            const int hE = static_cast<int>(pathplan::edgeHeading(u, v));
            for (int h = 0; h < NH; ++h) {
                const double c = w + turnCost(static_cast<pathplan::Heading>(h),
                                              static_cast<pathplan::Heading>(hE), cfg);
                const int a = sid(u, h), b = sid(v, hE);
                if (c < ds.d80[a][b]) {
                    ds.d80[a][b] = c;
                    ds.nxt80[a][b] = b;
                }
            }
        }
    }

    for (int k = 0; k < S; ++k)
        for (int i = 0; i < S; ++i) {
            if (ds.d80[i][k] >= INF) continue;
            for (int j = 0; j < S; ++j) {
                const double nd = ds.d80[i][k] + ds.d80[k][j];
                if (nd < ds.d80[i][j]) {
                    ds.d80[i][j] = nd;
                    ds.nxt80[i][j] = ds.nxt80[i][k];
                }
            }
        }

    ds.D.assign(N + 1, std::vector<double>(N + 1, INF));
    ds.hFrom.assign(N + 1, std::vector<int>(N + 1, -1));
    ds.hTo.assign(N + 1, std::vector<int>(N + 1, -1));
    for (int u = 1; u <= N; ++u)
        for (int v = 1; v <= N; ++v) {
            if (u == v) {
                ds.D[u][v] = 0.0;
                ds.hFrom[u][v] = ds.hTo[u][v] = -1;
                continue;
            }
            double best = INF;
            int bf = -1, bt = -1;
            for (int h = 0; h < NH; ++h)
                for (int h2 = 0; h2 < NH; ++h2) {
                    const double c = ds.d80[sid(u, h)][sid(v, h2)];
                    if (c < best) { best = c; bf = h; bt = h2; }
                }
            ds.D[u][v] = best;
            ds.hFrom[u][v] = bf;
            ds.hTo[u][v] = bt;
        }

    ds.goCost.assign(N + 1, std::vector<std::vector<double>>(
                                NH, std::vector<double>(N + 1, INF)));
    ds.goH.assign(N + 1, std::vector<std::vector<int>>(
                             NH, std::vector<int>(N + 1, -1)));
    for (int p = 1; p <= N; ++p)
        for (int h = 0; h < NH; ++h)
            for (int n = 1; n <= N; ++n) {
                if (p == n) {
                    ds.goCost[p][h][n] = 0.0;
                    ds.goH[p][h][n] = h;
                    continue;
                }
                double best = INF;
                int bh = -1;
                for (int h2 = 0; h2 < NH; ++h2) {
                    const double c = ds.d80[sid(p, h)][sid(n, h2)];
                    if (c < best) { best = c; bh = h2; }
                }
                ds.goCost[p][h][n] = best;
                ds.goH[p][h][n] = bh;
            }
    return ds;
}

// 状态路径重建:(u,h0) → (v,h1) 的节点序列(含两端);不可达返回空。
std::vector<int> buildStatePath(const Distances& ds, int u, int h0, int v, int h1) {
    std::vector<int> path;
    int cur = sid(u, h0);
    const int goal = sid(v, h1);
    path.push_back(u);
    int guard = 0;
    while (cur != goal && guard++ < S * 2) {
        const int nx = ds.nxt80[cur][goal];
        if (nx < 0) { path.clear(); return path; }
        cur = nx;
        path.push_back(cur / NH + 1);
    }
    return path;
}

// 节点级路径重建(自动选最优起止朝向):u → v。
std::vector<int> buildNodePath(const Distances& ds, int u, int v) {
    if (u == v) return {u};
    const int h0 = ds.hFrom[u][v], h1 = ds.hTo[u][v];
    if (h0 < 0 || h1 < 0) return {};
    return buildStatePath(ds, u, h0, v, h1);
}

// ── 覆盖回路(中国邮路精确解)─────────────────────────────────────
// 输入:需求边集 reqEdges(无向);输出:从 cur 出发、覆盖全部需求边、回 home 的节点序列。
std::vector<int> coveringCircuit(const Graph& g, const Distances& ds,
                                 const std::vector<Edge>& reqEdges,
                                 int cur, int home) {
    if (reqEdges.empty()) return buildNodePath(ds, cur, home);

    // 多重图遍历额度:需求边各 1 次;连通化/匹配附加路径的边累加次数。
    std::map<Edge, int> mult;
    for (const Edge& e : reqEdges) mult[normEdge(e.first, e.second)]++;

    // ── 1) 连通化:需求边诱导子图的连通分量 + 当前位置/家,用全图最短路连接 ──
    {
        std::vector<int> parent(N + 1);
        for (int i = 1; i <= N; ++i) parent[i] = i;
        std::function<int(int)> find = [&](int x) {
            return parent[x] == x ? x : parent[x] = find(parent[x]);
        };
        for (const auto& [e, cnt] : mult)
            if (cnt > 0) parent[find(e.first)] = find(e.second);

        std::vector<std::vector<int>> comps(N + 1);
        for (const auto& [e, cnt] : mult)
            if (cnt > 0) {
                const int r = find(e.first);
                comps[r].push_back(e.first);
                comps[r].push_back(e.second);
            }
        // 当前位置 cur 与家 home 若不在任何需求边端点上,作为独立分量一并连通
        auto hasIncident = [&](int u) {
            for (const auto& [e, cnt] : mult)
                if (cnt > 0 && (e.first == u || e.second == u)) return true;
            return false;
        };
        if (!hasIncident(cur)) comps[find(cur)].push_back(cur);
        if (!hasIncident(home)) comps[find(home)].push_back(home);
        for (auto& c : comps) {
            std::sort(c.begin(), c.end());
            c.erase(std::unique(c.begin(), c.end()), c.end());
        }

        while (true) {
            int ra = -1, rb = -1, ba = 0, bb = 0;
            double best = INF;
            for (int a = 1; a <= N; ++a) {
                if (comps[a].empty()) continue;
                for (int b = a + 1; b <= N; ++b) {
                    if (comps[b].empty()) continue;
                    for (int u : comps[a])
                        for (int v : comps[b]) {
                            if (ds.D[u][v] < best) {
                                best = ds.D[u][v];
                                ra = a; rb = b; ba = u; bb = v;
                            }
                        }
                }
            }
            if (ra < 0) break;  // 只剩一个分量
            // 连接路径并入 mult(重复走)
            const auto p = buildNodePath(ds, ba, bb);
            for (size_t k = 0; k + 1 < p.size(); ++k)
                mult[normEdge(p[k], p[k + 1])]++;
            // 合并分量
            comps[ra].insert(comps[ra].end(), comps[rb].begin(), comps[rb].end());
            comps[rb].clear();
        }
    }

    // ── 2) 方向感知平衡 + 双向边定向 + 有向欧拉路径 ──
    // 有向欧拉路径条件:每个节点 出度=入度,除起点 cur(出=入+1)与终点 home(入=出+1)。
    // need[v] = 双向边还需贡献的 (出-入):目标 0;初始 = target[v] - 固定边贡献。
    //   target[cur]=+1(起点多一出)、target[home]=-1(终点多一入)、其余 0。
    // 单方向边固定定向(need[u]-1、need[v]+1,即 u 已有一出、v 已有一入);
    // 双向边定向 a→b 贡献 need[a]-1、need[b]+1;
    // 主循环把 need>0 的节点(还需出边)沿未定向边路径定向到 need<0 的节点(还需入边)。
    {
        std::vector<int> need(N + 1, 0);
        std::vector<std::vector<int>> dirOut(N + 1);
        std::vector<std::pair<int, int>> undirEdges;  // 双向可走、尚未定向的边
        std::vector<std::vector<int>> undirAdj(N + 1);
        std::vector<char> used;                       // 与 undirEdges 同步(见 addUndir)

        auto addUndir = [&](int u, int v) {
            const int id = static_cast<int>(undirEdges.size());
            undirEdges.push_back({u, v});
            undirAdj[u].push_back(id);
            undirAdj[v].push_back(id);
            used.push_back(0);
        };
        auto addDir = [&](int u, int v) {   // 定向 u→v,更新 need
            dirOut[u].push_back(v);
            need[u] -= 1;
            need[v] += 1;
        };
        // 附加路径段:按方向可用性加入(双向→待定向;单方向→直接定向)
        auto addPathSeg = [&](int u, int v) {
            const bool ab = g.canGo(u, v), ba = g.canGo(v, u);
            if (ab && ba) addUndir(u, v);
            else if (ab) addDir(u, v);
            else if (ba) addDir(v, u);
        };

        // 遍历 mult 的每个遍历单位,按方向可用性分类
        for (const auto& [e, cnt] : mult) {
            if (cnt <= 0) continue;
            const bool ab = g.canGo(e.first, e.second);
            const bool ba = g.canGo(e.second, e.first);
            for (int k = 0; k < cnt; ++k) {
                if (ab && ba) {
                    addUndir(e.first, e.second);
                } else if (ab) {       // 仅 e.first→e.second
                    addDir(e.first, e.second);
                } else if (ba) {       // 仅 e.second→e.first
                    addDir(e.second, e.first);
                } else {
                    std::fprintf(stderr, "[规划] 警告:需求边 %d-%d 双向不可走,跳过\n",
                                 e.first, e.second);
                }
            }
        }
        if (cur != home) { need[cur] += 1; need[home] -= 1; }

        // 外层循环:主循环(need 归零)→ 回路定向(need 全 0 时处理剩余双向边)。
        // 回路定向的附加边若引入单方向边(need 变脏),回到主循环重新平衡。
        int guardOuter = 0;
        while (true) {
            if (++guardOuter > 400) {
                std::fprintf(stderr, "[规划] 警告:定向平衡迭代过多,强制退出\n");
                break;
            }

            // ── 主循环:need>0 节点(还需出边)沿未定向边路径,把亏欠传给 need<0 节点 ──
            while (true) {
                int n = -1;
                for (int u = 1; u <= N; ++u)
                    if (need[u] > 0) { n = u; break; }
                if (n < 0) break;   // need 全 0 → 回路定向

                // 从 n 沿未定向边 BFS,找 need<0 的节点 p
                std::vector<int> par(N + 1, -1);
                std::queue<int> q;
                q.push(n); par[n] = n;
                int p = -1;
                while (!q.empty() && p < 0) {
                    const int u = q.front(); q.pop();
                    for (int id : undirAdj[u]) {
                        if (used[id]) continue;
                        const int w = (undirEdges[id].first == u) ? undirEdges[id].second
                                                                  : undirEdges[id].first;
                        if (par[w] >= 0) continue;
                        par[w] = u;
                        if (need[w] < 0) { p = w; break; }
                        q.push(w);
                    }
                }
                if (p < 0) {
                    // 未定向子图不可达负节点:加附加边(最短路连到最近的 need<0 节点)
                    double best = INF;
                    int bestP = -1;
                    for (int u = 1; u <= N; ++u)
                        if (need[u] < 0 && ds.D[n][u] < best) { best = ds.D[n][u]; bestP = u; }
                    if (bestP < 0) break;  // 无法平衡(理论不应发生)
                    const auto path = buildNodePath(ds, n, bestP);
                    for (size_t i = 0; i + 1 < path.size(); ++i) addPathSeg(path[i], path[i + 1]);
                    continue;
                }
                // 定向路径 n→…→p(每条边沿 n→p 方向:n 获得出边、p 获得入边)
                std::vector<int> path;
                for (int u = p; u != n; u = par[u]) path.push_back(u);
                path.push_back(n);
                std::reverse(path.begin(), path.end());  // [n, ..., p]
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    const int a = path[i], b = path[i + 1];
                    for (int id : undirAdj[a]) {
                        if (used[id]) continue;
                        const int w = (undirEdges[id].first == a) ? undirEdges[id].second
                                                                  : undirEdges[id].first;
                        if (w == b) {
                            used[id] = 1;
                            addDir(a, b);
                            break;
                        }
                    }
                }
            }

            // ── 回路定向:need 全 0,剩余未定向双向边按"欧拉回路"定向 ──
            int remain = 0;
            for (char c : used) remain += !c;
            if (remain == 0) break;

            // 未定向子图奇度点 → D 最小完美匹配,附加边并入(可能引入单方向边 → 重来)
            std::vector<int> deg2(N + 1, 0);
            for (size_t i = 0; i < undirEdges.size(); ++i)
                if (!used[i]) {
                    deg2[undirEdges[i].first]++;
                    deg2[undirEdges[i].second]++;
                }
            std::vector<int> odd2;
            for (int u = 1; u <= N; ++u)
                if (deg2[u] % 2) odd2.push_back(u);

            if (!odd2.empty()) {
                const int m = static_cast<int>(odd2.size());
                const int full = (1 << m) - 1;
                std::vector<double> dp(static_cast<size_t>(1) << m, INF);
                std::vector<int> par2(static_cast<size_t>(1) << m, -1);
                dp[0] = 0.0;
                for (int mask = 0; mask <= full; ++mask) {
                    if (dp[mask] >= INF) continue;
                    int i = -1;
                    for (int b = 0; b < m; ++b)
                        if (!(mask >> b & 1)) { i = b; break; }
                    if (i < 0) continue;
                    for (int j = i + 1; j < m; ++j) {
                        if (mask >> j & 1) continue;
                        const int nmask = mask | (1 << i) | (1 << j);
                        const double c = dp[mask] + ds.D[odd2[i]][odd2[j]];
                        if (c < dp[nmask]) { dp[nmask] = c; par2[nmask] = mask; }
                    }
                }
                int mask = full;
                while (mask) {
                    const int pm = par2[mask];
                    if (pm < 0) break;
                    const int diff = mask ^ pm;
                    const int i = __builtin_ctz(static_cast<unsigned>(diff));
                    const int j = __builtin_ctz(static_cast<unsigned>(diff & ~(1 << i)));
                    const auto path = buildNodePath(ds, odd2[i], odd2[j]);
                    for (size_t k = 0; k + 1 < path.size(); ++k) addPathSeg(path[k], path[k + 1]);
                    mask = pm;
                }
                continue;   // 附加边可能改变 need → 外层循环重来
            }

            // 全偶度:DFS 沿未定向边走回路,每条边定向"遍历方向"(need 保持不变)
            std::function<void(int)> orientDFS = [&](int v) {
                for (int id : undirAdj[v]) {
                    if (used[id]) continue;
                    used[id] = 1;
                    const int w = (undirEdges[id].first == v) ? undirEdges[id].second
                                                              : undirEdges[id].first;
                    dirOut[v].push_back(w);
                    orientDFS(w);
                }
            };
            for (int v = 1; v <= N; ++v) orientDFS(v);
        }

        // ── 3) 有向 Hierholzer:从 cur 出发,到 home 结束 ──
        struct DEdge { int to; int eid; };
        std::vector<std::vector<DEdge>> outAdj(N + 1);
        int total = 0;
        for (int u = 1; u <= N; ++u)
            for (int w : dirOut[u]) outAdj[u].push_back({w, total++});
        std::vector<int> rem(total, 1);

        std::vector<int> circuit;
        // 有向 Hierholzer:直行优先 + 回走(立即原路返回)最后选,减少掉头
        std::function<void(int, int)> hierholzer = [&](int v, int hin) {
            while (true) {
                int pick = -1;
                // 1) 直行优先(前进方向不变)
                if (hin >= 0)
                    for (const auto& e : outAdj[v])
                        if (rem[e.eid] &&
                            static_cast<int>(pathplan::edgeHeading(v, e.to)) == hin) {
                            pick = e.eid; break;
                        }
                // 2) 非回走(避免 180° 掉头)
                if (pick < 0)
                    for (const auto& e : outAdj[v]) {
                        if (!rem[e.eid]) continue;
                        const int hOut = static_cast<int>(pathplan::edgeHeading(v, e.to));
                        if (hin < 0 || hOut != (hin + 2) % NH) { pick = e.eid; break; }
                    }
                // 3) 回走(无其他选择时)
                if (pick < 0)
                    for (const auto& e : outAdj[v])
                        if (rem[e.eid]) { pick = e.eid; break; }
                if (pick < 0) break;
                rem[pick] = 0;
                int w = -1;
                for (const auto& e : outAdj[v])
                    if (e.eid == pick) { w = e.to; break; }
                hierholzer(w, static_cast<int>(pathplan::edgeHeading(v, w)));
            }
            circuit.push_back(v);
        };
        hierholzer(cur, -1);
        std::reverse(circuit.begin(), circuit.end());
        if (circuit.empty() || circuit.front() != cur) {
            // 兜底:理论上不会发生(平衡 + 弱连通保证存在);退化走最短路
            return buildNodePath(ds, cur, home);
        }
        return circuit;
    }
}

// ── 收尾 DP(侦查点找齐后):剩余固定点 + 剩余隧道,精确最短闭环回家 ──
std::vector<int> finishDP(const Graph& g, const Distances& ds,
                          const MissionState& st, const CostConfig& cfg) {
    const int home = cfg.home;
    struct Item { bool edge; int a, b; };
    std::vector<Item> items;
    for (int n : st.fixed_left) items.push_back({false, n, n});
    for (const Edge& t : st.tunnel_left) items.push_back({true, t.first, t.second});
    // 剔除从当前位置不可达的需求(物理不可达:入边全被障碍阻断等),
    // 避免整次规划失败卡死任务——其余需求照常完成,丢分的是被堵死的那部分。
    {
        std::vector<Item> reachable;
        for (const Item& it : items) {
            if (!it.edge) {
                if (ds.D[st.current][it.a] < INF) {
                    reachable.push_back(it);
                } else {
                    std::fprintf(stderr, "[规划] 警告:固定点 %d 从 %d 不可达,跳过\n",
                                 it.a, st.current);
                }
            } else {
                const bool ok = (ds.D[st.current][it.a] < INF && g.canGo(it.a, it.b))
                             || (ds.D[st.current][it.b] < INF && g.canGo(it.b, it.a));
                if (ok) {
                    reachable.push_back(it);
                } else {
                    std::fprintf(stderr, "[规划] 警告:隧道 %d-%d 不可达/双向阻断,跳过\n",
                                 it.a, it.b);
                }
            }
        }
        items = std::move(reachable);
    }
    const int M = static_cast<int>(items.size());
    if (M == 0) return buildNodePath(ds, st.current, home);

    const int full = (1 << M) - 1;
    const float FINF = std::numeric_limits<float>::infinity();
    std::vector<float> dp(static_cast<size_t>(1 << M) * (N + 1) * NH, FINF);
    std::vector<uint32_t> par(static_cast<size_t>(1 << M) * (N + 1) * NH, 0);
    auto idx = [&](int mask, int p, int h) {
        return (static_cast<size_t>(mask) * (N + 1) + p) * NH + h;
    };
    for (int h = 0; h < NH; ++h) dp[idx(0, st.current, h)] = 0.0f;

    for (int mask = 0; mask <= full; ++mask) {
        for (int p = 1; p <= N; ++p) {
            for (int h = 0; h < NH; ++h) {
                const float cur = dp[idx(mask, p, h)];
                if (cur >= FINF) continue;
                for (int rem = full ^ mask; rem; rem &= rem - 1) {
                    const int k = __builtin_ctz(static_cast<unsigned>(rem));
                    const int nmask = mask | (1 << k);
                    if (!items[k].edge) {
                        const int n = items[k].a;
                        const double c = ds.goCost[p][h][n];
                        if (c >= INF) continue;
                        const int nh = ds.goH[p][h][n];
                        const float nd = cur + static_cast<float>(c);
                        if (nd < dp[idx(nmask, n, nh)]) {
                            dp[idx(nmask, n, nh)] = nd;
                            par[idx(nmask, n, nh)] =
                                (static_cast<uint32_t>(mask) << 8)
                                | (static_cast<uint32_t>(p) << 3)
                                | static_cast<uint32_t>(h);
                        }
                    } else {
                        const int a = items[k].a, b = items[k].b;
                        const Edge te = normEdge(a, b);
                        // 方向 a→b:先到 a(枚举到达朝向 hA),再穿越到 b
                        {
                            const int hEdge = static_cast<int>(pathplan::edgeHeading(a, b));
                            if (g.canGo(a, b)) {
                                const double w = cfg.base +
                                    (isTunnel(te, st.tunnel_left) ? cfg.tunnel_risk : 0.0);
                                for (int hA = 0; hA < NH; ++hA) {
                                    const double cToA = ds.d80[sid(p, h)][sid(a, hA)];
                                    if (cToA >= INF) continue;
                                    const double cCross = w + turnCost(
                                        static_cast<pathplan::Heading>(hA),
                                        static_cast<pathplan::Heading>(hEdge), cfg);
                                    const float nd = cur + static_cast<float>(cToA + cCross);
                                    if (nd < dp[idx(nmask, b, hEdge)]) {
                                        dp[idx(nmask, b, hEdge)] = nd;
                                        par[idx(nmask, b, hEdge)] =
                                            (static_cast<uint32_t>(mask) << 8)
                                            | (static_cast<uint32_t>(p) << 3)
                                            | static_cast<uint32_t>(h);
                                    }
                                }
                            }
                        }
                        // 方向 b→a
                        {
                            const int hEdge = static_cast<int>(pathplan::edgeHeading(b, a));
                            if (g.canGo(b, a)) {
                                const double w = cfg.base +
                                    (isTunnel(te, st.tunnel_left) ? cfg.tunnel_risk : 0.0);
                                for (int hB = 0; hB < NH; ++hB) {
                                    const double cToB = ds.d80[sid(p, h)][sid(b, hB)];
                                    if (cToB >= INF) continue;
                                    const double cCross = w + turnCost(
                                        static_cast<pathplan::Heading>(hB),
                                        static_cast<pathplan::Heading>(hEdge), cfg);
                                    const float nd = cur + static_cast<float>(cToB + cCross);
                                    if (nd < dp[idx(nmask, a, hEdge)]) {
                                        dp[idx(nmask, a, hEdge)] = nd;
                                        par[idx(nmask, a, hEdge)] =
                                            (static_cast<uint32_t>(mask) << 8)
                                            | (static_cast<uint32_t>(p) << 3)
                                            | static_cast<uint32_t>(h);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 最优终点:覆盖全部需求后离 home 最近
    int bestP = -1, bestH = -1;
    double best = INF;
    for (int p = 1; p <= N; ++p)
        for (int h = 0; h < NH; ++h) {
            const double v = dp[idx(full, p, h)];
            if (v >= INF) continue;
            const double c = ds.goCost[p][h][home];
            if (c >= INF) continue;
            if (v + c < best) { best = v + c; bestP = p; bestH = h; }
        }
    if (bestP < 0) return {};  // 不可达

    // 回溯完成顺序
    struct StepItem { int k, endpos, arriveH; };
    std::vector<StepItem> orderRev;
    int startH = 0;
    int mask = full, p = bestP, h = bestH;
    while (mask != 0) {
        const uint32_t packed = par[idx(mask, p, h)];
        const int pm = static_cast<int>(packed >> 8);
        const int pp = static_cast<int>((packed >> 3) & 31);
        const int ph = static_cast<int>(packed & 7);
        const int k = __builtin_ctz(static_cast<unsigned>(mask ^ pm));
        orderRev.push_back({k, p, h});
        if (pm == 0) startH = ph;  // 第一段的起始朝向
        mask = pm; p = pp; h = ph;
    }
    std::reverse(orderRev.begin(), orderRev.end());

    // 生成节点路径
    std::vector<int> route;
    route.push_back(st.current);
    int pos = st.current;
    int curH = startH;
    for (const auto& si : orderRev) {
        const Item& it = items[si.k];
        if (!it.edge) {
            const int n = it.a;
            const auto seg = buildStatePath(ds, pos, curH, n, si.arriveH);
            if (seg.empty()) return {};
            route.insert(route.end(), seg.begin() + 1, seg.end());
            pos = n;
            curH = si.arriveH;
        } else if (si.endpos == it.b) {  // 穿越 a→b
            const int hEdge = static_cast<int>(pathplan::edgeHeading(it.a, it.b));
            // 重算最优到达朝向(与 DP 转移一致:min over hA)
            int bestHA = -1;
            double bestC = INF;
            for (int hA = 0; hA < NH; ++hA) {
                const double c = ds.d80[sid(pos, curH)][sid(it.a, hA)] +
                                 turnCost(static_cast<pathplan::Heading>(hA),
                                          static_cast<pathplan::Heading>(hEdge), cfg);
                if (c < bestC) { bestC = c; bestHA = hA; }
            }
            if (bestHA < 0) return {};
            const auto seg = buildStatePath(ds, pos, curH, it.a, bestHA);
            if (seg.empty()) return {};
            route.insert(route.end(), seg.begin() + 1, seg.end());
            route.push_back(it.b);
            pos = it.b;
            curH = hEdge;
        } else {  // 穿越 b→a
            const int hEdge = static_cast<int>(pathplan::edgeHeading(it.b, it.a));
            int bestHB = -1;
            double bestC = INF;
            for (int hB = 0; hB < NH; ++hB) {
                const double c = ds.d80[sid(pos, curH)][sid(it.b, hB)] +
                                 turnCost(static_cast<pathplan::Heading>(hB),
                                          static_cast<pathplan::Heading>(hEdge), cfg);
                if (c < bestC) { bestC = c; bestHB = hB; }
            }
            if (bestHB < 0) return {};
            const auto seg = buildStatePath(ds, pos, curH, it.b, bestHB);
            if (seg.empty()) return {};
            route.insert(route.end(), seg.begin() + 1, seg.end());
            route.push_back(it.a);
            pos = it.a;
            curH = hEdge;
        }
    }

    // 回家
    {
        const int hGoal = ds.goH[pos][curH][home];
        if (hGoal < 0) return {};
        const auto seg = buildStatePath(ds, pos, curH, home, hGoal);
        if (seg.empty()) return {};
        route.insert(route.end(), seg.begin() + 1, seg.end());
    }
    return route;
}

}  // namespace

Edge normEdge(int a, int b) {
    if (a > b) std::swap(a, b);
    return {a, b};
}

MissionPlanner::MissionPlanner(const Graph& g, const CostConfig& cfg)
    : g_(g), cfg_(cfg) {
    tunnels_ = { {6, 7}, {10, 11}, {14, 15}, {18, 19} };
    for (auto& t : tunnels_) t = normEdge(t.first, t.second);

    nodes_ = g_.nodes();
    for (int u : nodes_) {
        for (const auto& e : g_.neighbors(u)) {
            if (u < e.to) edges_.insert(normEdge(u, e.to));
        }
    }
}

std::vector<int> MissionPlanner::plan(const MissionState& st) {
    const Distances ds = buildDistances(g_, tunnels_, cfg_);
    if (needSearch(st, cfg_)) {
        // 侦查点未找齐 → 覆盖回路:需求边 = 未搜索非隧道边 ∪ 剩余隧道
        std::vector<Edge> req;
        for (const Edge& e : edges_)
            if (!isTunnel(e, tunnels_) && !st.searched.count(e)) req.push_back(e);
        for (const Edge& t : st.tunnel_left) req.push_back(t);
        return coveringCircuit(g_, ds, req, st.current, cfg_.home);
    }
    // 侦查点找齐 → 收尾 DP
    return finishDP(g_, ds, st, cfg_);
}

}  // namespace mission
