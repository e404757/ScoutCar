#include "map.h"

#include <cmath>

// Topology mirrors map.txt:
//
//              [1]             node 1 touches only node 3
//              │
//          [2]-[3]-[4]         row: 2-3-4 ; 2 drops to 6, 4 drops to 7
//           │       │
//      [5]——[6]——[7]—[8]       row3: 5,6,7,8
//       │   │      │   │
//      [9]-[10]--[11]-[12]     row4: 9,10,11,12
//       │   │      │    │
//     [13]-[14]--[15]--[16]    row5: 13,14,15,16
//       │   │      │    │
//     [17]-[18]--[19]--[20]    row6: 17,18,19,20
//
// Every edge has unit weight (相邻格点距离相等).
Graph buildDefaultMap() {
    Graph g;

    // Upper connectors.
    g.addBidirectional(1, 3);
    g.addBidirectional(2, 3);
    g.addBidirectional(2, 6);
    g.addBidirectional(3, 4);
    g.addBidirectional(4, 7);

    // Row 3 and row 4.
    g.addBidirectional(5, 6);
    g.addBidirectional(5, 9);
    g.addBidirectional(6, 7);
    g.addBidirectional(6, 10);
    g.addBidirectional(7, 8);
    g.addBidirectional(7, 11);
    g.addBidirectional(8, 12);

    g.addBidirectional(9, 10);
    g.addBidirectional(9, 13);
    g.addBidirectional(10, 11);
    g.addBidirectional(10, 14);
    g.addBidirectional(11, 12);
    g.addBidirectional(11, 15);
    g.addBidirectional(12, 16);

    // Row 5 and row 6.
    g.addBidirectional(13, 14);
    g.addBidirectional(13, 17);
    g.addBidirectional(14, 15);
    g.addBidirectional(14, 18);
    g.addBidirectional(15, 16);
    g.addBidirectional(15, 19);
    g.addBidirectional(16, 20);

    g.addBidirectional(17, 18);
    g.addBidirectional(18, 19);
    g.addBidirectional(19, 20);

    return g;
}

// Rough (col, row) placement matching the drawing above. Used only to build an
// admissible Euclidean heuristic for A*.
const std::unordered_map<int, std::pair<double, double>>& nodePositions() {
    static const std::unordered_map<int, std::pair<double, double>> pos = {
        {1, {1.5, 0.0}},
        {2, {1.0, 1.0}}, {3, {1.5, 1.0}}, {4, {2.0, 1.0}},
        {5, {0.0, 2.0}}, {6, {1.0, 2.0}}, {7, {2.0, 2.0}}, {8, {3.0, 2.0}},
        {9, {0.0, 3.0}}, {10, {1.0, 3.0}}, {11, {2.0, 3.0}}, {12, {3.0, 3.0}},
        {13, {0.0, 4.0}}, {14, {1.0, 4.0}}, {15, {2.0, 4.0}}, {16, {3.0, 4.0}},
        {17, {0.0, 5.0}}, {18, {1.0, 5.0}}, {19, {2.0, 5.0}}, {20, {3.0, 5.0}},
    };
    return pos;
}
