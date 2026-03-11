

# Dijkstra's & Greedy Best-First-Search Algorithm
This works by visiting vertices in the graph starting with the object's starting point. It then repeatedly examines the closet not yet examined vertex adding its vertices to the set of vertices examined. It expands outwards from the starting point until it reaches the goal. This algorithm is guaranteed to find the shortest path from the starting point, as long as none of the edges have a negative cost.

The Greedy Best-First-Search algorithm works in a similar way, except that it has some estimate (heuristic) of how far from the goal any vertex is. Instead of selecting the vertex closest to the starting point, it selects the vertex closest to the goal. Greedy Best-First-search is not guaranteed to find a shortest path. However, it runs much quicker than Dijkstra's algorithm because it uses the heuristic function to guide its way towards the goal very quickly. 
# A* Algorithm
A* is like Dijkstra's algorithm in that in can be used to find a shortest path. A* is like the Greed Best First Search Algorithm in that it can use a heuristic to guide itself. It can be just as fast.

## A* Heuristic 

- At one extreme, if `h(n)` is 0, then only `g(n)` plays a role, and A* turns into Dijkstra's algorithm
- If `h(n)` is always lower than or equal to the cost of moving from n to the goal, then A* is guaranteed to find the shortest path. The lower `h(n)` is, the more node a* expands, making it slower.
- If `h(n)` is equal to cost of moving, then A* will only follow the best path and never expand anything else, making it very fast. Although you can't make this happen in all cases, you can make it exact in some special cases, It's nice to know that given perfect information A* will behave perfectly
- If `h(n)` is sometimes greater than the cost of moving, then A* is not guaranteed to find a shortest path, but it can run faster.
- At the other extreme, if `h(n)` is very high relative to `g(n)` then only `h(n)` plays a tole, and A* turns into Greedy Best-First-Search algorithm

[[C++ Mini Project - Code]]
# Templates in C++
A C++ template is a tool for creating generic classes or functions. This allows us to write code that works for any data type without rewriting it for each type.

- Avoid code duplication by allowing one function or class to work with multiple data types, mainly allowing generic functions and classes.
- Provides type safety unlike void* pointers or macros, and can be specialized for specific data types when needed.
- Form the basis of [STL containers](https://www.geeksforgeeks.org/cpp/containers-cpp-stl/) and [algorithms](https://www.geeksforgeeks.org/cpp/c-magicians-stl-algorithms/) like vector, map, and sort.

![[Pasted image 20260204100520.png]]

AStarAlgorithm.h

``` c++
#pragma once
#include <vector>
#include <utility>

struct Node {
    int row, col;
    int g;      // cost from start
    int h;      // heuristic (Manhattan distance to goal)
    int f;      // g + h
    int parentRow, parentCol;

    Node(int r, int c, int g, int h, int pr, int pc)
        : row(r), col(c), g(g), h(h), f(g + h), parentRow(pr), parentCol(pc) {}

    // For priority queue: lower f = higher priority
    bool operator>(const Node& other) const { return f > other.f; }
};

// Returns the path from start to goal as a list of (row, col) pairs.
// Returns empty vector if no path exists.
// grid: 0 = walkable, 1 = wall
std::vector<std::pair<int,int>> astar(
    const std::vector<std::vector<int>>& grid,
    std::pair<int,int> start,
    std::pair<int,int> goal
);

```

grid.h
```c++
#pragma once
#include <vector>
#include <utility>

// Print the grid; marks start (S), goal (G), and path (*) on it
void printGrid(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int,int>>& path,
    std::pair<int,int> start,
    std::pair<int,int> goal
);
```


AStarAlgorithm.cpp
```c++
#include "AStarAlgorithm.h"
#include <queue>          // priority_queue
#include <vector>         // vector
#include <unordered_map>  // unordered_map
#include <cmath>          // abs
#include <algorithm>      // reverse
#include <climits>        // INT_MAX

using namespace std;

// Manhattan distance heuristic
static int heuristic(int r, int c, int gr, int gc) {
    return abs(r - gr) + abs(c - gc);
}

// Encode (row, col) into a single int for fast lookup e.g row 3, col 2 on a 10-wide grid = 32
static int encode(int r, int c, int cols) {
    return r * cols + c;
}

vector<pair<int,int>> astar(
    const vector<vector<int>>& grid,
    pair<int,int> start,
    pair<int,int> goal)
{
    const int rows = grid.size();
    const int cols = grid[0].size();

    auto [sr, sc] = start;
    auto [gr, gc] = goal;

    // Validate start/goal
    if (grid[sr][sc] == 1 || grid[gr][gc] == 1)
        return {};

    // Min-heap: Node with lowest f on top, priority queue give you the smallest element first
    priority_queue<Node, vector<Node>, greater<Node>> openSet;

    // Track best g-cost seen for each cell
    vector<vector<int>> bestG(rows, vector<int>(cols, INT_MAX));
    bestG[sr][sc] = 0;

    // Store parent for path reconstruction: maps encoded pos -> (parentRow, parentCol)
    unordered_map<int, pair<int,int>> cameFrom;

    openSet.emplace(sr, sc, 0, heuristic(sr, sc, gr, gc), -1, -1);

    // 4-directional movement
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        int r = current.row, c = current.col;

        // Skip stale entries
        if (current.g > bestG[r][c])
            continue;

        // Record how we arrived here
        if (current.parentRow != -1) {
            cameFrom[encode(r, c, cols)] = {current.parentRow, current.parentCol};
        }

        // Goal reached — reconstruct path
        if (r == gr && c == gc) {
            vector<pair<int,int>> path;
            int cr = gr, cc = gc;
            while (!(cr == sr && cc == sc)) {
                path.emplace_back(cr, cc);
                auto [pr, pc] = cameFrom[encode(cr, cc, cols)];
                cr = pr;
                cc = pc;
            }
            path.emplace_back(sr, sc);
            reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        for (int d = 0; d < 4; ++d) {
            int nr = r + dr[d];
            int nc = c + dc[d];

            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                continue;
            if (grid[nr][nc] == 1)
                continue;

            int newG = current.g + 1;
            if (newG < bestG[nr][nc]) {
                bestG[nr][nc] = newG;
                int h = heuristic(nr, nc, gr, gc);
                openSet.emplace(nr, nc, newG, h, r, c);
            }
        }
    }

    return {}; // No path found
}
```

- ``heuristic()`` gets the Manhattan distance to the goal
- ``encode() ``turns rows and columns into a single number, for example for a 10 wide grid row 3, col 2 would be turned into 32. for an 11 wide grid row 3, col 2 would be turned into 35.
- line 37: ``priority_queue`` gives you the smallest element first
- line 44: ``unordered_map`` was used instead of ``vector<vector<pair<int, int>>>`` of the same size as the grid, because only visited cells get an entry and it is faster at finding a cell's parent
- ``emplace_back`` and ``push_back`` are similar but ``emplace_back`` is slightly more efficient because it skips the intermediate object creation
-  



# References
https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
http://geeksforgeeks.org/cpp/templates-cpp/
