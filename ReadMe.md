

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

# main.cpp

```c++
#include <iostream>
#include <vector>
#include "AStarAlgorithm.h"
#include "grid.h"
#include "Test.h"

int main() {
	//runAllTests();
    //return 0;
    
    srand(time(0));
    int attempts = 0;
    const int MAX_ATTEMPTS = 10;

    std::pair<int, int> gridSize = { 10, 10 };
    std::pair<int,int> start = {0, 0};
    std::pair<int,int> goal  = {9, 9};
    std::vector<std::pair<int, int>> path;
    std::vector<std::vector<int>> grid;

    do {
        grid = generateGrid(gridSize, start, goal);
        path = astar(grid, start, goal);
    } while (path.empty() && attempts < MAX_ATTEMPTS);

    if (path.empty()) {
        std::cout << "No valid grid found after " << MAX_ATTEMPTS << " attempts.\n";
        return 1;
    }

    else {
        std::cout << "Grid (S=start, G=goal, #=wall, .=open):\n\n";
        printGrid(grid, {}, start, goal);
        std::cout << "\nPath found (" << path.size() << " steps):\n\n";
        printGrid(grid, path, start, goal);
        std::cout << "\nCoordinates:\n";
        for (auto& [r, c] : path) {// ISO C++ 17 standard 
            std::cout << "  (" << r << ", " << c << ")\n";
        }
    }

    return 0;
}
```

- In order for `auto& [r,c] : path` to work I needed to use the ISO C++ 17 standard. The default in visual studio is ISO C++ 14 standard.
# AStarAlgorithm.h

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

# AStarAlgorithm.cpp
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
int heuristic(int r, int c, int gr, int gc) {
    return abs(r - gr) + abs(c - gc);
}

// Encode (row, col) into a single int for fast lookup e.g row 3, col 2 on a 10-wide grid = 32
int encode(int r, int c, int cols) {
    return r * cols + c;
}

// Check that (r,c) is inside the grid bounds
bool in_bounds(int r, int c, int rows, int cols) {
    return r >= 0 && r < rows && c >= 0 && c < cols;
}

// Explore 4-directional neighbors of `current`, updating bestG and pushing new nodes onto openSet
static void explore_neighbors(
    const vector<vector<int>>& grid,
    const Node& current,
    vector<vector<int>>& bestG,
    priority_queue<Node, vector<Node>, greater<Node>>& openSet,
    int rows,
    int cols,
    int gr,
    int gc)
{
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};

    for (int d = 0; d < 4; ++d) {
        int nr = current.row + dr[d];
        int nc = current.col + dc[d];

        if (!in_bounds(nr, nc, rows, cols))
            continue;
        if (grid[nr][nc] == 1)
            continue;

        int newG = current.g + 1;
        if (newG < bestG[nr][nc]) {
            bestG[nr][nc] = newG;
            int h = heuristic(nr, nc, gr, gc);
            openSet.emplace(nr, nc, newG, h, current.row, current.col);
        }
    }
}

// Reconstruct path from start to goal using cameFrom map
static vector<pair<int,int>> reconstruct_path(
    const unordered_map<int, pair<int,int>>& cameFrom,
    pair<int,int> start,
    pair<int,int> goal,
    int cols)
{
    vector<pair<int,int>> path;
    int sr = start.first, sc = start.second;
    int cr = goal.first, cc = goal.second;
    while (!(cr == sr && cc == sc)) {
        path.emplace_back(cr, cc);
        auto it = cameFrom.find(encode(cr, cc, cols));
        if (it == cameFrom.end()) {
            // Should not happen if called correctly, but return empty to be safe
            return {};
        }
        auto [pr, pc] = it->second;
        cr = pr;
        cc = pc;
    }
    path.emplace_back(sr, sc);
    reverse(path.begin(), path.end());
    return path;
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
            return reconstruct_path(cameFrom, start, goal, cols);
        }

        // Explore neighbors of current
        explore_neighbors(grid, current, bestG, openSet, rows, cols, gr, gc);
    }

    return {}; // No path found
}
```

- ``heuristic()`` gets the Manhattan distance to the goal
- ``encode() ``turns rows and columns into a single number, for example for a 10 wide grid row 3, col 2 would be turned into 32. for an 11 wide grid row 3, col 2 would be turned into 35.
- ``priority_queue`` gives you the smallest element first
-  ``unordered_map`` was used instead of ``vector<vector<pair<int, int>>>`` of the same size as the grid, because only visited cells get an entry and it is faster at finding a cell's parent
- ``emplace_back`` and ``push_back`` are similar but ``emplace_back`` is slightly more efficient because it skips the intermediate object creation

# grid.h
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

std::vector<std::vector<int>> generateGrid(std::pair<int, int> gridSize, std::pair<int, int> start, std::pair<int, int> goal);
```
## Grid.cpp
``` c++
std::vector<std::vector<int>> generateGrid(std::pair<int, int> gridSize, std::pair<int, int> start, std::pair<int, int> goal)
{
    int rows = gridSize.first;
    int cols = gridSize.second;

    std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            std::pair<int, int> pos = { r, c };
            if (pos == start || pos == goal)
                grid[r][c] = 0;             // always keep start/goal open
            else
                grid[r][c] = (rand() % 2);  // 0 or 1
        }
    }

    return grid;
}
```
I am generating a grid of size nxn, this function uses rand to generate a random grid of 0's and 1's. Doing it this way makes it very likely that the grid will not have a path. to counter act this I iterate through the grids(max of 10 times) until there is a path found.  

### Printing Grid
```c++
void printGrid(
    const std::vector<std::vector<int>>& grid,
    const std::vector<std::pair<int,int>>& path,
    std::pair<int,int> start,
    std::pair<int,int> goal)
{
    std::set<std::pair<int,int>> pathSet(path.begin(), path.end());
    for (int r = 0; r < (int)grid.size(); ++r) {
        for (int c = 0; c < (int)grid[0].size(); ++c) {
            std::pair<int,int> pos = {r, c};
            const char* token = " . ";
            const char* colorStart = "";
            const char* colorEnd = "\033[0m"; // reset

            if (pos == start) {
                token = " S ";
                colorStart = "\033[32m"; // green
            } else if (pos == goal) {
                token = " G ";
                colorStart = "\033[31m"; // red
            } else if (pathSet.count(pos)) {
                token = " * ";
                colorStart = "\033[33m"; // yellow
            } else if (grid[r][c] == 1) {
                token = "###";
                colorStart = "\033[37m"; // white
            } else {
                token = " . ";
                colorStart = "\033[37m"; // white (dim)
            }

            std::cout << colorStart << token << colorEnd;
        }
        std::cout << "\n";
    }
}
```

- This function prints a 2-D grid to the terminal with color coded symbols representing different cell types. Grid (S=start, G=goal, #=wall, .=open)
- Originally I printed the grid with no additional colors, this worked but it was not as easy to tell what was going on without additional colors.

![[Pasted image 20260318070504.png]]


# Testing

```c++
#include "Test.h"
#include "AStarAlgorithm.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace std;

// ===== Test: heuristic =====
// Tests Manhattan distance calculation
void test_heuristic() {
    // Test 1: Same position - distance should be 0
    assert(heuristic(0, 0, 0, 0) == 0);

    // Test 2: Horizontal distance
    assert(heuristic(0, 0, 0, 5) == 5);

    // Test 3: Vertical distance
    assert(heuristic(0, 0, 5, 0) == 5);

    // Test 4: Diagonal distance
    assert(heuristic(0, 0, 3, 4) == 7); // |3-0| + |4-0| = 7

    // Test 5: Negative coordinates
    assert(heuristic(5, 5, 2, 2) == 6); // |5-2| + |5-2| = 6

    cout << "✓ heuristic tests passed" << endl;
}

// ===== Test: encode =====
// Tests encoding (row, col) into a single int
void test_encode() {
    // Test 1: Basic encoding on a 10-wide grid
    // (3, 2) on 10-wide = 3*10 + 2 = 32
    assert(encode(3, 2, 10) == 32);

    // Test 2: First cell
    assert(encode(0, 0, 10) == 0);

    // Test 3: Different grid width
    assert(encode(2, 3, 5) == 13); // 2*5 + 3 = 13

    // Test 4: Last cell of first row
    assert(encode(0, 9, 10) == 9);

    // Test 5: First cell of second row
    assert(encode(1, 0, 10) == 10);

    cout << "✓ encode tests passed" << endl;
}

// ===== Test: in_bounds =====
// Tests boundary checking
void test_in_bounds() {
    // Test 1: Valid cell
    assert(in_bounds(5, 5, 10, 10) == true);

    // Test 2: Top-left corner
    assert(in_bounds(0, 0, 10, 10) == true);

    // Test 3: Bottom-right corner
    assert(in_bounds(9, 9, 10, 10) == true);

    // Test 4: Out of bounds - negative row
    assert(in_bounds(-1, 5, 10, 10) == false);

    // Test 5: Out of bounds - negative col
    assert(in_bounds(5, -1, 10, 10) == false);

    // Test 6: Out of bounds - row too large
    assert(in_bounds(10, 5, 10, 10) == false);

    // Test 7: Out of bounds - col too large
    assert(in_bounds(5, 10, 10, 10) == false);

    // Test 8: Edge cases
    assert(in_bounds(0, 0, 1, 1) == true);
    assert(in_bounds(1, 1, 1, 1) == false);

    cout << "in_bounds tests passed" << endl;
}

// ===== Test: astar with simple path =====
void test_astar_simple_path() {
    // 3x3 grid with no obstacles
    vector<vector<int>> grid = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };

    auto path = astar(grid, {0, 0}, {2, 2});
    
    // Path should not be empty
    assert(!path.empty());
    
    // Path should start at (0, 0)
    assert(path.front() == make_pair(0, 0));
    
    // Path should end at (2, 2)
    assert(path.back() == make_pair(2, 2));
    
    // Path length should be 5 (diagonal: right, right, down, down or similar)
    // For a 3x3 grid from (0,0) to (2,2), minimum path is 5 cells
    assert(path.size() == 5);

    cout << "astar simple path test passed" << endl;
}

// ===== Test: astar with obstacles =====
void test_astar_with_obstacles() {
    // 5x5 grid with obstacles
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    auto path = astar(grid, {0, 0}, {4, 4});
    
    // Path should exist
    assert(!path.empty());
    
    // Path should start at (0, 0)
    assert(path.front() == make_pair(0, 0));
    
    // Path should end at (4, 4)
    assert(path.back() == make_pair(4, 4));
    
    // All cells in path should be walkable (0)
    for (auto [r, c] : path) {
        assert(grid[r][c] == 0);
    }

    cout << "astar with obstacles test passed" << endl;
}

// ===== Test: astar start is wall =====
void test_astar_start_is_wall() {
    vector<vector<int>> grid = {
        {1, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };

    auto path = astar(grid, {0, 0}, {2, 2});
    
    // Path should be empty (start is a wall)
    assert(path.empty());

    cout << "astar start is wall test passed" << endl;
}

// ===== Test: astar goal is wall =====
void test_astar_goal_is_wall() {
    vector<vector<int>> grid = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 1}
    };

    auto path = astar(grid, {0, 0}, {2, 2});
    
    // Path should be empty (goal is a wall)
    assert(path.empty());

    cout << "astar goal is wall test passed" << endl;
}

// ===== Test: astar no path exists =====
void test_astar_no_path() {
    // 3x3 grid where goal is completely blocked
    vector<vector<int>> grid = {
        {0, 1, 0},
        {1, 1, 1},
        {0, 1, 0}
    };

    auto path = astar(grid, {0, 0}, {2, 2});
    
    // Path should be empty (no route to goal)
    assert(path.empty());

    cout << "astar no path test passed" << endl;
}

// ===== Test: astar start equals goal =====
void test_astar_start_equals_goal() {
    vector<vector<int>> grid = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0}
    };

    auto path = astar(grid, {1, 1}, {1, 1});
    
    // Path should contain just the start/goal cell
    assert(!path.empty());
    assert(path.size() == 1);
    assert(path[0] == make_pair(1, 1));

    cout << "astar start equals goal test passed" << endl;
}

// ===== Test: astar straight line =====
void test_astar_straight_line() {
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0}
    };

    auto path = astar(grid, {0, 0}, {0, 4});
    
    // Path should exist
    assert(!path.empty());
    
    // Path should go straight right
    assert(path.size() == 5); // (0,0), (0,1), (0,2), (0,3), (0,4)
    
    for (int i = 0; i < (int)path.size(); ++i) {
        assert(path[i].first == 0);
        assert(path[i].second == i);
    }

    cout << "astar straight line test passed" << endl;
}

// ===== Test: astar large grid =====
void test_astar_large_grid() {
    // 20x20 empty grid
    vector<vector<int>> grid(20, vector<int>(20, 0));

    auto path = astar(grid, {0, 0}, {19, 19});
    
    // Path should exist
    assert(!path.empty());
    
    // Path should start and end correctly
    assert(path.front() == make_pair(0, 0));
    assert(path.back() == make_pair(19, 19));
    
    // Minimum path length for Manhattan distance 38 is 39 cells
    assert(path.size() == 39);

    cout << "astar large grid test passed" << endl;
}

void runAllTests() {
    cout << "\n========== Running A* Algorithm Tests ==========" << endl;

    try {
        test_heuristic();
        test_encode();
        test_in_bounds();
        test_astar_simple_path();
        test_astar_with_obstacles();
        test_astar_start_is_wall();
        test_astar_goal_is_wall();
        test_astar_no_path();
        test_astar_start_equals_goal();
        test_astar_straight_line();
        test_astar_large_grid();

        cout << "\n========== All tests PASSED! ==========" << endl;
    } catch (const exception& e) {
        cout << "\nTest FAILED with exception: " << e.what() << endl;
    }
}
```

- (explain what each test does)



# References
https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
http://geeksforgeeks.org/cpp/templates-cpp/
