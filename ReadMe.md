# A* Algorithm Theory

## Background

Before explaining how the A* algorithm works, it is helpful to understand two simpler algorithms that it builds upon: Dijkstra's algorithm and Greedy Best-First Search.

## Dijkstra's Algorithm

Dijkstra's algorithm works by visiting nodes in a graph starting from the starting node. It repeatedly examines the closest not yet visited node, adding it to the set of visited nodes. It expands outwards from the starting point in all directions until it reaches the goal. This algorithm is guaranteed to find the shortest path from the starting node, as long as none of the edges have a negative cost.

## Greedy Best-First Search

Greedy Best-First Search works in a similar way to Dijkstra's, except that it uses a heuristic; an estimate of how far any given node is from the goal. Instead of selecting the node closest to the start, it selects the node that appears closest to the goal. Greedy Best-First Search is not guaranteed to find the shortest path, however it runs much faster than Dijkstra's algorithm because the heuristic guides it towards the goal more directly.

## A* Algorithm

A* combines the strengths of both algorithms. Like Dijkstra's, it can find the shortest path. Like Greedy Best-First Search, it uses a heuristic to guide itself towards the goal efficiently. When used with a good heuristic, A* can be just as fast as Greedy Best-First Search while still guaranteeing the shortest path.

A* assigns every node a score:

$$f(n) = g(n) + h(n)$$

- **g(n)**: the actual cost to reach node `n` from the start
- **h(n)**: the heuristic estimate of the cost from node `n` to the goal
- **f(n)**: the total estimated cost of the path through node `n`

The algorithm always expands the node with the lowest `f(n)` first, which is why a min-heap priority queue is used.

## Heuristics

A heuristic estimates how far a node is from the goal. The true shortest distance cannot be known without finding the path first, so instead a value that is easy to compute is used, such as Manhattan distance.

The behavior of A* depends heavily on the heuristic chosen:

- If `h(n) = 0`, A* behaves identically to Dijkstra's algorithm, expanding in all directions with no guidance.
- If `h(n)` is always less than or equal to the true cost, A* is guaranteed to find the shortest path. The lower `h(n)` is, the more nodes are expanded, making it slower.
- If `h(n)` equals the true cost exactly, A* only follows the optimal path and never expands unnecessary nodes, making it as fast as possible.
- If `h(n)` is sometimes greater than the true cost, A* may not find the shortest path, but it will run faster.
- If `h(n)` is very large relative to `g(n)`, A* behaves like Greedy Best-First Search.


## Manhattan Distance

Manhattan distance is commonly used as a heuristic for grid-based pathfinding. It measures the distance between two points by only moving along grid axes — no diagonals. The formula is:

$$d = |x_1 - x_2| + |y_1 - y_2|$$

For example, moving from cell `(1, 1)` to cell `(3, 4)` on a grid requires moving 2 steps down and 3 steps right, giving a Manhattan distance of 5. This maps naturally to grid movement where only up, down, left, and right are allowed.

# My Code

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
    //srand(time(0));
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

- I am using a random number generator that chooses between 0 and 1 for creating a grid, '0' is open and '1' is a wall. This causes some issues, there will often be no path from start to end. To mitigate this issue, I have included a do-while loop that will generate the grid and try find a path between the start and end. The loop will do this 10 times or until a path is found. This makes the chances of no path being found smaller because we essentially have 10 grids, making it very likely for there to be a path in 1 of them. 
- The reason I chose to do this is because I wanted to see the A* algorithm working during testing.
- In order for `auto& [r, c] : path` to work I needed to use the C++ 17. Structured binding does not exist in version 14. This was introduced in C++ 17. In Visual Studio the default version is 14.
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

- In the node struct 'g', 'h' and 'f' are stored separately to avoid recalculating 'f' every time a node is compared.
- The Node struct stores `parentRow` and `parentCol`, which record the coordinates of the cell that the algorithm came from when it first reach this node. Once the goal is reached, the parent fields are used to trace backwards from the goal to the start. Each node points to the cell it came from, which in turn points to its own parent. This chain is reversed to produce the final path. Without parent tracking you would have no way of knowing which cells the path passes through.
- `std::priority_queue` uses `<` by default to make a max-heap, but by overloading `>` and passing `std::greater<Node>`, you get a min-heap so the lowest `f` node is always processed first.
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

- ``heuristic()`` gets the Manhattan distance to the goal.
- ``encode() ``turns rows and columns into a single number, for example for a 10 wide grid row 3, col 2 would be turned into 32. for an 11 wide grid row 3, col 2 would be turned into 35.
- ``priority_queue`` gives you the smallest element first.
-  ``unordered_map`` was used instead of ``vector<vector<pair<int, int>>>`` of the same size as the grid, because only visited cells get an entry and it is faster at finding a cell's parent
- ``emplace_back`` and ``push_back`` are similar but ``emplace_back`` is slightly more efficient because it skips the intermediate object creation
- The `bestG` grid tracks the best known cost to reach each cell. the priority queue can hold multiple entries for the same cell, so when a node is popped its 'g' value is checked against `bestG`. If it is higher, the entry is skipped. This avoids extra entries in the queue.
- Instead of writing four separate if-statements for up, down, left and right, two small direction arrays `dr` and `dc` are used and looped over in `explore_neighbors`.
- Every cell in `bestG` is initialized to `INT_MAX`, representing infinity, meaning no route has been found to that cell yet. Any real path cost will always be lower, so the first time a cell is reached its cost will always be recorded.

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
- `generateGrid` takes `gridSize`, `start`, and `goal` as `std::pair<int, int>`. Using a pair to represent a 2D coordinate keeps related values together and avoids having separate `rows` and `cols` parameters.
- `generateGrid` handles creating the grid and `printGrid` handles displaying it.
## Grid.cpp

### Generating Grid
``` c++
#include "grid.h"
#include <iostream>
#include <set>
#include <random>

std::vector<std::vector<int>> generateGrid(std::pair<int, int> gridSize, std::pair<int, int> start, std::pair<int, int> goal)
{

    int rows = gridSize.first;
    int cols = gridSize.second;
    std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0));

    std::mt19937 rng(std::random_device{}());
    std::bernoulli_distribution dist(0.5);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            std::pair<int, int> pos = { r, c };
            if (pos == start || pos == goal)
                grid[r][c] = 0;             // always keep start/goal open
            else
                grid[r][c] = (dist(rng) ? 1 : 0);  // 0 or 1
        }
    }

    return grid;
}
```

- To generate a grid of size nxn, this function uses `std::mt19937` to fill the grid with random 0's and 1's. Doing it this way makes it very likely that the grid will not have a path. to counter act this I iterate through the grids(max of 10 times) until there is a path found.  
- I initially used `rand()` but this was replaced with `std::mt19937`, a modern random number generator that produces much higher quality randomness. `rand()` is considered outdated in modern C++ because it has a short cycle and poor distribution on some compilers.
- The start and goal cells are explicitly forced to 0 regardless of what `std::mt19937` produces. This guarantees A* always has a valid cell to begin and end on, preventing a situation where the algorithm immediately returns empty because the start or goal is a wall.


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
- ANSI escape codes are used to add colour to the terminal output. Each cell type has its own colour. For example green for start, red for goal, yellow for path cells, and white for walls and open cells. This makes the grid much easier to read at a glance.
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

For Testing the A* algorithm, I wanted to test all the functions/functionality.
## Helper Function Tests

### test_heuristic

- Verifies the Manhattan distance calculation.
- Checks zero distance when start equals goal, pure horizontal/vertical distances. To check absolute values are used correctly.

| Input          | Expected |
| -------------- | -------- |
| (0,0) -> (0,0) | 0        |
| (0,0) -> (0,5) | 5        |
| (0,0) -> (5,0) | 5        |
| (0,0) -> (3,4) | 7        |
| (5,5) -> (2,2  | 6        |

### test_encode

- Tests the function that flattens a 2D `(row, col)` position into a single integer using $row * width + col$.
- Confirms the formula works for the first cell, the last cell of a row, the first cell of the next row, and different grid widths.

| Input            | Expected |
| ---------------- | -------- |
| encode(3, 2, 10) | 32       |
| encode(0, 0, 10) | 0        |
| encode(2, 3, 5)  | 13       |
| encode(0, 9, 10) | 9        |
| encode(1, 0, 10) | 10       |

### test_in_bounds

- Checks boundary validation. Covers valid cells including corners, and all four out-of-bounds cases: negative row, negative column, row equal to height, and column equal to width. Also tests a 1×1 grid where only `(0,0)` is valid.

| Input           | Expected |
| --------------- | -------- |
| (5,5) on 10×10  | true     |
| (0,0) on 10×10  | true     |
| (9,9) on 10×10  | true     |
| (-1,5) on 10×10 | false    |
| (5,-1) on 10×10 | false    |
| (10,5) on 10×10 | false    |
| (5,10) on 10×10 | false    |
| (0,0) on 1×1    | true     |
| (1,1) on 1×1    | false    |

## A* Pathfinding Tests

### test_astar_simple_path

Runs A* on a clear 3×3 grid from (0,0) to (2,2). Confirms the path is non-empty, starts and ends at the correct cells, and has exactly 5 cells — the minimum for a Manhattan path across a 3×3 grid.

```
0 0 0
0 0 0
0 0 0
```

**Expected:** path of length 5, from (0,0) to (2,2).

### test_astar_with_obstacles

Runs A* on a 5×5 grid with two horizontal walls blocking direct routes. Confirms a path is still found, starts and ends correctly, and every cell along the path is walkable (value 0).

```
0 0 0 0 0
0 1 1 1 0
0 0 0 0 0
0 1 1 1 0
0 0 0 0 0
```

**Expected:** valid path from (0,0) to (4,4) through walkable cells only.

### test_astar_start_is_wall

Passes a grid where the start cell (0,0) is a wall. Expects an empty path, A* should bail out immediately rather than trying to navigate from an impassable cell.

```
1 0 0
0 0 0
0 0 0
```

**Expected:** empty path.

### test_astar_goal_is_wall

The goal cell (2,2) is a wall. Expects an empty path, there is no point navigating toward a destination that can never be reached.

```
0 0 0
0 0 0
0 0 1
```

**Expected:** empty path.

### test_astar_no_path

Uses a 3×3 grid where a cross of walls completely isolates (2,2) from (0,0). Confirms the algorithm returns an empty path rather than hanging or crashing.

```
0 1 0
1 1 1
0 1 0
```

**Expected:** empty path (no route exists).

### test_astar_start_equals_goal

Passes (1,1) as both start and goal on a clear 3×3 grid. The algorithm should recognise it is already at the destination and return a path containing exactly one cell.

**Expected:** path of length `1` containing only (1,1).

### test_astar_straight_line

Uses a single-row 1×5 grid, forcing the only possible path to go straight right. Confirms the path has exactly 5 cells in the correct column order.

```
0 0 0 0 0
```

**Expected:** (0,0) -> (0,1) -> (0,2) -> (0,3) -> (0,4), length 5.

### test_astar_large_grid

Runs A* on a 20×20 empty grid from `(0,0)` to `(19,19)`. Confirms the path exists, has the correct endpoints, and is exactly 39 cells long — the minimum for a Manhattan distance of 38.

**Expected:** path of length 39, from (0,0) to (19,19).


# References

- Amit Patel, _Introduction to A*_: https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html
- cppreference, _std::mt19937_: https://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine
- GeeksforGeeks, _C++ Templates_: https://www.geeksforgeeks.org/templates-cpp/
- Wikipedia, _Manhattan Distance_: https://en.wikipedia.org/wiki/Taxicab_geometry
