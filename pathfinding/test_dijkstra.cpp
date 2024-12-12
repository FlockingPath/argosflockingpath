#include "dijkstra.h"
#include <cassert>
#include <iostream>

int main() {
    Grid grid(5, 5);

    // Set some obstacles
    // grid.SetObstacle(1, 1);
    // grid.SetObstacle(1, 2);
    // grid.SetObstacle(1, 3);
    // grid.SetObstacle(2, 1);
    // grid.SetObstacle(3, 1);

    // Define start and target nodes
    int startX = 0, startY = 0;  // Starting point
    int targetX = 4, targetY = 4; // Target point

    // Calc the shortest path
    std::vector<Node> path = grid.CalcShortestPath(startX, startY, targetX, targetY);

    // Check if the path is correct
    if (path.empty()) {
        std::cerr << "No path found from (" << startX << ", " << startY << ") to (" << targetX << ", " << targetY << ")" << std::endl;
        return 1;
    }

    assert(path.front().x == startX && path.front().y == startY);  // Ensure it starts at the start node
    assert(path.back().x == targetX && path.back().y == targetY); // Ensure it ends at the target node

    // Print the path
    std::cout << "Path found:" << std::endl;
    for (const Node& node : path) {
        std::cout << "(" << node.x << ", " << node.y << ")" << std::endl;
    }

    std::cout << "Test passed!" << std::endl;
    return 0;
}