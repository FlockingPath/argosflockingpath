#include "dijkstra.h"
#include <vector>
#include <limits>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <iostream>

Grid::Grid(int width, int height) : mpwidth(width), mpheight(height) {
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            mpnodes[x][y] = {x, y, std::numeric_limits<float>::infinity(), nullptr};
        }
    }
}

void Grid::SetObstacle(int x, int y) {
    if (IsObstacle(x, y)) return;
    mpnodes[x][y].cost = std::numeric_limits<float>::infinity();
    std::cout << "Set obstacle at (" << x << ", " << y << ")" << std::endl;
}

bool Grid::IsObstacle(int x, int y) const {
    if (x >= 0 && x < mpwidth && y >= 0 && y < mpheight) {
        bool isObstacle = mpnodes.at(x).at(y).cost == std::numeric_limits<float>::infinity();
        return isObstacle;
    }
    return true;  // Out-of-bounds nodes should always be treated as obstacles
}

// Reset grid for dynamic obstacles
void Grid::ResetGrid() {
    for (int x = 0; x < mpwidth; ++x) {
        for (int y = 0; y < mpheight; ++y) {
            mpnodes[x][y].cost = std::numeric_limits<float>::infinity();
            mpnodes[x][y].parent = nullptr;
        }
    }
}

std::vector<Node> Grid::CalcShortestPath(int startX, int startY, int targetX, int targetY) {
    auto cmp = [](Node* left, Node* right) { return left->cost > right->cost; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> openSet(cmp);

    // Reset all nodes' costs to infinity
    for (int x = 0; x < mpwidth; ++x) {
        for (int y = 0; y < mpheight; ++y) {
            mpnodes[x][y].cost = std::numeric_limits<float>::infinity();
            mpnodes[x][y].parent = nullptr;
        }
    }

    // Set the cost of the starting node to 0
    mpnodes[startX][startY].cost = 0;
    openSet.push(&mpnodes[startX][startY]);

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        // If the target is reached, reconstruct the path
        if (current->x == targetX && current->y == targetY) {
            std::vector<Node> path;
            while (current) {
                path.push_back(*current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Define the possible neighbors
        std::vector<std::pair<int, int>> neighbors = {
            {current->x + 1, current->y}, {current->x - 1, current->y},
            {current->x, current->y + 1}, {current->x, current->y - 1}
        };

        for (auto& neighbor : neighbors) {
            int nx = neighbor.first, ny = neighbor.second;

            // Ensure the neighbor is within bounds
            if (nx >= 0 && nx < mpwidth && ny >= 0 && ny < mpheight) {
                // Check if the neighbor is not an obstacle
                if (!IsObstacle(nx, ny)) {
                    // Calculate the cost for this neighbor
                    float newCost = current->cost + 1; // (1 per move)

                    // If the new cost is lower than the current cost of the neighbor, update it
                    if (newCost < mpnodes[nx][ny].cost) {
                        mpnodes[nx][ny].cost = newCost;
                        mpnodes[nx][ny].parent = current;
                        openSet.push(&mpnodes[nx][ny]);
                    }
            }
            }
        }
    }

    return {};
}