#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <vector>
#include <limits>
#include <queue>
#include <unordered_map>

struct Node{
    int x,y; // Position in our grid environment
    float cost; // Cost from start Node
    Node* parent; // Reconstruct the path for dynamic obstacles

    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

class Grid {
public:
    Grid(int width, int height);
    std::vector<Node> CalcShortestPath(int startX, int startY, int targetX, int targetY);

    // Obstacle handling
    void SetObstacle(int x, int y);
    bool IsObstacle(int x, int y) const;
    // Utilized for dynamic obstacles.
    void ResetGrid();
    
private:
    int mpwidth, mpheight;
    std::unordered_map<int, std::unordered_map<int, Node>> mpnodes;
};

#endif