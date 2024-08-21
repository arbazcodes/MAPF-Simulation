#pragma once

#include <vector>
#include <set>
#include <utility> // For std::pair

enum Direction
{
    Up,
    Down,
    Left,
    Right,
    None
};

using Vertex = std::pair<int, int>; // Change Vertex to pair

class Graph
{
public:
    int width, height;
    std::set<Vertex> locations; // Change to pair

    Graph() = default;
    Graph(int w, int h);
    ~Graph() = default;
    std::vector<Vertex> GetNeighbors(const Vertex &v) const;
};
