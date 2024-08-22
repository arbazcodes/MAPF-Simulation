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

struct Vertex {
    int x, y;
    Direction direction;

    Vertex() = default;
    Vertex(int a, int b) {
        x = a;
        y = b;
        direction = Direction::Up;
    }

    bool operator<(const Vertex &other) const
    {
        return std::tie(x, y, direction) < std::tie(other.x, other.y, other.direction);
    }
};

class Graph
{
public:
    int width, height;
    std::set<Vertex> locations;

    Graph() = default;
    Graph(int w, int h);
    ~Graph();
    void Init();
    void Clear();
    std::vector<Vertex> GetNeighbors(const Vertex &v) const;
};
