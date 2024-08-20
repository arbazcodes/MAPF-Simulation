#pragma once

#include <vector>
#include <unordered_set>

enum Direction
{
    Up,
    Down,
    Left,
    Right,
    None
};

struct Vertex
{
    int x, y;
    Direction direction;
    Vertex() = default;
    Vertex(int a, int b) : x(a), y(b) {}
};

class Graph
{
public:
    int width, height;
    std::unordered_set<Vertex *> locations;

    Graph() = default;
    Graph(int w, int h);
    ~Graph();
    std::vector<Vertex *> GetNeighbors(const Vertex *v);
};
