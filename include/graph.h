#pragma once

#include <vector>
#include <unordered_set>

struct Vertex
{
    int x, y, time_occupied;
    bool occupied;
    Vertex() = default;
    Vertex(int a, int b) : x(a), y(b), time_occupied(-1), occupied(false) {}
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
