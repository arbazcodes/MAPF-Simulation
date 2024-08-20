#include "graph.h"
#include <iostream>
#include <stdexcept>
#include <queue>
#include <unordered_map>

Graph::Graph(int w, int h)
    : width(w), height(h)
{
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            Vertex *v = new Vertex(x, y);
            locations.insert(v);
        }
    }
}

Graph::~Graph()
{
    for (Vertex *v : locations)
    {
        delete v;
    }
    locations.clear();
}

std::vector<Vertex *> Graph::GetNeighbors(const Vertex *v)
{
    std::vector<Vertex *> neighbors;

    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};

    for (int i = 0; i < 4; ++i)
    {
        int nx = v->x + dx[i];
        int ny = v->y + dy[i];

        if (nx >= 0 && ny >= 0 && nx < width && ny < height)
        {
            Vertex neighbor(nx, ny);
            Vertex *neighbor_vertex = nullptr;
            for (Vertex *vert : locations)
            {
                if (vert->x == nx && vert->y == ny)
                {
                    neighbor_vertex = vert;
                    break;
                }
            }

            if (neighbor_vertex)
            {
                neighbors.push_back(neighbor_vertex);
            }
        }
    }

    return neighbors;
}

