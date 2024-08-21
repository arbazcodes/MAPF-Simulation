#include "graph.h"
#include <iostream>
#include <stdexcept>

Graph::Graph(int w, int h)
    : width(w), height(h)
{
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            locations.insert(std::make_pair(x, y)); // Use pair
        }
    }
}

std::vector<Vertex> Graph::GetNeighbors(const Vertex &v) const
{
    std::vector<Vertex> neighbors;

    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};

    for (int i = 0; i < 4; ++i)
    {
        int nx = v.first + dx[i];
        int ny = v.second + dy[i];

        if (nx >= 0 && ny >= 0 && nx < width && ny < height)
        {
            Vertex neighbor = {nx, ny};
            if (locations.find(neighbor) != locations.end())
            {
                neighbors.push_back(neighbor);
            }
        }
    }

    return neighbors;
}
