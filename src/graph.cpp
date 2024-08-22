#include "graph.h"
#include <iostream>
#include <stdexcept>

Graph::Graph(int w, int h): width(w), height(h){
    Init();
}

void Graph::Init(){
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            locations.insert(Vertex(x, y)); // Use pair
        }
    }
}

void Graph::Clear(){
    locations.clear();
}

Graph::~Graph(){
    Clear();
}

std::vector<Vertex> Graph::GetNeighbors(const Vertex &v) const
{
    std::vector<Vertex> neighbors;

    int dx[] = {0, 0, -1, 1, 0};
    int dy[] = {-1, 1, 0, 0, 0};

    std::vector<Direction> direction_vector = {Direction::Up, Direction::Down, Direction::Left, Direction::Right, Direction::None};

    for (int i = 0; i < 4; ++i)
    {
        int nx = v.x + dx[i];
        int ny = v.y + dy[i];

        if (nx >= 0 && ny >= 0 && nx < width && ny < height)
        {
            Vertex neighbor = {nx, ny};
            if (locations.find(neighbor) != locations.end())
            {
                neighbor.direction = direction_vector[i];
                neighbors.push_back(neighbor);
            }
        }
    }

    return neighbors;
}
