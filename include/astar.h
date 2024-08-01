#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <utility>

typedef std::pair<int, int> Pair;

enum Direction
{
    UP,
    DOWN,
    LEFT,
    RIGHT,
};

struct Constraint
{
    int id;
    int x;
    int y;
    int time;
};

struct State
{
    Pair position;
    Direction direction;
    int time_step;

    bool operator<(const State &other) const
    {
        return std::tie(position, direction, time_step) < std::tie(other.position, other.direction, other.time_step);
    }
};

std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid);

#endif // ASTAR_H
