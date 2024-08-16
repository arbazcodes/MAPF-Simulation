// astar.h

#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <utility>
#include <optional>
#include <map>
#include <set>

using Pair = std::pair<int, int>;

enum Direction
{
    UP,
    DOWN,
    LEFT,
    RIGHT,
    STAY
};

struct Constraint
{
    int type;
    int id;
    int x;
    int y;
    int time;

    bool operator<(const Constraint &other) const
    {
        return std::tie(type, x, y, time) < std::tie(other.type, other.x, other.y, other.time);
    }
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

std::vector<State> GetNeighbors(
    const State &current,
    const Pair &goal,
    const std::vector<std::vector<int>> &grid,
    const std::map<int, std::set<Pair>> &vertex_constraint_map,
    const std::map<int, std::set<Pair>> &edge_constraint_map,
    const std::vector<std::vector<int>> &stopping_constraint_map,
    const std::map<int, std::set<Pair>> &following_constraint_map);

#endif // ASTAR_H