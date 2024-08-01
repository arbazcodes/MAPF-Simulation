#include "astar.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>

// Function to calculate Manhattan distance
int ManhattanDistance(const Pair &a, const Pair &b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

// Function to calculate the cost of rotation between two directions
int RotationCost(Direction from, Direction to)
{
    return (from == to) ? 0 : 1;            
}

// A* algorithm for pathfinding
std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid)
{
    int rows = grid.size();
    int cols = (rows > 0) ? grid[0].size() : 0;

    std::vector<Pair> direction_vectors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    std::vector<Direction> directions = {RIGHT, DOWN, LEFT, UP};
    //  Priority queue to store open list
    std::priority_queue<
        std::tuple<int, State>,
        std::vector<std::tuple<int, State>>,
        std::greater<std::tuple<int, State>>>
        open_list;
    open_list.push({0, {start, UP, 0}}); // (priority, (position, direction, time_step))

    std::map<State, int> g_costs;
    g_costs[{start, UP, 0}] = 0;

    std::map<State, State> came_from;

    std::map<int, std::set<Pair>> constraint_map;
    for (const auto &constraint : constraints)
    {
        constraint_map[constraint.time].insert({constraint.x, constraint.y});
    }

    while (!open_list.empty())
    {
        auto [_, current] = open_list.top();
        open_list.pop();

        if (current.position == goal)
        {
            std::vector<std::vector<int>> path;
            while (came_from.find(current) != came_from.end())
            {
                path.push_back({current.position.first,
                                current.position.second,
                                static_cast<int>(current.direction),
                                current.time_step});
                current = came_from[current];
            }
            path.push_back({start.first, start.second, static_cast<int>(UP), 0});
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        for (size_t i = 0; i < 4; ++i)
        {
            Direction new_direction = directions[i];

            
            if((current.direction == LEFT && new_direction == RIGHT) || (current.direction == RIGHT && new_direction == LEFT))
                new_direction == current.direction;
            if((current.direction == UP && new_direction == DOWN) || (current.direction == DOWN && new_direction == UP))
                new_direction = current.direction;

            Pair direction_vector = direction_vectors[i];
            Pair next_cell = {
                current.position.first + direction_vector.first,
                current.position.second + direction_vector.second};
            int next_time_step = current.time_step + 1;

            // Check if the next cell is within bounds and not blocked
            if (next_cell.first < 0 || next_cell.first >= rows ||
                next_cell.second < 0 || next_cell.second >= cols ||
                grid[next_cell.first][next_cell.second] != 1)
            {
                continue;
            }

            // Check constraints
            if (constraint_map.find(next_time_step) != constraint_map.end() &&
                constraint_map[next_time_step].find(next_cell) != constraint_map[next_time_step].end())
            {
                continue;
            }

            // Determine the cost to transition to the new state
            int rotation_cost_value = RotationCost(current.direction, new_direction);
            int move_cost = 1;
            int final_g_cost = g_costs[current] + rotation_cost_value + move_cost;
            State final_state = {next_cell, new_direction, next_time_step};

            // Check if the new state is better than previously known states
            if (g_costs.find(final_state) == g_costs.end() ||
                final_g_cost < g_costs[final_state])
            {
                g_costs[final_state] = final_g_cost;
                int f_cost = final_g_cost + ManhattanDistance(final_state.position, goal);
                open_list.push({f_cost, final_state});
                came_from[final_state] = current;
            }
        }
    }

    return {};
}
