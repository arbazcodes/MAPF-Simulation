#include "astar.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>
#include <set>
#include <iostream>

// Function to calculate Manhattan distance
int ManhattanDistance(const Pair &a, const Pair &b)
{
    return std::sqrt(std::pow(std::abs(a.first - b.first), 2) + std::pow(std::abs(a.second - b.second), 2));
}

// Function to calculate the cost of rotation between two directions
int RotationCost(Direction from, Direction to)
{
    if (from == to)
        return 0;
    if ((from == UP && to == DOWN) || (from == DOWN && to == UP) ||
        (from == LEFT && to == RIGHT) || (from == RIGHT && to == LEFT))
    {
        return 1; // No rotation cost for opposite directions
    }
    return 2; // Default rotation cost
}

// Function to get constraint time for a given position
std::optional<int> GetConstraintTime(const Pair &position, const std::vector<std::vector<int>> &constraints)
{
    for (const auto &constraint : constraints)
    {
        if (position.first == constraint[0] && position.second == constraint[1])
            return constraint[2];
    }
    return std::nullopt;
}

// Function to get valid neighbors for a given state
std::vector<State> GetNeighbors(
    const State &current,
    const Pair &goal,
    const std::vector<std::vector<int>> &grid,
    std::map<int, std::set<Pair>> vertex_constraint_map,
    std::map<int, std::set<Pair>> edge_constraint_map,
    std::vector<std::vector<int>> stopping_constraint_map,
    std::map<int, std::set<Pair>> following_constraint_map)
{
    std::vector<State> neighbors;
    std::vector<Pair> direction_vectors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0, 0}};
    std::vector<Direction> directions = {DOWN, RIGHT, UP, LEFT, STAY};

    for (int i = 0; i < directions.size(); ++i)
    {
        Direction new_direction = directions[i];
        Pair direction_vector = direction_vectors[i];
        Pair next_cell = {current.position.first + direction_vector.first, current.position.second + direction_vector.second};
        int next_time_step = current.time_step + 1;

        if (next_cell.first < 0 || next_cell.first >= grid.size() || next_cell.second < 0 || next_cell.second >= grid[0].size() || grid[next_cell.first][next_cell.second] != 1)
        {
            continue;
        }

        if (vertex_constraint_map.find(next_time_step) != vertex_constraint_map.end() &&
            vertex_constraint_map[next_time_step].find(next_cell) != vertex_constraint_map[next_time_step].end())
        {
            continue;
        }
        if (edge_constraint_map.find(next_time_step) != edge_constraint_map.end() &&
            edge_constraint_map[next_time_step].find(next_cell) != edge_constraint_map[next_time_step].end())
        {
            continue;
        }
        if (following_constraint_map.find(next_time_step) != following_constraint_map.end() &&
            following_constraint_map[next_time_step].find(next_cell) != following_constraint_map[next_time_step].end())
        {
            continue;
        }
        bool is_stopping_constraint = false;
        for (const auto &constraint : stopping_constraint_map)
        {
            if (next_cell.first == constraint[0] && next_cell.second == constraint[1] && next_time_step == constraint[2])
            {
                is_stopping_constraint = true;
                break;
            }
        }
        if (is_stopping_constraint)
        {
            continue;
        }

        // If moving in the opposite direction, keep the same direction
        Direction current_direction = current.direction;
        if ((current_direction == UP && new_direction == DOWN) || (current_direction == DOWN && new_direction == UP) ||
            (current_direction == LEFT && new_direction == RIGHT) || (current_direction == RIGHT && new_direction == LEFT))
        {
            new_direction = current_direction; // Keep the same direction
        }

        neighbors.push_back({next_cell, new_direction, next_time_step});
    }

    return neighbors;
}

// A* algorithm implementation
std::vector<std::vector<int>> AStarAlgorithm(
    const Pair &start,
    const Pair &goal,
    const std::vector<Constraint> &constraints,
    const std::vector<std::vector<int>> &grid)
{
    int rows = grid.size();
    int cols = (rows > 0) ? grid[0].size() : 0;

    std::priority_queue<
        std::tuple<int, State>,
        std::vector<std::tuple<int, State>>,
        std::greater<std::tuple<int, State>>>
        open_list;
    open_list.push({0, {start, UP, 0}});

    std::map<State, int> g_costs;
    g_costs[{start, UP, 0}] = 0;

    std::map<State, State> came_from;

    std::map<int, std::set<Pair>> vertex_constraint_map;
    std::map<int, std::set<Pair>> edge_constraint_map;
    std::vector<std::vector<int>> stopping_constraint_map;
    std::map<int, std::set<Pair>> following_constraint_map;

    for (const auto &constraint : constraints)
    {
        if (constraint.type == 0)
        {
            vertex_constraint_map[constraint.time].insert({constraint.x, constraint.y});
        }
        else if (constraint.type == 1)
        {
            edge_constraint_map[constraint.time].insert({constraint.x, constraint.y});
        }
        else if (constraint.type == 2)
        {
            stopping_constraint_map.push_back({constraint.x, constraint.y, constraint.time});
        }
        else if (constraint.type == 3)
        {
            following_constraint_map[constraint.time].insert({constraint.x, constraint.y});
        }
    }

    while (!open_list.empty())
    {
        auto [_, current] = open_list.top();
        open_list.pop();

        if (current.position == goal)
        {
            // Check if the current time step is less than the stopping constraint time
            auto constraint_time = GetConstraintTime(current.position, stopping_constraint_map);
            if (constraint_time.has_value() && current.time_step < constraint_time.value())
            {
                std::vector<State> neighbors = GetNeighbors(current, goal, grid, vertex_constraint_map, edge_constraint_map, stopping_constraint_map, following_constraint_map);

                for (auto &neighbor : neighbors)
                {
                    if (neighbor.direction == STAY)
                        neighbor.direction = current.direction;
                    int rotation_cost_value = RotationCost(current.direction, neighbor.direction);
                    int move_cost = (current.position == neighbor.position) ? 1 : 2;
                    int final_g_cost = g_costs[current] + rotation_cost_value + move_cost;
                    State final_state = neighbor;

                    if (g_costs.find(final_state) == g_costs.end() || final_g_cost < g_costs[final_state])
                    {
                        g_costs[final_state] = final_g_cost;
                        int f_cost = final_g_cost + ManhattanDistance(final_state.position, goal);
                        open_list.push({f_cost, final_state});
                        came_from[final_state] = current;
                    }
                }
                continue; // Skip this goal state as it violates the stopping constraint
            }

            std::vector<std::vector<int>> path;
            while (came_from.find(current) != came_from.end())
            {
                path.push_back({current.position.first, current.position.second, static_cast<int>(current.direction), current.time_step});
                current = came_from[current];
            }
            path.push_back({start.first, start.second, static_cast<int>(UP), 0});
            std::reverse(path.begin(), path.end());
            return path;
        }

        std::vector<State> neighbors = GetNeighbors(current, goal, grid, vertex_constraint_map, edge_constraint_map, stopping_constraint_map, following_constraint_map);

        for (auto &neighbor : neighbors)
        {
            if(neighbor.direction == STAY)
                neighbor.direction = current.direction;
            int rotation_cost_value = RotationCost(current.direction, neighbor.direction);
            int move_cost = (current.position == neighbor.position) ? 1 : 2;
            int final_g_cost = g_costs[current] + rotation_cost_value + move_cost;
            State final_state = neighbor;

            if (g_costs.find(final_state) == g_costs.end() || final_g_cost < g_costs[final_state])
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
