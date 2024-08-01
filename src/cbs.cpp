#include "cbs.h"
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <optional>
#include <queue>

// Constructor
Cbs::Cbs(const std::vector<std::vector<int>> &grid) : grid(grid) {}

std::optional<std::vector<CostPath>> Cbs::LowLevel(
    const std::vector<Pair> &sources,
    const std::vector<Pair> &destinations,
    const std::vector<Constraint> &constraints) const
{
    std::vector<CostPath> solution;
    std::map<int, std::vector<Constraint>> constraint_by_id;

    for (const auto &constraint : constraints)
    {
        constraint_by_id[constraint.id].push_back(constraint);
    }

    for (int i = 0; i < sources.size(); ++i)
    {
        auto path = AStarAlgorithm(sources[i], destinations[i], constraint_by_id[i], grid);
        if (path.empty())
        {
            std::cout << "No path found for agent " << i << " with constraints." << std::endl;
            return std::nullopt;
        }
        solution.push_back(path);
    }

    // Find the maximum path length
    int max_length = 0;
    for (const auto &path : solution)
    {
        if (path.size() > max_length)
        {
            max_length = path.size();
        }
    }

    // // Extend all paths to match the maximum length
    // for (auto &path : solution)
    // {
    //     if (path.size() < max_length)
    //     {
    //         auto last_step = path.back();
    //         for (int t = path.size(); t < max_length; ++t)
    //         {
    //             path.push_back({last_step[0], last_step[1], last_step[2], static_cast<int>(t)});
    //         }
    //     }
    // }

    return solution;
}

// Calculate the total cost of a solution
int Cbs::FindTotalCost(const std::vector<CostPath> &solution) const
{
    int total_cost = 0;

    for (const auto &path : solution)
    {
        total_cost += path.size();
    }

    return total_cost;
}

std::vector<std::vector<int>> Cbs::FindConflicts(const std::vector<CostPath> &solution) const
{
    std::vector<std::vector<int>> Conflicts;

    for (int i = 0; i < solution.size(); ++i)
    {
        const std::vector<std::vector<int>> &path_1 = solution[i];

        for (int j = i + 1; j < solution.size(); ++j)
        {
            const std::vector<std::vector<int>> &path_2 = solution[j];

            for (int t = 0; t < path_1.size() && t < path_2.size(); ++t)
            {
                const auto &step_1 = path_1[t];
                const auto &step_2 = path_2[t];

                if (step_1.size() == 4 && step_2.size() == 4)
                {
                    int x1 = step_1[0];
                    int y1 = step_1[1];
                    int x2 = step_2[0];
                    int y2 = step_2[1];

                    if (x1 == x2 && y1 == y2)
                    {
                        Conflicts.push_back({i, j, x1, y1, t});
                    }
                }
            }
        }
    }

    return Conflicts;
}

std::vector<Constraint> Cbs::GenerateConstraints(const std::vector<std::vector<int>> &conflicts) const
{
    std::vector<Constraint> Constraints;

    for (const auto &p : conflicts)
    {
        Constraints.push_back({p[0], p[2], p[3], p[4]});
        Constraints.push_back({p[1], p[2], p[3], p[4]});
    }

    return Constraints;
}

std::vector<CostPath> Cbs::HighLevel(const std::vector<Pair> &sources, const std::vector<Pair> &destinations) const
{
    std::priority_queue<CbsNode> open;
    CbsNode root;
    root.constraints = {};

    auto initial_solution = LowLevel(sources, destinations, root.constraints);
    if (!initial_solution)
    {
        std::cout << "No initial solution found." << std::endl;
        return {};
    }

    root.solution = *initial_solution;
    root.cost = FindTotalCost(root.solution);
    open.push(root);

    while (!open.empty())
    {
        CbsNode current = open.top();
        open.pop();

        auto conflicts = FindConflicts(current.solution);

        if (conflicts.empty())
        {
            std::cout << "Solution found with total cost: " << root.cost << std::endl;
            return current.solution;
        }

        for (const auto &conflict : conflicts)
        {
            CbsNode child1 = current;
            CbsNode child2 = current;

            auto new_constraints = GenerateConstraints({conflict});

            auto new_solution1 = LowLevel(sources, destinations, child1.constraints);
            if (!new_solution1.has_value())
            {
                continue;
            }

            child1.constraints.push_back(new_constraints[0]);
            child1.solution = *new_solution1;
            child1.cost = FindTotalCost(child1.solution);
            open.push(child1);

            auto new_solution2 = LowLevel(sources, destinations, child2.constraints);
            if (!new_solution2.has_value())
            {
                continue;
            }
            child2.constraints.push_back(new_constraints[1]);
            child2.solution = *new_solution2;
            child2.cost = FindTotalCost(child2.solution);
            open.push(child2);
        }
    }

    std::cout << "No feasible solution found." << std::endl;
    return {};
}

