#include "tapf.h"

// Function to generate unique random positions within grid bounds
std::vector<std::vector<int>> GenerateUniqueRandomPositions(int num_positions, int grid_rows, int grid_cols, std::mt19937 &rng)
{
    if (num_positions > grid_rows * grid_cols)
    {
        throw std::invalid_argument("Number of positions requested exceeds the number of available grid cells.");
    }

    std::vector<std::vector<int>> all_positions;
    for (int row = 0; row < grid_rows; ++row)
    {
        for (int col = 0; col < grid_cols; ++col)
        {
            all_positions.push_back({row, col, 0});
        }
    }

    std::shuffle(all_positions.begin(), all_positions.end(), rng);
    all_positions.resize(num_positions);
    return all_positions;
}

// Function to ensure unique start and goal positions
void EnsureUniqueStartAndGoal(std::vector<std::vector<int>> &starts, std::vector<std::vector<int>> &goals, std::mt19937 &rng, int grid_rows, int grid_cols)
{
    std::set<std::vector<int>> start_set;
    std::set<std::vector<int>> goal_set;

    // Generate unique starts
    while (start_set.size() < starts.size())
    {
        auto new_starts = GenerateUniqueRandomPositions(starts.size(), grid_rows, grid_cols, rng);
        start_set.insert(new_starts.begin(), new_starts.end());
    }
    starts.assign(start_set.begin(), start_set.end());

    // Generate unique goals
    while (goal_set.size() < goals.size())
    {
        auto new_goals = GenerateUniqueRandomPositions(goals.size(), grid_rows, grid_cols, rng);
        goal_set.insert(new_goals.begin(), new_goals.end());
    }
    goals.assign(goal_set.begin(), goal_set.end());

    // Ensure no start position is the same as any goal position
    std::set<std::vector<int>> all_positions = start_set;
    all_positions.insert(goal_set.begin(), goal_set.end());

    // Regenerate goals if there are overlaps with start positions
    while (goal_set.size() < goals.size())
    {
        auto new_goals = GenerateUniqueRandomPositions(goals.size(), grid_rows, grid_cols, rng);
        for (const auto &goal : new_goals)
        {
            if (all_positions.find(goal) == all_positions.end())
            {
                goal_set.insert(goal);
                all_positions.insert(goal);
            }
        }
    }
    goals.assign(goal_set.begin(), goal_set.end());
}

std::vector<std::vector<std::vector<int>>> GenerateEndpoints(int num_agents, int grid_rows, int grid_cols)
{
    // Initialize the grid
    std::vector<std::vector<int>> grid(grid_rows, std::vector<int>(grid_cols, 1));

    std::random_device rd;
    std::mt19937 rng(rd());

    // Generate unique random sources and destinations
    auto starts = GenerateUniqueRandomPositions(num_agents, grid_rows, grid_cols, rng);
    auto goals = GenerateUniqueRandomPositions(num_agents, grid_rows, grid_cols, rng);

    // EnsureUniqueStartAndGoal(starts, goals, rng, grid_rows, grid_cols);

    return {{starts}, {goals}};
}
