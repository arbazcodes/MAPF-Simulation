#pragma once
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iterator>
#include <set>

using Pair = std::pair<int, int>;

std::vector<std::vector<int>> GenerateUniqueRandomPositions(int num_positions, int grid_rows, int grid_cols, std::mt19937 &rng);
void EnsureUniqueStartAndGoal(std::vector<Pair> &starts, std::vector<Pair> &goals, std::mt19937 &rng, int grid_rows, int grid_cols);
std::vector<std::vector<std::vector<int>>> GenerateEndpoints(int number_of_robots, int grid_rows, int grid_cols);