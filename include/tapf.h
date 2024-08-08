#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <iterator>
#include <set>
#include "cbs.h"

std::vector<Pair> GenerateUniqueRandomPositions(int num_positions, int grid_rows, int grid_cols, std::mt19937 &rng);
void EnsureUniqueStartAndGoal(std::vector<Pair> &starts, std::vector<Pair> &goals, std::mt19937 &rng, int grid_rows, int grid_cols);
std::vector<std::vector<Pair>> GenerateEndpoints(int number_of_robots, int grid_rows, int grid_cols);