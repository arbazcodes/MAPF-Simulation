#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <stack>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>

using namespace std;

typedef pair<int, int> Pair;
typedef pair<double, Pair> pPair;

struct Cell {
    double f, g, h;
    int parent_i, parent_j;
};

class AStar {
public:
    AStar(const char* file);
    vector<vector<vector<int>>> aStarSearch(vector<Pair> src, vector<Pair> dest);

private:
    vector<vector<int>> grid;
    int ROW, COL;
    void loadGrid(const char* file);
    bool isValid(int row, int col) const;
    bool isUnBlocked(int row, int col) const;
    bool isDestination(int row, int col, Pair dest) const;
    double calculateHValue(int row, int col, Pair dest) const;
    vector<vector<int>> tracePath(const vector<vector<Cell>>& cellDetails, Pair dest) const;
};

#endif
