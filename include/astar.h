#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <utility>

using namespace std;

typedef pair<int, int> Pair;
typedef pair<double, Pair> pPair;

struct Cell {
    int parent_i, parent_j;
    double f, g, h;
};

class AStar {
public:
    AStar(const char *file);

    vector<vector<int>> aStarSearch(Pair src, Pair dest);

    vector<vector<int>> grid;
    int ROW;
    int COL;

    void loadGrid(const char *file);
    bool isValid(int row, int col) const;
    bool isUnBlocked(int row, int col) const;
    bool isDestination(int row, int col, Pair dest) const;
    double calculateHValue(int row, int col, Pair dest) const;
    vector<vector<int>> tracePath(const vector<vector<Cell>>& cellDetails, Pair dest) const;
};

#endif
