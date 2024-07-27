#include "astar.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>
#include <cmath>
#include <queue>
#include <stack>
#include <string>
#include <iostream>

AStar::AStar(const char *file){
    loadGrid(file);
    ROW = grid.size();
    COL = grid[0].size();
}

void AStar::loadGrid(const char *file) {
    int tileCode;
    string line;
    ifstream fstream(file);
    if (fstream) {
        while (getline(fstream, line)) {
            istringstream sstream(line);
            vector<int> row;
            while (sstream >> tileCode)
                row.push_back(tileCode);
            grid.push_back(row);
        }
    }
}

bool AStar::isValid(int row, int col) const {
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

bool AStar::isUnBlocked(int row, int col) const {
    return grid[row][col] == 1;
}

bool AStar::isDestination(int row, int col, Pair dest) const {
    return row == dest.first && col == dest.second;
}

double AStar::calculateHValue(int row, int col, Pair dest) const {
    return abs(row - dest.first) + abs(col - dest.second);
}

vector<vector<int>> AStar::tracePath(const vector<vector<Cell>>& cellDetails, Pair dest) const {
    vector<vector<int>> path;
    stack<Pair> Path;
    int row = dest.first;
    int col = dest.second;
    
    while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col)) {
        Path.push({row, col});
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }
    Path.push({row, col});
    
    while (!Path.empty()) {
        auto p = Path.top();
        Path.pop();
        path.push_back({p.second, p.first});
    }
    return path;
}

vector<vector<int>> AStar::aStarSearch(Pair src, Pair dest) {

    swap(src.first, src.second);
    swap(dest.first, dest.second);
    vector<vector<int>> path;
    
    if (!isValid(src.first, src.second) || !isValid(dest.first, dest.second)) {
        cout << "Source or Destination is invalid\n";
        return path;
    }

    if (!isUnBlocked(src.first, src.second) || !isUnBlocked(dest.first, dest.second)) {
        cout << "Source or Destination is blocked\n";
        return path;
    }

    if (isDestination(src.first, src.second, dest)) {
        cout << "We are already at the destination\n";
        path.push_back({src.first, src.second});
        return path;
    }

    vector<vector<Cell>> cellDetails(ROW, vector<Cell>(COL));
    vector<vector<bool>> closedList(ROW, vector<bool>(COL, false));

    for (int i = 0; i < ROW; ++i) {
        for (int j = 0; j < COL; ++j) {
            cellDetails[i][j].f = numeric_limits<double>::max();
            cellDetails[i][j].g = numeric_limits<double>::max();
            cellDetails[i][j].h = numeric_limits<double>::max();
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    int i = src.first;
    int j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    priority_queue<pPair, vector<pPair>, greater<>> openList;
    openList.push({0.0, {i, j}});

    bool foundDest = false;

    vector<int> rowNum = {1, -1, 0, 0};
    vector<int> colNum = {0, 0, 1, -1};

    while (!openList.empty()) {
        pPair p = openList.top();
        openList.pop();

        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

        for (int k = 0; k < 4; ++k) {
            int newRow = i + rowNum[k];
            int newCol = j + colNum[k];

            if (isValid(newRow, newCol)) {
                if (isDestination(newRow, newCol, dest)) {
                    cellDetails[newRow][newCol].parent_i = i;
                    cellDetails[newRow][newCol].parent_j = j;
                    cout << "The destination cell is found\n";
                    path = tracePath(cellDetails, dest);
                    foundDest = true;
                    return path;
                } else if (!closedList[newRow][newCol] && isUnBlocked(newRow, newCol)) {
                    double gNew = cellDetails[i][j].g + 1.0;
                    double hNew = calculateHValue(newRow, newCol, dest);
                    double fNew = gNew + hNew;

                    if (cellDetails[newRow][newCol].f == numeric_limits<double>::max() || cellDetails[newRow][newCol].f > fNew) {
                        openList.push({fNew, {newRow, newCol}});
                        cellDetails[newRow][newCol].f = fNew;
                        cellDetails[newRow][newCol].g = gNew;
                        cellDetails[newRow][newCol].h = hNew;
                        cellDetails[newRow][newCol].parent_i = i;
                        cellDetails[newRow][newCol].parent_j = j;
                    }
                }
            }
        }
    }

    if (!foundDest) {
        cout << "Failed to find the Destination Cell\n";
    }

    return path;
}