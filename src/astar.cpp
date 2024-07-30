#include "astar.h"

AStar::AStar(const char *file) {
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
    return (grid[row][col] == 1);
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
        path.push_back({p.first, p.second});
    }
    return path;
}

vector<vector<vector<int>>> AStar::aStarSearch(vector<Pair> src, vector<Pair> dest) {
    vector<vector<vector<int>>> Paths;
    unordered_map<string, int> cellAvailability;

    auto encode = [](int x, int y) {
        return to_string(x) + "," + to_string(y);
    };

    for (size_t x = 0; x < src.size(); ++x) {
        // swap(src[x].first, src[x].second);
        // swap(dest[x].first, dest[x].second);
        vector<vector<int>> path;

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

        if (!isValid(src[x].first, src[x].second) || !isValid(dest[x].first, dest[x].second)) {
            cout << "Source or Destination is invalid\n";
            Paths.push_back(path);
            continue;
        }

        if (!isUnBlocked(src[x].first, src[x].second) || !isUnBlocked(dest[x].first, dest[x].second)) {
            cout << "Source or Destination is blocked\n";
            Paths.push_back(path);
            continue;
        }

        if (isDestination(src[x].first, src[x].second, dest[x])) {
            cout << "We are already at the destination\n";
            path.push_back({src[x].first, src[x].second});
            Paths.push_back(path);
            continue;
        }

        int timestep = 0;
        int i = src[x].first;
        int j = src[x].second;
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
            timestep++;
            i = p.second.first;
            j = p.second.second;
            closedList[i][j] = true;
            string currentNode = encode(i, j);

            cout << "Current cell: (" << i << ", " << j << ") at timestep " << timestep << endl;

            for (int k = 0; k < 4; ++k) {
                int newRow = i + rowNum[k];
                int newCol = j + colNum[k];
                string newNode = encode(newRow, newCol);

                if (isValid(newRow, newCol) && isUnBlocked(newRow, newCol)) {
                    bool isOccupiedAtCurrentTime = (cellAvailability.find(newNode) != cellAvailability.end() && cellAvailability[newNode] > timestep);
                    bool willBeFreeSoon = (cellAvailability.find(newNode) != cellAvailability.end() && cellAvailability[newNode] == timestep + 1);

                    if (isOccupiedAtCurrentTime && !willBeFreeSoon) {
                        cout << "Collision Detected at (" << newRow << ", " << newCol << ") at timestep " << timestep << endl;
                        continue;
                    }

                    if (isDestination(newRow, newCol, dest[x])) {
                        cellDetails[newRow][newCol].parent_i = i;
                        cellDetails[newRow][newCol].parent_j = j;
                        path = tracePath(cellDetails, dest[x]);
                        foundDest = true;
                        Paths.push_back(path);

                        for (const auto& p : path) {
                            string pathNode = encode(p[0], p[1]);
                            cellAvailability[pathNode] = timestep + 1; 
                        }
                        break;
                    } else {
                        double gNew = cellDetails[i][j].g + 1.0;
                        double hNew = calculateHValue(newRow, newCol, dest[x]);
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

            if (foundDest) break;
        }

        if (!foundDest) {
            cout << "Failed to find the Destination Cell\n";
        }
    }

    return Paths;
}