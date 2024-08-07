#include "grid.h"
#include <iostream>
#include <fstream>
#include <sstream>

void Grid::Load(const char *file, unsigned int levelWidth, unsigned int levelHeight)
{
    // clear old data
    this->Bricks.clear();
    // load from file
    unsigned int tileCode;
    std::string line;
    std::ifstream fstream(file);
    std::vector<std::vector<unsigned int>> tileData;
    if (fstream)
    {
        while (std::getline(fstream, line)) // read each line from level file
        {
            std::istringstream sstream(line);
            std::vector<unsigned int> row;
            while (sstream >> tileCode) // read each word separated by spaces
                row.push_back(tileCode);
            tileData.push_back(row);
        }
        if (tileData.size() > 0)
        {
            this->tileData = tileData; // Store the tile data
            this->init(tileData, levelWidth, levelHeight);
        }
    }
}

void Grid::init(std::vector<std::vector<unsigned int>> tileData, unsigned int levelWidth, unsigned int levelHeight)
{
    // calculate dimensions
    unsigned int height = tileData.size();
    unsigned int width = tileData[0].size();
    this->ROW = height;
    this->COL = width;
    float unit_width = levelWidth / static_cast<float>(width);
    float unit_height = levelHeight / height;
    this->unitWidth = unit_width;
    this->unitHeight = unit_height;

    // initialize level tiles based on tileData
    for (unsigned int y = 0; y < height; ++y)
    {
        for (unsigned int x = 0; x < width; ++x)
        {
            glm::vec3 color = glm::vec3(1.0f); // original: white
            if (tileData[y][x] == 1)
                color = glm::vec3(0.2f, 0.6f, 1.0f);
            else if (tileData[y][x] == 2)
                color = glm::vec3(0.0f, 0.7f, 0.0f);
            else if (tileData[y][x] == 3)
                color = glm::vec3(0.8f, 0.8f, 0.4f);
            else if (tileData[y][x] == 0)
                color = glm::vec3(0.0f, 0.0f, 0.0f);

            glm::vec2 pos(unit_width * x, unit_height * y);
            glm::vec2 size(unit_width, unit_height);
            this->Bricks.push_back(SimObject(pos, size, ResourceManager::GetTexture("block"), color));
        }
    }
}

void Grid::SetDestinationColor(const glm::vec2 &destination, const glm::vec3 &color)
{
    int x = static_cast<int>(destination.x / unitWidth);
    int y = static_cast<int>(destination.y / unitHeight);

    for (SimObject &tile : this->Bricks)
    {
        if (tile.Position == glm::vec2(unitWidth * x, unitHeight * y))
        {
            tile.Color = color;
            return;
        }
    }
}

void Grid::Draw(SpriteRenderer &renderer)
{
    for (SimObject &tile : this->Bricks)
        tile.Draw(renderer);
}
