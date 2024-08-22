#ifndef GAME_H
#define GAME_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <set>
#include "grid.h"
#include "robot.h"
#include "pibt.h"

const glm::vec2 INITIAL_VELOCITY(100.0f, 100.0f);
const float RADIUS = 30.0f;
#define ROWS 6
#define COLS 6
#define NUMBER_OF_ROBOTS 10

class Sim
{
public:
    unsigned int Width, Height;
    float UnitWidth, UnitHeight;
    Grid grid;
    int globalPathIndex = 1;
    int path_size;

    Sim(unsigned int width, unsigned int height);
    ~Sim();
    void Init();
    void Update(float dt);
    void Render();
    void Clear();

    bool StateChanged();
    void Replan();

    std::vector<std::vector<std::vector<int>>> CleanSolution(const std::vector<std::vector<std::vector<int>>> &solution);

    bool AllReached();
    bool AllRotated();
    bool AllReachedGoal();

private:
    std::vector<Robot *> Robots;
    std::set<int> idle_robots;
};

#endif
