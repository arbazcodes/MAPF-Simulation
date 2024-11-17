#ifndef PIBT_SIM_H
#define PIBT_SIM_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <set>
#include "pibt_alg.h"
#include "grid.h"
#include "pibt_robot.h"

const glm::vec2 INITIAL_VELOCITY(500.0f, 500.0f);
const float RADIUS = 30.0f;
#define ROWS 6
#define COLS 6
#define NUMBER_OF_ROBOTS 25

class PIBT_Sim
{
public:
    unsigned int Width, Height;
    float UnitWidth, UnitHeight;
    Grid grid;
    int globalPathIndex = 1;
    int path_size;
    std::vector<std::vector<int>> starts;
    std::vector<std::vector<int>> goals;

    PIBT_Sim(unsigned int width, unsigned int height);
    ~PIBT_Sim();
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
    std::vector<PIBT_Robot *> Robots;
    std::set<int> idle_robots;
};

#endif // PIBT_SIM
