#ifndef CBS_CBS_Sim_H
#define CBS_CBS_Sim_H

#include "grid.h"
#include "cbs_robot.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

const glm::vec2 INITIAL_VELOCITY(800.0f, 800.0f);
const float RADIUS = 30.0f;
#define ROWS 6
#define COLS 6
#define NUMBER_OF_ROBOTS 6

class CBS_Sim
{
public:
    unsigned int Width, Height;
    float UnitWidth, UnitHeight;
    Grid grid;

    CBS_Sim(unsigned int width, unsigned int height);
    ~CBS_Sim();
    void Init();
    void Update(float dt);
    void Render();
    void Clear();

    bool AllReached();
    bool AllRotated();
    bool AllReachedGoal();

private:
    std::vector<CBS_Robot *> Robots;
};

#endif // CBS_Sim