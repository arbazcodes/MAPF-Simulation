#ifndef GAME_H
#define GAME_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "grid.h"
#include "robot.h"

const glm::vec2 INITIAL_VELOCITY(300.0f, 300.0f);
const float RADIUS = 30.0f;
#define ROWS 6
#define COLS 6
#define NUMBER_OF_ROBOTS 3

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

    bool AllReached();
    bool AllRotated();
    bool AllReachedGoal();

private:
    std::vector<Robot *> Robots;
};

#endif
