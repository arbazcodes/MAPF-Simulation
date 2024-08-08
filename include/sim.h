#ifndef GAME_H
#define GAME_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "grid.h"
#include "robot.h"

const glm::vec2 INITIAL_VELOCITY(100.0f, 100.0f);
const float RADIUS = 25.0f;
#define NUMBER_OF_ROBOTS 4

class Sim
{
public:
    unsigned int Width, Height;
    float UnitWidth, UnitHeight;
    Grid grid;

    Sim(unsigned int width, unsigned int height);
    ~Sim();
    void Init();
    void Update(float dt);
    void Render();

private:
    std::vector<Robot *> Robots;
};

#endif
