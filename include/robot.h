#ifndef BALL_OBJECT_H
#define BALL_OBJECT_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <iostream>
#include <stdio.h>
#include "texture.h"
#include "sprite_renderer.h"
#include <vector>

class Robot
{
public:

    float       Radius;
    Texture2D   Sprite;
    bool isRotating = true;
    float InitialRotation, CurrentRotation, AngularVelocity;
    glm::vec2   InitialPosition, CurrentPosition, Velocity;
    std::vector<std::vector<int>> Path;
    unsigned int currentPathIndex = 1;

    Robot();
    Robot(glm::vec2 pos, float radius, glm::vec2 velocity, Texture2D sprite);

    glm::vec2 checkBounds(int x, int y, glm::vec2 movement, glm::vec2 targetPosition);
    void Rotate(float dt);
    void Move(float dt, float unit_width, float unit_height);
    void Reset(glm::vec2 position, glm::vec2 velocity);
    void Draw(SpriteRenderer &renderer);

    glm::vec2 GetPosition();
};

#endif
