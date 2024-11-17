#ifndef CBS_ROBOT_H
#define CBS_ROBOT_H

#include "texture.h"
#include "sprite_renderer.h"

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <iostream>
#include <stdio.h>
#include <vector>

class CBS_Robot
{
public:
    float Radius;
    Texture2D Sprite;
    bool isRotating = true;
    bool isMoving = false;
    float InitialRotation, CurrentRotation, AngularVelocity;
    bool reached, rotated, reachedGoal;
    int targetDirection, targetAngle;

    glm::vec2 InitialPosition, CurrentPosition, Velocity;
    glm::vec3 Color; // New attribute for color
    std::vector<std::vector<int>> Path;
    int currentPathIndex = 1;

    CBS_Robot();
    CBS_Robot(glm::vec2 pos, float radius, glm::vec2 velocity, Texture2D sprite, glm::vec3 color);
    void Rotate(float dt);
    void Move(float dt, float unit_width, float unit_height, bool AllRobotsReached = false, bool AllRobotsRotated = false);
    void Reset(glm::vec2 position, glm::vec2 velocity);
    void Draw(SpriteRenderer &renderer);
    glm::vec2 GetPosition();

private:
    glm::vec3 defaultColor = glm::vec3(1.0f, 1.0f, 1.0f); // Default color if not specified
};

#endif // CBS_ROBOT