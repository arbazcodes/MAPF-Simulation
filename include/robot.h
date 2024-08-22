#ifndef BALL_OBJECT_H
#define BALL_OBJECT_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <iostream>
#include <stdio.h>
#include "texture.h"
#include "sprite_renderer.h"
#include <vector>

enum Status {IDLE, DELIVERING};

class Robot
{
public:
    float Radius;
    Texture2D Sprite;
    int id;
    Status status = Status::DELIVERING;
    bool isRotating = true;
    bool isMoving = false;
    float InitialRotation, CurrentRotation, AngularVelocity;
    bool reached, rotated;
    int targetDirection, targetAngle;

    glm::vec2 InitialPosition, CurrentPosition, GoalPosition, Velocity;
    glm::vec3 Color; // New attribute for color
    std::vector<std::vector<int>> Path;
    int currentPathIndex;

    Robot();
    Robot(int i, glm::vec2 pos, glm::vec2 goal, float radius, glm::vec2 velocity, Texture2D sprite, glm::vec3 color);

    void UpdateStatus();
    bool ReachedGoal();
    void Rotate(float dt);
    void Move(float dt, float unit_width, float unit_height, bool AllRobotsReached = false, bool AllRobotsRotated = false);
    void Reset(glm::vec2 position, glm::vec2 velocity);
    void Draw(SpriteRenderer &renderer);
    glm::vec2 GetPosition();

private:
    glm::vec3 defaultColor = glm::vec3(1.0f, 1.0f, 1.0f); // Default color if not specified
};

#endif
