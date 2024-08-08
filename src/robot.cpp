#include "robot.h"

// Constructor with default values
Robot::Robot()
    : InitialPosition(0.0f, 0.0f), CurrentPosition(0.0f, 0.0f), Velocity(0.0f, 0.0f), Radius(1.0f), Sprite(), InitialRotation(0.0f), CurrentRotation(0.0f), AngularVelocity(50.0f), Color(glm::vec3(1.0f, 1.0f, 1.0f))
{
}

// Constructor with specified values
Robot::Robot(glm::vec2 pos, float radius, glm::vec2 velocity, Texture2D sprite, glm::vec3 color)
    : InitialPosition(pos), CurrentPosition(pos), Velocity(velocity), Radius(radius), Sprite(sprite), InitialRotation(0.0f), CurrentRotation(0.0f), AngularVelocity(300.0f), Color(color)
{
}

void Robot::Rotate(float dt)
{
    if (this->currentPathIndex >= Path.size())
        return; // No rotation needed if no path remaining

    int targetDirection = Path[this->currentPathIndex][2];
    float targetAngle;

    switch (targetDirection)
    {
    case 0:
        targetAngle = 0.0f;
        break; // Facing right
    case 1:
        targetAngle = 180.0f;
        break; // Facing left
    case 2:
        targetAngle = -90.0f;
        break; // Facing up
    case 3:
        targetAngle = 90.0f;
        break; // Facing dow
    case 4:
        targetAngle = CurrentRotation;
        break; 
    default:
        return; // Invalid direction
    }

    float angleDiff = targetAngle - this->CurrentRotation;

    while (angleDiff > 180.0f)
        angleDiff -= 360.0f;
    while (angleDiff < -180.0f)
        angleDiff += 360.0f;

    if (std::abs(angleDiff) < 1.0f)
    {
        this->CurrentRotation = targetAngle;
        this->isRotating = false;
        this->isMoving = true;
    }
    else
    {
        float rotationSpeed = this->AngularVelocity * dt;
        if (std::abs(angleDiff) < rotationSpeed)
        {
            this->CurrentRotation = targetAngle;
            this->isRotating = false;
            this->isMoving = true;
        }
        else
        {
            if (angleDiff > 0)
                this->CurrentRotation += rotationSpeed;
            else
                this->CurrentRotation -= rotationSpeed;
        }
    }
}

void Robot::Move(float dt, float unit_width, float unit_height)
{
    if (!this->isMoving)
        return;

    if (this->currentPathIndex >= Path.size())
        return;

    int x = this->Path[currentPathIndex][0] - this->Path[currentPathIndex - 1][0];
    int y = this->Path[currentPathIndex][1] - this->Path[currentPathIndex - 1][1];

    glm::vec2 targetPosition = glm::vec2(this->InitialPosition.x + (x * unit_width), this->InitialPosition.y + (y * unit_height));

    if (glm::distance(this->CurrentPosition, targetPosition) < 1.0f)
    {
        this->isRotating = true;
        this->isMoving = false;
        this->InitialPosition = CurrentPosition;
        this->currentPathIndex++;
        return;
    }

    glm::vec2 velocity = this->Velocity;
    velocity.x = x * velocity.x;
    velocity.y = y * velocity.y;

    glm::vec2 movement = velocity * dt;

    if (x > 0)
    {
        if (this->CurrentPosition.x + movement.x > targetPosition.x)
        {
            this->CurrentPosition.x = targetPosition.x;
            movement.x = 0;
        }
    }
    else
    {
        if (this->CurrentPosition.x + movement.x < targetPosition.x)
        {
            this->CurrentPosition.x = targetPosition.x;
            movement.x = 0;
        }
    }

    if (y > 0)
    {
        if (this->CurrentPosition.y + movement.y > targetPosition.y)
        {
            this->CurrentPosition.y = targetPosition.y;
            movement.y = 0;
        }
    }
    else
    {
        if (this->CurrentPosition.y + movement.y < targetPosition.y)
        {
            this->CurrentPosition.y = targetPosition.y;
            movement.y = 0;
        }
    }

    this->CurrentPosition += movement;
}

bool Robot::IsFacingTargetDirection(int targetDirection)
{
    float targetAngle;
    switch (targetDirection)
    {
    case 2:
        targetAngle = 0.0f;
        break; // Facing right
    case 1:
        targetAngle = -90.0f;
        break; // Facing left
    case 0:
        targetAngle = 90.0f;
        break; // Facing up
    case 3:
        targetAngle = 180.0f;
        break; // Facing down
    default:
        return false; // Invalid direction
    }

    float angleDiff = targetAngle - this->CurrentRotation;

    while (angleDiff > 180.0f)
        angleDiff -= 360.0f;
    while (angleDiff < -180.0f)
        angleDiff += 360.0f;

    return std::abs(angleDiff) < 1.0f;
}

void Robot::Reset(glm::vec2 position, glm::vec2 velocity)
{
    this->CurrentPosition = position;
    this->Velocity = velocity;
}

void Robot::Draw(SpriteRenderer &renderer)
{
    renderer.DrawSprite(Sprite, CurrentPosition, glm::vec2(Radius * 2, Radius * 2), CurrentRotation, Color);
}

glm::vec2 Robot::GetPosition()
{
    return CurrentPosition;
}
