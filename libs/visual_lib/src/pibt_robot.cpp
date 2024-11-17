#include "pibt_robot.h"

// Constructor with default values
PIBT_Robot::PIBT_Robot()
    : id(), InitialPosition(0.0f, 0.0f), GoalPosition(0.0f, 0.0f), CurrentPosition(0.0f, 0.0f), Velocity(0.0f, 0.0f), Radius(1.0f), Sprite(), InitialRotation(0.0f), CurrentRotation(0.0f), AngularVelocity(300.0f), Color(glm::vec3(1.0f, 1.0f, 1.0f))
{
    reached = false, rotated = false;
}

// Constructor with specified values
PIBT_Robot::PIBT_Robot(int i, glm::vec2 pos, glm::vec2 goal, float radius, glm::vec2 velocity, Texture2D sprite, glm::vec3 color, float current_rotation, glm::vec2 current_postion)
    : id(i), InitialPosition(pos), GoalPosition(goal), CurrentPosition(current_postion), Velocity(velocity), Radius(radius), Sprite(sprite), InitialRotation(0.0f), CurrentRotation(current_rotation), AngularVelocity(500.0f), Color(color)
{
    reached = false, rotated = false;
}

void PIBT_Robot::Rotate(float dt)
{

    targetDirection = Path[currentPathIndex][2];

    switch (targetDirection)
    {
    case 0: // Facing right
        targetAngle = 0.0f;
        break;
    case 1: // Facing left
        targetAngle = 180.0f;
        break;
    case 2: // Facing up
        targetAngle = -90.0f;
        break;
    case 3: // Facing down
        targetAngle = 90.0f;
        break;
    case 4: // No direction change
        targetAngle = CurrentRotation;
        break;
    default:
        return; // Invalid direction
    }

    float angleDiff = targetAngle - this->CurrentRotation;

    // Normalize angle difference to the range [-180, 180]
    while (angleDiff > 180.0f)
        angleDiff -= 360.0f;
    while (angleDiff < -180.0f)
        angleDiff += 360.0f;

    rotated = std::abs(angleDiff) < 1.0f;
    if (rotated)
    {
        this->CurrentRotation = targetAngle;
        this->isRotating = false;
        this->isMoving = true;
        return;
    }

    // Apply rotation based on angular velocity
    float rotationSpeed = this->AngularVelocity * dt;
    if (std::abs(angleDiff) < rotationSpeed)
    {
        this->CurrentRotation = targetAngle;
        rotated = true;
    }
    else
    {
        if (angleDiff > 0)
            this->CurrentRotation += rotationSpeed;
        else
            this->CurrentRotation -= rotationSpeed;

        if (this->CurrentRotation >= 360.0f)
            this->CurrentRotation -= 360.0f;
        else if (this->CurrentRotation < 0.0f)
            this->CurrentRotation += 360.0f;
    }
}

void PIBT_Robot::Move(float dt, float unit_width, float unit_height, bool AllPIBT_RobotsReached, bool AllPIBT_RobotsRotated)
{
    if (!this->isMoving)
        return;

    if (!AllPIBT_RobotsRotated)
        return;

    int x, y;

    if (!reached)
    {
        x = this->Path[currentPathIndex][0] - this->Path[currentPathIndex - 1][0];
        y = this->Path[currentPathIndex][1] - this->Path[currentPathIndex - 1][1];
    }
    else
    {
        x = 0;
        y = 0;
    }


    glm::vec2 targetPosition = glm::vec2(this->InitialPosition.x + (x * unit_width), this->InitialPosition.y + (y * unit_height));

    reached = glm::distance(this->CurrentPosition, targetPosition) < 1.0f;

    if (reached)
    {
        this->isRotating = true;
        this->isMoving = false;
        this->InitialPosition = glm::vec2(((float)this->Path[currentPathIndex][0] * unit_width) + unit_width / 2 - Radius,
                                          ((float)this->Path[currentPathIndex][1] * unit_height) + unit_height / 2 - Radius);
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

bool PIBT_Robot::ReachedGoal()
{
    bool reachedGoal = (glm::distance(this->CurrentPosition, GoalPosition) < 5.0f);
    return reachedGoal;
}

void PIBT_Robot::UpdateStatus()
{
    if (currentPathIndex >= Path.size() || ReachedGoal())
        this->status = IDLE;
    else
        this->status = DELIVERING;
}

void PIBT_Robot::Reset(glm::vec2 position, glm::vec2 velocity)
{
    this->CurrentPosition = position;
    this->Velocity = velocity;
}

void PIBT_Robot::Draw(SpriteRenderer &renderer)
{
    renderer.DrawSprite(Sprite, CurrentPosition, glm::vec2(Radius * 2, Radius * 2), CurrentRotation, Color);
}

glm::vec2 PIBT_Robot::GetPosition()
{
    return CurrentPosition;
}
