#include "robot.h"

// Constructor with default values
Robot::Robot()
    : InitialPosition(0.0f, 0.0f), CurrentPosition(0.0f, 0.0f), Velocity(0.0f, 0.0f), Radius(1.0f), Sprite(), InitialRotation(0.0f), CurrentRotation(0.0f), AngularVelocity(50.0f)
{ }

// Constructor with specified values
Robot::Robot(glm::vec2 pos, float radius, glm::vec2 velocity, Texture2D sprite)
    : InitialPosition(pos), CurrentPosition(pos), Velocity(velocity), Radius(radius), Sprite(sprite), InitialRotation(0.0f), CurrentRotation(0.0f), AngularVelocity(100.0f)
{}

void Robot::Rotate(float dt)
{
    if (this->currentPathIndex < 1)
        return; // No rotation needed if no previous path

    int targetDirection = Path[this->currentPathIndex][2];
    float targetAngle;

    if (targetDirection == 2)
        targetAngle = 0.0f; // Facing right
    else if (targetDirection == 1)
        targetAngle = -90.0f; // Facing left
    else if (targetDirection == 0)
        targetAngle = 90.0f; // Facing up
    else if (targetDirection == 3)
        targetAngle = 180.0f; // Facing down

    float angleDiff = targetAngle - this->CurrentRotation;

    // Normalize angle difference to the range (-180, 180)
    while (angleDiff > 180.0f)
        angleDiff -= 360.0f;
    while (angleDiff < -180.0f)
        angleDiff += 360.0f;

    if (std::abs(angleDiff) < 1.0f)
    {
        this->CurrentRotation = targetAngle;
        this->isRotating = false;
    }
    else
    {
        float rotationSpeed = this->AngularVelocity * dt;
        if (std::abs(angleDiff) < rotationSpeed)
        {
            this->CurrentRotation = targetAngle;
            this->isRotating = false;
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

void Robot::Move(float dt, float unit_width, float unit_height){
            
        int x = this->Path[currentPathIndex][0] - this->Path[currentPathIndex - 1][0];
        int y = this->Path[currentPathIndex][1] - this->Path[currentPathIndex - 1][1];

        glm::vec2 targetPosition = glm::vec2(this->InitialPosition.x + (x * unit_width), this->InitialPosition.y + (y * unit_height));

        if (glm::distance(this->CurrentPosition, targetPosition) < 1.0f) {
            this->isRotating = true;
            this->InitialPosition = CurrentPosition;
            this->currentPathIndex++;
        }

        glm::vec2 velocity = this->Velocity;
        velocity.x = x * velocity.x;
        velocity.y = y * velocity.y;
        
        glm::vec2 movement = velocity * dt;

        if ( x > 0 )
        {
            if ( this->CurrentPosition.x + movement.x > targetPosition.x )  { this->CurrentPosition.x = targetPosition.x; movement.x = 0; }
        }
        else
        {
            if ( this->CurrentPosition.x + movement.x < targetPosition.x ) { this->CurrentPosition.x = targetPosition.x; movement.x = 0; } 
        }

        if ( y > 0 )
        {
            if ( this->CurrentPosition.y + movement.y > targetPosition.y ) { this->CurrentPosition.y = targetPosition.y; movement.y = 0; }
        }
        else
        {
            if ( this->CurrentPosition.y + movement.y < targetPosition.y ) { this->CurrentPosition.y = targetPosition.y; movement.y = 0; }
        }

        this->CurrentPosition += movement;
}

void Robot::Reset(glm::vec2 CurrentPosition, glm::vec2 velocity)
{
    CurrentPosition = CurrentPosition;
    Velocity = velocity;
}

void Robot::Draw(SpriteRenderer &renderer)
{
    renderer.DrawSprite(Sprite, CurrentPosition, glm::vec2(Radius * 2, Radius * 2), CurrentRotation);
}

glm::vec2 Robot::GetPosition()
{
    return CurrentPosition;
}
