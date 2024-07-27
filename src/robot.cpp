#include "robot.h"

// Constructor with default values
Robot::Robot()
    : InitialPosition(0.0f, 0.0f), CurrentPosition(0.0f, 0.0f), Velocity(0.0f, 0.0f), Radius(1.0f), Sprite(), InitialRotation(0.0f), CurrentRotation(0.0f), AngularVelocity(50.0f)
{ }

// Constructor with specified values
Robot::Robot(glm::vec2 pos, float radius, glm::vec2 velocity, Texture2D sprite)
    : InitialPosition(pos), CurrentPosition(pos), Velocity(velocity), Radius(radius), Sprite(sprite), InitialRotation(0.0f), CurrentRotation(0.0f), AngularVelocity(100.0f)
{}

void Robot::Rotate(float dt){

    int x = this->Path[this->currentPathIndex][0] - this->Path[this->currentPathIndex - 1][0];
    int y = this->Path[this->currentPathIndex][1] - this->Path[this->currentPathIndex - 1][1];

    float targetAngle;
            
    if(x == 1)
        targetAngle =  90;  
    if(x == -1)
        targetAngle =  -90;
    if(y == 1)
        targetAngle = 180; 
    if(y == -1)
        targetAngle = 0; 

    if(std::abs(this->CurrentRotation - targetAngle) < 1.0f)
        this->isRotating = false;

    float direction = glm::normalize(targetAngle - this->CurrentRotation);
    float rotation = direction * this->AngularVelocity * dt;
    this->CurrentRotation += rotation;
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
