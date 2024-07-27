#include "sim_object.h"


SimObject::SimObject() 
    : Position(0.0f, 0.0f), Size(1.0f, 1.0f), Color(1.0f), Sprite(){ }

SimObject::SimObject(glm::vec2 pos, glm::vec2 size, Texture2D sprite, glm::vec3 color, glm::vec2 velocity) 
    : Position(pos), Size(size), Color(color),  Sprite(sprite){ }

void SimObject::Draw(SpriteRenderer &renderer)
{
    renderer.DrawSprite(this->Sprite, this->Position, this->Size, 0.0, this->Color);
}