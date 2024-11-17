#ifndef SIM_OBJECT_H
#define SIM_OBJECT_H

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "texture.h"
#include "sprite_renderer.h"

class SimObject
{
public:
    glm::vec2   Position, Size;
    glm::vec3   Color;
    Texture2D   Sprite;	

    SimObject();
    SimObject(glm::vec2 pos, glm::vec2 size, Texture2D sprite, glm::vec3 color = glm::vec3(1.0f), glm::vec2 velocity = glm::vec2(0.0f, 0.0f));

    virtual void Draw(SpriteRenderer &renderer);
};

#endif