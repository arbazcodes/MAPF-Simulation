#ifndef GAMELEVEL_H
#define GAMELEVEL_H
#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include "sim_object.h"
#include "sprite_renderer.h"
#include "resource_manager.h"

class Grid
{
public:
    std::vector<SimObject> Bricks;
    unsigned int unitWidth, unitHeight;

    Grid() {}

    void Load(const char *file, unsigned int levelWidth, unsigned int levelHeight);
    void Draw(SpriteRenderer &renderer);
private:
    void init(std::vector<std::vector<unsigned int>> tileData, unsigned int levelWidth, unsigned int levelHeight);
};

#endif