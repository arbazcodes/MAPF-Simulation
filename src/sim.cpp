#include "sim.h"
#include "resource_manager.h"
#include "sprite_renderer.h"
#include "sim_object.h"
#include "robot.h"
#include "astar.h"

// Game-related State data
SpriteRenderer  *Renderer;
std::vector<glm::vec2> InitialPositions;

Sim::Sim(unsigned int width, unsigned int height) 
    : Width(width), Height(height)
{ 
}

Sim::~Sim()
{
    delete Renderer;
    for (auto robot : Robots) {
        delete robot;
    }
    Robots.clear();
}


void Sim::Init()
{
    ResourceManager::LoadShader("C:/Users/Lenovo/Desktop/Sim/shaders/sprite.vs", "C:/Users/Lenovo/Desktop/sim/shaders/sprite.fs", nullptr, "sprite");
    
    // configure shaders
    glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(this->Width), 
        static_cast<float>(this->Height), 0.0f, -1.0f, 1.0f);
    ResourceManager::GetShader("sprite").Use().SetInteger("image", 0);
    ResourceManager::GetShader("sprite").SetMatrix4("projection", projection);

    // set render-specific controls
    Renderer = new SpriteRenderer(ResourceManager::GetShader("sprite"));

    // load textures
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Sim/textures/background.jpg", false, "background");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Sim/textures/robot.png", true, "robot");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Sim/textures/block.png", false, "block");

    // load grid
    grid.Load("C:/Users/Lenovo/Desktop/Sim/levels/one.lvl", this->Width, this->Height);
    this->UnitWidth = grid.unitWidth;
    this->UnitHeight = grid.unitHeight;

    AStar astar("C:/Users/Lenovo/Desktop/Sim/levels/one.lvl");
    AStar astar1("C:/Users/Lenovo/Desktop/Sim/levels/one.lvl");

    InitialPositions = {
        glm::vec2((1.0f * UnitWidth) + UnitWidth/2 - RADIUS, (1.0f * UnitHeight) + UnitHeight/2 - RADIUS),
        glm::vec2((5.0f * UnitWidth) + UnitWidth/2 - RADIUS, (4.0f * UnitHeight) + UnitHeight/2 - RADIUS)
    };

    Robots.push_back(new Robot(InitialPositions[0], RADIUS, INITIAL_VELOCITY, ResourceManager::GetTexture("robot")));
    Robots[0]->Path = astar.aStarSearch(std::make_pair(1, 1), std::make_pair(5, 1));
    for (const auto& p : Robots[0]->Path) {
        std::cout << "(" << p[0] << "," << p[1] << ") ";
    }
    std::cout << std::endl;
    Robots.push_back(new Robot(InitialPositions[1], RADIUS, INITIAL_VELOCITY, ResourceManager::GetTexture("robot")));
    Robots[1]->Path = astar1.aStarSearch(std::make_pair(5, 4), std::make_pair(2, 2));
    for (const auto& p : Robots[1]->Path) {
        std::cout << "(" << p[0] << "," << p[1] << ") ";
    }
}

void Sim::Update(float dt)
{
    for (auto robot : Robots) {
        if (robot->currentPathIndex < robot->Path.size()) {
            robot->Rotate(dt);

            if(!robot->isRotating)
                robot->Move(dt, this->UnitWidth, this->UnitHeight);
        }
    }
}


void Sim::Render()
{
    Renderer->DrawSprite(ResourceManager::GetTexture("background"), glm::vec2(0.0f, 0.0f), glm::vec2(this->Width, this->Height), 0.0f);
    this->grid.Draw(*Renderer);

    for (auto robot : Robots) {
        robot->Draw(*Renderer);
    }
}
