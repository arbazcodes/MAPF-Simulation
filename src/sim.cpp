#include "sim.h"
#include "resource_manager.h"
#include "sprite_renderer.h"
#include "sim_object.h"
#include "robot.h"
#include "astar.h"
#include "cbs.h"

// Game-related State data
SpriteRenderer *Renderer;
std::vector<glm::vec2> InitialPositions;

Sim::Sim(unsigned int width, unsigned int height)
    : Width(width), Height(height)
{
}

Sim::~Sim()
{
    delete Renderer;
    for (auto robot : Robots)
    {
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
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Sim1/textures/background.jpg", false, "background");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Sim1/textures/robot.png", true, "robot");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Sim1/textures/block.png", false, "block");

    // load grid
    grid.Load("C:/Users/Lenovo/Desktop/Sim1/levels/one.lvl", this->Width, this->Height);
    this->UnitWidth = grid.unitWidth;
    this->UnitHeight = grid.unitHeight;

    InitialPositions = {
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (0.0f * UnitHeight) + UnitHeight / 2 - RADIUS),
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (2.0f * UnitHeight) + UnitHeight / 2 - RADIUS),
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (1.0f * UnitHeight) + UnitHeight / 2 - RADIUS)};

    std::vector<Pair> starts = {
        {0, 0},
        //{0, 2},
        {0, 1},
        {1, 1},
        {1, 0},
        {2, 1},
        {2, 2},
        //{2, 0}
    };
    std::vector<Pair> goals = {
        {0, 2},
        //{0, 0},
        {1, 1},
        {1, 0},
        {2, 1},
        {1, 2},
        {2, 0},
        //{2, 2}
    };

    std::vector<std::vector<int>> Grid = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

    Cbs cbsAlgorithm(Grid);

    std::vector<CostPath> solution = cbsAlgorithm.HighLevel(starts, goals);

    for (int i = 0; i < NUMBER_OF_ROBOTS; ++i)
    {
        glm::vec2 InitialPosition = glm::vec2(((float)starts[i].first * UnitWidth) + UnitWidth / 2 - RADIUS, ((float)starts[i].second * UnitHeight) + UnitHeight / 2 - RADIUS);
        glm::vec3 robotColor = glm::vec3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
        Robots.push_back(new Robot(InitialPosition, RADIUS, INITIAL_VELOCITY, ResourceManager::GetTexture("robot"), robotColor));
        Robots[i]->Path = solution[i];

        glm::vec2 goalPosition = glm::vec2((float)goals[i].first * UnitWidth, (float)goals[i].second * UnitHeight);
        grid.SetDestinationColor(goalPosition, robotColor);

        for (const auto &step : Robots[i]->Path)
        {
            std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ", " << step[3] << ") ";
        }
        std::cout << std::endl;
    }
}

void Sim::Update(float dt)
{
    for (auto robot : Robots)
    {
        if (robot->currentPathIndex < robot->Path.size())
        {
            if (robot->isRotating)
            {
                robot->Rotate(dt);
            }
            else
            {
                robot->Move(dt, this->UnitWidth, this->UnitHeight);
            }
        }
    }
}

void Sim::Render()
{
    Renderer->DrawSprite(ResourceManager::GetTexture("background"), glm::vec2(0.0f, 0.0f), glm::vec2(this->Width, this->Height), 0.0f);
    this->grid.Draw(*Renderer);

    for (auto robot : Robots)
    {
        robot->Draw(*Renderer);
    }
}
