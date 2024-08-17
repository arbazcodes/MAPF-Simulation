#include "sim.h"
#include "resource_manager.h"
#include "sprite_renderer.h"
#include "sim_object.h"
#include "robot.h"
#include "astar.h"
#include "cbs.h"
#include "tapf.h"
#include <unistd.h>
#include "pibt.h"

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

void Sim::Clear()
{
    ResourceManager::Clear();
    Robots.clear();
}

void Sim::Init()
{
    ResourceManager::LoadShader("C:/Users/Lenovo/Desktop/Simulation/shaders/sprite.vs", "C:/Users/Lenovo/Desktop/Simulation/shaders/sprite.fs", nullptr, "sprite");

    // configure shaders
    glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(this->Width),
                                      static_cast<float>(this->Height), 0.0f, -1.0f, 1.0f);
    ResourceManager::GetShader("sprite").Use().SetInteger("image", 0);
    ResourceManager::GetShader("sprite").SetMatrix4("projection", projection);

    // set render-specific controls
    Renderer = new SpriteRenderer(ResourceManager::GetShader("sprite"));

    // load textures
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Simulation/textures/background.jpg", false, "background");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Simulation/textures/robot.png", true, "robot");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Simulation/textures/block.png", false, "block");

    // load grid
    grid.Load("C:/Users/Lenovo/Desktop/Simulation/levels/one.lvl", this->Width, this->Height);
    this->UnitWidth = grid.unitWidth;
    this->UnitHeight = grid.unitHeight;

    InitialPositions = {
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (0.0f * UnitHeight) + UnitHeight / 2 - RADIUS),
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (2.0f * UnitHeight) + UnitHeight / 2 - RADIUS),
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (1.0f * UnitHeight) + UnitHeight / 2 - RADIUS)};

    auto endpoints = GenerateEndpoints(NUMBER_OF_ROBOTS, ROWS, COLS);

    std::vector<Pair> starts = endpoints[0];
    std::vector<Pair> goals = endpoints[1];

    // Create a PIBT planner
    pibt planner(COLS, ROWS, starts, goals);

    try
    {
        // Run the PIBT algorithm
        planner.run();

        // Print results
        std::cout << "Final positions of agents:\n";
        for (const Agent *agent : planner.agents)
        {
            std::cout << "Agent " << agent->id << " - Path: ";
            for (auto vertex : agent->Path)
                std::cout << "(" << vertex.first << ", " << vertex.second << ") ";
            std::cout << std::endl;

            // Convert agent path to robot path format
            std::vector<std::vector<int>> robotPath;
            for (auto &step : agent->Path)
            {
                robotPath.push_back({step.first, step.second, 0, 0}); // Convert to (x, y, 0, 0) format
            }

            glm::vec2 InitialPosition = glm::vec2(((float)agent->Path[0].first * UnitWidth) + UnitWidth / 2 - RADIUS, ((float)agent->Path[0].second * UnitHeight) + UnitHeight / 2 - RADIUS);
            glm::vec3 robotColor = glm::vec3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
            Robots.push_back(new Robot(InitialPosition, RADIUS, INITIAL_VELOCITY, ResourceManager::GetTexture("robot"), robotColor));
            Robots.back()->Path = robotPath; // Set robot path
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        Clear();
        Init();
    }

    // Set destination colors
    for (size_t i = 0; i < goals.size(); ++i)
    {
        glm::vec2 goalPosition = glm::vec2((float)goals[i].first * UnitWidth, (float)goals[i].second * UnitHeight);
        grid.SetDestinationColor(goalPosition, Robots[i]->Color); // Correct attribute name
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
                bool allReached = AllReached();
                bool allRotated = AllRotated();
                robot->Move(dt, this->UnitWidth, this->UnitHeight, allReached, allRotated);
            }
        }else{
            robot->reachedGoal = true;
        }
    }
    if(AllReachedGoal())
    {
        sleep(0.7);
        Clear();
        Init();
    }
}

bool Sim::AllReached()
{
    for (auto robot : Robots)
    {
        if (!robot->reached && robot->currentPathIndex < robot->Path.size())
            return false;
    }

    return true;
}
bool Sim::AllRotated()
{
    for (auto robot : Robots)
    {
        if (!robot->rotated && robot->currentPathIndex < robot->Path.size())
            return false;
    }

    return true;
}

bool Sim::AllReachedGoal()
{
    for (auto robot : Robots)
    {
        if (!robot->reachedGoal)
            return false;
    }

    return true;
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
