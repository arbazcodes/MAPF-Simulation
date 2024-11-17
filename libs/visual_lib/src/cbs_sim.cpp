#include "cbs_sim.h"
#include "resource_manager.h"
#include "sprite_renderer.h"
#include "sim_object.h"
#include "bounded_astar.h"
#include "cbs_alg.h"
#include "tapf.h"
#include <unistd.h>

// Game-related State data
SpriteRenderer *Renderer;
std::vector<glm::vec2> InitialPositions;

CBS_Sim::CBS_Sim(unsigned int width, unsigned int height)
    : Width(width), Height(height)
{
}

CBS_Sim::~CBS_Sim()
{
    delete Renderer;
    for (auto robot : Robots)
    {
        delete robot;
    }
    Robots.clear();
}

void CBS_Sim::Clear()
{
    ResourceManager::Clear();
    Robots.clear();
}

void CBS_Sim::Init()
{
    ResourceManager::LoadShader("resources/shaders/sprite.vs", "resources//shaders/sprite.fs", nullptr, "sprite");

    // configure shaders
    glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(this->Width),
                                      static_cast<float>(this->Height), 0.0f, -1.0f, 1.0f);
    ResourceManager::GetShader("sprite").Use().SetInteger("image", 0);
    ResourceManager::GetShader("sprite").SetMatrix4("projection", projection);

    // set render-specific controls
    Renderer = new SpriteRenderer(ResourceManager::GetShader("sprite"));

    // load textures
    ResourceManager::LoadTexture("resources/textures/background.jpg", false, "background");
    ResourceManager::LoadTexture("resources/textures/robot.png", true, "robot");
    ResourceManager::LoadTexture("resources/textures/block.png", false, "block");

    // load grid
    grid.Load("resources/levels/6x6.lvl", this->Width, this->Height);
    this->UnitWidth = grid.unitWidth;
    this->UnitHeight = grid.unitHeight;

    InitialPositions = {
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (0.0f * UnitHeight) + UnitHeight / 2 - RADIUS),
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (2.0f * UnitHeight) + UnitHeight / 2 - RADIUS),
        glm::vec2((0.0f * UnitWidth) + UnitWidth / 2 - RADIUS, (1.0f * UnitHeight) + UnitHeight / 2 - RADIUS)};

    auto endpoints = GenerateEndpoints(NUMBER_OF_ROBOTS, ROWS, COLS);

    std::vector<std::vector<int>> start_vecs = endpoints[0];
    std::vector<std::vector<int>> goal_vecs = endpoints[1];

    std::vector<std::pair<int, int>> start_pairs, goal_pairs;

    start_pairs.resize(start_vecs.size());
    goal_pairs.resize(goal_vecs.size());

    for (int i = 0; i < start_vecs.size() && goal_vecs.size(); i++){
        start_pairs[i].first = start_vecs[i][0];
        start_pairs[i].second = start_vecs[i][1];

        goal_pairs[i].first = goal_vecs[i][0];
        goal_pairs[i].second = goal_vecs[i][1];
    }
    
    std::vector<std::vector<int>> Grid(ROWS, std::vector<int>(COLS, 1));

    Cbs cbsAlgorithm(Grid);

    auto paths = cbsAlgorithm.HighLevel(start_pairs, goal_pairs, false);

    if(!paths.has_value())
    {
        std::cout << "TIMED OUT!!!!." << std::endl;
        Clear();
        Init();
    }else{

        auto solution = paths.value();
        for (int i = 0; i < NUMBER_OF_ROBOTS; ++i)
        {
            glm::vec2 InitialPosition = glm::vec2(((float)solution[i][0][0] * UnitWidth) + UnitWidth / 2 - RADIUS, ((float)solution[i][0][1] * UnitHeight) + UnitHeight / 2 - RADIUS);
            glm::vec3 robotColor = glm::vec3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
            Robots.push_back(new CBS_Robot(InitialPosition, RADIUS, INITIAL_VELOCITY, ResourceManager::GetTexture("robot"), robotColor));
            Robots[i]->Path = solution[i];

            glm::vec2 goalPosition = glm::vec2((float) goal_pairs[i].first * UnitWidth, (float) goal_pairs[i].second * UnitHeight);
            grid.SetDestinationColor(goalPosition, robotColor);

            for (const auto &step : Robots[i]->Path)
            {
                std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ", " << step[3] << ") ";
            }
            std::cout << std::endl;
        }
    }

}

void CBS_Sim::Update(float dt)
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

bool CBS_Sim::AllReached()
{
    for (auto robot : Robots)
    {
        if (!robot->reached && robot->currentPathIndex < robot->Path.size())
            return false;
    }

    return true;
}
bool CBS_Sim::AllRotated()
{
    for (auto robot : Robots)
    {
        if (!robot->rotated && robot->currentPathIndex < robot->Path.size())
            return false;
    }

    return true;
}

bool CBS_Sim::AllReachedGoal()
{
    for (auto robot : Robots)
    {
        if (!robot->reachedGoal)
            return false;
    }

    return true;
}

void CBS_Sim::Render()
{
    Renderer->DrawSprite(ResourceManager::GetTexture("background"), glm::vec2(0.0f, 0.0f), glm::vec2(this->Width, this->Height), 0.0f);
    this->grid.Draw(*Renderer);

    for (auto robot : Robots)
    {
        robot->Draw(*Renderer);
    }
}