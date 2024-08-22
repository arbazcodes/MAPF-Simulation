#include "sim.h"
#include "resource_manager.h"
#include "sprite_renderer.h"
#include "sim_object.h"
#include "robot.h"
#include "tapf.h"
#include <unistd.h>
#include "pibt.h"
#include <chrono>

// Game-related State data
SpriteRenderer *Renderer;
std::vector<glm::vec2> InitialPositions;

using CostPath = std::vector<std::vector<int>>;

Sim::Sim(unsigned int width, unsigned int height)
    : Width(width), Height(height)
{
}

Sim::~Sim()
{
    Clear();
}

void Sim::Clear()
{
    ResourceManager::Clear();
    delete Renderer;
    for (auto robot : Robots)
    {
        delete robot;
    }
    Robots.clear();
    globalPathIndex = 1;
    idle_robots.clear();
}

void Sim::Init()
{
    ResourceManager::LoadShader("C:/Users/Lenovo/Desktop/Simulation - Copy/shaders/sprite.vs", "C:/Users/Lenovo/Desktop/Simulation/shaders/sprite.fs", nullptr, "sprite");

    // configure shaders
    glm::mat4 projection = glm::ortho(0.0f, static_cast<float>(this->Width),
                                      static_cast<float>(this->Height), 0.0f, -1.0f, 1.0f);
    ResourceManager::GetShader("sprite").Use().SetInteger("image", 0);
    ResourceManager::GetShader("sprite").SetMatrix4("projection", projection);

    // set render-specific controls
    Renderer = new SpriteRenderer(ResourceManager::GetShader("sprite"));

    // load textures
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Simulation - Copy/textures/background.jpg", false, "background");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Simulation - Copy/textures/robot.png", true, "robot");
    ResourceManager::LoadTexture("C:/Users/Lenovo/Desktop/Simulation - Copy/textures/block.png", false, "block");

    // load grid
    grid.Load("C:/Users/Lenovo/Desktop/Simulation - Copy/levels/6x6.lvl", this->Width, this->Height);
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
    pibt *planner;

    try
    {
        int recursive_run = 0;
        int failure_count = 0;
        std::chrono::duration<double> total_duration(0);
        std::chrono::_V2::system_clock::time_point start_time;
        start_time = std::chrono::high_resolution_clock::now();

        while (recursive_run < 10)
        {
            // Create a new planner instance
            planner = new pibt(COLS, ROWS, starts, goals);
            // Run the PIBT algorithm with a timeout
            planner->timesteps = 0;
            planner->failed = false; // Reset failure flag
            planner->run();

            if (planner->failed)
            {
                recursive_run += 1;
                if (recursive_run == 10)
                {
                    // failure_count++;
                    std::cout << "Failed or Timed Out!\n";
                }
            }
            else
            {
                break;
            }
        }

        std::vector<CostPath> sol;

        // Print results
        // std::cout << "Final positions of agents:\n";
        for (const Agent *agent : planner->agents)
        {
            // Convert agent path to robot path format
            std::vector<std::vector<int>> robotPath;
            // std::cout << "Agent " << agent->id << " - Path: ";
            for (const auto vertex : agent->Path)
            {
                // std::cout << "(" << vertex[0]<< ", " << vertex[1] << ", " << vertex[2] << ") ";
                robotPath.push_back({vertex[0], vertex[1], vertex[2]});
            }
            sol.push_back(robotPath);
            // std::cout << std::endl;
        }

        planner->Clear();
        delete planner;

        auto solution = CleanSolution(sol);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> iteration_duration = end_time - start_time;
        total_duration += iteration_duration;

        path_size = solution[0].size();
        idle_robots = {};
        for (int i = 0; i < NUMBER_OF_ROBOTS; ++i)
        {
            glm::vec2 InitialPosition = glm::vec2(((float)solution[i].front()[0] * UnitWidth) + UnitWidth / 2 - RADIUS, ((float)solution[i].front()[1] * UnitHeight) + UnitHeight / 2 - RADIUS);
            glm::vec2 GoalPosition = glm::vec2(((float)solution[i].back()[0] * UnitWidth) + UnitWidth / 2 - RADIUS, ((float)solution[i].back()[1] * UnitHeight) + UnitHeight / 2 - RADIUS);
            glm::vec3 robotColor = glm::vec3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
            Robots.push_back(new Robot(i, InitialPosition, GoalPosition, RADIUS, INITIAL_VELOCITY, ResourceManager::GetTexture("robot"), robotColor));
            Robots[i]->Path = solution[i];
            glm::vec2 destination = glm::vec2((float)solution[i].back()[0] * UnitWidth, (float)solution[i].back()[1] * UnitHeight);
            grid.SetDestinationColor(destination, robotColor);

            for (const auto &step : Robots[i]->Path)
            {
                std::cout << "(" << step[0] << ", " << step[1] << ", " << step[2] << ") ";
            }
            std::cout << std::endl;
        }

        std::cout << "\nDensity (Agents / Number of Cells: " << (float)((NUMBER_OF_ROBOTS / (float)(ROWS * COLS)) * 100) << "%" << std::endl;
        std::cout << "Iteration Time: " << iteration_duration.count() << " seconds" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
        Clear();
        Init();
    }
}

void Sim::Update(float dt)
{
    if (StateChanged())
    {
        Replan();
    }
    else
    {
        if (AllReached() && AllRotated())
        {
            if (globalPathIndex < path_size)
            {
                globalPathIndex++;
                for (auto robot : Robots)
                {
                    robot->isRotating = true;
                    robot->isMoving = false;
                    robot->reached = false;
                }
            }
        }
        for (auto robot : Robots)
        {
            if (globalPathIndex < robot->Path.size())
            {
                robot->currentPathIndex = globalPathIndex;
                if (robot->isRotating)
                {
                    robot->Rotate(dt);
                }
                else if (robot->isMoving)
                {
                    bool allReached = AllReached();
                    bool allRotated = AllRotated();
                    robot->Move(dt, this->UnitWidth, this->UnitHeight, allReached, allRotated);
                }
            }
            robot->UpdateStatus();
        }
    }
}

bool Sim::StateChanged()
{
    for (auto robot : Robots)
    {
        if (robot->status == Status::IDLE)
            idle_robots.insert(robot->id);
        else
            idle_robots.erase(robot->id);
    }
    if (idle_robots.size() == NUMBER_OF_ROBOTS)
        return true;
    return false;
}

void Sim::Replan()
{
    // std::vector<Pair> newStarts;
    // std::vector<Pair> newGoals;

    // std::vector<std::vector<int>> current_positions;
    // for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
    // {
    //     current_positions.push_back({Robots[i]->Path[globalPathIndex]});
    // }

    sleep(1);
    Clear();
    Init();
}

bool Sim::AllReached()
{
    for (auto robot : Robots)
    {
        if (!robot->reached)
            return false;
    }
    return true;
}
bool Sim::AllRotated()
{
    for (auto robot : Robots)
    {
        if (!robot->rotated)
            return false;
    }

    return true;
}

bool Sim::AllReachedGoal()
{
    for (auto robot : Robots)
    {
        if (robot->status == DELIVERING)
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

std::vector<std::vector<std::vector<int>>> Sim::CleanSolution(const std::vector<std::vector<std::vector<int>>> &sol)
{

    std::vector<std::vector<std::vector<int>>> solution = sol;

    int max_length = solution[0].size();

    for (int k = 0; k < solution.size(); k++)
    {
        for (int i = 0; i + 2 < max_length; i++)
        {
            if (solution[k][i][0] == solution[k][i + 2][0] && solution[k][i][1] == solution[k][i + 2][1])
            {
                solution[k][i + 1][0] = solution[k][i + 2][0];
                solution[k][i + 1][1] = solution[k][i + 2][1];
                solution[k][i + 1][2] = solution[k][i + 2][2];
            }
        }
    }
    return solution;
}
