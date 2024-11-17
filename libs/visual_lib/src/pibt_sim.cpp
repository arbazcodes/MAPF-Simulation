#include <tapf.h>
#include <unistd.h>
#include <chrono>
#include "pibt_sim.h"
#include "resource_manager.h"
#include "sprite_renderer.h"
#include "sim_object.h"

// Game-related State data
SpriteRenderer *Renderer;
std::vector<glm::vec2> InitialPositions;
std::vector<glm::vec3> RobotsColors;

using CostPath = std::vector<std::vector<int>>;

PIBT_Sim::PIBT_Sim(unsigned int width, unsigned int height)
    : Width(width), Height(height)
{
}

PIBT_Sim::~PIBT_Sim()
{
    Clear();
}

void PIBT_Sim::Clear()
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

void PIBT_Sim::Init()
{
    ResourceManager::LoadShader("resources/shaders/sprite.vs", "resources/shaders/sprite.fs", nullptr, "sprite");

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

    for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
    {
        RobotsColors.push_back(glm::vec3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX));
    }

    auto endpoints = GenerateEndpoints(NUMBER_OF_ROBOTS, ROWS, COLS);

    starts = endpoints[0];
    goals = endpoints[1];

    for (int i = 0; i < starts.size(); i++)
    {
        std::cout << "Start: (" << starts[i][0] << ", " << starts[i][1] << ", " << starts[i][2] << ")---" << "Gaol: (" << goals[i][0] << ", " << goals[i][1] << ", " << goals[i][2] << ")" << std::endl;
    }

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
                planner->SortAgentsById();
                break;
            }
        }

        std::vector<CostPath> sol;

        // Convert agent path to robot path format
        for (const Agent *agent : planner->agents)
        {
            std::vector<std::vector<int>> robotPath;
            for (const auto vertex : agent->Path)
            {
                robotPath.push_back({vertex[0], vertex[1], vertex[2]});
            }
            sol.push_back(robotPath);
        }

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
            glm::vec3 robotColor = RobotsColors[i];
            Robots.push_back(new PIBT_Robot(i, InitialPosition, GoalPosition, RADIUS, INITIAL_VELOCITY, ResourceManager::GetTexture("robot"), robotColor, 0.0f, InitialPosition));
            Robots[i]->Path = solution[i];
            glm::vec2 destination = glm::vec2((float)solution[i].back()[0] * UnitWidth, (float)solution[i].back()[1] * UnitHeight);
            grid.SetDestinationColor(destination, robotColor);
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

void PIBT_Sim::Update(float dt)
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

bool PIBT_Sim::StateChanged()
{
    for (auto robot : Robots)
    {
        if (robot->status == Status::IDLE)
            idle_robots.insert(robot->id);
        else
            idle_robots.erase(robot->id);
    }
    if (!idle_robots.empty())
        return true;
    return false;
}

void PIBT_Sim::Replan()
{
    std::cout << "\nREPLANNED!!!\n";
    std::vector<std::vector<int>> newStarts;
    for (auto robot : Robots)
    {
        if(robot->currentPathIndex < robot->Path.size())
            newStarts.push_back(robot->Path[robot->currentPathIndex]);
    }
    std::vector<std::vector<int>> newGoals = goals;
    for (auto id : idle_robots)
    {
        bool found = false;
        while (!found)
        {
            bool duplicate = false;
            std::vector<std::vector<std::vector<int>>> endpoints = GenerateEndpoints(1, ROWS, COLS);
            for (int i = 0; i < newStarts.size(); i++)
            {
                if (endpoints[0][0][0] == newStarts[id][0] && endpoints[0][0][1] == newStarts[id][1] ||
                    endpoints[0][0][0] == newGoals[i][0] && endpoints[0][0][1] == newGoals[i][1])
                {
                    duplicate = true;
                }
            }
            if (!duplicate)
            {

                found = true;
                newGoals[id] = endpoints[0][0];
            }
        }
    }
    for (int i = 0; i < newStarts.size(); i++)
    {
        std::cout << "New Start: (" << newStarts[i][0] << ", " << newStarts[i][1] << ", " << newStarts[i][2] << ")---" << 
        "New Gaol: (" << newGoals[i][0] << ", " << newGoals[i][1] << ", " << newGoals[i][2] << ")" << std::endl;
    }

        goals = newGoals;

    idle_robots.clear();

    grid.Load("resources/levels/6x6.lvl", this->Width, this->Height);

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
            planner = new pibt(COLS, ROWS, newStarts, newGoals);
            planner->timesteps = 0;
            planner->failed = false;
            planner->run();

            if (planner->failed)
            {
                recursive_run += 1;
                if (recursive_run == 10)
                {
                    std::cout << "Failed or Timed Out!\n";
                }
            }
            else
            {
                planner->SortAgentsById();
                break;
            }
        }

        std::vector<CostPath> sol;
        for (const Agent *agent : planner->agents)
        {
            std::vector<std::vector<int>> robotPath;
            for (const auto vertex : agent->Path)
            {
                robotPath.push_back({vertex[0], vertex[1], vertex[2]});
            }
            sol.push_back(robotPath);
        }

        delete planner;
        globalPathIndex = 1;

        auto solution = CleanSolution(sol);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> iteration_duration = end_time - start_time;
        total_duration += iteration_duration;
        path_size = solution[0].size();

        std::vector<float> current_rotations;
        std::vector<glm::vec2> current_positions;
        for (auto robot : Robots)
        {
            current_rotations.push_back(robot->CurrentRotation);
            current_positions.push_back(robot->CurrentPosition);
        }

        Robots.clear();
        for (int i = 0; i < NUMBER_OF_ROBOTS; ++i)
        {
            glm::vec2 InitialPosition = glm::vec2(((float)newStarts[i][0] * UnitWidth) + UnitWidth / 2 - RADIUS,
                                                  ((float)newStarts[i][1] * UnitHeight) + UnitHeight / 2 - RADIUS);
            glm::vec2 GoalPosition = glm::vec2(((float)newGoals[i][0] * UnitWidth) + UnitWidth / 2 - RADIUS,
                                               ((float)newGoals[i][1] * UnitHeight) + UnitHeight / 2 - RADIUS);
            glm::vec2 destination = glm::vec2((float)newGoals[i][0] * UnitWidth, (float)newGoals[i][1] * UnitHeight);
            glm::vec3 robotColor = RobotsColors[i];
            Robots.push_back(new PIBT_Robot(i, InitialPosition, GoalPosition, RADIUS, INITIAL_VELOCITY,
                                       ResourceManager::GetTexture("robot"), robotColor, current_rotations[i], InitialPosition));
            Robots[i]->Path = solution[i];
            grid.SetDestinationColor(destination, robotColor);
        }

        std::cout << "\nDensity (Agents / Number of Cells: " << (float)((NUMBER_OF_ROBOTS / (float)(ROWS * COLS)) * 100) << "%" << std::endl;
        std::cout << "Iteration Time: " << iteration_duration.count() << " seconds" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << '\n';
    }
}

bool PIBT_Sim::AllReached()
{
    for (auto robot : Robots)
    {
        if (!robot->reached)
            return false;
    }
    return true;
}
bool PIBT_Sim::AllRotated()
{
    for (auto robot : Robots)
    {
        if (!robot->rotated)
            return false;
    }

    return true;
}

bool PIBT_Sim::AllReachedGoal()
{
    for (auto robot : Robots)
    {
        if (robot->status == DELIVERING)
            return false;
    }

    return true;
}

void PIBT_Sim::Render()
{
    Renderer->DrawSprite(ResourceManager::GetTexture("background"), glm::vec2(0.0f, 0.0f), glm::vec2(this->Width, this->Height), 0.0f);
    this->grid.Draw(*Renderer);

    for (auto robot : Robots)
    {
        robot->Draw(*Renderer);
    }
}

std::vector<std::vector<std::vector<int>>> PIBT_Sim::CleanSolution(const std::vector<std::vector<std::vector<int>>> &sol)
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
            if (solution[k][i][2] == solution[k][i + 2][2])
            {
                solution[k][i + 1][2] = solution[k][i + 2][2];
            }
        }
    }
    return solution;
}
