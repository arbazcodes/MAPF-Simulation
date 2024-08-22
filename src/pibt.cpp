#include "pibt.h"
#include <algorithm>
#include <iostream>
#include <random>

// TODO: Improve Heuristic
int pibt::HeuristicDistance(const Vertex &start, const Vertex &goal, Direction current_direction)
{
    // Manhattan distance
    int distance = std::abs(start.x - goal.x) + std::abs(start.y - goal.y);
    int movement_cost = (start.x == goal.x && start.y == goal.y) ? 0 : 1;
    int direction_change_cost;
    if ((current_direction == Direction::Up && start.direction == Direction::Down) ||
        (current_direction == Direction::Down && start.direction == Direction::Up) ||
        (current_direction == Direction::Left && start.direction == Direction::Right) ||
        (current_direction == Direction::Right && start.direction == Direction::Left) ||
        (current_direction == Direction::None))
        direction_change_cost = 0;
    else
        direction_change_cost = 1;

    return distance; //+ movement_cost + direction_change_cost;
}

pibt::pibt(int w, int h,
           const std::vector<std::pair<int, int>> &starts,
           const std::vector<std::pair<int, int>> &goals)
    : graph(w, h)
{
    Init(starts, goals);
}

void pibt::Init(const std::vector<std::pair<int, int>> &starts,
                const std::vector<std::pair<int, int>> &goals)
{
    // Create a list of unique priorities
    const size_t num_agents = starts.size();
    std::vector<float> priorities(num_agents);

    // Initialize priorities with evenly spaced values
    for (size_t i = 0; i < num_agents; ++i)
    {
        priorities[i] = static_cast<float>(i) / num_agents;
    }

    // Shuffle priorities to ensure randomness
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(priorities.begin(), priorities.end(), g);

    for (size_t i = 0; i < num_agents; ++i)
    {
        const auto &start = starts[i];
        const auto &goal = goals[i];
        Vertex start_vertex = {start.first, start.second};
        Vertex goal_vertex = {goal.first, goal.second};

        if (graph.locations.find(start_vertex) == graph.locations.end() ||
            graph.locations.find(goal_vertex) == graph.locations.end())
        {
            throw std::runtime_error("Invalid start or goal location.");
        }

        // int init_dist = disable_dist_init ? 0 : HeuristicDistance(start_vertex, goal_vertex);

        Agent *agent = new Agent{
            static_cast<int>(i), // id
            start_vertex,        // current location
            {-1, -1},            // next location (initialize with the current location)
            start_vertex,        // start
            goal_vertex,         // goal
            priorities[i],       // unique priority
            false,               // reached goal
            Direction::Up,       // initialize current direction
            {}                   // initialize path
        };
        agent->Path.push_back({start_vertex.x, start_vertex.y, Direction::None});
        agents.push_back(agent);
    }
}

void pibt::Clear(){
    for (Agent *agent : agents)
    {
        delete agent;
    }
    agents.clear();
    graph.Clear();
}

pibt::~pibt()
{
    Clear();
}

Agent *pibt::FindConflictingAgent(const Vertex &v, const Agent *agent)
{
    for (auto ak : agents)
    {
        if (ak->v_now.x == v.x && ak->v_now.y == v.y && ak->v_next.x == -1 && ak->v_next.y == -1 && ak->id != agent->id)
        {
            return ak;
        }
    }
    return nullptr;
}

bool pibt::allReached()
{
    for (auto agent : agents)
    {
        if (!agent->reached_goal)
            return false;
    }
    return true;
}

void pibt::PrintAgents()
{
    for (auto agent : agents)
    {
        std::cout << "Agent ID: " << agent->id << '\n';
        std::cout << "Current Location: (" << agent->v_now.x << ", " << agent->v_now.y << ")\n";
        std::cout << "Next Location: ";
        if (agent->v_next.x != -1 && agent->v_next.y != - 1)
        {
            std::cout << "(" << agent->v_next.x << ", " << agent->v_next.y << ")\n";
        }
        else
        {
            std::cout << "None\n";
        }
        std::cout << "Goal Location: (" << agent->goal.x << ", " << agent->goal.y << ")\n";
        std::cout << "Priority: " << agent->priority << '\n';
        std::cout << "Reached Goal: " << (agent->reached_goal ? "Yes" : "No") << '\n';
    }
}

// Function to determine next move for an agent
bool pibt::PIBT(Agent *ai, Agent *aj)
{
    float ai_original_priority = ai->priority;
    if (aj)
    {
        ai->priority = std::max(ai->priority, aj->priority);
    }

    auto compare = [&](const Vertex &v, const Vertex &u)
    {
        int d_v = HeuristicDistance(v, ai->goal, ai->current_direction);
        int d_u = HeuristicDistance(u, ai->goal, ai->current_direction);
        return d_v < d_u;
    };

    std::vector<Vertex> candidates = graph.GetNeighbors(ai->v_now);
    std::sort(candidates.begin(), candidates.end(), compare);

    bool found_valid_move = false;

    bool tried_backtracking = false;

    for (const Vertex &u : candidates)
    {
        bool vertex_conflict = false;
        for (auto ak : agents)
        {
            if (ak->v_next.x == u.x && ak->v_next.y == u.y && ak->id != ai->id)
            {
                vertex_conflict = true;
                break;
            }
        }

        bool higher_priority_conflict = false;
        for (auto ak : agents)
        {
            if (ak->v_now.x == u.x && ak->v_now.y == u.y && ak->id != ai->id && ak->priority >= ai->priority)
            {
                higher_priority_conflict = true;
                break;
            }
        }

        bool follow_conflict = false;
        for (auto ak : agents)
        {
            if (ak->id != ai->id && ak->v_next.x != -1 && ak->v_next.y != -1 && ak->v_now.x == u.x && ak->v_now.y == u.y)
            {
                follow_conflict = true;
                break;
            }
        }

        if (vertex_conflict || higher_priority_conflict || follow_conflict || (aj && aj->v_now.x == u.x && aj->v_now.y == u.y))
        {
            continue;
        }

        ai->v_next = u;
        Agent *conflicting_agent = FindConflictingAgent(u, ai);

        bool priority_inherit_done = false;

        if (conflicting_agent && conflicting_agent->priority < ai->priority)
        {
            tried_backtracking = true;

            if (!PIBT(conflicting_agent, ai))
            {
                priority_inherit_done = false;
                continue;
            }
            else
            {
                priority_inherit_done = true;
            }
        }

        if (!tried_backtracking)
        {
            found_valid_move = true;
            break;
        }
        else
        {
            if (priority_inherit_done)
            {
                ai->v_next = ai->v_now;
                found_valid_move = true;
                break;
            }
            else
            {
                ai->v_next = Vertex(-1, -1);
                continue;
            }
        }
    }

    if (!found_valid_move)
    {
        ai->v_next = ai->v_now;
    }

    ai->priority = ai_original_priority;
    return found_valid_move;
}

void pibt::run()
{
    auto compare = [](Agent *a, const Agent *b)
    {
        return a->priority > b->priority;
    };

    while (!allReached())
    {
        for (auto *agent : agents)
        {
            if (agent->v_now.x != agent->goal.x && agent->v_next.y != agent->goal.y)
                agent->priority++;
            else
                agent->reached_goal = true;

            if (agent->v_next.x != -1 && agent->v_next.y != -1)
            {
                Direction new_direction = agent->v_next.direction;
                // Maintain direction consistency for opposite moves
                if ((new_direction == Direction::Up && agent->current_direction == Direction::Down) ||
                    (new_direction == Direction::Down && agent->current_direction == Direction::Up) ||
                    (new_direction == Direction::Left && agent->current_direction == Direction::Right) ||
                    (new_direction == Direction::Right && agent->current_direction == Direction::Left) ||
                    (new_direction == Direction::None))
                {
                    new_direction = agent->current_direction;
                }

                agent->Path.push_back({agent->v_next.x, agent->v_next.y, new_direction});
                agent->current_direction = new_direction; // Update previous direction
                agent->v_now = agent->v_next;
                agent->v_next = Vertex(-1, -1); // Set to invalid
            }
        }

        std::sort(agents.begin(), agents.end(), compare);

        for (auto *agent : agents)
        {
            if (agent->v_next.x == -1 && agent->v_next.y == -1)
            {
                PIBT(agent, nullptr);
            }
        }

        timesteps++;

        if (timesteps > (agents.size() * std::max(graph.width, graph.height) * 10))
        {
            failed = true;
            timesteps = 0;
            return;
        }
    }
}
