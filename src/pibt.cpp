#include "pibt.h"
#include <algorithm>
#include <iostream>
#include <random>

int pibt::HeuristicDistance(const Vertex *start, const Vertex *goal)
{
    return std::abs(start->x - goal->x) + std::abs(start->y - goal->y);
}

pibt::pibt(int w, int h,
           const std::vector<std::pair<int, int>> &starts,
           const std::vector<std::pair<int, int>> &goals)
    : graph(w, h),
      agents(),
      disable_dist_init(false)
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
        Vertex *start_vertex = nullptr;
        Vertex *goal_vertex = nullptr;

        for (Vertex *v : graph.locations)
        {
            if (v->x == start.first && v->y == start.second)
            {
                start_vertex = v;
            }
            if (v->x == goal.first && v->y == goal.second)
            {
                goal_vertex = v;
            }
        }

        if (!start_vertex || !goal_vertex)
        {
            throw std::runtime_error("Invalid start or goal location.");
        }

        int init_dist = disable_dist_init ? 0 : HeuristicDistance(start_vertex, goal_vertex);

        Agent *agent = new Agent{
            static_cast<int>(i), // id
            start_vertex,        // current location
            nullptr,             // next location
            start_vertex,        // start
            goal_vertex,         // goal
            priorities[i],       // unique priority
            false,               // reached goal
            Direction::None,     // initialize current direction
            Direction::None,     // initialize previous direction
            {}                   // initialize path
        };
        agent->Path.push_back({start_vertex->x, start_vertex->y, Direction::None});
        agents.push_back(agent);
    }
}

pibt::~pibt()
{
    for (Agent *agent : agents)
    {
        delete agent;
    }
}

Agent *pibt::FindConflictingAgent(const Vertex *v, const Agent *agent)
{
    for (auto ak : agents)
    {
        if (ak->v_now == v && ak->v_next == nullptr && ak->id != agent->id)
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
        std::cout << "Current Location: (" << agent->v_now->x << ", " << agent->v_now->y << ")\n";
        std::cout << "Next Location: ";
        if (agent->v_next)
        {
            std::cout << "(" << agent->v_next->x << ", " << agent->v_next->y << ")\n";
        }
        else
        {
            std::cout << "None\n";
        }
        std::cout << "Goal Location: (" << agent->goal->x << ", " << agent->goal->y << ")\n";
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

    auto compare = [&](Vertex *const v, Vertex *const u)
    {
        int d_v = HeuristicDistance(v, ai->goal);
        int d_u = HeuristicDistance(u, ai->goal);
        return d_v < d_u;
    };

    std::vector<Vertex *> candidates = graph.GetNeighbors(ai->v_now);
    candidates.push_back(ai->v_now); // Include current vertex as a candidate
    std::sort(candidates.begin(), candidates.end(), compare);

    bool found_valid_move = false;

    for (Vertex *u : candidates)
    {
        bool vertex_conflict = false;
        for (auto ak : agents)
        {
            if (ak->v_next == u && ak->id != ai->id)
            {
                vertex_conflict = true;
                break;
            }
        }

        bool follow_conflict = false;
        for (auto ak : agents)
        {
            if (ak->v_now == u && ak->id != ai->id)
            {
                follow_conflict = true;
                break;
            }
        }

        follow_conflict = false;

        if (vertex_conflict || follow_conflict || (aj && aj->v_now == u))
        {
            continue;
        }

        ai->v_next = u;
        Agent *conflicting_agent = FindConflictingAgent(u, ai);

        if (conflicting_agent && conflicting_agent->priority < ai->priority)
        {
            if (!PIBT(conflicting_agent, ai))
            {
                continue;
            }
        }

        found_valid_move = true;
        break;
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
            if (!(agent->v_now == agent->goal))
                agent->priority++;
            else
                agent->reached_goal = true;

            if (agent->v_next != nullptr)
            {
                agent->current_direction = Direction::None;

                if (agent->v_next->x == agent->v_now->x && agent->v_next->y == agent->v_now->y - 1)
                    agent->current_direction = Direction::Up;
                else if (agent->v_next->x == agent->v_now->x && agent->v_next->y == agent->v_now->y + 1)
                    agent->current_direction = Direction::Down;
                else if (agent->v_next->x == agent->v_now->x - 1 && agent->v_next->y == agent->v_now->y)
                    agent->current_direction = Direction::Left;
                else if (agent->v_next->x == agent->v_now->x + 1 && agent->v_next->y == agent->v_now->y)
                    agent->current_direction = Direction::Right;

                // Maintain direction consistency for opposite moves
                if ((agent->current_direction == Direction::Up && agent->prev_direction == Direction::Down) ||
                    (agent->current_direction == Direction::Down && agent->prev_direction == Direction::Up) ||
                    (agent->current_direction == Direction::Left && agent->prev_direction == Direction::Right) ||
                    (agent->current_direction == Direction::Right && agent->prev_direction == Direction::Left))
                {
                    agent->current_direction = agent->prev_direction;
                }

                agent->Path.push_back({agent->v_next->x, agent->v_next->y, agent->current_direction});
                agent->prev_direction = agent->current_direction; // Update previous direction
                agent->v_now = agent->v_next;
                agent->v_next = nullptr;
            }
        }

        std::sort(agents.begin(), agents.end(), compare);

        for (auto *agent : agents)
        {
            if (agent->v_next == nullptr)
            {
                PIBT(agent, nullptr);
            }
        }
    }
}
