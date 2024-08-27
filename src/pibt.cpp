#include "pibt.h"
#include <algorithm>
#include <iostream>
#include <random>

int pibt::HeuristicDistance(const Vertex *start, const Vertex *goal)
{
    return std::abs(start->x - goal->x) + std::abs(start->y - goal->y);
}

pibt::pibt(int w, int h,
           const std::vector<std::vector<int>> &starts,
           const std::vector<std::vector<int>> &goals)
    : graph(w, h),
      agents()
{
    // Create a list of unique priorities
    const int num_agents = starts.size();
    std::vector<float> priorities(num_agents);

    // Initialize priorities with evenly spaced values
    for (int i = 0; i < num_agents; ++i)
    {
        priorities[i] = (float)(i) / num_agents;
    }

    // Shuffle priorities to ensure randomness
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(priorities.begin(), priorities.end(), g);

    for (int i = 0; i < num_agents; ++i)
    {
        const auto &start = starts[i];
        const auto &goal = goals[i];
        Vertex *start_vertex = nullptr;
        Vertex *goal_vertex = nullptr;

        for (Vertex *v : graph.locations)
        {
            if (v->x == start[0] && v->y == start[1])
            {
                start_vertex = v;
            }
            if (v->x == goal[0] && v->y == goal[1])
            {
                goal_vertex = v;
            }
        }

        if (!start_vertex || !goal_vertex)
        {
            throw std::runtime_error("Invalid start or goal location.");
        }

        Agent *agent = new Agent(
            (int)(i),            // id
            start_vertex,        // current location
            nullptr,             // next location
            start_vertex,        // start
            goal_vertex,         // goal
            priorities[i],       // unique priority
            false,               // reached goal
            (Direction)start[2] // initialize current direction
        );
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

void pibt::SortAgentsById()
{
    std::sort(agents.begin(), agents.end(), [](const Agent *a, const Agent *b)
              { return a->id < b->id; });
}

// Function to determine next move for an agent
bool pibt::PIBT(Agent *ai, Agent *aj)
{
    auto compare = [&](Vertex *const v, Vertex *const u)
    {
        int d_v = HeuristicDistance(v, ai->goal);
        int d_u = HeuristicDistance(u, ai->goal);
        return d_v < d_u;
    };

    std::vector<Vertex *> candidates = graph.GetNeighbors(ai->v_now);
    candidates.push_back(ai->v_now); // Include current vertex as a candidate
    std::sort(candidates.begin(), candidates.end(), compare);

    for (Vertex *u : candidates)
    {
        bool vertex_conflict = false;
        for (auto ak : agents)
        {
            if (ak->id == ai->id)
                continue;
            if (ak->v_next != nullptr)
            {
                if (ak->v_next == u)
                {
                    vertex_conflict = true;
                    break;
                }
                if (ak->v_now == u)
                {
                    vertex_conflict = true;
                    break;
                }
            }
        }

        if (vertex_conflict || (aj && aj->v_now == u))
        {
            continue;
        }

        ai->v_next = u;
        bool found_valid_move = true;
        bool inherited = false;

        for (auto ak : agents)
        {
            if (ak->id == ai->id)
                continue;

            if (!(ak->v_now == u))
                continue;
            if (ak->v_next != nullptr)
                continue;

            if (PIBT(ak, ai))
            {
                inherited = true;
            }
            else
            {
                found_valid_move = false;
            }

            // break;
        }

        if (!found_valid_move)
        {
            // ai->v_next = ai->v_now;
            ai->v_next = nullptr;
            continue;
        }

        int dx = ai->v_now->x - u->x;
        int dy = ai->v_now->y - u->y;
        bool moving_side = (ai->current_direction == 0 || ai->current_direction == 1) && dx;
        bool moving_side_up = (ai->current_direction == 2 || ai->current_direction == 3) && dy;

        if ((found_valid_move && inherited) || moving_side || moving_side_up)
        {
            ai->v_next = ai->v_now;
            if (moving_side || moving_side_up)
                ai->current_direction = u->direction;
        }

        return found_valid_move;
    }

    ai->v_next = ai->v_now;

    return false;
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
                Direction new_direction = Direction::None;
                if (agent->v_next->x == agent->v_now->x && agent->v_next->y == agent->v_now->y - 1)
                    new_direction = Direction::Up;
                else if (agent->v_next->x == agent->v_now->x && agent->v_next->y == agent->v_now->y + 1)
                    new_direction = Direction::Down;
                else if (agent->v_next->x == agent->v_now->x - 1 && agent->v_next->y == agent->v_now->y)
                    new_direction = Direction::Left;
                else if (agent->v_next->x == agent->v_now->x + 1 && agent->v_next->y == agent->v_now->y)
                    new_direction = Direction::Right;

                // Maintain direction consistency for opposite moves
                if ((new_direction == Direction::Up && agent->current_direction == Direction::Down) ||
                    (new_direction == Direction::Down && agent->current_direction == Direction::Up) ||
                    (new_direction == Direction::Left && agent->current_direction == Direction::Right) ||
                    (new_direction == Direction::Right && agent->current_direction == Direction::Left) ||
                    (new_direction == Direction::None))
                {
                    new_direction = agent->current_direction;
                }

                agent->Path.push_back({agent->v_next->x, agent->v_next->y, (int) new_direction});
                agent->current_direction = new_direction; // Update previous direction
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
        ++timesteps;

        timesteps++;

        if (timesteps > (agents.size() * std::max(graph.width, graph.height) * 10))
        {
            failed = true;
            timesteps = 0;
            return;
        }
    }
}
