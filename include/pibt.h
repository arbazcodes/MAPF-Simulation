#pragma once

#include "graph.h"
#include <vector>

// PIBT agent
struct Agent
{
    int id;
    Vertex *v_now;
    Vertex *v_next;
    Vertex *start;
    Vertex *goal;
    float priority;
    // float tie_breaker;
    bool reached_goal;
    Direction current_direction;
    std::vector<std::vector<int>> Path;
};

// Alias for a collection of agents
using Agents = std::vector<Agent *>;

// PIBT class
class pibt
{
public:
    int timesteps = 0;
    bool failed = false;

    Graph graph;
    Agents agents;
    bool disable_dist_init;

    pibt(int w, int h,
         const std::vector<std::pair<int, int>> &starts,
         const std::vector<std::pair<int, int>> &goals);
    ~pibt();

    void run();

    bool PIBT(Agent *ai, Agent *aj = nullptr);
    Agent *FindConflictingAgent(const Vertex *v, const Agent *agent);
    bool allReached();

private:
    int HeuristicDistance(const Vertex *start, const Vertex *goal, Direction current_direction);
    void PrintAgents();
};
