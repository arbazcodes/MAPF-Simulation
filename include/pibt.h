#pragma once

#include "graph.h"
#include <vector>
#include <unordered_map>

enum Direction
{
    Up,
    Down,
    Left,
    Right,
    None
};

// PIBT agent
struct Agent
{
    int id;
    Vertex *v_now;
    Vertex *v_next;
    Vertex *start;
    Vertex *goal;
    float priority;
    bool reached_goal;
    std::vector<std::vector<int>> Path;
};

// Alias for a collection of agents
using Agents = std::vector<Agent *>;

// PIBT class
class pibt
{
public:
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
    bool AllReached();

private:
    std::unordered_map<Vertex *, Agent *> occupied_now;
    std::unordered_map<Vertex *, Agent *> occupied_next;

    int HeuristicDistance(const Vertex *start, const Vertex *goal);
    void PrintAgents();
};
