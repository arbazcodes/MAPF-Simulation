#pragma once

#include "graph.h"
#include <vector>
#include <unordered_map>

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
    Direction current_direction;
    std::vector<std::vector<int>> Path;

    Agent(int i, Vertex *vnow, Vertex *vnext, Vertex *s, Vertex *g, float p, bool reached_goal, Direction cd) : id(i), v_now(vnow), v_next(vnext), start(s), goal(g), priority(p), reached_goal(reached_goal), current_direction(cd)
    {
        Path = {{s->x, s->y, s->direction}, {s->x, s->y, s->direction}};
    }
};

// Alias for a collection of agents
using Agents = std::vector<Agent *>;

// PIBT class
class pibt
{
public:
    Graph graph;
    Agents agents;
    bool failed = false;
    int timesteps = 0;

    pibt(int w, int h,
         const std::vector<std::vector<int>> &starts,
         const std::vector<std::vector<int>> &goals);
    ~pibt();

    void run();

    bool PIBT(Agent *ai, Agent *aj = nullptr);
    Agent *FindConflictingAgent(const Vertex *v, const Agent *agent);
    bool allReached();
    void SortAgentsById();

private:
    std::unordered_map<Vertex *, Agent *> occupied_now;
    std::unordered_map<Vertex *, Agent *> occupied_next;

    int HeuristicDistance(const Vertex *start, const Vertex *goal);
    void PrintAgents();
};
