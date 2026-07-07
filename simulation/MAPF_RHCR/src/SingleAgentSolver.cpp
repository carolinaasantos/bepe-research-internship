// Adapted from RHCR (Rolling-Horizon Collision Resolution).
//
// Original source:
// https://github.com/Jiaoyang-Li/RHCR
//
// Modifications in this file:
// - Replaced the unchecked G.heuristics.at(goal)[curr] lookup in compute_h_value()
//   with a bounds-checked version that uses Manhattan distance as a fallback for
//   obstacle-override nodes (cells reachable only via per-agent traversable sets)
//   whose heuristic table was never computed.
// - Added the same safe fallback for intermediate goal-to-goal heuristic segments.
// - Added can_enter_override_obstacle(): determines whether an agent may step onto
//   a cell that is dynamically blocked or an obstacle override. A dynamic obstacle
//   cell (box on grid) is only enterable when it is exactly the agent's current
//   goal (pickup); opened/delivery cells are freely traversable.

#include "SingleAgentSolver.h"


double SingleAgentSolver::compute_h_value(const BasicGraph& G, int curr, int goal_id,
                             const vector<pair<int, int> >& goal_location) const
{
    if (goal_id < 0 || goal_id >= (int)goal_location.size() || curr < 0 || curr >= G.size())
        return INT_MAX;

    auto it = G.heuristics.find(goal_location[goal_id].first);
    double h;
    bool use_manhattan_fallback = false;
    int first_goal = goal_location[goal_id].first;
    const auto* traversable = current_traversable();
    if (traversable != nullptr)
    {
        // Fallback only for obstacle-override nodes. Otherwise keep disconnected as INT_MAX.
        if ((G.types[curr] == "Obstacle" && traversable->find(curr) != traversable->end()) ||
            (G.types[first_goal] == "Obstacle" && traversable->find(first_goal) != traversable->end()))
            use_manhattan_fallback = true;
    }

    if (it == G.heuristics.end() || curr >= (int)it->second.size() || it->second[curr] >= INT_MAX)
    {
        if (use_manhattan_fallback)
            h = G.get_Manhattan_distance(curr, first_goal);
        else
            return INT_MAX;
    }
    else
        h = it->second[curr];
    goal_id++;
    while (goal_id < (int) goal_location.size())
    {
        int next_goal = goal_location[goal_id].first;
        int prev_goal = goal_location[goal_id - 1].first;
        auto h_it = G.heuristics.find(next_goal);
        if (h_it == G.heuristics.end() || prev_goal < 0 || prev_goal >= (int)h_it->second.size() ||
            h_it->second[prev_goal] >= INT_MAX)
        {
            bool allow_segment_fallback = false;
            if (traversable != nullptr)
            {
                if ((G.types[prev_goal] == "Obstacle" && traversable->find(prev_goal) != traversable->end()) ||
                    (G.types[next_goal] == "Obstacle" && traversable->find(next_goal) != traversable->end()))
                    allow_segment_fallback = true;
            }
            if (allow_segment_fallback)
                h += G.get_Manhattan_distance(prev_goal, next_goal);
            else
                return INT_MAX;
        }
        else
            h += h_it->second[prev_goal];
        goal_id++;
    }
    return h;
}

bool SingleAgentSolver::can_enter_override_obstacle(const BasicGraph& G, int next_loc, int goal_id,
                                    const vector<pair<int, int> >& goal_location) const
{
    if (next_loc < 0 || next_loc >= G.size())
        return false;

    // Dynamically closed cells (boxes currently occupying a cell) can only be
    // entered when they are exactly the agent's current goal (pickup).
    if (G.is_dynamic_blocked(next_loc))
    {
        if (goal_id < 0 || goal_id >= (int)goal_location.size())
            return false;
        return next_loc == goal_location[goal_id].first;
    }

    const auto* traversable = current_traversable();
    if (traversable == nullptr)
        return true;

    if (G.types[next_loc] != "Obstacle")
        return true;

    if (traversable->find(next_loc) == traversable->end())
        return true;

    // This obstacle was already opened (pickup completed or delivery reopened),
    // so it should be freely traversable.
    if (!G.is_dynamic_blocked(next_loc))
        return true;

    if (goal_id < 0 || goal_id >= (int)goal_location.size())
        return false;

    return next_loc == goal_location[goal_id].first;
}
