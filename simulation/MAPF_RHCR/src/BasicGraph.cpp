// ------------------------------------------------------------------
// Adapted from RHCR (Rolling-Horizon Collision Resolution).
//
// Original source:
// https://github.com/Jiaoyang-Li/RHCR
//
// Modifications in this file:
// - Added is_traversable() method: centralises obstacle/dynamic-block reachability
//   check used by valid_move() and get_weight().
// - Added valid_move() method: replaces direct weights[loc][dir] < WEIGHT_MAX checks
//   throughout the codebase; respects dynamic_blocked cells, per-agent traversable
//   overrides, and allows an agent to leave a cell that was dynamically closed under it.
// - Updated get_neighbors(int), get_neighbors(State), get_reverse_neighbors(State)
//   to accept an optional traversable pointer and delegate to valid_move().
// - Updated get_weight() to accept a traversable pointer and return a fallback cost
//   of 1 for agent-specific override edges instead of WEIGHT_MAX.
// - Removed the post-hoc Obstacle re-labelling pass that set types[j]="Obstacle"
//   for disconnected nodes; this was incompatible with dynamic obstacle cells.
// ------------------------------------------------------------------

#include "BasicGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>


void BasicGraph::print_map() const
{  
    std::cout << "***type***" << std::endl;
    for (std::string t : types)
        std::cout << t << ",";
    std::cout << std::endl;

    std::cout << "***weights***" << std::endl;
    for (std::vector<double> n : weights)
    {
        for (double w : n)
        {
            std::cout << w << ",";
        }
        std::cout << std::endl;
    }
}


int BasicGraph::get_rotate_degree(int dir1, int dir2) const
{
    if (dir1 == dir2)
        return 0;
    else if (abs(dir1 - dir2) == 1 || abs(dir1 - dir2) == 3)
        return 1;
    else
        return 2;
}

// ------------------------------------------------------------------
// Centralised reachability check used by both valid_move() and
// get_weight().
//
// Centralising the logic here guarantees that every call site —
// whether computing edge costs or expanding neighbours — applies
// the same traversability rules, so heuristic values and actual path
// expansion can never disagree.
//
// A cell is reachable when:
//   - It is not typed as "Obstacle" in the static map, or
//   - It is explicitly listed in the caller's per-agent traversable
//     set (covering pickup/delivery cells temporarily assigned to
//     a specific agent).
//
// Dynamic obstacle cells (boxes currently present on the grid) are
// never freely reachable; they always require explicit permission.
// ------------------------------------------------------------------

bool BasicGraph::is_traversable(int loc, const unordered_set<int>* traversable) const
{
    if (loc < 0 || loc >= this->size())
        return false;
    if (dynamic_blocked.find(loc) != dynamic_blocked.end())
        return traversable != nullptr && traversable->find(loc) != traversable->end();
    if (types[loc] != "Obstacle")
        return true;
    return traversable != nullptr && traversable->find(loc) != traversable->end();
}

// ------------------------------------------------------------------
// Unified edge validity check that replaces all direct
// weights[loc][dir] < WEIGHT_MAX comparisons throughout the codebase.
//
// Three behaviours were added beyond the original static check:
//
// 1. Dynamic blocked source: if the cell the agent is leaving was
//    closed by a delivery after the agent arrived, static edge
//    weights are WEIGHT_MAX. The agent must still be allowed to
//    leave to prevent deadlock (see escape-route logic below).
//
// 2. Dynamic blocked destination: entering a cell that currently
//    holds a box requires the destination to appear in the per-agent
//    traversable set. The finer goal-level restriction — only
//    passable when the cell is the agent's current search goal —
//    is enforced by the calling solver via can_enter_override_obstacle().
//
// 3. Agent-specific override edges: when either endpoint is listed
//    in the traversable set and the standard weight would be
//    WEIGHT_MAX, the edge is treated as passable so the agent can
//    reach its assigned pickup/delivery cell.
// ------------------------------------------------------------------

bool BasicGraph::valid_move(int loc, int dir, const unordered_set<int>* traversable) const
{
    if (loc < 0 || loc >= this->size() || dir < 0 || dir >= 4)
        return false;
    bool from_dynamic_blocked = (dynamic_blocked.find(loc) != dynamic_blocked.end());
    if (!from_dynamic_blocked && !is_traversable(loc, traversable))
        return false;
    int to = loc + move[dir];
    if (to < 0 || to >= this->size() || get_Manhattan_distance(loc, to) != 1)
        return false;
    bool to_dynamic_blocked = (dynamic_blocked.find(to) != dynamic_blocked.end());
    if (to_dynamic_blocked)
    {
        /// ------------------------------------------------------------------
        // Entering a cell that currently holds a box requires explicit
        // per-agent permission.
        //
        // Without this check, agents could route through occupied cells
        // freely. The traversable set carries the coarse permission (this
        // agent has an assignment involving this cell); the finer goal-level
        // restriction (only enterable when it is the current search goal)
        // is delegated to can_enter_override_obstacle() in the calling solver
        // so that the decision is made with full goal context available.
        // ------------------------------------------------------------------

        if (traversable == nullptr || traversable->find(to) == traversable->end())
            return false;
    }
    else if (!is_traversable(to, traversable))
        return false;
    if (weights[loc][dir] < WEIGHT_MAX - 1)
        return true;

    // ------------------------------------------------------------------
    // Escape-route exception for dynamically closed source cells.
    //
    // A box may have been delivered onto the agent's current cell after
    // the agent arrived there. At that point the static edge weights
    // stored in the template map are WEIGHT_MAX, but blocking all
    // outgoing moves would trap the agent permanently. Allowing departure
    // from a dynamically closed cell unconditionally resolves this without
    // requiring the template map to be rebuilt.
    // ------------------------------------------------------------------

    if (from_dynamic_blocked)
        return true;

    // ------------------------------------------------------------------
    // Agent-specific override for WEIGHT_MAX edges.
    //
    // When an agent's traversable set includes a pickup or delivery cell
    // that is typed as an obstacle in the static map, the template edge
    // weights to and from that cell are WEIGHT_MAX. Checking both
    // endpoints here ensures the agent can cross into or out of its
    // assigned obstacle cell even though the static template forbids it.
    // ------------------------------------------------------------------

    if (traversable == nullptr)
        return false;
    if (traversable->find(loc) != traversable->end() || traversable->find(to) != traversable->end())
        return true;

    // Backward-compatible obstacle-based override.
    bool from_obstacle = (types[loc] == "Obstacle");
    bool to_obstacle = (types[to] == "Obstacle");
    if (!(from_obstacle || to_obstacle))
        return false;
    return false;
}

// ------------------------------------------------------------------
// Updated to accept an optional per-agent traversable set and
// delegate edge validity to valid_move().
//
// The original implementation tested weights[v][i] < WEIGHT_MAX
// directly. That check did not account for dynamic obstacle cells
// or agent-specific traversable overrides. Delegating to valid_move()
// keeps the reachability logic in one place and automatically picks
// up all three override behaviours (dynamic source, dynamic
// destination, and explicit override edges).
// ------------------------------------------------------------------

list<int> BasicGraph::get_neighbors(int v, const unordered_set<int>* traversable) const
{
    list<int> neighbors;
    if (v < 0)
        return neighbors;

    for (int i = 0; i < 4; i++) // move
        if (valid_move(v, i, traversable))
            neighbors.push_back(v + move[i]);

    return neighbors;
}

// ------------------------------------------------------------------
// Same update as get_neighbors(int): edge validity is now delegated
// to valid_move() with the optional traversable set so that dynamic
// obstacle permissions and agent-specific overrides are respected
// consistently across all overloads.
// ------------------------------------------------------------------

list<State> BasicGraph::get_neighbors(const State& s, const unordered_set<int>* traversable) const
{
    list<State> neighbors;
    if (s.location < 0)
        return neighbors;
    if (s.orientation >= 0)
    {
        neighbors.push_back(State(s.location, s.timestep + 1, s.orientation)); // wait
        if (valid_move(s.location, s.orientation, traversable))
            neighbors.push_back(State(s.location + move[s.orientation], s.timestep + 1, s.orientation)); // move
        int next_orientation1 = s.orientation + 1;
        int next_orientation2 = s.orientation - 1;
        if (next_orientation2 < 0)
            next_orientation2 += 4;
        else if (next_orientation1 > 3)
            next_orientation1 -= 4;
        neighbors.push_back(State(s.location, s.timestep + 1, next_orientation1)); // turn left
        neighbors.push_back(State(s.location, s.timestep + 1, next_orientation2)); // turn right
    }
    else
    {
        neighbors.push_back(State(s.location, s.timestep + 1)); // wait
        for (int i = 0; i < 4; i++) // move
            if (valid_move(s.location, i, traversable))
                neighbors.push_back(State(s.location + move[i], s.timestep + 1));
    }
    return neighbors;
}    

// ------------------------------------------------------------------
// Same update as the forward get_neighbors() overloads: valid_move()
// is called with the traversable pointer so that reverse-neighbour
// expansion during heuristic precomputation respects the same
// reachability rules as forward search, preventing inconsistencies
// between precomputed distances and actual path costs.
// ------------------------------------------------------------------

std::list<State> BasicGraph::get_reverse_neighbors(const State& s, const unordered_set<int>* traversable) const
{
    std::list<State> rneighbors;
    // no wait actions
    if (s.orientation >= 0)
    {
        if (s.location - move[s.orientation] >= 0 && s.location - move[s.orientation] < this->size() &&
            valid_move(s.location - move[s.orientation], s.orientation, traversable))
            rneighbors.push_back(State(s.location - move[s.orientation], -1, s.orientation)); // move
        int next_orientation1 = s.orientation + 1;
        int next_orientation2 = s.orientation - 1;
        if (next_orientation2 < 0)
            next_orientation2 += 4;
        else if (next_orientation1 > 3)
            next_orientation1 -= 4;
        rneighbors.push_back(State(s.location, -1, next_orientation1)); // turn right
        rneighbors.push_back(State(s.location, -1, next_orientation2)); // turn left
    }
    else
    {
        for (int i = 0; i < 4; i++) // move
            if (s.location - move[i] >= 0 && s.location - move[i] < this->size() &&
                    valid_move(s.location - move[i], i, traversable))
                rneighbors.push_back(State(s.location - move[i]));
    }
    return rneighbors;
}

// ------------------------------------------------------------------
// Updated to accept an optional per-agent traversable set and return
// a finite cost for agent-specific override edges.
//
// When valid_move() accepts an edge because one or both endpoints 
// appear in the traversable set, a fallback cost of 1 is returned 
// instead of WEIGHT_MAX.
//
// Using a finite cost here is necessary for consistency: if the
// heuristic precomputation (which calls get_weight() via
// get_reverse_neighbors()) sees WEIGHT_MAX while the forward search
// sees a passable edge, the heuristic becomes inadmissible and the
// planner may produce suboptimal or incorrect paths.
// ------------------------------------------------------------------

double BasicGraph::get_weight(int from, int to, const unordered_set<int>* traversable) const
{
    if (from == to) // wait or rotate
        return weights[from][4];
    int dir = get_direction(from, to);
    if (dir >= 0 && valid_move(from, dir, traversable))
    {
        if (weights[from][dir] < WEIGHT_MAX - 1)
            return weights[from][dir];
        return 1; // fallback cost for agent-specific override edges
    }
    else
        return WEIGHT_MAX;
}


int BasicGraph::get_direction(int from, int to) const
{
    for (int i = 0; i < 4; i++)
    {
        if (move[i] == to - from)
            return i;
    }
    if (from == to)
        return 4;
    return -1;
}



bool BasicGraph::load_heuristics_table(std::ifstream& myfile)
{
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> >::iterator beg;
    std::string line;
    
    getline(myfile, line); //skip "table_size"
    getline(myfile, line);
    boost::tokenizer< boost::char_separator<char> > tok(line, sep);
    beg = tok.begin();
	int N = atoi ( (*beg).c_str() ); // read number of cols
	beg++;
	int M = atoi ( (*beg).c_str() ); // read number of rows
	if (M != this->size())
	    return false;
	for (int i = 0; i < N; i++)
	{
		getline (myfile, line);
        int loc = atoi(line.c_str());
        getline (myfile, line);        
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	    beg = tok.begin();
        std::vector<double> h_table(this->size());
        for (int j = 0; j < this->size(); j++)
        {
            h_table[j] = atof((*beg).c_str());
            beg++;
        }
        heuristics[loc] = h_table;
    }
	return true;
}


void BasicGraph::save_heuristics_table(std::string fname)
{
    std::ofstream myfile;
	myfile.open (fname);
	myfile << "table_size" << std::endl << 
        heuristics.size() << "," << this->size() << std::endl;
	for (auto h_values: heuristics) 
	{
        myfile << h_values.first << std::endl;
		for (double h : h_values.second) 
		{
            myfile << h << ",";
		}
		myfile << std::endl;
	}
	myfile.close();
}

// ------------------------------------------------------------------
// The original implementation iterated over nodes here and
// re-labelled any location with g_val == DBL_MAX as "Obstacle"
// (types[j] = "Obstacle"). That pass was removed.
//
// It is incompatible with dynamic obstacle cells: a cell that is
// currently unreachable because a box is placed there may become
// reachable again once the box is collected. Permanently marking it
// as an obstacle would prevent future traversal even after the
// dynamic state changes. Callers must instead handle DBL_MAX
// heuristic values by falling back to Manhattan distance, as done
// in SingleAgentSolver::compute_h_value().
// ------------------------------------------------------------------

std::vector<double> BasicGraph::compute_heuristics(int root_location)
{
    std::vector<double> res(this->size(), DBL_MAX);
	fibonacci_heap< StateTimeAStarNode*, compare<StateTimeAStarNode::compare_node> > heap;
    unordered_set< StateTimeAStarNode*, StateTimeAStarNode::Hasher, StateTimeAStarNode::EqNode> nodes;

    State root_state(root_location);
    if(consider_rotation)
    {
        for (auto neighbor : get_reverse_neighbors(root_state))
        {
            StateTimeAStarNode* root = new StateTimeAStarNode(State(root_location, -1,
                    get_direction(neighbor.location, root_state.location)), 0, 0, nullptr, 0);
            root->open_handle = heap.push(root);  // add root to heap
            nodes.insert(root);       // add root to hash_table (nodes)
        }
    }
    else
    {
        StateTimeAStarNode* root = new StateTimeAStarNode(root_state, 0, 0, nullptr, 0);
        root->open_handle = heap.push(root);  // add root to heap
        nodes.insert(root);       // add root to hash_table (nodes)
    }

	while (!heap.empty()) 
    {
        StateTimeAStarNode* curr = heap.top();
		heap.pop();
		for (auto next_state : get_reverse_neighbors(curr->state))
		{
			double next_g_val = curr->g_val + get_weight(next_state.location, curr->state.location);
            StateTimeAStarNode* next = new StateTimeAStarNode(next_state, next_g_val, 0, nullptr, 0);
			auto it = nodes.find(next);
			if (it == nodes.end()) 
			{  // add the newly generated node to heap and hash table
				next->open_handle = heap.push(next);
				nodes.insert(next);
			}
			else 
			{  // update existing node's g_val if needed (only in the heap)
				delete(next);  // not needed anymore -- we already generated it before
                StateTimeAStarNode* existing_next = *it;
				if (existing_next->g_val > next_g_val) 
				{
					existing_next->g_val = next_g_val;
					heap.increase(existing_next->open_handle);
				}
			}
		}
	}
	// iterate over all nodes and populate the distances
	for (auto it = nodes.begin(); it != nodes.end(); it++)
	{
        StateTimeAStarNode* s = *it;
		res[s->state.location] = std::min(s->g_val, res[s->state.location]);
		delete (s);
	}
	nodes.clear();
	heap.clear();
    return res;
}


int BasicGraph::get_Manhattan_distance(int loc1, int loc2) const
{
    return abs(loc1 / cols - loc2 / cols) + abs(loc1 % cols - loc2 % cols);
}


void BasicGraph::copy(const BasicGraph& copy)
{
    rows = copy.get_rows();
    cols = copy.get_cols();
    weights = copy.get_weights();
}
