// ------------------------------------------------------------------
// Adapted from RHCR (Rolling-Horizon Collision Resolution).
//
// Original source:
// https://github.com/Jiaoyang-Li/RHCR
//
// Modifications in this file:
// - Added load_locations_from_txt(): reads start/goal pairs per agent from a
//   .txt file (--locations flag) enabling fixed pre-assigned routes.
// - Added load_traverse_from_txt(): reads per-agent traversable obstacle cell
//   lists from a .txt file (--traverse flag).
// - Added all_assigned_goals_finished(): checks whether every agent has reached
//   all its pre-assigned goals for early termination in fixed_locations_mode.
// - Added reset_dynamic_obstacles_from_assigned_goals(): reconstructs the dynamic
//   obstacle state from the remaining unfinished goals on startup.
// - Added refresh_effective_traversable(): rebuilds per-agent traversable sets by
//   merging base_agent_traversable with dynamic_open_obstacles and propagating the
//   current dynamic_closed_obstacles to the graph.
// - Added close_delivery_obstacles flag: when false, delivering a box reopens the
//   cell instead of closing it, enabling continuous re-use of delivery locations.
// - Added debug_positions flag and print_positions_at_timestep(): prints each
//   agent's (row, col, orientation) at every timestep when enabled.
// - Added mandatory_goal_wait support: agents hold at each pickup/delivery goal
//   for a configurable number of timesteps.
// - Renamed output file from assigned_goal_paths.txt to rhcr_paths.txt.
// - Refactored solve(): introduced has_full_plan() and apply_plan() lambdas to
//   centralise plan validation; added cascade fallback (PBS/ECBS → WHCA → LRA)
//   when the primary solver fails; normalised empty goal lists to hold-in-place
//   pseudo-goals; introduced effective_time_limit to cap per-call solver time.
// - Added consecutive_failed_plans counter and terminated_no_solution flag:
//   terminates the simulation after max_consecutive_failed_plans consecutive
//   replanning failures.
// - Added timesteps_without_progress counter: terminates if no task is completed
//   within max_no_progress_timesteps timesteps.
// - Fixed solve_by_WHCA() to handle the empty new_agents case (full-fleet replanning)
//   and guard against iterator overrun.
// - Initialised last_solve_success, consecutive_failed_plans, timesteps_without_progress
//   and terminated_no_solution in initialize_solvers().
// - Added --goal_wait, --close_delivery_obstacles, --max_failed_plans,
//   --max_no_progress_timesteps and --debug_positions to save_results() output.
// ------------------------------------------------------------------

#include "BasicSystem.h"
#include <stdlib.h>
#include <boost/tokenizer.hpp>
#include <algorithm>
#include <sstream>
#include <queue>


BasicSystem::BasicSystem(const BasicGraph& G, MAPFSolver& solver): G(G), solver(solver), num_of_tasks(0) {}

BasicSystem::~BasicSystem() {}


// TODO: implement the random instance generator
/*bool BasicSystem::load_config(std::string fname)
{
    std::string line;
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open()) {
        std::cout << "Config file " << fname << " does not exist. " << std::endl;
        return false;
    }
    getline(myfile, line);
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char> > tok(line, sep);
    boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin();
    duration = atoi((*beg).c_str());
    getline(myfile, line);
    tok.assign(line, sep);
    fiducial = atoi((*beg).c_str());
    getline(myfile, line);
    tok.assign(line, sep);
    double length = atoi((*beg).c_str()) / fiducial;
    getline(myfile, line);
    tok.assign(line, sep);
    double width = atoi((*beg).c_str()) / fiducial;
    getline(myfile, line);
    tok.assign(line, sep);
    double v_max = atoi((*beg).c_str()) * duration / fiducial / 1000;
    getline(myfile, line);
    tok.assign(line, sep);
    double a_max = atoi((*beg).c_str()) * duration * duration / fiducial / 1000000;
    getline(myfile, line);
    tok.assign(line, sep);
    double rotate90 = atoi((*beg).c_str()) / duration;
    getline(myfile, line);
    tok.assign(line, sep);
    int rotate180 = atoi((*beg).c_str()) / duration;
    getline(myfile, line);
    tok.assign(line, sep);
    planning_window = atoi((*beg).c_str()) / duration;
    getline(myfile, line);
    tok.assign(line, sep);
    simulation_time = atoi((*beg).c_str()) / duration;

    myfile.close();
return true;

}
bool BasicSystem::generate_random_MAPF_instance()
{
    std::cout << "*** Generating instance " << seed << " ***" << std::endl;
    clock_t t = std::clock();
    // initialize_start_locations();
    // initialize_goal_locations();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    // print_MAPF_instance();
    
    return true;
}


void BasicSystem::print_MAPF_instance() const
{
    std::cout << "*** instance " << seed << " ***" << std::endl;
    for (int k = 0; k < (int)starts.size(); k++)
    {
        cout << "Agent " << k << ": " << starts[k];
        for (int goal : goal_locations[k])
            cout << "->" << goal;
        cout << endl;
    }
}


void BasicSystem::save_MAPF_instance(std::string fname) const
{
    std::ofstream stats;
    stats.open(fname, std::ios::app);
    stats << starts.size();
    for (int k = 0; k < (int)starts.size(); k++)
    {
        stats << k << "," << starts[k].location;
        for (int goal : goal_locations[k])
            cout << "," << goal;
        cout << endl;
    }
    stats.close();
}

bool BasicSystem::read_MAPF_instance(std::string fname)
{
    std::string line;
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open()) {
        std::cout << "MAPF instance file " << fname << " does not exist. " << std::endl;
        return false;
    }

    std::cout << "*** Reading instance " << fname << " ***" << std::endl;

    getline(myfile, line);
    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char> > tok(line, sep);
    boost::tokenizer<boost::char_separator<char> >::iterator beg = tok.begin();
    this->num_of_drives = atoi((*beg).c_str()); // read number of cols
    this->starts.resize(num_of_drives);
    this->goal_locations.resize(num_of_drives);

    for (int i = 0; i < num_of_drives; i++) {
        getline(myfile, line);
        boost::tokenizer<boost::char_separator<char> > tok(line, sep);
        beg = tok.begin();
        beg++; // skip id
        starts[i] = State(std::atoi(beg->c_str()));
        goal_locations[i].emplace_back(std::atoi(beg->c_str()));
    }

    myfile.close();
    return true;
}


bool BasicSystem::run()
{
	bool sol = pbs.run(starts, goal_locations, time_limit, vector<Path>(), PriorityGraph());
	pbs.save_results(outfile, std::to_string(num_of_drives) + "," + std::to_string(seed));
	pbs.best_node->priorities.save_as_digraph("goal_node.gv");
	return sol;
}
*/

bool BasicSystem::load_locations()
{
    loaded_locations_from_txt = false;
    assigned_start_locations.clear();
    assigned_goal_locations.clear();
    if (!locations_file.empty())
    {
        bool loaded = load_locations_from_txt(locations_file);
        loaded_locations_from_txt = loaded;
        return loaded;
    }

	string fname = G.map_name + "_rotation=" + std::to_string(consider_rotation) +
		"_" + std::to_string(num_of_drives) + ".agents";
    std::ifstream myfile (fname.c_str());
    if (!myfile.is_open())
		return false;

    string line;
    getline (myfile,line);
    boost::char_separator<char> sep(",");

    if (atoi(line.c_str()) != num_of_drives)
    {
        cout << "The agent file does not match the settings." << endl;
        exit(-1);
    }
    for (int k = 0; k < num_of_drives; k++)
    {
        getline (myfile, line);
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator beg=tok.begin();
        // starts
        int start_loc = atoi((*beg).c_str());
        beg++;
        int start_orient = atoi((*beg).c_str());
        beg++;
        starts[k] = State(start_loc, 0, start_orient);
        paths[k].emplace_back(starts[k]);
        finished_tasks[k].push_back(std::make_pair(start_loc, 0));
        // goals
        int goal = atoi((*beg).c_str());
        goal_locations[k].emplace_back(goal, 0);
    }
    myfile.close();
	return true;
}

// ------------------------------------------------------------------
// In fixed_locations_mode, task completion is determined by exhaustion
// of the preloaded goal sequence rather than by the normal task
// generation pipeline.
//
// This helper provides a global termination condition that ends the
// simulation immediately once every assigned goal has been completed,
// avoiding unnecessary replanning cycles after all work is finished.
// ------------------------------------------------------------------

bool BasicSystem::all_assigned_goals_finished() const
{
    if (assigned_goal_locations.empty() ||
        assigned_goal_progress.size() != assigned_goal_locations.size())
        return false;

    for (int k = 0; k < (int)assigned_goal_locations.size(); k++)
    {
        if (assigned_goal_progress[k] < (int)assigned_goal_locations[k].size())
            return false;
    }
    return true;
}

// ------------------------------------------------------------------
// Fixed-location mode allows task assignment to be defined externally
// rather than generated online by the task scheduler.
//
// Each agent receives a predefined sequence of pickup/delivery goals
// loaded from a text file, ensuring deterministic execution across
// different runs and facilitating reproducible experiments.
//
// This mode is primarily intended for evaluating planner behaviour
// under a fixed task allocation rather than dynamic task assignment.
// ------------------------------------------------------------------

bool BasicSystem::load_locations_from_txt(const std::string& fname)
{
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open())
        return false;

    std::vector<int> starts_from_file;
    std::vector<int> orientations_from_file;
    std::vector<std::vector<int> > goals_from_file;
    std::string line;
    int declared_agents = -1;
    while (getline(myfile, line))
    {
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos)
            line = line.substr(0, comment_pos);
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        std::vector<int> vals;
        int val;
        while (ss >> val)
            vals.push_back(val);
        if (vals.empty())
            continue;

        if (declared_agents < 0 && vals.size() == 1)
        {
            declared_agents = vals[0];
            continue;
        }

        int offset = 0;
        // Optional leading agent id (must match line index among records).
        if (vals.size() >= 3 && vals[0] == (int)starts_from_file.size())
            offset = 1;

        if ((int)vals.size() - offset < 2)
            continue;

        int start = vals[offset];
        int orient = -1;
        std::vector<int> goals;
        for (int i = offset + 1; i < (int)vals.size(); i++)
        {
            goals.push_back(vals[i]);
        }
        starts_from_file.push_back(start);
        orientations_from_file.push_back(orient);
        goals_from_file.push_back(goals);
    }
    myfile.close();

    if (declared_agents >= 0 && declared_agents != (int)starts_from_file.size())
    {
        std::cout << "The locations txt file declares " << declared_agents
                  << " agents but contains " << starts_from_file.size() << " records." << std::endl;
        return false;
    }
    if ((int)starts_from_file.size() != num_of_drives)
    {
        std::cout << "The locations txt file has " << starts_from_file.size()
                  << " records, but --agentNum is " << num_of_drives << "." << std::endl;
        return false;
    }

    for (int k = 0; k < num_of_drives; k++)
    {
        int start_loc = starts_from_file[k];
        int start_orient = orientations_from_file[k];
        const auto it_traverse = solver.path_planner.agent_traversable.find(k);
        const unordered_set<int>* traverse_set = (it_traverse == solver.path_planner.agent_traversable.end()) ? nullptr : &(it_traverse->second);
        if (start_loc < 0 || start_loc >= G.size())
        {
            std::cout << "Invalid start location in " << fname << " for agent " << k << std::endl;
            return false;
        }
        if (goals_from_file[k].empty())
        {
            std::cout << "No goal is provided in " << fname << " for agent " << k << std::endl;
            return false;
        }
        if (G.types[start_loc] == "Obstacle" && (traverse_set == nullptr || traverse_set->find(start_loc) == traverse_set->end()))
        {
            std::cout << "Start is an obstacle in " << fname << " for agent " << k << std::endl;
            return false;
        }
        for (int goal_loc : goals_from_file[k])
        {
            if (goal_loc < 0 || goal_loc >= G.size() ||
                (G.types[goal_loc] == "Obstacle" && (traverse_set == nullptr || traverse_set->find(goal_loc) == traverse_set->end())))
            {
                std::cout << "Invalid or obstacle goal location in " << fname << " for agent " << k << std::endl;
                return false;
            }
        }
        int prev_loc = start_loc;
        for (int goal_loc : goals_from_file[k])
        {
            if (!reachable_with_override(prev_loc, goal_loc, traverse_set))
            {
                std::cout << "Disconnected segment for agent " << k << ": "
                          << prev_loc << " -> " << goal_loc
                          << ". Check map directions/obstacles and traverse.txt." << std::endl;
                return false;
            }
            prev_loc = goal_loc;
        }

        starts[k] = State(start_loc, 0, start_orient);
        paths[k].emplace_back(starts[k]);
        finished_tasks[k].push_back(std::make_pair(start_loc, 0));
        for (int goal_loc : goals_from_file[k])
            goal_locations[k].emplace_back(goal_loc, 0);
        assigned_start_locations.push_back(start_loc);
        assigned_goal_locations.push_back(goals_from_file[k]);
    }
    reset_dynamic_obstacles_from_assigned_goals();
    refresh_effective_traversable();
    return true;
}


bool BasicSystem::reachable_with_override(int start, int goal, const unordered_set<int>* traversable) const
{
    if (start == goal)
        return true;
    std::vector<char> visited(G.size(), 0);
    std::queue<int> q;
    visited[start] = 1;
    q.push(start);
    while (!q.empty())
    {
        int u = q.front();
        q.pop();
        for (int d = 0; d < 4; d++)
        {
            if (!G.valid_move(u, d, traversable))
                continue;
            int v = u + G.move[d];
            if (v < 0 || v >= G.size() || visited[v])
                continue;
            if (v == goal)
                return true;
            visited[v] = 1;
            q.push(v);
        }
    }
    return false;
}


void BasicSystem::update_start_locations()
{
    for (int k = 0; k < num_of_drives; k++)
    {
        starts[k] = State(paths[k][timestep].location, 0, paths[k][timestep].orientation);
    }
}


void BasicSystem::apply_goal_wait_to_solver_goals(vector<vector<pair<int, int> > >& solver_goals,
                                                  const vector<int>* global_agent_ids) const
{
    for (int idx = 0; idx < (int)solver_goals.size(); idx++)
    {
        int agent = (global_agent_ids == nullptr) ? idx : (*global_agent_ids)[idx];
        if (agent < 0 || agent >= (int)goal_hold_end_timestep.size() || solver_goals[idx].empty())
            continue;
        if (goal_hold_end_timestep[agent] < 0)
            continue;

        int remaining_wait = goal_hold_end_timestep[agent] - timestep;
        if (remaining_wait > solver_goals[idx].front().second)
            solver_goals[idx].front().second = remaining_wait;
    }
}

void BasicSystem::update_paths(const std::vector<Path*>& MAPF_paths, int max_timestep = INT_MAX)
{
    for (int k = 0; k < num_of_drives; k++)
    {
        if (paths[k].empty())
            paths[k].emplace_back(starts[k].location, 0, starts[k].orientation);

        while ((int)paths[k].size() < timestep)
        {
            const auto& prev = paths[k].back();
            int idx = (int)paths[k].size();
            paths[k].emplace_back(prev.location, idx, prev.orientation);
        }

        State anchor;
        if ((int)paths[k].size() > timestep)
            anchor = paths[k][timestep];
        else
            anchor = State(paths[k].back().location, timestep, paths[k].back().orientation);

        int length = min(max_timestep, (int) MAPF_paths[k]->size());
        if (length <= 0)
            continue;

        paths[k].resize(timestep + length);

        const auto it_traverse = solver.path_planner.agent_traversable.find(k);
        const unordered_set<int>* traverse_set =
            (it_traverse == solver.path_planner.agent_traversable.end()) ? nullptr : &(it_traverse->second);

        for (int t = 0; t < length; t++)
        {
            int abs_t = timestep + t;
            State next = (t == 0) ? anchor : MAPF_paths[k]->at(t);
            if (next.location < 0 || next.location >= G.size())
            {
                if (abs_t > 0 && abs_t - 1 < (int)paths[k].size())
                    next = State(paths[k][abs_t - 1].location, abs_t, paths[k][abs_t - 1].orientation);
                else
                    next = State(anchor.location, abs_t, anchor.orientation);
            }
            next.timestep = abs_t;
            paths[k][abs_t] = next;

            if (abs_t > 0)
            {
                auto& prev = paths[k][abs_t - 1];
                auto& curr = paths[k][abs_t];
                if (curr.location < 0 || curr.location >= G.size())
                {
                    curr.location = prev.location;
                    curr.orientation = prev.orientation;
                }
                else if (prev.location >= 0 && prev.location < G.size() &&
                         curr.location != prev.location &&
                         G.get_Manhattan_distance(prev.location, curr.location) > 1)
                {
                    int best = prev.location;
                    int best_dist = G.get_Manhattan_distance(prev.location, curr.location);
                    for (int dir = 0; dir < 4; dir++)
                    {
                        if (!G.valid_move(prev.location, dir, traverse_set))
                            continue;
                        int cand = prev.location + G.move[dir];
                        int dist = G.get_Manhattan_distance(cand, curr.location);
                        if (dist < best_dist)
                        {
                            best = cand;
                            best_dist = dist;
                        }
                    }
                    curr.location = best;
                    if (curr.location == prev.location)
                        curr.orientation = prev.orientation;
                }
                curr.timestep = abs_t;
            }
        }
    }
}

void BasicSystem::update_paths(const std::vector<Path>& MAPF_paths, int max_timestep = INT_MAX)
{
    for (int k = 0; k < num_of_drives; k++)
    {
        if (paths[k].empty())
            paths[k].emplace_back(starts[k].location, 0, starts[k].orientation);

        while ((int)paths[k].size() < timestep)
        {
            const auto& prev = paths[k].back();
            int idx = (int)paths[k].size();
            paths[k].emplace_back(prev.location, idx, prev.orientation);
        }

        State anchor;
        if ((int)paths[k].size() > timestep)
            anchor = paths[k][timestep];
        else
            anchor = State(paths[k].back().location, timestep, paths[k].back().orientation);

        int length = min(max_timestep, (int) MAPF_paths[k].size());
        if (length <= 0)
            continue;

        paths[k].resize(timestep + length);

        const auto it_traverse = solver.path_planner.agent_traversable.find(k);
        const unordered_set<int>* traverse_set =
            (it_traverse == solver.path_planner.agent_traversable.end()) ? nullptr : &(it_traverse->second);

        for (int t = 0; t < length; t++)
        {
            int abs_t = timestep + t;
            State next = (t == 0) ? anchor : MAPF_paths[k][t];
            if (next.location < 0 || next.location >= G.size())
            {
                if (abs_t > 0 && abs_t - 1 < (int)paths[k].size())
                    next = State(paths[k][abs_t - 1].location, abs_t, paths[k][abs_t - 1].orientation);
                else
                    next = State(anchor.location, abs_t, anchor.orientation);
            }
            next.timestep = abs_t;
            paths[k][abs_t] = next;

            if (abs_t > 0)
            {
                auto& prev = paths[k][abs_t - 1];
                auto& curr = paths[k][abs_t];
                if (curr.location < 0 || curr.location >= G.size())
                {
                    curr.location = prev.location;
                    curr.orientation = prev.orientation;
                }
                else if (prev.location >= 0 && prev.location < G.size() &&
                         curr.location != prev.location &&
                         G.get_Manhattan_distance(prev.location, curr.location) > 1)
                {
                    int best = prev.location;
                    int best_dist = G.get_Manhattan_distance(prev.location, curr.location);
                    for (int dir = 0; dir < 4; dir++)
                    {
                        if (!G.valid_move(prev.location, dir, traverse_set))
                            continue;
                        int cand = prev.location + G.move[dir];
                        int dist = G.get_Manhattan_distance(cand, curr.location);
                        if (dist < best_dist)
                        {
                            best = cand;
                            best_dist = dist;
                        }
                    }
                    curr.location = best;
                    if (curr.location == prev.location)
                        curr.orientation = prev.orientation;
                }
                curr.timestep = abs_t;
            }
        }
    }
}

void BasicSystem::update_initial_paths(vector<Path>& initial_paths) const
{
    initial_paths.clear();
    initial_paths.resize(num_of_drives);
    for (int k = 0; k < num_of_drives; k++)
    {
        // check whether the path traverse every goal locations
        int i = (int)goal_locations[k].size() - 1;
        int j = (int)paths[k].size() - 1;
        while (i >= 0 && j >= 0)
        {
            while (j >= 0 && paths[k][j].location != goal_locations[k][i].first &&
			paths[k][j].timestep >= goal_locations[k][i].second)
                j--;
            i--;
        }
        if (j < 0)
            continue;
          
        if ((int) paths[k].size() <= timestep + planning_window)
            continue;

        initial_paths[k].resize(paths[k].size() - timestep);
        for (int t = 0; t < (int)initial_paths[k].size(); t++)
        {
            initial_paths[k][t] = paths[k][timestep + t];
            initial_paths[k][t].timestep = t;
        }
    }
}

void BasicSystem::update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const
{
    initial_constraints.clear();
    for (int k = 0; k < num_of_drives; k++)
    {
        int prev_location = -1;
        for (int t = timestep; t > max(0, timestep - k_robust); t--)
        {
            int curr_location = paths[k][t].location;
            if (curr_location < 0)
                continue;
            else if (curr_location != prev_location)
            {
                initial_constraints.emplace_back(k, curr_location, t + k_robust + 1 - timestep);
                prev_location = curr_location;
            }
        }
    }
}


bool BasicSystem::check_collisions(const vector<Path>& input_paths) const
{
	for (int a1 = 0; a1 < (int)input_paths.size(); a1++)
	{
		for (int a2 = a1 + 1; a2 < (int)input_paths.size(); a2++)
		{
			// TODO: add k-robust
			size_t min_path_length = input_paths[a1].size() < input_paths[a2].size() ? input_paths[a1].size() : input_paths[a2].size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = input_paths[a1].at(timestep).location;
				int loc2 = input_paths[a2].at(timestep).location;
				if (loc1 == loc2)
					return true;
				else if (timestep < min_path_length - 1
					&& loc1 == input_paths[a2].at(timestep + 1).location
					&& loc2 == input_paths[a1].at(timestep + 1).location)
					return true;
			}
			if ((hold_endpoints || useDummyPaths) && input_paths[a1].size() != input_paths[a2].size())
			{
				int a1_ = input_paths[a1].size() < input_paths[a2].size() ? a1 : a2;
				int a2_ = input_paths[a1].size() < input_paths[a2].size() ? a2 : a1;
				int loc1 = input_paths[a1_].back().location;
				for (size_t timestep = min_path_length; timestep < input_paths[a2_].size(); timestep++)
				{
					int loc2 = input_paths[a2_].at(timestep).location;
					if (loc1 == loc2)
						return true;
				}
			}
		}
	}
	return false;
}

bool BasicSystem::congested() const
{
	if (simulation_window <= 1)
		return false;
    int wait_agents = 0;
    for (int k = 0; k < (int)paths.size(); k++)
    {
        // Skip agents intentionally waiting at a goal (mandatory_goal_wait still active)
        if (k < (int)goal_hold_end_timestep.size() && goal_hold_end_timestep[k] >= 0)
            continue;
        const auto& path = paths[k];
        int t = 0;
        while (t < simulation_window && path[timestep].location == path[timestep + t].location &&
                path[timestep].orientation == path[timestep + t].orientation)
            t++;
        if (t == simulation_window)
        {
            // The agent appears stuck for the whole window. Before counting it as
            // congested, check whether its path shows movement starting right after
            // the window. This happens when mandatory_goal_wait == simulation_window:
            // the hold period ends exactly at the window boundary, goal_hold_end was
            // reset to -1 inside move(), but the planned path still shows the agent
            // waiting (correctly) and then moving at timestep+simulation_window.
            if ((int)path.size() > timestep + simulation_window &&
                path[timestep + simulation_window].location != path[timestep].location)
                continue;
            wait_agents++;
        }
    }
    return wait_agents > num_of_drives / 2;  // more than half of drives didn't make progress
}

void BasicSystem::print_positions_at_timestep(int t) const
{
    // ------------------------------------------------------------------
    // Debug position tracing outputs the exact state of every agent at each
    // timestep, including location and orientation.
    //
    // This functionality is intended for validation and debugging of
    // planner behaviour, particularly when investigating deadlocks,
    // unexpected waiting behaviour, or dynamic obstacle interactions.
    // ------------------------------------------------------------------

    if (!debug_positions)
        return;
    std::cout << "[DEBUG_POS] t=" << t;
    for (int k = 0; k < num_of_drives; k++)
    {
        if ((int)paths[k].size() <= t)
            continue;
        const State& s = paths[k][t];
        int row = s.location / G.cols;
        int col = s.location % G.cols;
        std::cout << " | a" << k
                  << ":loc=" << s.location
                  << "(" << row << "," << col << ")"
                  << ",o=" << s.orientation;
    }
    std::cout << std::endl;
}

// move all agents from start_timestep to end_timestep
// return a list of finished tasks
list<tuple<int, int, int>> BasicSystem::move()
{
    int start_timestep = (timestep == 0) ? 0 : timestep + 1;
    int end_timestep = timestep + simulation_window;
    bool traversability_changed = false;

	list<tuple<int, int, int>> finished_tasks; // <agent_id, location, timestep>

    for (int t = start_timestep; t <= end_timestep; t++)
    {
        for (int k = 0; k < num_of_drives; k++) {
            // Agents waits at its current locations if no future paths are assigned
            while ((int) paths[k].size() <= t)
            { // This should not happen?
                State final_state = paths[k].back();
                paths[k].emplace_back(final_state.location, final_state.timestep + 1, final_state.orientation);
            }
        }
    }


    for (int t = start_timestep; t <= end_timestep; t++)
    {
        for (int k = 0; k < num_of_drives; k++)
        {
            State curr = paths[k][t];

            if (k < (int)force_depart_location.size() && force_depart_location[k] >= 0 &&
                curr.location != force_depart_location[k] &&
                curr.timestep > force_depart_from_timestep[k])
            {
                force_depart_location[k] = -1;
                force_depart_from_timestep[k] = -1;
            }

            /* int wait_times = 0; // wait time at the current location
            while (wait_times < t && paths[k][t - wait_times] != curr)
            {
                wait_times++;
            }*/

            // remove goals if necessary
            if ((!hold_endpoints || paths[k].size() == t + 1) && !goal_locations[k].empty() &&
                curr.location == goal_locations[k].front().first &&
                curr.timestep >= goal_locations[k].front().second)
            {
                if (goal_hold_end_timestep[k] < 0)
                {
                    // ------------------------------------------------------------------
                    // Mandatory goal waiting introduces a configurable dwell time at every
                    // pickup and delivery location.
                    //
                    // This models real-world loading/unloading operations and ensures that
                    // agents remain occupying the goal location for a fixed duration before
                    // continuing toward the next assigned task.
                    // ------------------------------------------------------------------

                    goal_hold_end_timestep[k] = curr.timestep + mandatory_goal_wait;
                    // Replan as soon as a goal is first reached to reserve it during the wait period.
                    end_timestep = std::min(end_timestep, t);
                }

                if (curr.timestep < goal_hold_end_timestep[k])
                    continue;

                // Fixed assigned-goal mode:
                // odd-numbered reached goal (1st,3rd,...) -> pickup -> open obstacle globally
                // even-numbered reached goal (2nd,4th,...) -> drop   -> close obstacle globally
                if (k < (int)assigned_goal_locations.size() &&
                    k < (int)assigned_goal_progress.size() &&
                    assigned_goal_progress[k] < (int)assigned_goal_locations[k].size() &&
                    assigned_goal_locations[k][assigned_goal_progress[k]] == curr.location)
                {
                    assigned_goal_progress[k] += 1;
                    // Fixed-goal mode models boxes on pickup/delivery goals regardless of
                    // the static map cell type, so dynamic open/close must always be applied.
                    int reached_idx_1based = assigned_goal_progress[k];
                    if (reached_idx_1based % 2 == 1) // pickup -> box removed
                    {
                        dynamic_closed_obstacles.erase(curr.location);
                        dynamic_open_obstacles.insert(curr.location);
                    }
                    else // delivery -> box appears
                    {
                        // ------------------------------------------------------------------
                        // The original behaviour closes a delivery location after task
                        // completion, preventing subsequent tasks from reusing that cell.
                        //
                        // When close_delivery_obstacles is disabled, delivery cells remain
                        // available after completion, allowing repeated use of the same
                        // delivery location throughout the simulation.
                        // ------------------------------------------------------------------

                        if (close_delivery_obstacles)
                        {
                            dynamic_closed_obstacles.insert(curr.location);
                            dynamic_open_obstacles.erase(curr.location);
                        }
                        else
                        {
                            dynamic_closed_obstacles.erase(curr.location);
                            dynamic_open_obstacles.insert(curr.location);
                        }
                    }
                    traversability_changed = true;
                }
                goal_locations[k].erase(goal_locations[k].begin());
                goal_hold_end_timestep[k] = -1;
                if (k < (int)force_depart_location.size())
                {
                    force_depart_location[k] = curr.location;
                    force_depart_from_timestep[k] = curr.timestep;
                }
                // Replan immediately when mandatory hold ends to avoid extra waiting
                // at the completed pickup/delivery due stale window suffixes.
                end_timestep = std::min(end_timestep, t);
                finished_tasks.emplace_back(k, curr.location, t);
            }

            // check whether the move is valid
            if (t > 0)
            {
                State prev = paths[k][t - 1];
                const auto it_traverse = solver.path_planner.agent_traversable.find(k);
                const unordered_set<int>* traverse_set =
                    (it_traverse == solver.path_planner.agent_traversable.end()) ? nullptr : &(it_traverse->second);

                if (curr.location == prev.location)
                {
                    if (G.get_rotate_degree(prev.orientation, curr.orientation) == 2)
                    {
                        cout << "Drive " << k << " rotates 180 degrees from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
                else if (consider_rotation)
                {
                    if (prev.orientation != curr.orientation)
					{
						cout << "Drive " << k << " rotates while moving from " << prev << " to " << curr << endl;
						save_results();
						exit(-1);
					}
					else if ( !G.valid_move(prev.location, prev.orientation, traverse_set) ||
                        prev.location + G.move[prev.orientation] != curr.location)
                    {
                        cout << "Drive " << k << " jump from " << prev << " to " << curr
                             << " (dir=" << prev.orientation
                             << ", prev_trav=" << G.is_traversable(prev.location, traverse_set)
                             << ", curr_trav=" << G.is_traversable(curr.location, traverse_set)
                             << ", valid_move=" << G.valid_move(prev.location, prev.orientation, traverse_set)
                             << ")" << endl;
                        save_results();
                        exit(-1);
                    }
                }
				else
				{
					int dir = G.get_direction(prev.location, curr.location);
					if (dir < 0 || !G.valid_move(prev.location, dir, traverse_set))
					{
						cout << "Drive " << k << " jump from " << prev << " to " << curr
                             << " (dir=" << dir
                             << ", prev_type=" << G.types[prev.location]
                             << ", curr_type=" << G.types[curr.location]
                             << ", prev_trav=" << G.is_traversable(prev.location, traverse_set)
                             << ", curr_trav=" << G.is_traversable(curr.location, traverse_set)
                             << ", valid_move=" << (dir >= 0 ? G.valid_move(prev.location, dir, traverse_set) : 0)
                             << ")" << endl;
						save_results();
						exit(-1);
					}
				}
            }

            // Check whether this move has conflicts with other agents
			if (G.types[curr.location] != "Magic")
			{
				for (int j = k + 1; j < num_of_drives; j++)
				{
					for (int i = max(0, t - k_robust); i <= min(t + k_robust, end_timestep); i++)
					{
						if ((int)paths[j].size() <= i)
							break;
						if (paths[j][i].location == curr.location)
						{
							cout << "Drive " << k << " at " << curr << " has a conflict with drive " << j
								<< " at " << paths[j][i] << endl;
							save_results(); //TODO: write termination reason to files
							exit(-1);
						}
					}
				}
			}
        }
        if (traversability_changed)
        {
            // Drop all not-yet-executed suffixes; map traversability changed at t,
            // so future segments must be replanned from this state.
            for (int a = 0; a < num_of_drives; a++)
            {
                if ((int)paths[a].size() > t + 1)
                    paths[a].resize(t + 1);
            }
            // Starts used by refresh_effective_traversable must reflect current timestep.
            for (int a = 0; a < num_of_drives; a++)
                starts[a] = State(paths[a][t].location, 0, paths[a][t].orientation);
            refresh_effective_traversable();
            traversability_changed = false;
        }
        // Debug only
        //print_positions_at_timestep(t);
    }
    last_move_timestep = end_timestep;
    return finished_tasks;
}


void BasicSystem::add_partial_priorities(const vector<Path>& initial_paths, PriorityGraph& initial_priorities) const
{
    list<int> low_priorities;
    list<int> high_priorities;
    for (int k = 0; k < num_of_drives; k++)
    {
        if (initial_paths[k].empty())
            low_priorities.push_back(k);
        else
            high_priorities.push_back(k);
    }

    for (auto low : low_priorities)
    {
        for (auto high : high_priorities)
            initial_priorities.add(low, high);
    }
}

void BasicSystem::save_results()
{
	if (screen)
		std::cout << "*** Saving " << seed << " ***" << std::endl;
    clock_t t = std::clock();
    std::ofstream output;

    // settings
    output.open(outfile + "/config.txt", std::ios::out);
    output << "map: " << G.map_name << std::endl
        << "#drives: " << num_of_drives << std::endl
        << "seed: " << seed << std::endl
        << "solver: " << solver.get_name() << std::endl
        << "time_limit: " << time_limit << std::endl
        << "simulation_window: " << simulation_window << std::endl
        << "planning_window: " << planning_window << std::endl
        << "simulation_time: " << simulation_time << std::endl
        << "robust: " << k_robust << std::endl
        << "rotate: " << consider_rotation << std::endl
        << "use_dummy_paths: " << useDummyPaths << std::endl
        << "hold_endpoints: " << hold_endpoints << std::endl
        << "mandatory_goal_wait: " << mandatory_goal_wait << std::endl
        << "close_delivery_obstacles: " << close_delivery_obstacles << std::endl
        << "max_failed_plans: " << max_consecutive_failed_plans << std::endl
        << "max_no_progress_timesteps: " << max_no_progress_timesteps << std::endl;

    output.close();

    // tasks
    output.open(outfile + "/tasks.txt", std::ios::out);
    output << num_of_drives << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        int prev = finished_tasks[k].front().first;
        for (auto task : finished_tasks[k])
        {
            output << task.first << "," << task.second << ",";
            if (task.second != 0)
            {
                auto it = G.heuristics.find(task.first);
                if (it != G.heuristics.end() && prev >= 0 && prev < (int)it->second.size())
                    output << it->second[prev];
                else
                    output << G.get_Manhattan_distance(prev, task.first);
            }
            output << ";";
            prev = task.first;
        }
        for (auto goal : goal_locations[k]) // tasks that have not been finished yet
        {
            output << goal.first << ",-1,;";
        }
        output << std::endl;
    }
    output.close();

    // paths
    output.open(outfile + "/paths.txt", std::ios::out);
    output << num_of_drives << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        for (auto p : paths[k])
        {
            if (p.timestep <= timestep)
                output << p << ";";
        }
        output << std::endl;
    }
    output.close();

    // exact assigned-goal paths (only when starts/goals are provided via locations txt)
    if (!assigned_start_locations.empty() && assigned_start_locations.size() == paths.size() &&
        assigned_goal_locations.size() == paths.size())
    {
        output.open(outfile + "/rhcr_paths.txt", std::ios::out);
        save_assigned_goal_paths(output);
        output.close();
    }
    saving_time = (std::clock() - t) / CLOCKS_PER_SEC;
	if (screen)
		std::cout << "Done! (" << saving_time << " s)" << std::endl;
}


void BasicSystem::save_assigned_goal_paths(std::ofstream& output) const
{
    output << num_of_drives << std::endl;
    output << "# format per line: agent,start,goals,reached_all,reached_times,path=(loc[row,col]@t;...)" << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        int start = assigned_start_locations[k];
        const auto& goals = assigned_goal_locations[k];
        output << "agent " << k << "," << start << ",";
        for (int g = 0; g < (int)goals.size(); g++)
        {
            output << goals[g];
            if (g + 1 < (int)goals.size())
                output << "|";
        }
        output << ",";

        int last_t = -1;
        int next_goal_id = 0;
        std::vector<int> reached_times(goals.size(), -1);
        for (int t = 0; t < (int)paths[k].size(); t++)
        {
            if (paths[k][t].timestep < 0)
                continue;
            last_t = t;
            if (next_goal_id < (int)goals.size() && paths[k][t].location == goals[next_goal_id])
            {
                reached_times[next_goal_id] = paths[k][t].timestep;
                next_goal_id++;
            }
            if (next_goal_id == (int)goals.size())
                break;
        }

        bool reached = (next_goal_id == (int)goals.size());
        output << (reached ? "1" : "0") << ",";
        for (int i = 0; i < (int)reached_times.size(); i++)
        {
            output << reached_times[i];
            if (i + 1 < (int)reached_times.size())
                output << "|";
        }
        output << ",";
        output << "path=";
        for (int t = 0; t <= last_t; t++)
        {
            if (paths[k][t].timestep < 0)
                continue;
            int loc = paths[k][t].location;
            int row = loc / G.get_cols();
            int col = loc % G.get_cols();
            output << loc << "[" << row << "," << col << "]@" << paths[k][t].timestep << ";";
        }
        output << std::endl;
    }
}


void BasicSystem::update_travel_times(unordered_map<int, double>& travel_times)
{
    if (travel_time_window <= 0)
        return;

    travel_times.clear();
    unordered_map<int, int> count;

    int t_min = max(0, timestep - travel_time_window);
    if (t_min >= timestep)
        return;
    for (auto path : paths)
    {
        int t = timestep;
        while (t >= t_min)
        {
            int loc = path[t].location;
            int dir = path[t].orientation;
            int wait = 0;
            while (t > wait && path[t - 1 - wait].location == loc && path[t - 1 - wait].orientation == dir)
                wait++;
            auto it = travel_times.find(loc);
            if (it == travel_times.end())
            {
                travel_times[loc] = wait;
                count[loc] = 1;
            }
            else
            {
                travel_times[loc] += wait;
                count[loc] += 1;
            }
            t = t - 1 - wait;
        }
    }

    for (auto it : count)
    {
        if (it.second > 1)
        {
            travel_times[it.first] /= it.second;
        }
    }
}

// ------------------------------------------------------------------
// Planning execution was refactored to separate:
//
// - Plan validation (has_full_plan()).
// - Plan application (apply_plan()).
// - Solver selection and fallback handling.
//
// This reduces duplicated logic and ensures that all planning methods
// are evaluated using identical acceptance criteria.
//
// A cascading fallback strategy was also introduced so that failures in
// higher-quality planners can automatically fall back to more robust
// alternatives before declaring planning failure.
// ------------------------------------------------------------------

void BasicSystem::solve()
{
    LRA_called = false;
	LRAStar lra(G, solver.path_planner);
	lra.simulation_window = simulation_window;
	lra.k_robust = k_robust;
	solver.clear();
	vector<vector<pair<int, int> > > solver_goal_locations(goal_locations);
    // Some MAPF / single-agent planners assume each agent has at least one goal.
    // Normalize empty goal lists to a hold-at-current-location pseudo-goal.
    for (int k = 0; k < num_of_drives; k++)
    {
        if (solver_goal_locations[k].empty())
            solver_goal_locations[k].emplace_back(starts[k].location, 0);
    }
	apply_goal_wait_to_solver_goals(solver_goal_locations);
    bool solved_this_round = false;
    // ------------------------------------------------------------------
    // effective_time_limit provides a unified upper bound for each planner
    // invocation regardless of the selected planning backend.
    //
    // This prevents excessive solver runtimes from stalling the simulation
    // and ensures predictable planning latency across replanning cycles.
    // ------------------------------------------------------------------

    int effective_time_limit = time_limit;
    if (!assigned_goal_locations.empty())
        effective_time_limit = std::min(effective_time_limit, 10);
        //effective_time_limit = std::min(effective_time_limit, 30);
    effective_time_limit = std::max(1, effective_time_limit);

    auto enforce_departure_after_goal_wait = [&](vector<Path>& candidate_paths)
    {
        (void)candidate_paths;
    };
    auto has_full_plan = [&](const vector<Path>& candidate_paths)
    {
        if ((int)candidate_paths.size() != num_of_drives)
            return false;
        for (const auto& p : candidate_paths)
        {
            if (p.empty())
                return false;
        }
        return true;
    };
    auto apply_plan = [&](vector<Path>& candidate_paths)
    {
        if (!has_full_plan(candidate_paths))
            return false;
        enforce_departure_after_goal_wait(candidate_paths);
        if (check_collisions(candidate_paths))
        {
            lra.resolve_conflicts(candidate_paths);
            candidate_paths = lra.solution;
            enforce_departure_after_goal_wait(candidate_paths);
            if (!has_full_plan(candidate_paths) || check_collisions(candidate_paths))
                return false;
        }
        update_paths(candidate_paths);
        return true;
    };
	if (solver.get_name() == "LRA")
	{
		// predict travel time
		unordered_map<int, double> travel_times;
		update_travel_times(solver.travel_times);

		bool sol = solver.run(starts, solver_goal_locations, effective_time_limit);
        if (sol && has_full_plan(solver.solution))
        {
            vector<Path> candidate_paths = solver.solution;
            solved_this_round = apply_plan(candidate_paths);
        }
	}
	else if (solver.get_name() == "WHCA")
	{
		update_initial_constraints(solver.initial_constraints);

		bool sol = solver.run(starts, solver_goal_locations, effective_time_limit);
		if (sol)
        {
            vector<Path> candidate_paths = solver.solution;
            solved_this_round = apply_plan(candidate_paths);
        }
        if (!solved_this_round)
        {
            vector<Path> candidate_paths = solver.solution;
            lra.resolve_conflicts(candidate_paths);
            candidate_paths = lra.solution;
            solved_this_round = apply_plan(candidate_paths);
        }
	}
	 else // PBS or ECBS
	 {
		 //PriorityGraph initial_priorities;
		 update_initial_constraints(solver.initial_constraints);

		 // solve
		 if (hold_endpoints || useDummyPaths)
		 {
		 vector<State> new_starts;
		 vector< vector<pair<int, int> > > new_goal_locations;
			 for (int i : new_agents)
			 {
				 if (i < 0 || i >= num_of_drives || solver_goal_locations[i].empty())
					 continue;
				 new_starts.emplace_back(starts[i]);
				 new_goal_locations.emplace_back(solver_goal_locations[i]);
			 }
			 vector<Path> planned_paths(num_of_drives);
			 solver.initial_rt.clear();
			 auto p = new_agents.begin();
			 for (int i = 0; i < num_of_drives; i++)
			 {
                 planned_paths[i].resize(paths[i].size() - timestep);
                 for (int t = 0; t < (int)planned_paths[i].size(); t++)
                 {
                     planned_paths[i][t] = paths[i][timestep + t];
                     planned_paths[i][t].timestep = t;
                 }
				 if (p == new_agents.end() || *p != i)
				 {
					 solver.initial_rt.insertPath2CT(planned_paths[i]);
				 }
				 else
					 ++p;
			 }
			 if (!new_starts.empty())
			 {
				 bool sol;
                if (timestep == 0)
                    sol = solver.run(new_starts, new_goal_locations, 10 * effective_time_limit);
                else
                    sol = solver.run(new_starts, new_goal_locations, effective_time_limit);
                if (sol)
				 {
					 auto pt = solver.solution.begin();
					 for (int i : new_agents)
					 {
						 planned_paths[i] = *pt;
						 ++pt;
					 }
				 }
				 else
				 {
					 solve_by_WHCA(planned_paths, new_starts, new_goal_locations);
				 }
                 solved_this_round = apply_plan(planned_paths);
			 }
             else
             {
                 solved_this_round = true; // no active agents need replanning
             }
		 }
		 else
		 {
			 // Pass only the next immediate goal per agent to keep PBS paths short.
			 // With all chained goals at once, paths span the whole map and PBS
			 // cannot find a valid priority ordering (any ordering leaves some
			 // agent with no route). Replanning at each pickup/delivery handles
			 // the remaining goals in subsequent calls.
			 // Exception: when an agent is in a mandatory hold period at the current
			 // goal, also include the following goal so the solver plans movement
			 // past the hold period. Without this, the path shows the agent stuck
			 // at the hold location after the wait ends, triggering false congestion.
			 vector<vector<pair<int,int>>> next_goals(num_of_drives);
			 for (int k = 0; k < num_of_drives; k++) {
				 if (!solver_goal_locations[k].empty()) {
					 next_goals[k] = { solver_goal_locations[k].front() };
					 if (k < (int)goal_hold_end_timestep.size() &&
					     goal_hold_end_timestep[k] >= 0 &&
					     solver_goal_locations[k].size() > 1)
						 next_goals[k].push_back(solver_goal_locations[k][1]);
				 }
                 if (next_goals[k].empty())
                     next_goals[k].emplace_back(starts[k].location, 0);
			 }
			 bool sol = solver.run(starts, next_goals, effective_time_limit);
             if (sol)
             {
                 if (log)
                     solver.save_constraints_in_goal_node(outfile + "/goal_nodes/" + std::to_string(timestep) + ".gv");
                 vector<Path> candidate_paths = solver.solution;
                 solved_this_round = apply_plan(candidate_paths);
             }
             if (!solved_this_round)
             {
                 // Retry with WHCA over all agents when PBS/ECBS fails at this step.
                 vector<Path> whca_paths(num_of_drives);
                 auto old_new_agents = new_agents;
                 new_agents.clear();
                 for (int a = 0; a < num_of_drives; a++)
                 {
                     new_agents.push_back(a);
                 }
                 bool whca_sol = false;
                 whca_sol = solve_by_WHCA(whca_paths, starts, next_goals);
                 new_agents = old_new_agents;
                 if (whca_sol)
                     solved_this_round = apply_plan(whca_paths);
             }
             if (!solved_this_round)
             {
                 vector<Path> candidate_paths = solver.solution;
                 lra.resolve_conflicts(candidate_paths);
                 candidate_paths = lra.solution;
                 solved_this_round = apply_plan(candidate_paths);
             }
		 }
		 if (log)
			 solver.save_search_tree(outfile + "/search_trees/" + std::to_string(timestep) + ".gv");

	 }
    last_solve_success = solved_this_round;
    if (solved_this_round)
    {
        consecutive_failed_plans = 0;
    }
    else
    {
        // ------------------------------------------------------------------
        // Repeated planning failures generally indicate that the current
        // problem configuration is unsolvable or that the planner is unable to
        // make progress under the imposed constraints.
        //
        // The simulation is therefore terminated after a configurable number of
        // consecutive failures to avoid infinite replanning loops.
        // ------------------------------------------------------------------

        consecutive_failed_plans++;
        std::cout << "NO SOLUTION at timestep " << timestep
                  << " (consecutive failed replans: " << consecutive_failed_plans << ")" << std::endl;
        if (consecutive_failed_plans >= max_consecutive_failed_plans)
        {
            terminated_no_solution = true;
            std::cout << "Terminating: no solution found for " << consecutive_failed_plans
                      << " consecutive replans." << std::endl;
        }
    }
	 solver.save_results(outfile + "/solver.csv", std::to_string(timestep) + "," 
										+ std::to_string(num_of_drives) + "," + std::to_string(seed));
}

// ------------------------------------------------------------------
// WHCA originally assumed that at least one newly affected agent would
// always require replanning.
//
// Additional checks were introduced to correctly support full-fleet
// replanning scenarios and to prevent iterator advancement beyond the
// valid container range when no new agents are present.
// ------------------------------------------------------------------

bool BasicSystem::solve_by_WHCA(vector<Path>& planned_paths,
	const vector<State>& new_starts, const vector< vector<pair<int, int> > >& new_goal_locations)
{
	WHCAStar whca(G, solver.path_planner);
    whca.k_robust = k_robust;
    whca.window = INT_MAX;
    whca.hold_endpoints = hold_endpoints || useDummyPaths;
    whca.screen = screen;
	whca.initial_rt.hold_endpoints = true;
	whca.initial_rt.map_size = G.size();
	whca.initial_rt.k_robust = k_robust;
	whca.initial_rt.window = INT_MAX;
	whca.initial_rt.copy(solver.initial_rt);
    whca.initial_solution.resize(new_starts.size());
    if (whca.hold_endpoints)
    {
        if (timestep == 0)
        {
            for (int i = 0; i < (int)new_starts.size(); i++)
                whca.initial_solution[i].emplace_back(starts[i]); // hold initial location
        }
        else
        {
            whca.initial_solution.clear();
            for (auto agent : new_agents)
                whca.initial_solution.emplace_back(planned_paths[agent]); // hold old paths
        }
    }
	bool sol = false;
    int whca_time_limit = time_limit;
    if (!assigned_goal_locations.empty())
        whca_time_limit = std::min(whca_time_limit, 10);
    whca_time_limit = std::max(1, whca_time_limit);
    if (timestep == 0)
    {
        sol = whca.run(new_starts, new_goal_locations, whca_time_limit);
    }
    else
    {
        sol = whca.run(new_starts, new_goal_locations, whca_time_limit);
    }
	whca.save_results(outfile + "/solver.csv", std::to_string(timestep) + ","
		+ std::to_string(num_of_drives) + "," + std::to_string(seed));
    if (sol)
    {
        if (new_agents.empty())
        {
            planned_paths = whca.solution;
        }
        else
        {
            auto pt = whca.solution.begin();
            for (int i : new_agents)
            {
                if (pt == whca.solution.end())
                    break;
                planned_paths[i] = *pt;
                ++pt;
            }
        }
    }
	whca.clear();
	return sol;
}

void BasicSystem::initialize_solvers()
{
	solver.k_robust = k_robust;
	solver.window = planning_window;
	solver.hold_endpoints = hold_endpoints || useDummyPaths;
	solver.screen = screen;

	solver.initial_rt.hold_endpoints = true;
	solver.initial_rt.map_size = G.size();
	solver.initial_rt.k_robust = k_robust;
	solver.initial_rt.window = INT_MAX;

    solver.path_planner.agent_traversable.clear();
    if (!traverse_file.empty())
        load_traverse_from_txt(traverse_file);
    base_agent_traversable.assign(num_of_drives, unordered_set<int>());
    for (int a = 0; a < num_of_drives; a++)
    {
        auto it = solver.path_planner.agent_traversable.find(a);
        if (it != solver.path_planner.agent_traversable.end())
            base_agent_traversable[a] = it->second;
    }
    dynamic_open_obstacles.clear();
    dynamic_closed_obstacles.clear();
    assigned_goal_progress.assign(num_of_drives, 0);
    goal_hold_end_timestep.assign(num_of_drives, -1);
    force_depart_location.assign(num_of_drives, -1);
    force_depart_from_timestep.assign(num_of_drives, -1);
    last_solve_success = true;
    consecutive_failed_plans = 0;

    // ------------------------------------------------------------------
    // Progress is measured through successful completion of assigned tasks.
    //
    // If no task completion occurs for an extended period, the system is
    // assumed to be effectively stalled, even if valid plans continue to be
    // generated.
    //
    // This safeguard prevents simulations from running indefinitely in
    // situations where agents are trapped in cyclic or non-productive
    // behaviour.
    // ------------------------------------------------------------------

    timesteps_without_progress = 0;
    terminated_no_solution = false;
    refresh_effective_traversable();
}

// ------------------------------------------------------------------
// Dynamic obstacle state must remain consistent when a simulation is
// resumed or initialized from a predefined goal sequence.
//
// This routine reconstructs the obstacle configuration from the set of
// remaining unfinished goals so that obstacle availability matches the
// logical task state instead of relying solely on runtime updates.
// ------------------------------------------------------------------

void BasicSystem::reset_dynamic_obstacles_from_assigned_goals()
{
    dynamic_open_obstacles.clear();
    dynamic_closed_obstacles.clear();

    int n = std::min((int)assigned_goal_locations.size(), (int)assigned_goal_progress.size());
    for (int k = 0; k < n; k++)
    {
        const auto& goals = assigned_goal_locations[k];
        int next_goal = std::max(0, assigned_goal_progress[k]);
        for (int i = next_goal; i < (int)goals.size(); i++)
        {
            if (i % 2 == 0)
                dynamic_closed_obstacles.insert(goals[i]);
        }
    }
}

// ------------------------------------------------------------------
// Effective traversability is derived from multiple sources:
//
// 1. Static per-agent permissions loaded from traverse files.
// 2. Dynamically opened obstacle locations.
// 3. Dynamically closed obstacle locations.
//
// Rebuilding the effective traversable sets centralizes this logic and
// guarantees that all planning components operate on a consistent view
// of the current obstacle state.
// ------------------------------------------------------------------

void BasicSystem::refresh_effective_traversable()
{
    // Apply dynamic "box present" obstacles at graph level.
    const_cast<BasicGraph&>(G).set_dynamic_blocked(dynamic_closed_obstacles);

    solver.path_planner.agent_traversable.clear();
    for (int a = 0; a < num_of_drives; a++)
    {
        auto& s = solver.path_planner.agent_traversable[a];
        if (a < (int)base_agent_traversable.size())
            s.insert(base_agent_traversable[a].begin(), base_agent_traversable[a].end());
        s.insert(dynamic_open_obstacles.begin(), dynamic_open_obstacles.end());
    }
}

// ------------------------------------------------------------------
// Traversable obstacle overrides are loaded per agent from an external
// file so that different agents may be granted access to different
// obstacle locations.
//
// These permissions are used to model reserved pickup/delivery cells
// that remain blocked for most agents while still being reachable by
// the agent responsible for servicing the corresponding task.
// ------------------------------------------------------------------

bool BasicSystem::load_traverse_from_txt(const std::string& fname)
{
    std::ifstream myfile(fname.c_str());
    if (!myfile.is_open())
        return false;

    std::string line;
    int declared_agents = -1;
    int line_agent = 0;
    while (getline(myfile, line))
    {
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos)
            line = line.substr(0, comment_pos);
        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        std::vector<int> vals;
        int v;
        while (ss >> v)
            vals.push_back(v);
        if (vals.empty())
            continue;

        if (declared_agents < 0 && vals.size() == 1)
        {
            declared_agents = vals[0];
            continue;
        }

        int agent = line_agent;
        int offset = 0;
        if (vals.size() >= 2 && vals[0] == line_agent)
        {
            agent = vals[0];
            offset = 1;
        }

        if (agent >= 0 && agent < num_of_drives)
        {
            for (int i = offset; i < (int)vals.size(); i++)
            {
                int loc = vals[i];
                if (0 <= loc && loc < G.size())
                    solver.path_planner.agent_traversable[agent].insert(loc);
            }
        }
        line_agent++;
    }
    myfile.close();
    if (declared_agents >= 0 && declared_agents != line_agent)
    {
        std::cout << "Traverse file declares " << declared_agents << " agents but has " << line_agent << " data lines." << std::endl;
    }
    return true;
}

bool BasicSystem::load_records()
{
	boost::char_separator<char> sep1(";");
	boost::char_separator<char> sep2(",");
	string line;

	// load paths
	std::ifstream myfile(outfile + "/paths.txt");

	if (!myfile.is_open())
			return false;

	timestep = INT_MAX;
	getline(myfile, line);
	if (atoi(line.c_str()) != num_of_drives)
	{
		cout << "The path file does not match the settings." << endl;
		exit(-1);
	}
	for (int k = 0; k < num_of_drives; k++)
	{
		getline(myfile, line);
		boost::tokenizer< boost::char_separator<char> > tok1(line, sep1);
		for (auto task : tok1)
		{
			boost::tokenizer< boost::char_separator<char> > tok2(task, sep2);
			boost::tokenizer< boost::char_separator<char> >::iterator beg = tok2.begin();
			int loc = atoi((*beg).c_str());
			beg++;
			int orientation = atoi((*beg).c_str());
			beg++;
			int time = atoi((*beg).c_str());
			paths[k].emplace_back(loc, time, orientation);
		}
		timestep = min(timestep, paths[k].back().timestep);
	}
	myfile.close();

	// pick the timestep
	timestep = int((timestep - 1) / simulation_window) * simulation_window; //int((timestep - 1) / simulation_window) * simulation_window;

	// load tasks
	myfile.open(outfile + "/tasks.txt");
	if (!myfile.is_open())
		return false;

	getline(myfile, line);
	if (atoi(line.c_str()) != num_of_drives)
	{
		cout << "The task file does not match the settings." << endl;
		exit(-1);
	}
	for (int k = 0; k < num_of_drives; k++)
	{
		getline(myfile, line);
		boost::tokenizer< boost::char_separator<char> > tok1(line, sep1);
		for (auto task : tok1)
		{
			boost::tokenizer< boost::char_separator<char> > tok2(task, sep2);
			boost::tokenizer< boost::char_separator<char> >::iterator beg = tok2.begin();
			int loc = atoi((*beg).c_str());
			beg++;
			int time = atoi((*beg).c_str());
			if (time >= 0 && time <= timestep)
			{
				finished_tasks[k].emplace_back(loc, time);
				timestep = max(timestep, time);
			}
			else
			{
				goal_locations[k].emplace_back(loc, 0);
			}
		}
	}
	myfile.close();
	return true;
}
