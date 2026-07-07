// Adapted from RHCR (Rolling-Horizon Collision Resolution).
//
// Original source:
// https://github.com/Jiaoyang-Li/RHCR
//
// Modifications in this file:
// - Added fixed_locations_mode: when start/goal locations are loaded from a .txt
//   file, the simulation uses pre-assigned routes and stops as soon as all agents
//   reach their final goals.
// - Added assign_random_travel_location(): fallback goal assignment for agents
//   whose induct/eject queues are empty, picking any non-obstacle cell at random.
// - Guarded induct/eject assignment with !G.inducts.empty() / !G.ejects.empty()
//   checks to avoid crashes on maps that have no induct or eject stations.
// - Added Travel-type goal handling in update_goal_locations().
// - Changed simulation loop from a for-loop with fixed step to a while-loop with
//   variable step (last_move_timestep) so the horizon advances correctly when
//   move() returns early after a goal is reached mid-window.
// - Integrated terminated_no_solution flag: breaks the simulation loop on
//   consecutive replan failures detected in BasicSystem::solve().
// - Added timesteps_without_progress tracking: terminates if no task finishes
//   within max_no_progress_timesteps consecutive timesteps.
// - Skipped the congestion check in fixed_locations_mode to avoid false positives
//   during mandatory_goal_wait holds.
// - Fixed load_records() path: skips file loading when locations_file is non-empty
//   (file-based starts/goals supersede the record-based initialisation).
// - Added validation on startup to exit(-1) with a diagnostic message when a
//   start→goal segment is disconnected or invalid.
// - Initialised fixed_locations_mode = false in the constructor.
// - Excluded Travel and Obstacle cell types from the valid initial-goal filter.


#include "SortingSystem.h"
#include <stdlib.h>
#include "PBS.h"
#include <boost/tokenizer.hpp>
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"
#include <algorithm>


SortingSystem::SortingSystem(const SortingGrid& G, MAPFSolver& solver): BasicSystem(G, solver), c(8), G(G) {}


SortingSystem::~SortingSystem() {}


void SortingSystem::initialize_start_locations()
{
    int N = G.size();
    std::vector<bool> used(N, false);

    // Choose random start locations
    // Any non-obstacle locations can be start locations
    // Start locations should be unique
	for (int k = 0; k < num_of_drives;)
	{
		int loc = rand() % N;
		if (G.types[loc] != "Obstacle" && !used[loc])
		{
			int orientation = -1;
			if (consider_rotation)
			{
				orientation = rand() % 4;
			}
			starts[k] = State(loc, 0, orientation);
			paths[k].emplace_back(starts[k]);
			used[loc] = true;
			finished_tasks[k].emplace_back(loc, 0);
			k++;
		}
	}
}


void SortingSystem::initialize_goal_locations()
{
	if (hold_endpoints || useDummyPaths)
		return;
    // Choose random goal locations
    // a close induct location can be a goal location, or
    // any eject locations can be goal locations
    // Goal locations are not necessarily unique
    for (int k = 0; k < num_of_drives; k++)
    {
		int goal;
		if (!G.inducts.empty() && !G.ejects.empty() && k % 2 == 0) // to induction
		{
			goal = assign_induct_station(starts[k].location);
			drives_in_induct_stations[goal]++;
		}
		else if (!G.ejects.empty()) // to ejection
		{
			goal = assign_eject_station();
		}
		else
		{
			goal = assign_random_travel_location();
		}
		goal_locations[k].emplace_back(goal, 0);
    }
}


void SortingSystem::update_goal_locations()
{
    if (fixed_locations_mode)
    {
        // Keep finished agents at their current locations in fixed start/goal mode.
        for (int k = 0; k < num_of_drives; k++)
        {
            if (goal_locations[k].empty())
                goal_locations[k].emplace_back(paths[k][timestep].location, timestep);
        }
        return;
    }

	for (int k = 0; k < num_of_drives; k++)
	{
		pair<int, int> curr(paths[k][timestep].location, timestep); // current location

		pair<int, int> goal; // The last goal location
		if (goal_locations[k].empty())
		{
			goal = curr;
		}
		else
		{
			goal = goal_locations[k].back();
		}
		int min_timesteps = G.get_Manhattan_distance(curr.first, goal.first); // cannot use h values, because graph edges may have weights  // G.heuristics.at(goal)[curr];
		min_timesteps = max(min_timesteps, goal.second);
		while (min_timesteps <= simulation_window)
			// The agent might finish its tasks during the next planning horizon
		{
			// assign a new task
			int next;
			if (G.types[goal.first] == "Induct")
			{
				next = assign_eject_station();
			}
			else if (G.types[goal.first] == "Eject")
			{
				next = assign_induct_station(curr.first);
				drives_in_induct_stations[next]++; // the drive will go to the next induct station
			}
            else if (G.types[goal.first] == "Travel")
            {
                next = assign_random_travel_location();
            }
			else
			{
				std::cout << "ERROR in update_goal_function()" << std::endl;
				std::cout << "The fiducial type should not be " << G.types[curr.first] << std::endl;
				exit(-1);
			}
			goal_locations[k].emplace_back(next, 0);
			min_timesteps += G.get_Manhattan_distance(next, goal.first); // G.heuristics.at(next)[goal];
			min_timesteps = max(min_timesteps, goal.second);
			goal = make_pair(next, 0);
		}
	}
}


int SortingSystem::assign_induct_station(int curr) const
{
    if (drives_in_induct_stations.empty())
        return assign_random_travel_location();

    int assigned_loc;
	double min_cost = DBL_MAX;
	for (auto induct : drives_in_induct_stations)
	{
		double cost = G.heuristics.at(induct.first)[curr] + c * induct.second;
		if (cost < min_cost)
		{
			min_cost = cost;
			assigned_loc = induct.first;
		}
	}
    return assigned_loc;
}


int SortingSystem::assign_eject_station() const
{
    if (G.ejects.empty())
        return assign_random_travel_location();

	int n = rand() % G.ejects.size();
	boost::unordered_map<std::string, std::list<int> >::const_iterator it = G.ejects.begin();
	std::advance(it, n);
	int p = rand() % it->second.size();
	auto it2 = it->second.begin();
	std::advance(it2, p);
	return *it2;
}


int SortingSystem::assign_random_travel_location() const
{
    while (true)
    {
        int loc = rand() % G.size();
        if (G.types[loc] != "Obstacle")
            return loc;
    }
}

void SortingSystem::simulate(int simulation_time)
{
    std::cout << "*** Simulating " << seed << " ***" << std::endl;
    this->simulation_time = simulation_time;
    initialize();

    while (timestep < simulation_time)
	{
        if (fixed_locations_mode && all_assigned_goals_finished())
        {
            std::cout << "All agents reached assigned goals. Stopping simulation early." << std::endl;
            break;
        }

		std::cout << "Timestep " << timestep << std::endl;

		update_start_locations();
		update_goal_locations();
		solve();
        if (terminated_no_solution)
        {
            std::cout << "Terminating simulation: no solution for many consecutive replans." << std::endl;
            break;
        }

		// move drives
        int old_timestep = timestep;
		auto new_finished_tasks = move();
		std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl;
        int moved_steps = std::max(1, last_move_timestep - old_timestep);

		// update tasks
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;
			finished_tasks[id].emplace_back(loc, t);
			num_of_tasks++;
			if (G.types[loc] == "Induct")
			{
				drives_in_induct_stations[loc]--; // the drive will leave the current induct station
			}
		}
        if (new_finished_tasks.empty())
        {
            timesteps_without_progress += moved_steps;
            if (timesteps_without_progress >= max_no_progress_timesteps)
            {
                terminated_no_solution = true;
                std::cout << "Terminating simulation: no tasks finished for "
                          << timesteps_without_progress << " timesteps." << std::endl;
                break;
            }
        }
        else
        {
            timesteps_without_progress = 0;
        }
		
		

		// In fixed_locations_mode agents have pre-assigned goals and the simulation
		// ends via all_assigned_goals_finished(). The congestion check would fire
		// false positives during mandatory_goal_wait holds, so skip it.
		if (!fixed_locations_mode && congested())
		{
			cout << "***** Too many traffic jams ***" << endl;
			break;
		}

        // Move horizon may be shortened inside move() when a goal is reached.
        int next_timestep = last_move_timestep;
        if (next_timestep <= timestep)
            next_timestep = timestep + simulation_window;
        timestep = next_timestep;
	}

    update_start_locations();
    std::cout << std::endl << "Done!" << std::endl;
    save_results();
}


void SortingSystem::initialize()
{
	initialize_solvers();
    fixed_locations_mode = false;

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);

	for (const auto induct : G.inducts)
	{
		drives_in_induct_stations[induct.second] = 0;
	}

    bool succ = false;
    if (locations_file.empty())
    {
	    succ = load_records(); // continue simulating from the records
    }
	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
            if (!locations_file.empty())
            {
                std::cout << "Failed to load locations from " << locations_file << std::endl;
                std::cout << "Fix disconnected/invalid start-goal segments and run again." << std::endl;
                exit(-1);
            }
			cout << "Randomly generating initial locations" << endl;
			initialize_start_locations();
			initialize_goal_locations();
		}
        else if (loaded_locations_from_txt)
        {
            fixed_locations_mode = true;
        }
	}

	// initialize induct station counter
	for (int k = 0; k < num_of_drives; k++)
	{
		// goals
		int goal = goal_locations[k].back().first;
		if (G.types[goal] == "Induct")
		{
			drives_in_induct_stations[goal]++;
		}
		else if (G.types[goal] != "Eject" && G.types[goal] != "Travel" && G.types[goal] != "Obstacle")
		{
			std::cout << "ERROR in the type of goal locations" << std::endl;
			std::cout << "The fiducial type of the goal of agent " << k << " is " << G.types[goal] << std::endl;
			exit(-1);
		}
	}
}
