#pragma once
#include "BasicGraph.h"
#include "States.h"
#include "PriorityGraph.h"
#include "PBS.h"
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"


class BasicSystem
{
public:
    // params for MAPF algotithms
	MAPFSolver& solver;
	bool hold_endpoints;
	bool useDummyPaths;
    int time_limit;
    int travel_time_window;
	//string potential_function;
	//double potential_threshold;
	//double suboptimal_bound;
    int screen;
    bool log;
    bool debug_positions = false;
    int num_of_drives;
    int seed;
    int simulation_window;
    int planning_window;
    int simulation_time;
    int mandatory_goal_wait = 5; // timesteps to stay on every pickup/delivery goal
    bool close_delivery_obstacles = true; // delivery cells stay blocked after drop-off

    // params for drive model
    bool consider_rotation;
    int k_robust;

    BasicSystem(const BasicGraph& G, MAPFSolver& solver);
    ~BasicSystem();

	// TODO
    /*bool load_config(std::string fname);
    bool generate_random_MAPF_instance();
    bool run();
	void print_MAPF_instance() const;
	void save_MAPF_instance(std::string fname) const;
	bool read_MAPF_instance(std::string fname);*/

    // I/O
    std::string outfile;
    std::string locations_file; // optional .txt file for start/goal definitions
    std::string traverse_file; // optional .txt file for per-agent obstacle overrides
    void save_results();
	double saving_time = 0; // time for saving results to files, in seconds
    int num_of_tasks; // number of finished tasks

	list<int> new_agents; // used for replanning a subgroup of agents

    // used for MAPF instance
    vector<State> starts;
    vector< vector<pair<int, int> > > goal_locations;
	// unordered_set<int> held_endpoints;
    int timestep;

    // record movements of drives
    std::vector<Path> paths;
    std::vector<std::list<std::pair<int, int> > > finished_tasks; // location + finish time

    bool congested() const;
	bool check_collisions(const vector<Path>& input_paths) const;

    // update
    void update_start_locations();
    void update_travel_times(unordered_map<int, double>& travel_times);
    void update_paths(const std::vector<Path*>& MAPF_paths, int max_timestep);
    void update_paths(const std::vector<Path>& MAPF_paths, int max_timestep);
    void update_initial_paths(vector<Path>& initial_paths) const;
    void update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const;
    
	void add_partial_priorities(const vector<Path>& initial_paths, PriorityGraph& initial_priorities) const;
	list<tuple<int, int, int>> move(); // return finished tasks
	void solve();
	void initialize_solvers();
	bool load_records();
	bool load_locations();
    bool all_assigned_goals_finished() const;
    bool loaded_locations_from_txt = false;
    int last_move_timestep = 0;
    bool last_solve_success = true;
    int max_consecutive_failed_plans = 20;
    int max_no_progress_timesteps = 200;
    int consecutive_failed_plans = 0;
    int timesteps_without_progress = 0;
    bool terminated_no_solution = false;


protected:
	bool solve_by_WHCA(vector<Path>& planned_paths,
		const vector<State>& new_starts, const vector< vector<pair<int, int> > >& new_goal_locations);
    bool LRA_called = false;

private:
	const BasicGraph& G;
    void print_positions_at_timestep(int t) const;
    void apply_goal_wait_to_solver_goals(vector<vector<pair<int, int> > >& solver_goals,
                                         const vector<int>* global_agent_ids = nullptr) const;
    bool load_locations_from_txt(const std::string& fname);
    bool load_traverse_from_txt(const std::string& fname);
    bool reachable_with_override(int start, int goal, const unordered_set<int>* traversable) const;
    void reset_dynamic_obstacles_from_assigned_goals();
    void refresh_effective_traversable();
    void save_assigned_goal_paths(std::ofstream& output) const;
    vector<int> assigned_start_locations;
    vector<vector<int> > assigned_goal_locations;
    vector<int> assigned_goal_progress;
    vector<unordered_set<int> > base_agent_traversable;
    unordered_set<int> dynamic_open_obstacles;
    unordered_set<int> dynamic_closed_obstacles;
    vector<int> goal_hold_end_timestep; // absolute timestep when the current goal can be completed
    vector<int> force_depart_location; // completed goal location that must be left on next plan
    vector<int> force_depart_from_timestep; // absolute timestep when goal completion happened
};
