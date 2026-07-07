#include "SortingGraph.h"
#include <fstream>
#include <boost/tokenizer.hpp>
#include "StateTimeAStar.h"
#include <sstream>
#include <random>
#include <chrono>

bool SortingGrid::load_map(std::string fname)
{
    std::string line;
    std::ifstream myfile ((fname).c_str());
	if (!myfile.is_open())
    {
	    std::cout << "Map file " << fname << " does not exist. " << std::endl;
        return false;
    }
	
    std::cout << "*** Loading map ***" << std::endl;
    clock_t t = std::clock();
	std::size_t pos = fname.rfind('.');      // position of the file extension
    map_name = fname.substr(0, pos);     // get the name without extension
    getline (myfile, line); // skip the words "grid size"
	getline(myfile, line);
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	this->rows = atoi((*beg).c_str()); // read number of cols
	beg++;
	this->cols = atoi((*beg).c_str()); // read number of rows
	move[0] = 1;
	move[1] = -cols;
	move[2] = -1;
	move[3] = cols;

	getline(myfile, line); // read headers
    // Map external column names to internal direction indices.
    // Internal directions: 0=E, 1=N, 2=W, 3=S.
    std::array<int, 4> file_dir_col = {5, 6, 7, 8}; // default: legacy positional assumption
    {
        boost::tokenizer< boost::char_separator<char> > header_tok(line, sep);
        std::vector<std::string> headers;
        for (auto it = header_tok.begin(); it != header_tok.end(); ++it)
            headers.emplace_back(it->c_str());
        for (int i = 0; i < (int)headers.size(); i++)
        {
            if (headers[i] == "weight_to_EAST")
                file_dir_col[0] = i;
            else if (headers[i] == "weight_to_NORTH")
                file_dir_col[1] = i;
            else if (headers[i] == "weight_to_WEST")
                file_dir_col[2] = i;
            else if (headers[i] == "weight_to_SOUTH")
                file_dir_col[3] = i;
        }
    }

	//read tyeps, station ids and edge weights
	this->types.resize(rows * cols);
	this->weights.resize(rows * cols);
	for (int i = 0; i < rows * cols; i++)
	{
		getline(myfile, line);
		boost::tokenizer< boost::char_separator<char> > tok(line, sep);
		beg = tok.begin();
		beg++; // skip id
		std::string raw_type = std::string(beg->c_str()); // read type
        this->types[i] = (raw_type == "Obstacle") ? "Obstacle" : "Travel";
		beg++;
		if (raw_type == "Induct")
			this->inducts[beg->c_str()] = i; // read induct station id
		else if (raw_type == "Eject")
		{
			boost::unordered_map<std::string, std::list<int> >::iterator it = ejects.find(beg->c_str());
			if (it == ejects.end())
			{
				this->ejects[beg->c_str()] = std::list<int>();
			}
			this->ejects[beg->c_str()].push_back(i); // read eject station id
		}
		beg++;
		beg++; // skip x
		beg++; // skip y
		weights[i].resize(5);
        std::vector<std::string> remaining_tokens;
        for (; beg != tok.end(); ++beg)
            remaining_tokens.emplace_back(beg->c_str());
        // remaining_tokens should contain direction weights + wait weight.
        // Use header-based indices when present; fallback defaults preserve compatibility.
        for (int dir = 0; dir < 4; dir++)
        {
            int idx = file_dir_col[dir] - 5; // normalize to remaining_tokens offset
            if (idx < 0 || idx >= (int)remaining_tokens.size())
                weights[i][dir] = WEIGHT_MAX;
            else if (remaining_tokens[idx] == "inf")
                weights[i][dir] = WEIGHT_MAX;
            else
                weights[i][dir] = std::stod(remaining_tokens[idx]);
        }
        int wait_idx = 9 - 5; // "weight_for_WAIT"
        if (wait_idx >= 0 && wait_idx < (int)remaining_tokens.size() && remaining_tokens[wait_idx] != "inf")
            weights[i][4] = std::stod(remaining_tokens[wait_idx]);
        else
            weights[i][4] = WEIGHT_MAX;
	}

	myfile.close();
    double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
    std::cout << "Map size: " << rows << "x" << cols << " with " << inducts.size() << " induct stations and " <<
        ejects.size() << " eject stations." << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
    return true;
}


void SortingGrid::preprocessing(bool consider_rotation)
{
	std::cout << "*** PreProcessing map ***" << std::endl;
	clock_t t = std::clock();
	this->consider_rotation = consider_rotation;
	std::string fname;
	if (consider_rotation)
		fname = map_name + "_rotation_heuristics_table.txt";
	else
		fname = map_name + "_heuristics_table.txt";
	std::ifstream myfile(fname.c_str());
	bool succ = false;
	if (myfile.is_open())
	{
		succ = load_heuristics_table(myfile);
		myfile.close();
	}

    bool need_recompute = !succ;
    if (!need_recompute)
	{
        for (int loc = 0; loc < this->size(); loc++)
        {
            if (types[loc] != "Obstacle" && heuristics.find(loc) == heuristics.end())
            {
                need_recompute = true;
                break;
            }
        }
    }
	if (need_recompute)
	{
        heuristics.clear();
        for (int loc = 0; loc < this->size(); loc++)
        {
            if (types[loc] != "Obstacle")
                heuristics[loc] = compute_heuristics(loc);
        }
		save_heuristics_table(fname);
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}
