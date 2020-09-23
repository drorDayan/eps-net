/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, Dec 2018
*/

/*driver.cpp
* Solve a LA-MAPF instance on a general graph.
* vertex/edge conflicts are preprocessed and saved
*/

#include "ICBSSearch.h"
#include "searchgraph.h"

#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <cstdlib>
#include <cmath>


#include "boost/program_options.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>




namespace pt = boost::property_tree;
using namespace std;

int main(int argc, char** argv) {

	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("output,o", po::value<std::string>()->required(), "output file")
		("solver,s", po::value<std::string>()->required(), "solvers (ICBS, ASYM, MAX")
		("agentNum,k", po::value<int>()->required(), "number of agents")
		("graphInfo,g", po::value<bool>()->default_value(false), "Graph Information")
		("heuristic,h", po::value<bool>()->default_value(false), "using heuristics in the high level")
		("weight,w", po::value<double>()->default_value(1), "weight for ECBS")
		("lookahead,l", po::value<int>()->default_value(0), "lookahead steps for MAX")
		("cutoffTime,t", po::value<int>(), "cutoff time (s)")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	// read the map file and construct its two-dim array
	searchGraph_t G;
	std::unordered_map<std::string, vertex_t> V;
	std::unordered_map<std::string, edge_t> E;
	std::vector<vertex_t> starts, goals;
	loadSearchGraph(G, V, E, starts, goals, vm["map"].as<string>());

	if (vm["graphInfo"].as<bool>())
	{
		getGraphInfo(G);
		return 0;
	}

	//Cutoff Time
	int TIME_LIMIT = TIME_LIMIT_DEFAULT;
	if (vm.count("cutoffTime"))
		TIME_LIMIT = vm["cutoffTime"].as<int>() * 1000;

	bool useHeuristics = false;
	int lookahead = 0;

	constraint_strategy s;
	if(vm["solver"].as<string>() == "ICBS")
		s = constraint_strategy::ICBS;
	else if (vm["solver"].as<string>() == "ASYM")
		s = constraint_strategy::ASYM;
	else if (vm["solver"].as<string>() == "MAX")
	{
		s = constraint_strategy::MAX;
		useHeuristics = vm["heuristic"].as<bool>();
		lookahead = vm["lookahead"].as<int>();
	}
	else
	{
		std::cerr << "Error solvers !" << endl;
		return -1;
	}
		
	ICBSSearch icbs(TIME_LIMIT, G, starts, goals, vm["agentNum"].as<int>(), s, vm["weight"].as<double>(), lookahead, useHeuristics);
	bool res;
	res = icbs.runICBSSearch();
	ofstream stats;
	stats.open(vm["output"].as<string>(), ios::app);
	stats << icbs.runtime << "," <<
		icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
		icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
		vm["map"].as<string>() << "," << icbs.solution_cost << "," << 
		icbs.min_sum_f_vals - icbs.dummy_start->g_val << "," <<
		vm["solver"].as<string>() << "," << 
		vm["agentNum"].as<int>() << "," << vm["lookahead"].as<int>() << "," <<
		vm["heuristic"].as<bool>() << endl;
	stats.close();

	return 0;
}
