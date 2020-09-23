/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, Dec 2018
*/

/*driver.cpp
* Solve a LA-MAPF instance with rectangele agents on 2D grid.
* vertex conflicts happen when two agents overlap
*  edge conflicts happen when two agents swap their lcoations
 */


#include "agents_loader.h"
#include "epea_search.h"
#include "icbs_search.h"

#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

using namespace std;

int main(int argc, char** argv) {

	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("solver,s", po::value<std::string>()->required(), "solvers (EPEA, ICBS, SYM, ASYM, MAX, MAXpH")
		("lookahead,l", po::value<int>()->default_value(0), "lookahead steps for MDDs")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("minSize,i", po::value<int>()->default_value(0), "minimal size of agents")
		("maxSize,x", po::value<int>()->default_value(0), "maxmimal size of agents")
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

	// read the map file 
	MapLoader ml(vm["map"].as<string>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), ml, vm["agentNum"].as<int>(), vm["minSize"].as<int>(), vm["maxSize"].as<int>());

 
	//Cutoff Time
	int TIME_LIMIT = TIME_LIMIT_DEFAULT;
	if(vm.count("cutoffTime"))
		TIME_LIMIT = vm["cutoffTime"].as<int>() * 1000;

	if(vm["solver"].as<string>() == "EPEA")
	{	
		EPEASearch epea(TIME_LIMIT, ml, al);
		bool res;
		res = epea.runEPEASearch();
		ofstream stats;
		stats.open(vm["output"].as<string>(), ios::app);
		stats << epea.runtime << "," << "," << "," <<
			epea.num_expanded << "," << epea.num_generated << "," <<
			vm["agents"].as<string>() << "," << epea.solution_cost << "," << epea.solution_depth << "," <<
			vm["solver"].as<string>() << endl;
		stats.close();
	}
	else //CBS variants
	{
		constraint_strategy s;
		if(vm["solver"].as<string>() == "ICBS")
			s = constraint_strategy::BASIC;
		else if (vm["solver"].as<string>() == "ASYM")
			s = constraint_strategy::ASYMMETRIC;
		else if (vm["solver"].as<string>() == "SYM")
			s = constraint_strategy::SYMMETRIC;
		else if (vm["solver"].as<string>() == "MAX")
			s = constraint_strategy::MAX;
		else if (vm["solver"].as<string>() == "MAXpH")
				s = constraint_strategy::MAXpH;
		else
			return -1;
		ICBSSearch icbs(TIME_LIMIT, ml, al, 1.0, s, vm["lookahead"].as<int>());
		bool res;
		res = icbs.runICBSSearch();
		ofstream stats;
		stats.open(vm["output"].as<string>(), ios::app);
		stats << icbs.runtime << "," <<
			icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
			icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
			vm["agents"].as<string>() << "," << icbs.solution_cost << "," << 
			icbs.min_f_val - icbs.dummy_start->g_val << "," <<
			vm["solver"].as<string>() << "," << vm["lookahead"].as<int>() << endl;
		stats.close();
	}

	return 0;
}
