//=======================================================================

#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
//#include <algorithm>  // for remove_if
#include <ctime>
using namespace boost;
using namespace std;

int RANDOM_WALK_STEPS = 100000;

AgentsLoader::AgentsLoader(string fname, const MapLoader &ml, int agentsNum = 0, int minSize = 0, int maxSize = 0){

  string line;

  ifstream myfile (fname.c_str());

	if (myfile.is_open()) 
	{
		getline (myfile,line);
		char_separator<char> sep(",");
		tokenizer< char_separator<char> > tok(line, sep);
		tokenizer< char_separator<char> >::iterator beg=tok.begin();
		this->num_of_agents = atoi ( (*beg).c_str() );

		for (int i=0; i<num_of_agents; i++) 
		{
			  getline (myfile, line);
			  tokenizer< char_separator<char> > col_tok(line, sep);
			  tokenizer< char_separator<char> >::iterator c_beg=col_tok.begin();
			  pair<int,int> curr_pair;
			  // read start [row,col] for agent i
			  curr_pair.first = atoi ( (*c_beg).c_str() );
			  c_beg++;
			  curr_pair.second = atoi ( (*c_beg).c_str() );
			  this->initial_locations.push_back(curr_pair);
			  // read goal [row,col] for agent i
			  c_beg++;
			  curr_pair.first = atoi ( (*c_beg).c_str() );
			  c_beg++;
			  curr_pair.second = atoi ( (*c_beg).c_str() );
			  this->goal_locations.push_back(curr_pair);
			  // read max velocity and accelration for agent i
			 /* c_beg++;
			  this->max_v.push_back(atof((*c_beg).c_str()));
			  c_beg++;
			  this->max_w.push_back(atof((*c_beg).c_str()));
			  c_beg++;
			  this->max_a.push_back(atof((*c_beg).c_str()));*/
			  c_beg++;
			  this->sizes.push_back((int)atof((*c_beg).c_str()));
		}
		myfile.close();
	} 
	else if(agentsNum > 0 && maxSize >= minSize && minSize > 0)//Generate agents randomly
	{
		  this->num_of_agents = agentsNum;
		  bool* starts = new bool[ml.rows * ml.cols];
		  for (int i = 0; i<ml.rows * ml.cols; i++)
			  starts[i] = false;
		  bool* goals = new bool[ml.rows * ml.cols];
		  for (int i = 0; i<ml.rows * ml.cols; i++)
			  goals[i] = false;
		  // Choose random start locations
		  for (int k = 0; k < agentsNum; k++)
		  {
			  int x = rand() % ml.rows, y = rand() % ml.cols;
			  int start = x * ml.cols +y;
			  int size = rand() % (maxSize - minSize + 1) + minSize;
			  bool flag = true;
			  for(int i = 0; i < size && flag; i++)
				for(int j = 0; j < size && flag; j++)
					if (ml.my_map[start + i * ml.cols + j] || starts[start + i * ml.cols + j])
						flag = false;
			  if (flag)
			  {
					// update start
					this->initial_locations.push_back(make_pair(x,y));
					for (int i = 0; i < size; i++)
						for (int j = 0; j < size; j++)
							starts[start + i * ml.cols + j] = true;
					// update size
					this->sizes.push_back(size);

					// random walk
					int loc = start;
					bool* temp_map = new bool[ml.rows * ml.cols];
					for (int p = 0; p < ml.rows * ml.cols; p++)
					{
						bool flag = true;
						for (int i = 0; i < size && flag; i++)
							for (int j = 0; j < size && flag; j++)
								if (p + i * ml.cols + j < ml.cols * ml.rows && ml.my_map[p + i * ml.cols + j])
									flag = false;
						temp_map[p] = flag;
					}
					for (int walk = 0; walk < RANDOM_WALK_STEPS; walk++)
					{
						int directions[] = {0, 1, 2, 3, 4};
						random_shuffle(directions, directions + 5);
						int i = 0;
						for(; i< 5; i++)
						{
							int next_loc = loc + ml.moves_offset[directions[i]];
							if (0 <= next_loc && next_loc < ml.rows * ml.cols && temp_map[next_loc])
							{
								loc = next_loc;
								break;
							}
						}
					}
					// find goal
					bool flag = false;
					int goal = loc;
					while (!flag)
					{
						int directions[] = { 0, 1, 2, 3, 4 };
						random_shuffle(directions, directions + 5);
						int i = 0;
						for (; i< 5; i++)
						{
							int next_loc = goal + ml.moves_offset[directions[i]];
							if (0 <= next_loc && next_loc < ml.rows * ml.cols && temp_map[next_loc])
							{
								goal = next_loc;
								break;
							}
						}
						flag = true;
						for (int i = 0; i < size && flag; i++)
							for (int j = 0; j < size && flag; j++)
								if (goals[goal + i * ml.cols + j])
									flag = false;
					}
					//update goal
					this->goal_locations.push_back(make_pair(goal / ml.cols, goal % ml.cols));
					for (int i = 0; i < size; i++)
						for (int j = 0; j < size; j++)
							goals[goal + i * ml.cols + j] = true;
					// update others
					/*this->max_v.push_back(1);
					this->max_w.push_back(1);
					this->max_a.push_back(1);*/
			  }
			  else
			  {
				  k--;
			  }
		  }
		  saveToFile(fname);
	}
	else
	{
	  cerr << "Agent file \"" << fname << "\" not found." << std::endl;
	  exit(10);
	}
}

void AgentsLoader::printAgentsInitGoal () {
  cout << "AGENTS:" << endl;;
  for (int i=0; i<num_of_agents; i++) {
    cout << "Agent" << i << " : I=(" << initial_locations[i].first << "," << initial_locations[i].second << ") ; G=(" <<
      goal_locations[i].first << "," << goal_locations[i].second << ")" << endl;
  }
  cout << endl;
}

AgentsLoader::~AgentsLoader() {
  // vectors are on stack, so they are freed automatically
}

// create an empty object
AgentsLoader::AgentsLoader() {
  num_of_agents = 0;
}

// returns the agents' ids if they occupy [row,col] (first for start, second for goal)
pair<int, int> AgentsLoader::agentStartOrGoalAt(int row, int col) {
  int f = -1;
  int s = -1;
  for (vector< pair<int, int> >::iterator it = initial_locations.begin(); it != initial_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      f = (int)std::distance(initial_locations.begin(), it);
  for (vector< pair<int, int> >::iterator it = goal_locations.begin(); it != goal_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      s = (int)std::distance(goal_locations.begin(), it);
  return make_pair(f, s);
}


void AgentsLoader::clearLocationFromAgents(int row, int col) {
  pair<int, int> idxs = agentStartOrGoalAt(row, col);
  if ( idxs.first != -1 ) {  // remove the agent who's start is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.first );
    goal_locations.erase ( goal_locations.begin() + idxs.first );
    num_of_agents--;
  }
  idxs = agentStartOrGoalAt(row, col);
  if ( idxs.second != -1 ) {  // remove the agent who's goal is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.second );
    goal_locations.erase( goal_locations.begin() + idxs.second );
    num_of_agents--;
  }
}


// add an agent
void AgentsLoader::addAgent(int start_row, int start_col, int goal_row, int goal_col) {
  this->initial_locations.push_back(make_pair(start_row, start_col));
  this->goal_locations.push_back(make_pair(goal_row, goal_col));
  num_of_agents++;
}

void AgentsLoader::saveToFile(std::string fname) {
  ofstream myfile;
  myfile.open(fname);
  myfile << num_of_agents << endl;
  for (int i = 0; i < num_of_agents; i++)
    myfile << initial_locations[i].first << "," << initial_locations[i].second << ","
           << goal_locations[i].first << "," << goal_locations[i].second << ","
		   /*<< max_v[i] << "," << max_a[i] << "," << max_w[i] << ","*/ << sizes[i] << endl;
  myfile.close();
}
