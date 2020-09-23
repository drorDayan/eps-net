#include "single_agent_icbs.h"
#include <boost/heap/fibonacci_heap.hpp>

void SingleAgentICBS::updatePath(Node* goal) 
{
	path.clear();
	Node* curr = goal;
	while (curr->timestep != 0) 
	{
		path.resize(path.size() + 1);
		path.back().location = curr->loc;
		curr = curr->parent;
	}
	path.resize(path.size() + 1);
	path.back().location = start_location;
	reverse(path.begin(), path.end());
	path_cost = goal->g_val;
}

inline void SingleAgentICBS::releaseClosedListNodes(hashtable_t* allNodes_table) 
{
	hashtable_t::iterator it;
	for (it = allNodes_table->begin(); it != allNodes_table->end(); it++)
	{
		delete ((*it).second);
	}
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
int SingleAgentICBS::extractLastGoalTimestep(int goal_location, const std::vector<std::list< std::pair<int, int> > >* cons) const
{
	int x_g = goal_location / num_col, y_g = goal_location % num_col;
	if (cons != NULL) 
	{
		for (int t = static_cast<int>(cons->size()) - 1; t > 0; t--) 
		{
			for (std::list< std::pair<int, int> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) 
			{
				if ((*it).second < 0) // edge constraint
					continue;
				else if ((*it).first / num_col <= x_g && x_g <= (*it).second / num_col
					&& (*it).first % num_col <= y_g && y_g <= (*it).second % num_col) 
				{
					return (t);
				}
			}
		}
	}
	return -1;
}

// Checks if a vaild path found (wrt my_map and constraints)
// Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
inline bool SingleAgentICBS::isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)  const 
{
	if (cons == NULL)
		return false;
	int x = next_id / num_col, y = next_id % num_col;

	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_timestep < static_cast<int>(cons->size())) {
		for (std::list< std::pair<int, int> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it) 
		{
			if ((*it).second >= 0 && (*it).first / num_col <= x && x <= (*it).second / num_col
				&& (*it).first % num_col <= y && y <= (*it).second % num_col) 
				return true;
		}
	}

	// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
	if (next_timestep > 0 && next_timestep - 1 < static_cast<int>(cons->size())) 
	{
		for (std::list< std::pair<int, int> >::const_iterator it = cons->at(next_timestep - 1).begin(); it != cons->at(next_timestep - 1).end(); ++it) 
		{
			if ((*it).second <= 0 && (*it).first == curr_id && (*it).second == -next_id - 1) 
				return true;
		}
	}

	return false;
}


// Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
// Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
int SingleAgentICBS::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, bool* res_table, int max_plan_len) {
	int retVal = 0;
	if (next_timestep >= max_plan_len) 
	{
		// check vertex constraints (being at an agent's goal when he stays there because he is done planning)
		if (res_table[next_id + (max_plan_len - 1)*map_size] == true)
			retVal++;
		// Note -- there cannot be edge conflicts when other agents are done moving
	}
	else 
	{
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		if (res_table[next_id + next_timestep*map_size] == true)
			retVal++;
		// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
		// which means that res_table is occupied with another agent for [curr_id,next_timestep] and [next_id,next_timestep-1]
		if (res_table[curr_id + next_timestep*map_size] && res_table[next_id + (next_timestep - 1)*map_size])
			retVal++;
	}
	return retVal;
}

void SingleAgentICBS::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) 
{
	for (Node* n : open_list) 
	{
		if (n->getFVal() > old_lower_bound && n->getFVal() <= new_lower_bound) 
			n->focal_handle = focal_list.push(n);
	}
}

bool SingleAgentICBS::validMove(int curr, int next) const
{
	if (next < 0 && next >= map_size)
		return false;
	int curr_x = curr / num_col;
	int curr_y = curr % num_col;
	int next_x = next / num_col;
	int next_y = next % num_col;
	return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}


// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleAgentICBS::findPath(double f_weight, const std::vector < std::list< std::pair<int, int> > >* constraints, bool* res_table, int max_plan_len) {
	// clear data structures if they had been used before
	// (note -- nodes are deleted before findPath returns)
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	num_expanded = 0;
	num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()

	 // generate start and add it to the OPEN list
	 // param: int loc, int g_val, int h_val, Node* parent, int timestep, int num_internal_conf, bool in_openlist
	Node* start = new Node(start_location, 0, my_heuristic[start_location], NULL, 0, 0, false); 
	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table[start] = start;
	min_f_val = start->getFVal();
	lower_bound = f_weight * min_f_val;

	int lastGoalConsTime = extractLastGoalTimestep(goal_location, constraints); // the minimal plan length for the agent

	while (!focal_list.empty()) 
	{
		Node* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);
		curr->in_openlist = false;
		num_expanded++;

		// check if the popped node is a goal
		if (curr->loc == goal_location && curr->timestep > lastGoalConsTime) 
		{
			updatePath(curr);
			releaseClosedListNodes(&allNodes_table);
			return true;
		}

		// iterator over all possible actions
		for (int i = 0; i < 5; i++)
		{
			int next_id = curr->loc + moves_offset[i];
			int next_timestep = curr->timestep + 1;
			if (validMove(curr->loc, next_id) //valid move
				&& my_gvals_lb[next_id] < H_MAX //no obstacles
				&& !isConstrained(curr->loc, next_id, next_timestep, constraints)) // not constrained
			{	
				// compute cost to next_id via curr node
				int next_g_val = curr->g_val + 1;
				int next_h_val = my_heuristic[next_id];
				int next_internal_conflicts = 0;
				if (max_plan_len > 0)  // check if the reservation table is not empty (that is tha max_length of any other agent's plan is > 0)
					next_internal_conflicts = curr->num_internal_conf + numOfConflictsForStep(curr->loc, next_id, next_timestep, res_table, max_plan_len);
				// generate (maybe temporary) node
				Node* next = new Node(next_id, next_g_val, next_h_val, curr, next_timestep, next_internal_conflicts, false); 
				
				// try to retrieve it from the hash table
				it = allNodes_table.find(next);
				if (it == allNodes_table.end()) // next is a new node
				{
					next->open_handle = open_list.push(next);
					next->in_openlist = true;
					num_generated++;
					if (next->getFVal() <= lower_bound)
						next->focal_handle = focal_list.push(next);
					allNodes_table[next] = next;
				}
				else // update existing node's if needed (only in the open_list)
				{  
					delete(next);  // not needed anymore -- we already generated it before
					Node* existing_next = (*it).second;
					if (existing_next->in_openlist == true) // if its in the open list
					{  
						if (existing_next->getFVal() > next_g_val + next_h_val ||
							(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) 
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
							bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
							bool update_open = false;
							if ((next_g_val + next_h_val) <= lower_bound)   // if the new f-val qualify to be in FOCAL
							{
								if (existing_next->getFVal() > lower_bound)
									add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
								else
									update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
							}
							if (existing_next->getFVal() > next_g_val + next_h_val)
								update_open = true;
							
							// update existing node
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							existing_next->num_internal_conf = next_internal_conflicts;
							if (update_open) 
								open_list.increase(existing_next->open_handle);  // increase because f-val improved
							if (add_to_focal) 
								existing_next->focal_handle = focal_list.push(existing_next);
							if (update_in_focal)
								focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
						}
					} // end update a node in open list
					else // if its in the closed list (reopen)
					{  
						if (existing_next->getFVal() > next_g_val + next_h_val ||
							(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) 
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							existing_next->num_internal_conf = next_internal_conflicts;
							existing_next->open_handle = open_list.push(existing_next);
							existing_next->in_openlist = true;
							if (existing_next->getFVal() <= lower_bound)
								existing_next->focal_handle = focal_list.push(existing_next);
						}
					}  // end update a node in closed list
				}  // end update an existing node
			}
		}  // end for loop that generates successors
		  
		 // update FOCAL if min f-val increased
		if (open_list.size() == 0)  // in case OPEN is empty, no path found...
			return false;
		Node* open_head = open_list.top();
		if (open_head->getFVal() > min_f_val) 
		{
			double new_min_f_val = open_head->getFVal();
			double new_lower_bound = f_weight * new_min_f_val;
			updateFocalList(lower_bound, new_lower_bound, f_weight);
			min_f_val = new_min_f_val;
			lower_bound = new_lower_bound;
		}
	}  // end while loop
	
	// no path found
	path.clear();
	releaseClosedListNodes(&allNodes_table);
	return false;
}

SingleAgentICBS::~SingleAgentICBS() 
{
	delete[] my_map;
	delete[] my_heuristic;
	delete[] my_gvals_lb;
	delete (empty_node);
	delete (deleted_node);
}


SingleAgentICBS::SingleAgentICBS(int start_location, int goal_location, 
	const int* my_heuristic, const int* my_gvals_lb, const bool* my_map, int map_size, const int* moves_offset,
	int size, int num_col) 
{
	this->my_heuristic = my_heuristic;
	this->my_gvals_lb = my_gvals_lb;
	this->my_map = my_map;
	this->moves_offset = moves_offset;

	this->start_location = start_location;
	this->goal_location = goal_location;
	this->map_size = map_size;

	this->num_expanded = 0;
	this->num_generated = 0;
	this->path_cost = 0;
	this->lower_bound = 0;
	this->min_f_val = 0;

	this->agent_size = size;
	this->num_col = num_col;

	// initialize allNodes_table (hash table)
	empty_node = new Node();
	empty_node->loc = -1;
	deleted_node = new Node();
	deleted_node->loc = -2;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);
}

