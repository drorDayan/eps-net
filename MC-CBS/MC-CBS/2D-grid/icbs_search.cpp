#include "icbs_search.h"
#include "compute_heuristic.h"
#include <iostream>
#include <ctime>

// return agent_id's location for the given timestep
// Note -- if timestep is longer than its plan length,
// then the location remains the same as its last cell)
inline int ICBSSearch::getAgentLocation(int agent_id, size_t timestep) const
{
	if (timestep >= paths[agent_id]->size())
		return paths[agent_id]->at(paths[agent_id]->size() - 1).location;// if last timestep > plan length, agent remains in its last location
	return paths[agent_id]->at(timestep).location;	// otherwise, return its location for that timestep
}

//return true iff agent1 and agent2 switched locations at timestep [t,t+1]
// used for detecting edge conflicts
inline bool ICBSSearch::switchedLocations(int agent1_id, int agent2_id, size_t timestep) const
{
	// if both agents at their goal, they are done moving (cannot switch places)
	if (timestep >= paths[agent1_id]->size() && timestep >= paths[agent2_id]->size())
		return false;
	if (getAgentLocation(agent1_id, timestep) == getAgentLocation(agent2_id, timestep + 1) &&
		getAgentLocation(agent1_id, timestep + 1) == getAgentLocation(agent2_id, timestep))
		return true;
	return false;
}

// return true if the two agents overlap when a1 stay at loc1 and a2 stays at loc2
//used for detecting vertex conflicts
inline bool ICBSSearch::overlaped(int loc1, int loc2, int size1, int size2)  const
{
	int x1 = loc1 / num_col;
	int x2 = loc2 / num_col;
	int y1 = loc1 % num_col;
	int y2 = loc2 % num_col;
	return (((x2 >= x1 && ((x2 - x1) < size1)) || ((x1 >= x2) && ((x1 - x2) < size2))) &&
		(((y1 >= y2) && ((y1 - y2) < size2)) || ((y2 >= y1) && ((y2 - y1) < size1))));
}


// Returns the maximal path length (among all agent)
size_t ICBSSearch::getPathsMaxLength() const
{
	size_t retVal = 0;
	for (int ag = 0; ag < num_of_agents; ag++)
		if (paths[ag] != NULL && paths[ag]->size() > retVal)
			retVal = paths[ag]->size();
	return retVal;
}

// Generates a boolean reservation table for paths (cube of map_size*max_timestep).
// This is used by the low-level ECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
void ICBSSearch::updateReservationTable(bool* res_table, size_t max_path_len, int exclude_agent) {
	for (int ag = 0; ag < num_of_agents; ag++) 
	{
		if (ag != exclude_agent && paths[ag] != NULL) 
		{
			for (size_t timestep = 0; timestep < max_path_len; timestep++)
			{
				// compute the four corners of the agent's shape
				int id = getAgentLocation(ag, timestep);
				int min_x = 0, min_y = 0, max_x = map_size / num_col - 1, max_y = num_col - 1;
				if (id / num_col > al.sizes[exclude_agent] - 1)
					min_x = id / num_col - (al.sizes[exclude_agent] - 1);
				if (id % num_col > al.sizes[exclude_agent] - 1)
					min_y = id % num_col - (al.sizes[exclude_agent] - 1);
				if (id / num_col + al.sizes[ag] - 1< map_size / num_col - 1)
					max_x = id / num_col + al.sizes[ag] - 1;
				if (id % num_col + al.sizes[ag] - 1 < num_col - 1)
					max_y = id % num_col + al.sizes[ag] - 1;
				
				//block every cell inside the shape
				for (int i = min_x; i <= max_x; i++)
				{
					for (int j = min_y; j <= max_y; j++)
					{
						res_table[timestep * map_size + i*num_col + j] = true;
					}
				}

			}
		}
	}
}

// backtracking to collect all constraints imposed to this agent
vector < list< pair<int, int> > >* ICBSSearch::collectConstraints(ICBSNode* curr, int agent_id)
{
	// extract all constraints on leaf_node->agent_id
	list < tuple<int, int, int> > constraints; 
	int max_timestep = -1;
	while (curr != dummy_start) {
		if (curr->agent_id == agent_id) {
			constraints.push_front(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		curr = curr->parent;
	}

	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > >(max_timestep + 1, list< pair<int, int> >());
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++)
	{
		cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));
	}
	return cons_vec;
}

// compute heuristics for the high-level search
int ICBSSearch::computeHeuristics(const ICBSNode& curr)
{
	if (curr.cardinalConf.empty())
		return 0;

	// build weighted conflict graph
	vector<vector<int>> w(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
		w[i].resize(num_of_agents, 0);
	for (list<Conflict*>::const_iterator it = curr.cardinalConf.begin(); it != curr.cardinalConf.end(); ++it)
	{
		if((*it)->hasBetterWeight(w[(*it)->a1][(*it)->a2], w[(*it)->a2][(*it)->a1]))
		{
			w[(*it)->a1][(*it)->a2] = (*it)->weight1;
			w[(*it)->a2][(*it)->a1] = (*it)->weight2;
		}
	}

	vector<vector<bool>> x(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
		x[i].resize(num_of_agents, false);
	return minimumWVC(w, x, 0, 0);
}

//brute-force search to compute minimum edge-weighted vertex cover
int ICBSSearch::minimumWVC(vector<vector<int>>& w, vector<vector<bool>>& x, int row, int col) const
{
	if (row < w.size() - 1)
		for (int i = row; i < w.size(); i++)
		{
			int c = 0;
			for (int j = col; j < w[i].size(); j++)
			{
				if (w[i][j] > 0)
				{
					int newRow,newCol;
					if(j == w[i].size() - 1)
					{
						newRow = i + 1;
						newCol = newRow + 1;
					}
					else
					{
						newRow = i;
						newCol = j + 1;
					}
					x[i][j] = true;
					x[j][i] = false;
					int h1 = minimumWVC(w, x, newRow, newCol);
					x[i][j] = false;
					x[j][i] = true;
					int h2 = minimumWVC(w, x, newRow, newCol);
					return min(h1,h2);
				}
			}
			col = i + 2;
		}
	int h = 0;
	for (int i = 0; i < w.size(); i++)
	{
		int c = 0;
		for (int j = 0; j < w[i].size(); j++)
		{
			if (x[i][j] && c < w[i][j])
				c = w[i][j];
		}
		h += c;
	}
	return h;
}

// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void ICBSSearch::copyConflicts(const list<Conflict*>& conflicts, list<Conflict*>& copy, int excluded_agent) const
{
	for (list<Conflict*>::const_iterator it = conflicts.begin(); it != conflicts.end(); ++it)
	{
		if ((*it)->a1 != excluded_agent && (*it)->a2 != excluded_agent)
		{
			Conflict* conf = new Conflict(**it);
			copy.push_back(conf);
		}
	}
}

// find conflicts between two agents
void ICBSSearch::findConflicts(ICBSNode& curr, int a1, int a2) const
{
	int min_path_length = paths[a1]->size() < paths[a2]->size() ?
		(int)paths[a1]->size() : (int)paths[a2]->size();
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (overlaped(loc1, loc2, al.sizes[a1], al.sizes[a2])) // vertex conflict
		{
			curr.unknownConf.push_back(new Conflict(timestep, a1, a2, loc1, loc2));
		}
		else if (switchedLocations(a1, a2, timestep))// edge conflict
		{
			curr.unknownConf.push_back(new Conflict(timestep, a1, a2, loc1, -loc2 - 1));
		}
	}
	if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (int timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (overlaped(loc1, loc2, al.sizes[a1_], al.sizes[a2_]))
			{// It's at least a semi vertex conflict
				curr.unknownConf.push_front(new Conflict(timestep, a1_, a2_, loc1, loc2, conflict_type::GEQSEMI));
				curr.unknownConf.front()->weight1 = timestep - (int) paths[a1_]->size() + 1;
			}
		}
	}
}

// find conflicts in the node
void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (curr.parent != NULL)// not root node
	{
		// Copy from parent
		copyConflicts(curr.parent->cardinalConf, curr.cardinalConf, curr.agent_id);
		copyConflicts(curr.parent->semiConf, curr.semiConf, curr.agent_id);
		copyConflicts(curr.parent->nonConf, curr.nonConf, curr.agent_id);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, curr.agent_id);

		// detect new agent conflict
		int a1 = curr.agent_id;
		for (int a2 = 0; a2 < num_of_agents; a2++)
		{
			if (a1 == a2)
				continue;
			findConflicts(curr, a1, a2);
		}
	}
	else // root node
	{
		for(int a1 = 0; a1 < num_of_agents ; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
}

// build MDD
void ICBSSearch::buildMDD(ICBSNode& curr, int id, int lookahead)
{
	curr.mdds[id] = new MDD();
	vector < list< pair<int, int> > >* cons_vec = collectConstraints(&curr, id);
	curr.mdds[id]->buildMDD(*cons_vec, (int) paths[id]->size(), lookahead, *search_engines[id]);
	curr.mdds[id]->numPointers = 1;
	delete cons_vec;
}


// Classify conflicts and choose the one with the highest proiority
Conflict* ICBSSearch::chooseConflict(ICBSNode &parent)
{
	if (parent.cardinalConf.empty() && parent.semiConf.empty() && parent.nonConf.empty() && parent.unknownConf.empty())
		return NULL; // No conflict, i.e., it is the goal

	Conflict* chosenCon = NULL;
	if (cons_strategy == constraint_strategy::MAX || cons_strategy == constraint_strategy::MAXpH)  // classify all unknown conflicts
	{
		// classsify all conflicts one by one
		while (!parent.unknownConf.empty())
		{
			Conflict* conflict = parent.unknownConf.front();
			parent.unknownConf.pop_front();
						
			if (conflict->type == conflict_type::GEQSEMI)// If the conflict happens after the first agent reached its goal. 
			{  // so it is a vertex conflict and it is at least semi-cardinal for the first agent
				if (parent.mdds[conflict->a2] == NULL)
				{
					buildMDD(parent, conflict->a2, lookahead);
				}
				// Initialize with asymmetric constraints here.
				conflict->ComputeAsymConstraints(num_col, map_size / num_col, al.sizes[conflict->a1], al.sizes[conflict->a2]);
					
				// compute the minimal rectangle constraints for a2 with each weight
				vector<int> mdd2_x_min(lookahead + 1, conflict->loc2 / num_col);
				vector<int> mdd2_x_max(lookahead + 1, conflict->loc2 / num_col);
				vector<int> mdd2_y_min(lookahead + 1, conflict->loc2 % num_col);
				vector<int> mdd2_y_max(lookahead + 1, conflict->loc2 % num_col);
				conflict->getMinimalRectangle(mdd2_x_min, mdd2_x_max, mdd2_y_min, mdd2_y_max,
					lookahead, parent.mdds[conflict->a2]->levels[conflict->timestep], num_col);
				
				//if cons2 area can cover its mdd area, then it is a cardinal conflict; otherwise, semicardinal
				for (int i = lookahead; i >= 0; i--)
				{
					if (conflict->cons2[0] / num_col <= mdd2_x_min[i] && mdd2_x_max[i] <= conflict->cons2[1] / num_col &&
						conflict->cons2[0] % num_col <= mdd2_y_min[i] && mdd2_y_max[i] <= conflict->cons2[1] % num_col)
					{
						conflict->weight2 = i + 1;
						conflict->type = conflict_type::CARDINAL;
						parent.cardinalConf.push_back(conflict);
						break;
					}
				}
				if (conflict->type != conflict_type::CARDINAL)
				{
					conflict->weight2 = 0;
					conflict->type = conflict_type::SEMI;
					parent.semiConf.push_front(conflict);
				}
			}
			else // If the conflict happens before the two agents reached their goals 
			{
				if (parent.mdds[conflict->a1] == NULL)
				{
					buildMDD(parent, conflict->a1, lookahead);
				}
				if (parent.mdds[conflict->a2] == NULL)
				{
					buildMDD(parent, conflict->a2, lookahead);
				}
					
				if (conflict->loc2 < 0) // Edge conflict
				{
					conflict->computeConstraintsForEdgeConflict(parent.mdds[conflict->a1]->levels,
						parent.mdds[conflict->a2]->levels);
					if (conflict->type == conflict_type::CARDINAL)
					{
						parent.cardinalConf.push_back(conflict);
					}
					else if (conflict->type == conflict_type::SEMI)
					{
						parent.semiConf.push_back(conflict);
					}
					else
					{
						parent.nonConf.push_back(conflict);
					}
				}
					
				// get the smallest rectangle constraint with each weight for a1
				vector<int> mdd1_x_min(lookahead + 1, conflict->loc1 / num_col);
				vector<int> mdd1_x_max(lookahead + 1, conflict->loc1 / num_col);
				vector<int> mdd1_y_min(lookahead + 1, conflict->loc1 % num_col);
				vector<int> mdd1_y_max(lookahead + 1, conflict->loc1 % num_col);
				conflict->getMinimalRectangle(mdd1_x_min, mdd1_x_max, mdd1_y_min, mdd1_y_max, 
					lookahead, parent.mdds[conflict->a1]->levels[conflict->timestep], num_col);
				// get the smallest rectangle constraint with each weight for a2
				vector<int> mdd2_x_min(lookahead + 1, conflict->loc2 / num_col);
				vector<int> mdd2_x_max(lookahead + 1, conflict->loc2 / num_col);
				vector<int> mdd2_y_min(lookahead + 1, conflict->loc2 % num_col);
				vector<int> mdd2_y_max(lookahead + 1, conflict->loc2 % num_col);
				conflict->getMinimalRectangle(mdd2_x_min, mdd2_x_max, mdd2_y_min, mdd2_y_max,
					lookahead, parent.mdds[conflict->a2]->levels[conflict->timestep], num_col);

				int size1 = al.sizes[conflict->a1];
				int size2 = al.sizes[conflict->a2];
				int minCost = -1;
				for (int i = lookahead; i >= 0 && i >= minCost; i--)
				{
					for (int j = lookahead; j >= 0 && j >= minCost; j--)
					{
						if (min(i, j) <= minCost)
							break;
												
						std::pair<int, int> min1(mdd1_x_min[i], mdd1_y_min[i]);
						std::pair<int, int> max1(mdd1_x_max[i], mdd1_y_max[i]);
						std::pair<int, int> min2(mdd2_x_min[i], mdd2_y_min[i]);
						std::pair<int, int> max2(mdd2_x_max[i], mdd2_y_max[i]);
						// if the two rectangle constraints are mutually disjunctive
						if (conflict->disjunctive(min1, max1, min2, max2, size1, size2)) // cardinal
						{
							conflict->computeMaximalDisjunctiveConstraint2(min1, max1, size1, size2, num_col, map_size / num_col);
							conflict->weight1 = i + 1;
							conflict->weight2 = j + 1;
							conflict->type = conflict_type::CARDINAL;
							minCost = min(i, j);
						}
					}
				}
				if (conflict->type == conflict_type::CARDINAL)
				{
					parent.cardinalConf.push_back(conflict);
					continue;
				}
				if (lookahead > minCost)
				{
					std::pair<int, int> loc2(conflict->loc2 / num_col, conflict->loc2 % num_col);
					for (int i = lookahead; i >= 0 && i > minCost; i--)
					{
						std::pair<int, int> min1(mdd1_x_min[i], mdd1_y_min[i]);
						std::pair<int, int> max1(mdd1_x_max[i], mdd1_y_max[i]);
						if (conflict->disjunctive(min1, max1, loc2, loc2, size1, size2)) //semi (1st agent)
						{
							conflict->computeMaximalDisjunctiveConstraint2(min1, max1, size1, size2, num_col, map_size / num_col);
							conflict->weight1 = i + 1;
							conflict->weight2 = 0;
							conflict->type = conflict_type::SEMI;
							minCost = i;
							break;
						}
					}
				}
				if (lookahead > minCost)
				{
					std::pair<int, int> loc1(conflict->loc1 / num_col, conflict->loc1 % num_col);
					for (int i = lookahead; i >= 0 && i > minCost; i--)
					{
						std::pair<int, int> min2(mdd2_x_min[i], mdd2_y_min[i]);
						std::pair<int, int> max2(mdd2_x_max[i], mdd2_y_max[i]);
						if (conflict->disjunctive(loc1, loc1, min2, max2, size1, size2)) //semi (2nd agent)
						{
							conflict->swap();
							conflict->computeMaximalDisjunctiveConstraint2(min2, max2, size2, size1, num_col, map_size / num_col);
							conflict->weight1 = i + 1;
							conflict->weight2 = 0;
							conflict->type = conflict_type::SEMI;
							minCost = i;
							break;
						}
					}
				}
				if (conflict->type == conflict_type::SEMI)
				{
					parent.semiConf.push_back(conflict);
				}
				else //non
				{
					int min_x1 = min(max(conflict->loc2 / num_col - size1 + 1, mdd1_x_min[0]), conflict->loc1 / num_col);
					int max_x1 = max(min(conflict->loc2 / num_col + size2 - 1, mdd1_x_max[0]), conflict->loc1 / num_col);
					int min_y1 = min(max(conflict->loc2 % num_col - size1 + 1, mdd1_y_min[0]), conflict->loc1 % num_col);
					int max_y1 = max(min(conflict->loc2 % num_col + size2 - 1, mdd1_y_max[0]), conflict->loc1 % num_col);
					conflict->computeMaximalDisjunctiveConstraint2(make_pair(min_x1, min_y1), make_pair(max_x1, max_y1), 
						size1, size2, num_col, map_size / num_col);
					conflict->weight1 = 0;
					conflict->weight2 = 0;
					conflict->type = conflict_type::NON;
					parent.nonConf.push_back(conflict);
				}
			}
		}
		if (!parent.cardinalConf.empty()) // Choose maximal cost cardinal
		{
			Conflict* choose = parent.cardinalConf.front();
			for (list<Conflict*>::iterator it = parent.cardinalConf.begin(); it != parent.cardinalConf.end(); ++it)
			{
				if ((*it)->hasBetterWeight(choose))
					choose = (*it);
			}
			return choose;
		}
		else if (!parent.semiConf.empty())
		{
			Conflict* choose = parent.semiConf.front();
			for (list<Conflict*>::iterator it = parent.semiConf.begin(); it != parent.semiConf.end(); ++it)
			{
				if ((*it)->hasBetterWeight(choose))
					choose = (*it);
			}
			return choose;
		}
		else // Choose the earliest non
		{
			Conflict* choose = parent.nonConf.front();
			for (list<Conflict*>::iterator it = parent.nonConf.begin(); it != parent.nonConf.end(); ++it)
				if ((*it)->timestep < choose->timestep)
					choose = (*it);
			return choose;
		}
	}
	else // classify unknown conflicts until found a cardinal conflict
	{
		// Try to find a cardinal conflict in unknownConf
		while(!parent.unknownConf.empty())
		{
			int count = (int)parent.unknownConf.size();
			vector<int> needMDD(num_of_agents, 0);
			for(; count > 0; count--)
			{
				Conflict* conflict = parent.unknownConf.front();
				parent.unknownConf.pop_front();

				// if this is an edge conflict
				if (conflict->loc2 < 0) 
				{
					conflict->computeConstraintsForEdgeConflict(parent.mdds[conflict->a1]->levels,
						parent.mdds[conflict->a2]->levels);
					if (conflict->type == conflict_type::CARDINAL)
					{
						return conflict;
					}
					else if (conflict->type == conflict_type::SEMI)
					{
						parent.semiConf.push_back(conflict);
					}
					else
					{
						parent.nonConf.push_back(conflict);
					}
					continue;
				}

				// build MDDs if necessary
				if (conflict->type != conflict_type::GEQSEMI && parent.mdds[conflict->a1] == NULL)
				{
					needMDD[conflict->a1]++;
					parent.unknownConf.push_back(conflict);
					continue;
				}
				if (parent.mdds[conflict->a2] == NULL)
				{
					needMDD[conflict->a2]++;
					parent.unknownConf.push_back(conflict);
					continue;
				}

				// add constraints
				if (cons_strategy == constraint_strategy::ASYMMETRIC)
					conflict->ComputeAsymConstraints(num_col, map_size / num_col, al.sizes[conflict->a1], al.sizes[conflict->a2]);
				else if (cons_strategy == constraint_strategy::SYMMETRIC)
					conflict->ComputeSymConstraints(num_col, map_size / num_col, al.sizes[conflict->a1], al.sizes[conflict->a2]);
				else if (cons_strategy == constraint_strategy::BASIC)
				{
					conflict->cons1[0] = conflict->loc1;
					conflict->cons1[1] = conflict->loc1;
					conflict->cons2[0] = conflict->loc2;
					conflict->cons2[1] = conflict->loc2;
				}

				// classify this vertex conflict
				bool cardinal1 = true, cardinal2 = true;
				// if the conflict haapens after a1 reaches its goal,
				// we know for sure that it is cardinal for a1;
				// else, we need to check whether all MDD nodes are inside the rectangle for a1
				if(conflict->type != conflict_type::GEQSEMI)  
				{ 
					for (list<MDDNode*>::iterator it = parent.mdds[conflict->a1]->levels[conflict->timestep].begin();
						it != parent.mdds[conflict->a1]->levels[conflict->timestep].end(); ++it)
					{
						if (conflict->cons1[0] / num_col <= (*it)->location / num_col &&
							(*it)->location / num_col <= conflict->cons1[1] / num_col &&
							conflict->cons1[0] % num_col <= (*it)->location % num_col &&
							(*it)->location % num_col <= conflict->cons1[1] % num_col)
							continue;
						else
						{
							cardinal1 = false;
							break;
						}
					}
				}
				// check whether all MDD nodes are inside the rectangle for a2
				for (list<MDDNode*>::iterator it = parent.mdds[conflict->a2]->levels[conflict->timestep].begin();
					it != parent.mdds[conflict->a2]->levels[conflict->timestep].end(); ++it)
				{ 
					if (conflict->cons2[0] / num_col <= (*it)->location / num_col &&
						(*it)->location / num_col <= conflict->cons2[1] / num_col &&
						conflict->cons2[0] % num_col <= (*it)->location % num_col &&
						(*it)->location % num_col <= conflict->cons2[1] % num_col)
						continue;
					else
					{
						cardinal2 = false;
						break;
					}
				}
							
				if (cardinal1 && cardinal2)
				{
					conflict->type = conflict_type::CARDINAL;
					return conflict;
				}
				else if (cardinal1 || cardinal2)
				{
					conflict->type = conflict_type::SEMI;
					parent.semiConf.push_back(conflict);
				}
				else
				{
					conflict->type = conflict_type::NON;
					parent.nonConf.push_back(conflict);
				}
			} // No cardinal found

			// build MDD for the most needed agent
			int maxCount = 0, id = -1;
			for(int i = 0; i < num_of_agents; i++)
			{
				if (needMDD[i] > maxCount)
				{
					maxCount = needMDD[i];
					id = i;
				}
			}
			if (id >= 0)
			{
				buildMDD(parent, id, 0);
			}
		}

		// no cardinal conflicts found
		if (!parent.semiConf.empty()) // Choose the earliest semi
		{
			Conflict* choose = parent.semiConf.front();
			for (list<Conflict*>::iterator it = parent.semiConf.begin(); it != parent.semiConf.end(); ++it)
				if ((*it)->timestep < choose->timestep)
					choose = (*it);
			return choose;
		}
		else // Choose the earliest non
		{
			Conflict* choose = parent.nonConf.front();
			for (list<Conflict*>::iterator it = parent.nonConf.begin(); it != parent.nonConf.end(); ++it)
				if ((*it)->timestep < choose->timestep)
					choose = (*it);
			return choose;
		}
	}
	
}

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr, ICBSNode* root_node)
{
	paths = paths_found_initially;
	paths_costs = paths_costs_found_initially;
	vector<bool> updated(num_of_agents, false);  // initialized for false
	 /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
	 * because younger nodes take into account ancesstors' nodes constraints. */
	while (curr != root_node) 
	{
		if (updated[curr->agent_id] == false)
		{
			paths[curr->agent_id] = &(curr->path);
			paths_costs[curr->agent_id] = curr->path_cost;
			updated[curr->agent_id] = true;
		}
		curr = curr->parent;
	}
}


// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them.
// returns true only if such a path exists (otherwise false and path remain empty).
inline bool ICBSSearch::updateICBSNode(ICBSNode* leaf_node, ICBSNode* root_node) {
	// extract all constraints on leaf_node->agent_id
	list < tuple<int, int, int> > constraints;  // each constraint is <loc1, loc2, T>
	int agent_id = leaf_node->agent_id;
	ICBSNode* curr = leaf_node;
	int max_timestep = -1;
	while (curr != root_node) 
	{
		if (curr->agent_id == agent_id) 
		{
			constraints.push_front(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		curr = curr->parent;
	}

	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > >(max_timestep + 1, list< pair<int, int> >());
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) 
	{
		cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));
	}

	// build reservation table
	int max_plan_len = (int)getPathsMaxLength();
	bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
	updateReservationTable(res_table, max_plan_len, agent_id);

	// find a path w.r.t cons_vec (and prioretize by res_table).
	bool foundSol = search_engines[agent_id]->findPath(focal_w, cons_vec, res_table, max_plan_len);
	LL_num_expanded += search_engines[agent_id]->num_expanded;
	LL_num_generated += search_engines[agent_id]->num_generated;

	// update leaf's path to the one found and its low-level search's min f-val
	if (foundSol) 
	{
		leaf_node->path = vector<pathEntry>(*(search_engines[agent_id]->getPath()));
		leaf_node->path_cost = search_engines[agent_id]->path_cost;
	}

	// release memory allocated here and return
	delete (cons_vec);
	delete[] res_table;
	return foundSol;
}

// generate a child node
void ICBSSearch::generateChild(ICBSNode* n1, const ICBSNode* curr)
{
	if (updateICBSNode(n1, dummy_start) == true)
	{
		// Copy MDDs from parent
		for (int i = 0; i < curr->mdds.size(); i++)
		{
			n1->mdds[i] = NULL;
			continue;
			if (i == n1->agent_id)
			{
				n1->mdds[i] = NULL;
			}
			else if(curr->mdds[i] != NULL)
			{
				n1->mdds[i] = curr->mdds[i];
				n1->mdds[i]->numPointers++;
			}
			else
			{
				n1->mdds[i] = NULL;
			}
		}
		
		// new g_val equals old g_val plus the new path length found for the agent minus its old path length
		n1->g_val = curr->g_val - paths_costs[n1->agent_id] + n1->path_cost;

		// update n1's path for computing conflicts
		vector<pathEntry>* temp_old_path = paths[n1->agent_id];
		paths[n1->agent_id] = &(n1->path);
		findConflicts(*n1);
		n1->num_of_collisions = (int)(n1->unknownConf.size() + n1->cardinalConf.size() + n1->nonConf.size() + n1->semiConf.size());
		paths[n1->agent_id] = temp_old_path;  // restore the old path (for n2)
		 // update lower bounds and handles
		n1->f_val = max(n1->g_val, curr->f_val); 
		n1->open_handle = open_list.push(n1);
		HL_num_generated++;
		n1->time_generated = HL_num_generated;
		if (n1->f_val <= focal_list_threshold)
			n1->focal_handle = focal_list.push(n1);
		allNodes_table.push_back(n1);
	}
	else {
		delete (n1);
		n1 = NULL;
	}
	
}

// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ICBSSearch::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) 
{
	for (ICBSNode* n : open_list) 
	{
		if (n->f_val > old_lower_bound &&
			n->f_val <= new_lower_bound)
			n->focal_handle = focal_list.push(n);
	}
}

void ICBSSearch::printStrategy(constraint_strategy s) const
{
	switch (s)
	{
	case constraint_strategy::BASIC:
		std::cout << "   ICBS: ";
		break;
	case constraint_strategy::ASYMMETRIC:
		std::cout << "   ASYM: ";
		break;
	case constraint_strategy::SYMMETRIC:
		std::cout << "    SYM: ";
		break;
	case constraint_strategy::MAX:
		std::cout << "  MAX+" << lookahead << ": ";
		break;
	case constraint_strategy::MAXpH:
		std::cout << "MAXpH+" << lookahead << ": ";
		break;
	default:
		std::cout << "Strategy " << cons_strategy << " does not exists" << std::endl;
		exit(9);
	}
}

void ICBSSearch::printStatistics() const
{
	std::cout << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
		HL_num_expanded << " ; " << HL_num_generated << " ; " <<
		LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << endl;
}

// Runs the algorithm until the problem is solved or time is exhausted
bool ICBSSearch::runICBSSearch() 
{
	printStrategy(cons_strategy);

	// set timer
	std::clock_t start;
	start = std::clock();

	// start is already in the open_list
	while (!focal_list.empty() && !solution_found) 
	{
		runtime = (std::clock() - start);
		if (runtime > TIME_LIMIT) // timeout
		{  
			std::cout << "TIMEOUT  ; ";
			printStatistics();
			solution_found = false;
			break;
		}

		ICBSNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);
		
		// get the paths of current node
		updatePaths(curr, dummy_start);

	
		if(curr->conflict == NULL) // not choose conflict yet
		{
			curr->conflict = chooseConflict(*curr); // choose conflict
			if(cons_strategy == constraint_strategy::MAXpH) // need to compute heuristics
			{
				curr->h_val = computeHeuristics(*curr);
				curr->f_val = curr->g_val + curr->h_val;
				
				if (curr->f_val > focal_list_threshold) // put the node back to open_list
				{	
					curr->open_handle = open_list.push(curr);
					ICBSNode* open_head = open_list.top();
					if (open_head->f_val > min_f_val) // update  focal_list if necessary
					{
						min_f_val = open_head->f_val;
						double new_focal_list_threshold = min_f_val * focal_w;
						updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
						focal_list_threshold = new_focal_list_threshold;
					}
					continue;
				}
			}
		}
		
		// if we fail to find a conflict, then we find a solution
		if (curr->conflict == NULL) 
		{  
			runtime = (std::clock() - start);
			solution_found = true;
			solution_cost = curr->g_val;
			printStatistics();
			break;
		}
  
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		// generate the two successors that resolve  the chosen conflict
		ICBSNode* n1 = new ICBSNode();
		ICBSNode* n2 = new ICBSNode();
		n1->agent_id = curr->conflict->a1;
		n2->agent_id = curr->conflict->a2;
		n1->constraint = make_tuple(curr->conflict->cons1[0], curr->conflict->cons1[1], curr->conflict->timestep);
		n2->constraint = make_tuple(curr->conflict->cons2[0], curr->conflict->cons2[1], curr->conflict->timestep);
		n1->parent = curr;
		n2->parent = curr;
		n1->mdds.resize(num_of_agents, NULL);
		n2->mdds.resize(num_of_agents, NULL);

		generateChild(n1, curr);
		generateChild(n2, curr);

		//Clear expanded node's MDDs and conflicts lists in order to save some memory.
		for (int i = 0; i < curr->mdds.size(); i++)
		{
			if(curr->mdds[i] != NULL)
			{
				curr->mdds[i]->numPointers--;
				if(curr->mdds[i]->numPointers == 0)
					delete curr->mdds[i];
			}
		}
		curr->mdds.clear();
		delete curr->conflict;

		// if open_list is empty, then no solution exists.
		if (open_list.size() == 0) 
		{
			solution_found = false;
			solution_cost = -2;
			std::cout << "No solutions  ; ";
			printStatistics();
			break;
		}


		ICBSNode* open_head = open_list.top();
		if (open_head->f_val > min_f_val) // update  focal_list if necessary
		{
			min_f_val = open_head->f_val;
			double new_focal_list_threshold = min_f_val * focal_w;
			updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
			focal_list_threshold = new_focal_list_threshold;
		}
	}  // end of while loop

	return solution_found;
}

ICBSSearch::ICBSSearch(int TIME_LIMIT, const MapLoader& ml, const AgentsLoader& al, 
	double f_w, constraint_strategy c, int lookahead = 0) :
	lookahead(lookahead), TIME_LIMIT(TIME_LIMIT), cons_strategy(c), focal_w(f_w)
{
	HL_num_expanded = 0;
	HL_num_generated = 0;
	LL_num_expanded = 0;
	LL_num_generated = 0;

	this->num_col = ml.cols;
	this->al = al;
	num_of_agents = al.num_of_agents;
	map_size = ml.rows*ml.cols;
	solution_found = false;
	solution_cost = -1;
	paths_costs = vector <int>(num_of_agents);
	paths_costs_found_initially = vector <int>(num_of_agents);
	search_engines = vector < SingleAgentICBS* >(num_of_agents);

	// compute heuristics
	for (int i = 0; i < num_of_agents; i++) 
	{
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic ch(init_loc, goal_loc, al.sizes[i], ml.get_map(), ml.rows, ml.cols, ml.moves_offset);
		search_engines[i] = new SingleAgentICBS(init_loc, goal_loc, 
			ch.getHVals(), ch.getEstimatedGVals(),
			ml.get_map(), ml.rows*ml.cols,
			ml.moves_offset, al.sizes[i], ml.cols);
	}

	// initialize allNodes_table (hash table)
	empty_node = new ICBSNode();
	empty_node->time_generated = -2; empty_node->agent_id = -2;
	deleted_node = new ICBSNode();
	deleted_node->time_generated = -3; deleted_node->agent_id = -3;

	// initialize all initial paths
	paths_found_initially.resize(num_of_agents, NULL);
	for (int i = 0; i < num_of_agents; i++) 
	{
		paths = paths_found_initially;
		int max_plan_len = (int)getPathsMaxLength();
		bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
		updateReservationTable(res_table, max_plan_len, i);
		if (search_engines[i]->findPath(f_w, NULL, res_table, max_plan_len) == false)
		{
			std::cout << "NO SOLUTION EXISTS" << std::endl;
			exit(8);
		}
		paths_found_initially[i] = new vector<pathEntry>(*(search_engines[i]->getPath()));
		paths_costs_found_initially[i] = search_engines[i]->path_cost;
		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
		delete[] res_table;
	}

	paths = paths_found_initially;
	paths_costs = paths_costs_found_initially;

	// generate dummy start and update data structures
	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;
	dummy_start->g_val = 0;
	dummy_start->h_val = 0;
	for (int i = 0; i < num_of_agents; i++)
		dummy_start->g_val += paths_costs[i];
	dummy_start->f_val = dummy_start->g_val;
	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);
	HL_num_generated++;
	dummy_start->time_generated = HL_num_generated;
	allNodes_table.push_back(dummy_start);
	dummy_start->mdds.resize(num_of_agents, NULL);
	findConflicts(*dummy_start);
	min_f_val = dummy_start->f_val;
	focal_list_threshold = focal_w * min_f_val;
}

inline void ICBSSearch::releaseClosedListNodes() 
{
	for (list<ICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete *it;
}

ICBSSearch::~ICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	releaseClosedListNodes();
	delete (empty_node);
	delete (deleted_node);
}
