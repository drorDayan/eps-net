#include "ICBSSearch.h"
#include <map>
#include <ctime>

void ICBSSearch::collectConstraints(ICBSNode* curr, int agent_id, vector < list< pair<vertex_t, vertex_t> > >& cons_vec)
{
	// extract all constraints on leaf_node->agent_id
	list < tuple<vertex_t, vertex_t, int> > constraints; 
	int max_timestep = -1;
	while (curr != dummy_start) {
		if (curr->agent_id == agent_id) {
			for(auto constraint: curr->constraints)
			{	
				constraints.push_front(constraint);
				if (get<2>(constraint) > max_timestep) // calc constraints' max_timestep
					max_timestep = get<2>(constraint);
			}
		}
		curr = curr->parent;
	}
	
	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	cons_vec.resize(max_timestep + 1);
	for (list< tuple<vertex_t, vertex_t, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) 
	{
		cons_vec[get<2>(*it)].push_back(make_pair(get<0>(*it), get<1>(*it)));
	}
}

int ICBSSearch::getHLHeuristic(const ICBSNode& curr)
{
	if (curr.cardinalConf.size() < 2)
		return curr.cardinalConf.size();
	vector<vector<int>> w(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
		w[i].resize(num_of_agents, 0);
	for (list<Conflict*>::const_iterator it = curr.cardinalConf.begin(); it != curr.cardinalConf.end(); ++it)
	{
		if(cons_strategy == constraint_strategy::MAX)
		{
			if (min(w[(*it)->a1][(*it)->a2], w[(*it)->a2][(*it)->a1]) < min((*it)->cost1, (*it)->cost2) ||
				(min(w[(*it)->a1][(*it)->a2], w[(*it)->a2][(*it)->a1]) == min((*it)->cost1, (*it)->cost2) && max(w[(*it)->a1][(*it)->a2], w[(*it)->a2][(*it)->a1]) < max((*it)->cost1, (*it)->cost2)))
			{
				w[(*it)->a1][(*it)->a2] = (*it)->cost1;
				w[(*it)->a2][(*it)->a1] = (*it)->cost2;
			}
		}
	}

	vector<vector<bool>> x(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
		x[i].resize(num_of_agents, false);
	int rst2 = admissibleH(w, x, 0,0);
	return rst2;
}

//brute-force search to compute minimum edge-weighted vertex cover
int ICBSSearch::admissibleH(vector<vector<int>>& w, vector<vector<bool>>& x, int row, int col)
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
					int h1 = admissibleH(w, x, newRow, newCol);
					x[i][j] = false;
					x[j][i] = true;
					int h2 = admissibleH(w, x, newRow, newCol);
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

// Return true if there is a conflict
bool ICBSSearch::detectCollision(vertex_t v1, vertex_t v2)
{
	if(v1 == v2)
		return true;
	else if(G[v1].generalizedVertexConflicts.find(v2) == G[v1].generalizedVertexConflicts.end())
		return false;
	else
		return true;
}

bool ICBSSearch::detectCollision(vertex_t v1_from, vertex_t v2_from, vertex_t v1, vertex_t v2)
{
	if(v1 == v1_from && v2 == v2_from) //Both wait, so no edge conflict
		return false;
	if(v1 == v1_from) //a1 waits
	{
		for (auto e : G[v1].generalizedVertexEdgeConflicts)
		{
			if ((e.m_source == v2_from && e.m_target == v2) || (e.m_source == v2 && e.m_target == v2_from))
				return true;
		}
		return false;
	}
	else if(v2 == v2_from) //a2 waits
	{
		for (auto e : G[v2].generalizedVertexEdgeConflicts)
		{
			if ((e.m_source == v1_from && e.m_target == v1) || (e.m_source == v1 && e.m_target == v1_from))
				return true;
		}
		return false;
	}
	else if(v1 == v2_from && v1_from == v2) // Traverse the same edge
		return true;
	else //Both move through different edges
	{
		edge_t e1 = (boost::edge(v1_from, v1, G)).first;
		for(auto e: G[e1].generalizedEdgeConflicts)
		{
			if((e.m_source == v2_from && e.m_target == v2) || (e.m_source == v2 && e.m_target == v2_from))
				return true;
		}
		return false;
	}
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
void ICBSSearch::findConflicts(ICBSNode& curr, int a1, int a2)
{
	int min_path_length = paths[a1]->size() < paths[a2]->size() ?
		paths[a1]->size() : paths[a2]->size();
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		vertex_t v1 = paths[a1]->at(timestep).vertex;
		vertex_t v2 = paths[a2]->at(timestep).vertex;
		if (detectCollision(v1, v2))
		{
			curr.unknownConf.push_front(new Conflict(timestep, a1, a2, v1, v2));
		}
		else if (timestep > 0 && detectCollision(paths[a1]->at(timestep - 1).vertex, paths[a2]->at(timestep - 1).vertex, v1, v2))
		{
			curr.unknownConf.push_front(new Conflict(timestep, a1, a2, paths[a1]->at(timestep - 1).vertex, paths[a2]->at(timestep - 1).vertex, v1, v2)); // edge conflict
		}
	}
	if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		vertex_t v1 = paths[a1_]->back().vertex;
		for (int timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			vertex_t v2 = paths[a2_]->at(timestep).vertex;
			if (detectCollision(v1, v2))
			{
				curr.unknownConf.push_front(new Conflict(timestep, a1_, a2_, v1, v2, conflict_type::GEQSEMI));
			}
			else if (timestep < paths[a2_]->size()  && detectCollision(v1, paths[a2_]->at(timestep - 1).vertex, v1, v2))
			{
				curr.unknownConf.push_front(new Conflict(timestep, a1_, a2_, v1, paths[a2_]->at(timestep - 1).vertex, v1, v2, conflict_type::GEQSEMI)); // edge conflict
			}
		}
	}
}

// find conflicts in the node
void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (curr.parent != NULL) // not root node
	{
		// Copy from parent
		copyConflicts(curr.parent->cardinalConf, curr.cardinalConf, curr.agent_id);
		copyConflicts(curr.parent->semiConf, curr.semiConf, curr.agent_id);
		copyConflicts(curr.parent->nonConf, curr.nonConf, curr.agent_id);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, curr.agent_id);
		
		// detect new agent conflict
		int a1 = curr.agent_id, a2 = 0;
		for (; a2 < num_of_agents; a2++)
		{
			if (a1 != a2)
				findConflicts(curr, a1, a2);
		}
	}
	else
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

void ICBSSearch::buildMDD(ICBSNode& curr, int id, int lookahead)
{
	curr.mdds[id] = new MDD();
	vector < list< pair<vertex_t, vertex_t> > > cons_vec;
	collectConstraints(&curr, id, cons_vec);
	curr.mdds[id]->buildMDD(G, cons_vec, paths[id]->size(), lookahead, *search_engines[id]);
	curr.mdds[id]->numPointers = 1;
}

bool ICBSSearch::isCardinal(const list<tuple<vertex_t, vertex_t, int>>& cons, const list<MDDNode*>& level)
{
	list<MDDNode> copy;
	for (auto node: level)
		copy.push_back(MDDNode(*node));
	for (auto con : cons)
	{
		if(get<1>(con) == SIZE_MAX) // vertex constraint
		{
			for(list<MDDNode>::iterator it = copy.begin(); it != copy.end(); it++)
				if (it->vertex == get<0>(con))
				{
					copy.erase(it);
					break;
				}
		}
		else // edge constraint
		{
			for (list<MDDNode>::iterator it = copy.begin(); it != copy.end(); it++)
			{
				if (it->vertex == get<1>(con))
				{
					for (list<MDDNode*>::iterator it2 = it->parents.begin(); it2 != it->parents.end(); it2++)
					{
						if ((*it2)->vertex == get<0>(con))
						{
							it->parents.erase(it2);
							if (it->parents.empty())
								copy.erase(it);
							break;
						}
					}
					break;
				}
				else if (it->vertex == get<0>(con))
				{
					for (list<MDDNode*>::iterator it2 = it->parents.begin(); it2 != it->parents.end(); it2++)
					{
						if ((*it2)->vertex == get<1>(con))
						{
							it->parents.erase(it2);
							if (it->parents.empty())
								copy.erase(it);
							break;
						}
					}
					break;
				}
			}
		}
		if(copy.empty())
			return true;
	}
	return false;
}

Conflict* ICBSSearch::chooseConflict(ICBSNode &parent)
{
	Conflict* chosenCon = NULL;
	if (parent.cardinalConf.empty() && parent.semiConf.empty() && parent.nonConf.empty() && parent.unknownConf.empty())
		return NULL; // No conflict, i.e., it is the goal

	// Try to find a cardinal conflict in unknownConf
	while(!parent.unknownConf.empty())
	{
		int count = parent.unknownConf.size();
		Conflict* con = parent.unknownConf.front();
		parent.unknownConf.pop_front();
		if(con->type == conflict_type::GEQSEMI) //Conflict happend after one agent reached its goal. 
		{ // Add asymmetric constraints here.
			if (parent.mdds[con->a2] == NULL)
			{
				buildMDD(parent, con->a2, lookahead);
			}
			if (cons_strategy == constraint_strategy::ICBS)
				con->ComputeICBSConstraint();
			else
				con->ComputeAsymConstraints(G, false);
				
			//if cons1 area can cover its mdd area, then it is a cardinal conflict; otherwise, semicardinal
			if (cons_strategy == constraint_strategy::MAX)
			{
				con->cost1 = con->timestep - paths[con->a1]->size() + 2;
				con->cost2 = con->getReward(con->con2, parent.mdds[con->a2]->levels[con->timestep], lookahead);
				parent.nonConf.push_back(con);
				if(useh && con->cost1 > 0 && con->cost2 > 0)
					parent.cardinalConf.push_back(con); // To compute h for high level
			}
			else if(isCardinal(con->con2, parent.mdds[con->a2]->levels[con->timestep]))
			{
				con->type = conflict_type::CARDINAL;
				parent.cardinalConf.push_back(con);
				if (!useh)
					return con;
			}
			else
			{
				con->type = conflict_type::SEMI;
				parent.semiConf.push_front(con);
			}
		}
		else 
		{
			if (parent.mdds[con->a1] == NULL)
				buildMDD(parent, con->a1, lookahead);
			if (parent.mdds[con->a2] == NULL)
				buildMDD(parent, con->a2, lookahead);
			if (cons_strategy == constraint_strategy::ICBS)
				con->ComputeICBSConstraint();
			else if (cons_strategy == constraint_strategy::ASYM)
				con->ComputeAsymConstraints(G, true);
			else if (cons_strategy == constraint_strategy::MAX)
			{
				con->ComputeMaxRewardConstraints(G, parent.mdds[con->a1]->levels[con->timestep], parent.mdds[con->a2]->levels[con->timestep], lookahead);
				parent.nonConf.push_back(con);
				if(useh && con->cost1 > 0 && con->cost2 > 0)
					parent.cardinalConf.push_back(con); // Only for compute heuristic
				continue;
			}

			//if cons1 area can cover its mdd area, then it is a cardinal conflict; otherwise, semicardinal

			bool p1 = isCardinal(con->con1, parent.mdds[con->a1]->levels[con->timestep]);
			bool p2 = isCardinal(con->con2, parent.mdds[con->a2]->levels[con->timestep]);
			if (p1 && p2)
			{
				con->type = conflict_type::CARDINAL;
				parent.cardinalConf.push_back(con);
				if(!useh)
					return con;
			}
			else if(p1 || p2)
			{
				con->type = conflict_type::SEMI;
				parent.semiConf.push_front(con);
			}
			else
			{
				con->type = conflict_type::NON;
				parent.nonConf.push_back(con);
			}
		}
	}
	if(cons_strategy == constraint_strategy::MAX)
	{
		Conflict* choose = parent.nonConf.front();
		for (auto con : parent.nonConf)
			if (min(con->cost1, con->cost2) > min(choose->cost1, choose->cost2) ||
				(min(con->cost1, con->cost2) == min(choose->cost1, choose->cost2) && con->cost1 + con->cost2 > choose->cost1 + choose->cost2 ) ||
				(con->cost1 + con->cost2 == choose->cost1 + choose->cost2 && min(con->cost1, con->cost2) == min(choose->cost1, choose->cost2) &&
					con->con1.size() + con->con2.size() < choose->con1.size() + choose->con2.size()))
				choose = con;
		return choose;
	}
	else if (!parent.cardinalConf.empty()) // Choose the earliest cardinal
	{
		Conflict* choose = parent.cardinalConf.front();
		for (list<Conflict*>::iterator it = parent.cardinalConf.begin(); it != parent.cardinalConf.end(); ++it)
			if ((*it)->timestep < choose->timestep)
				choose = (*it);
		return choose;
	}
	else if (!parent.semiConf.empty()) // Choose the earliest semi
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

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr, ICBSNode* root_node) 
{
	paths = paths_found_initially;
	ll_min_f_vals = ll_min_f_vals_found_initially;
	paths_costs = paths_costs_found_initially;
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr != root_node) 
	{
		if (updated[curr->agent_id] == false) 
		{
			paths[curr->agent_id] = &(curr->path);
			ll_min_f_vals[curr->agent_id] = curr->ll_min_f_val;
			paths_costs[curr->agent_id] = curr->path_cost;
			updated[curr->agent_id] = true;
		}
		curr = curr->parent;
	}
}

// find all constraints on this agent (recursing to the root) and compute (and store) a path satisfying them.
// returns true only if such a path exists (otherwise false and path remain empty).
inline bool ICBSSearch::updateICBSNode(ICBSNode* leaf_node, ICBSNode* root_node)
{
	// extract all constraints on leaf_node->agent_id
	list < tuple<vertex_t, vertex_t, int> > constraints;  // each constraint is <L1,cols, rows, T>
	int agent_id = leaf_node->agent_id;
	ICBSNode* curr = leaf_node;
	int max_timestep = -1;
	while (curr != root_node)
	{
		if (curr->agent_id == agent_id) 
		{
			for(auto con: curr->constraints)
			{
				constraints.push_back(con);
				if (get<2>(con) > max_timestep) // calc constraints' max_timestep
					max_timestep = get<2>(con);
			}
		}
		curr = curr->parent;
	}


	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<vertex_t, vertex_t> > > cons_vec(max_timestep + 1);
	for (list< tuple<vertex_t, vertex_t, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) 
	{
		cons_vec[get<2>(*it)].push_back(make_pair(get<0>(*it), get<1>(*it)));
	}


	// find a path w.r.t cons_vec (and prioretize by res_table).
	bool foundSol = search_engines[agent_id]->findPath(cons_vec, paths, HL_w);
	LL_num_expanded += search_engines[agent_id]->num_expanded;
	LL_num_generated += search_engines[agent_id]->num_generated;


	// update leaf's path to the one found and its low-level search's min f-val
	if (foundSol)
	{
		leaf_node->path = vector<pathEntry>(*(search_engines[agent_id]->getPath()));
		leaf_node->ll_min_f_val = search_engines[agent_id]->min_f_val;
		leaf_node->path_cost = search_engines[agent_id]->path_cost;
	}

	// release memory allocated here and return
	return foundSol;
}



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
		n1->ll_min_f_val = n1->path_cost;

		// update n1's path for computing conflicts
		vector<pathEntry>* temp_old_path = paths[n1->agent_id];
		paths[n1->agent_id] = &(n1->path);\
		findConflicts(*n1);
		n1->num_of_collisions = n1->unknownConf.size() + n1->cardinalConf.size() + n1->nonConf.size() + n1->semiConf.size(); 
		paths[n1->agent_id] = temp_old_path;  // restore the old path (for n2)
		// update lower bounds and handles
		n1->sum_min_f_vals = max(n1->g_val, curr->sum_min_f_vals);
		n1->open_handle = open_list.push(n1);
		HL_num_generated++;
		n1->time_generated = HL_num_generated;
		if (n1->sum_min_f_vals <= focal_list_threshold)
			n1->focal_handle = focal_list.push(n1);
		allNodes_table.push_back(n1);
	}
	else {
		delete (n1);
		n1 = NULL;
	}
	
}

// To save memory
void ICBSSearch::compact(ICBSNode& node)
{
	vector<vector<Conflict*>> keep(num_of_agents);

	for(int i = 0; i < num_of_agents; i++)
		keep[i].resize(num_of_agents, NULL);
	for(list<Conflict*>::iterator it = node.cardinalConf.begin(); it != node.cardinalConf.end(); it++)
	{
		int a1 = min((*it)->a1, (*it)->a2);
		int a2 = max((*it)->a1, (*it)->a2);
		if(keep[a1][a2] == NULL || (*it)->timestep < keep[a1][a2]->timestep)
			keep[a1][a2] = (*it);
	}
	for(list<Conflict*>::iterator it = node.semiConf.begin(); it != node.semiConf.end(); it++)
	{
		int a1 = min((*it)->a1, (*it)->a2);
		int a2 = max((*it)->a1, (*it)->a2);
		if (keep[a1][a2] == NULL ||
			((*it)->type == conflict_type::SEMI && (*it)->timestep < keep[a1][a2]->timestep))
			keep[a1][a2] = (*it);
	}
	if (cons_strategy == constraint_strategy::ICBS || cons_strategy == constraint_strategy::ASYM)
	{
		for (list<Conflict*>::iterator it = node.nonConf.begin(); it != node.nonConf.end(); it++)
		{
			int a1 = min((*it)->a1, (*it)->a2);
			int a2 = max((*it)->a1, (*it)->a2);
			if (keep[a1][a2] == NULL ||
				((*it)->type == conflict_type::NON && (*it)->timestep < keep[a1][a2]->timestep))
				keep[a1][a2] = (*it);
		}
	}
	else if(cons_strategy == constraint_strategy::MAX)
	{
		for (list<Conflict*>::iterator it = node.nonConf.begin(); it != node.nonConf.end(); it++)
		{
			int a1 = min((*it)->a1, (*it)->a2);
			int a2 = max((*it)->a1, (*it)->a2);
			if (keep[a1][a2] == NULL ||
				min((*it)->cost1, (*it)->cost2) > min(keep[a1][a2]->cost1, keep[a1][a2]->cost2)  ||
				(min((*it)->cost1, (*it)->cost2) == min(keep[a1][a2]->cost1, keep[a1][a2]->cost2) && max((*it)->cost1, (*it)->cost2) > max(keep[a1][a2]->cost1, keep[a1][a2]->cost2)) ||
				(min((*it)->cost1, (*it)->cost2) == min(keep[a1][a2]->cost1, keep[a1][a2]->cost2) && max((*it)->cost1, (*it)->cost2) == max(keep[a1][a2]->cost1, keep[a1][a2]->cost2) && 
				(*it)->con1.size() + (*it)->con2.size() < keep[a1][a2]->con1.size() + keep[a1][a2]->con2.size()))
				keep[a1][a2] = (*it);
		}
	}
	//Delete useless conflicts to save memory
	for (list<Conflict*>::iterator it = node.cardinalConf.begin(); it != node.cardinalConf.end();)
	{
		int a1 = min((*it)->a1, (*it)->a2);
		int a2 = max((*it)->a1, (*it)->a2);
		if(keep[a1][a2] != (*it) && node.conflict != (*it))
		{
			delete (*it);
			it = node.cardinalConf.erase(it);
		}
		else
			it++;
	}
	for (list<Conflict*>::iterator it = node.semiConf.begin(); it != node.semiConf.end();)
	{
		int a1 = min((*it)->a1, (*it)->a2);
		int a2 = max((*it)->a1, (*it)->a2);
		if (keep[a1][a2] != (*it) && node.conflict != (*it))
		{
			delete (*it);
			it = node.semiConf.erase(it);
		}
		else
			it++;
	}
	for (list<Conflict*>::iterator it = node.nonConf.begin(); it != node.nonConf.end();)
	{
		int a1 = min((*it)->a1, (*it)->a2);
		int a2 = max((*it)->a1, (*it)->a2);
		if (keep[a1][a2] != (*it) && node.conflict != (*it))
		{
			delete (*it);
			it = node.nonConf.erase(it);
		}
		else
			it++;
	}
}

// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ICBSSearch::updateFocalList(double old_lower_bound, double new_lower_bound)
{
	for (ICBSNode* n : open_list) 
	{
		if (n->sum_min_f_vals > old_lower_bound &&
			n->sum_min_f_vals <= new_lower_bound)
			n->focal_handle = focal_list.push(n);
	}
}

//Compute Heuristics
void ICBSSearch::computeHeuristics(vector<double>& heuristics, vertex_t goal, const searchGraph_t& G)
{
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode> nodes;
	nodes.set_empty_key(NULL);
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode>::iterator it; // will be used for find()

	LLNode* root = new LLNode(goal, 0, 0, NULL, 0);
	root->open_handle = heap.push(root);  // add root to heap
	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	while (!heap.empty()) {
		LLNode* curr = heap.top();
		heap.pop();

		auto neighbours = boost::adjacent_vertices(curr->vertex, G);
		for (auto newV : make_iterator_range(neighbours))
		{
			//DROR: here costs are updated, we might need it to be dist(G[newV].pos, G[curr->vertex].pos)
			double next_g_val = curr->g_val + (G[newV].pos - G[curr->vertex].pos).norm();// 1;
			LLNode* next = new LLNode(newV, next_g_val, 0, NULL, 0);
			it = nodes.find(next);
			if (it == nodes.end()) {  // add the newly generated node to heap and hash table
				next->open_handle = heap.push(next);
				nodes[next] = next->open_handle;
			}
			else {  // update existing node's g_val if needed (only in the heap)
				delete(next);  // not needed anymore -- we already generated it before
				LLNode* existing_next = (*it).first;
				open_handle = (*it).second;
				if (existing_next->g_val > next_g_val) {
					existing_next->g_val = next_g_val;
					heap.update(open_handle);
				}
			}
		}
	}
	// iterate over all nodes and populate the num_of_collisionss
	heuristics.resize(boost::num_vertices(G), DBL_MAX);
	for (it = nodes.begin(); it != nodes.end(); it++) {
		LLNode* s = (*it).first;
		heuristics[s->vertex] = s->g_val;
		delete (s);
	}
	nodes.clear();
	heap.clear();
}

bool ICBSSearch::runICBSSearch() 
{
	switch (cons_strategy)
	{
	case constraint_strategy::ASYM:
		cout << "ASYM: ";
		break;
	case constraint_strategy::ICBS:
		cout << "ICBS: ";
		break;
	case constraint_strategy::MAX:
		cout << "MAX";
		if(useh)
			cout << "pH";
		cout << "+" << lookahead << ":";
		break;
	default:
		return false;
	}
	// set timer
	std::clock_t start;
	start = std::clock();

	// start is already in the open_list
	while (!focal_list.empty() && !solution_found) 
	{
		runtime = (std::clock() - start);
		if (runtime > TIME_LIMIT) 
		{ 
			cout << "TIMEOUT  ; " << solution_cost << " ; " << min_sum_f_vals - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << endl;
			solution_found = false;
			break;
		}

		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		updatePaths(curr, dummy_start);

		if(curr->conflict == NULL)
		{
			curr->conflict = chooseConflict(*curr);
			if(useh)
			{
				curr->h_val = getHLHeuristic(*curr);
				curr->sum_min_f_vals = curr->g_val + curr->h_val;
				if(cons_strategy == constraint_strategy::MAX)
					curr->cardinalConf.clear();
				compact(*curr);
				if (curr->sum_min_f_vals > focal_list_threshold)
				{	
					curr->open_handle = open_list.push(curr);
					ICBSNode* open_head = open_list.top();
					if (open_head->sum_min_f_vals > min_sum_f_vals) 
					{
						min_sum_f_vals = open_head->sum_min_f_vals;
						double new_focal_list_threshold = min_sum_f_vals * HL_w;
						updateFocalList(focal_list_threshold, new_focal_list_threshold);
						focal_list_threshold = new_focal_list_threshold;
					}
					continue;
				}
			}
			else
				compact(*curr);
		}
		if (curr->conflict == NULL)
		{  // found a solution (and finish the while look)
			runtime = (std::clock() - start); 
			solution_found = true;
			//DROR: This shows that the solution_cost is the g_val
			solution_cost = curr->g_val;
			cout << solution_cost << " ; " << solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; ";
			cout << endl;
			
		}
		else
		{  
			HL_num_expanded++;
			curr->time_expanded = HL_num_expanded;
			// generate the two successors that resolve one of the conflicts
			ICBSNode* n1 = new ICBSNode();
			ICBSNode* n2 = new ICBSNode();
			n1->agent_id = curr->conflict->a1;
			n2->agent_id = curr->conflict->a2;
			n1->constraints = curr->conflict->con1;
			n2->constraints = curr->conflict->con2;
			n1->parent = curr;
			n2->parent = curr;
			n1->mdds.resize(num_of_agents, NULL);
			n2->mdds.resize(num_of_agents, NULL);

			generateChild(n1, curr);
			generateChild(n2, curr);


			//Clear expanded node's MDDs and conflicts lists. in order to save some memory
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
			for (list<Conflict*>::iterator it = curr->cardinalConf.begin(); it != curr->cardinalConf.end(); ++it)
				delete (*it);
			for(list<Conflict*>::iterator it = curr->semiConf.begin(); it != curr->semiConf.end(); ++it)
				delete (*it);
			for (list<Conflict*>::iterator it = curr->nonConf.begin(); it != curr->nonConf.end(); ++it)
				delete (*it);
			for (list<Conflict*>::iterator it = curr->unknownConf.begin(); it != curr->unknownConf.end(); ++it)
				delete (*it);
			curr->cardinalConf.clear();
			curr->semiConf.clear();
			curr->nonConf.clear();
			curr->unknownConf.clear();

			if (open_list.size() == 0) {
				solution_found = false;
				break;
			}
			ICBSNode* open_head = open_list.top();
			if (open_head->sum_min_f_vals > min_sum_f_vals) 
			{
				min_sum_f_vals = open_head->sum_min_f_vals;
				double new_focal_list_threshold = HL_w * min_sum_f_vals;
				updateFocalList(focal_list_threshold, new_focal_list_threshold);
				focal_list_threshold = new_focal_list_threshold;
			}
		}  // end generating successors
	}  // end of while loop


	if (focal_list.empty() && solution_cost < 0)
	{
		solution_cost = -2;
		cout << "No solutions  ; " << solution_cost << " ; " << min_sum_f_vals - dummy_start->g_val << " ; " <<
			HL_num_expanded << " ; " << HL_num_generated << " ; " <<
			LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << 
			"|Open|=" << open_list.size() << endl;
		solution_found = false;
	}
	return solution_found;
}

ICBSSearch::ICBSSearch(int TIME_LIMIT, const searchGraph_t& G, const std::vector<vertex_t>& starts, const std::vector<vertex_t>& goals, int num_of_agents, constraint_strategy c, double HL_w, int lookahead, bool useh) :
	G(G), HL_w(HL_w), num_of_agents(num_of_agents), lookahead(lookahead), useh(useh), TIME_LIMIT(TIME_LIMIT)
{
	cons_strategy = c;
	HL_num_expanded = 0;
	HL_num_generated = 0;
	LL_num_expanded = 0;
	LL_num_generated = 0;


	solution_found = false;
	solution_cost = -1;
	ll_min_f_vals = vector <double>(num_of_agents);
	paths_costs = vector <double>(num_of_agents);
	ll_min_f_vals_found_initially = vector <double>(num_of_agents);
	paths_costs_found_initially = vector <double>(num_of_agents);
	search_engines = vector < SingleAgentECBS* >(num_of_agents);
	int delta = starts.size() / num_of_agents;
	for (int i = 0; i < num_of_agents; i++) 
	{
		vector<double> heuristics;
		computeHeuristics(heuristics,  goals[i * delta], G);
		search_engines[i] = new SingleAgentECBS(starts[i * delta], goals[i * delta], heuristics, G, i);
	}


	// initialize all initial paths to NULL
	paths_found_initially.resize(num_of_agents, NULL);


	// initialize paths_found_initially
	for (int i = 0; i < num_of_agents; i++) 
	{
		paths = paths_found_initially;
		vector < list< pair<vertex_t, vertex_t> > > constraints;
		if (search_engines[i]->findPath(constraints, paths, HL_w) == false)
			cout << "NO SOLUTION EXISTS";
		paths_found_initially[i] = new vector<pathEntry>(*(search_engines[i]->getPath()));
		ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
		paths_costs_found_initially[i] = search_engines[i]->path_cost;
		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
	}

	paths = paths_found_initially;
	ll_min_f_vals = ll_min_f_vals_found_initially;
	paths_costs = paths_costs_found_initially;

	// generate dummy start and update data structures
	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;
	dummy_start->g_val = 0;
	for (int i = 0; i < num_of_agents; i++)
		dummy_start->g_val += paths_costs[i];
	dummy_start->ll_min_f_val = 0;
	dummy_start->sum_min_f_vals = dummy_start->g_val;
	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);
	HL_num_generated++;
	dummy_start->time_generated = HL_num_generated;
	allNodes_table.push_back(dummy_start);
	dummy_start->mdds.resize(num_of_agents, NULL);
	findConflicts(*dummy_start);

	min_sum_f_vals = dummy_start->sum_min_f_vals;
	focal_list_threshold = HL_w * dummy_start->sum_min_f_vals;
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
}
