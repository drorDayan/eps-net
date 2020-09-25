#include "single_agent_ecbs.h"
#include <vector>
#include <list>

#include <boost/heap/fibonacci_heap.hpp>
#include <google/dense_hash_map>



using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


SingleAgentECBS::SingleAgentECBS(vertex_t startV, vertex_t goalV, const std::vector<double>& my_heuristic, const searchGraph_t& G, int agent_id) :
    my_heuristic(my_heuristic), G(G), startV(startV), goalV(goalV), agent_id(agent_id)
{
  this->num_expanded = 0;
  this->num_generated = 0;
  this->path_cost = 0;
  this->lower_bound = 0;
  this->min_f_val = 0;
  // initialize allNodes_table (hash table)
  allNodes_table.set_empty_key(NULL);
}


void SingleAgentECBS::updatePath(LLNode* goal)
{
  path.resize(goal->timestep + 1);
  LLNode* curr = goal;

  for(int t = goal->timestep; t >=0; t--)
  {
	path[t].vertex = curr->vertex;
    curr = curr->parent;
  }
  path_cost = goal->g_val;
}

inline void SingleAgentECBS::releaseClosedListNodes(hashtable_t* allNodes_table) {
  hashtable_t::iterator it;
  for (it=allNodes_table->begin(); it != allNodes_table->end(); it++) {
    delete ( (*it).second );  
  }
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
int SingleAgentECBS::extractLastGoalTimestep(const std::vector< std::list< std::pair<vertex_t, vertex_t> > >& cons) 
{
  if (!cons.empty()) {
    for ( int t = static_cast<int>(cons.size())-1; t > 0; t-- ) 
	{
      for (std::list< std::pair<vertex_t, vertex_t> >::const_iterator it = cons[t].begin(); it != cons[t].end(); ++it)
	  {
			if(it->first == goalV)
			  return (t);
        }
      }
    }
  return -1;
}


// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
//        cons[timestep] is a list of <loc1,loc2> of (vertex/edge) constraints for that timestep.
inline bool SingleAgentECBS::isConstrained(vertex_t currV, vertex_t nextV, int next_timestep, const std::vector< std::list< std::pair<vertex_t, vertex_t> > >& cons)  const
{
  if(cons.empty())
	return false;
  if (next_timestep < cons.size()) 
  {
		for ( std::list< std::pair<vertex_t, vertex_t>>::const_iterator it = cons[next_timestep].begin(); it != cons[next_timestep].end(); ++it )
		{
				if ( it->first == nextV && it->second == SIZE_MAX) // check vertex constraints (being in next_id at next_timestep is disallowed)
					return true;
				else if ((it->first == currV && it->second == nextV) || (it->first == nextV && it->second == currV)) // check edge constraints
					return true;
		}
	}
  return false;
}



int SingleAgentECBS::numOfConflictsForStep(vertex_t currV, vertex_t nextV, int next_timestep, const std::vector<std::vector<pathEntry>*>& paths) 
{
	int retVal = 0;
	for (int ag = 0; ag < paths.size(); ag++)
	{
		if(ag == agent_id || paths[ag] == NULL)
			continue;
		if(paths[ag]->size() <= next_timestep)
		{
			vertex_t v = paths[ag]->back().vertex;
			if (v== nextV || G[nextV].generalizedVertexConflicts.find(v) != G[nextV].generalizedVertexConflicts.end()) // vertex conflict at goal
				retVal++;
			else if (currV != nextV) //vertex edge conflict at goal
			{
				edge_t e = (boost::edge(currV, nextV, G)).first;
				for (auto e : G[v].generalizedVertexEdgeConflicts)
				{
					if ((e.m_source == currV && e.m_target == nextV) || (e.m_source == nextV && e.m_target == currV))
					{
						retVal++;
						break;
					}
				}
			}
		}
		else
		{
			vertex_t v = paths[ag]->at(next_timestep).vertex;
			if(v == nextV || G[nextV].generalizedVertexConflicts.find(v) != G[nextV].generalizedVertexConflicts.end()) // vertex conflict
				retVal++;
			else if (next_timestep > 0) //Edge conflict
			{
				vertex_t v_from = paths[ag]->at(next_timestep - 1).vertex;
				if(v_from == v && currV == nextV) // Both wait
					continue;
				else if (v_from == v) // One waits
				{
					edge_t e = (boost::edge(currV, nextV, G)).first;
					for (auto e : G[v].generalizedVertexEdgeConflicts)
					{
						if ((e.m_source == currV && e.m_target == nextV) || (e.m_source == nextV && e.m_target == currV))
						{
							retVal++;
							break;
						}
					}
				}
				else if (currV == nextV) //The other waits
				{
					edge_t e = (boost::edge(v_from, v, G)).first;
					for (auto e : G[v].generalizedVertexEdgeConflicts)
					{
						if ((e.m_source == v_from && e.m_target == v) || (e.m_source == v && e.m_target == v_from))
						{
							retVal++;
							break;
						}
					}
				}
				else if(v_from == nextV && v == currV) // Traverse the same edge
					retVal++;
				else // Both move through different edges
				{
					edge_t e1 = (boost::edge(v_from, v, G)).first;
					for (auto e : G[e1].generalizedEdgeConflicts)
					{
						if ((e.m_source == currV && e.m_target == nextV) || (e.m_source == nextV && e.m_target == currV))
						{
							retVal++;
							break;
						}
					}
				}
			}
		}
	}

  return retVal;
}


void SingleAgentECBS::updateFocalList(double old_lower_bound, double new_lower_bound)
{
  for (LLNode* n : open_list)
  {
    if ( n->getFVal() > old_lower_bound &&
         n->getFVal() <= new_lower_bound )
	{
		n->focal_handle = focal_list.push(n);
    }
  }
}


bool SingleAgentECBS::findPath(const std::vector < std::list< std::pair<vertex_t, vertex_t> > >& constraints, const std::vector<std::vector<pathEntry>*>& paths, double w) 
{
	// clear data structures if they had been used before
	// (note -- nodes are deleted before findPath returns)
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	num_expanded = 0;
	num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()

	 // generate start and add it to the OPEN list
	LLNode* start = new LLNode(startV, 0, my_heuristic[startV], NULL, 0, 0, false);
	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table[start] = start;
	min_f_val = start->getFVal();
	lower_bound = w * min_f_val;

	int lastGoalConsTime = extractLastGoalTimestep(constraints);

	while (!focal_list.empty())
	{
		LLNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);

		curr->in_openlist = false;
		num_expanded++;

		// check if the popped node is a goal
		if (curr->vertex == goalV && curr->timestep > lastGoalConsTime)
		{
			updatePath(curr);
			releaseClosedListNodes(&allNodes_table);
			return true;
		}

		//Move action
		auto neighbours = boost::adjacent_vertices(curr->vertex, G);
		for (auto nextV : make_iterator_range(neighbours))// Try every possible move. We only add backward edges in this step.
		{
			int next_timestep = curr->timestep + 1;
			if (!isConstrained(curr->vertex, nextV, next_timestep, constraints))
			{
				// compute cost to next_id via curr node
				//DROR this is where the length dependend cost should be used
				double cost = (G[nextV].pos - G[curr->vertex].pos).norm();// 1;
				double next_g_val = curr->g_val + cost;
				double next_h_val = my_heuristic[nextV];
				int next_internal_conflicts = 0;
				
				next_internal_conflicts = curr->num_internal_conf + numOfConflictsForStep(curr->vertex, nextV, next_timestep, paths);
				// generate (maybe temporary) node
				LLNode* next = new LLNode(nextV, next_g_val, next_h_val,
					curr, next_timestep, next_internal_conflicts, false);
				it = allNodes_table.find(next);

				if (it == allNodes_table.end())
				{
					next->open_handle = open_list.push(next);
					next->in_openlist = true;
					num_generated++;
					if (next->getFVal() <= lower_bound)
						next->focal_handle = focal_list.push(next);
					allNodes_table[next] = next;
				}
				else {  // update existing node's if needed (only in the open_list)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it).second;
					if (existing_next->in_openlist == true) 
					{  // if its in the open list
						if (existing_next->getFVal() > next_g_val + next_h_val ||
							(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) 
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
							bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
							bool update_open = false;
							if ((next_g_val + next_h_val) <= lower_bound)
							{  // if the new f-val qualify to be in FOCAL
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
					}
					else 
					{  // if its in the closed list (reopen)
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

		   //Wait action
		int next_timestep = curr->timestep + 1;
		if (!isConstrained(curr->vertex, curr->vertex, next_timestep, constraints))
		{
			// compute cost to next_id via curr node
			//DROR: This is the cost for staying in place, this should be 0, but it doesn't realy work...
			double cost = 0.01;
			double next_g_val = curr->g_val +cost;
			double next_h_val = my_heuristic[curr->vertex];
			int next_internal_conflicts = 0;
			next_internal_conflicts = curr->num_internal_conf + numOfConflictsForStep(curr->vertex, curr->vertex, next_timestep, paths);
			// generate (maybe temporary) node
			LLNode* next = new LLNode(curr->vertex, next_g_val, next_h_val,
				curr, next_timestep, next_internal_conflicts, false);
			it = allNodes_table.find(next);

			if (it == allNodes_table.end()) 
			{
				next->open_handle = open_list.push(next);
				next->in_openlist = true;
				num_generated++;
				if (next->getFVal() <= lower_bound)
					next->focal_handle = focal_list.push(next);
				allNodes_table[next] = next;
			}
			else 
			{  // update existing node's if needed (only in the open_list)
				delete(next);  // not needed anymore -- we already generated it before
				LLNode* existing_next = (*it).second;
				if (existing_next->in_openlist == true) 
				{  // if its in the open list
					if (existing_next->getFVal() > next_g_val + next_h_val ||
						(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts))
					{
						// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
						bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
						bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
						bool update_open = false;
						if ((next_g_val + next_h_val) <= lower_bound) 
						{  // if the new f-val qualify to be in FOCAL
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
				}
				else
				{  // if its in the closed list (reopen)
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
		}  // end if case for grid not blocked

		 // update FOCAL if min f-val increased
		if (open_list.size() == 0)  // in case OPEN is empty, no path found...
			return false;
		LLNode* open_head = open_list.top();
		if (open_head->getFVal() > min_f_val)
		{
			double new_min_f_val = open_head->getFVal();
			double new_lower_bound = w * new_min_f_val;

			updateFocalList(lower_bound, new_lower_bound);
			min_f_val = new_min_f_val;
			lower_bound = new_lower_bound;
		}
	}  // end while loop
	   // no path found
	path.clear();
	releaseClosedListNodes(&allNodes_table);
	return false;
}


SingleAgentECBS::~SingleAgentECBS() {}
