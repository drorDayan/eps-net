#pragma once
#include "node.h"
#include <list>
#include <google/dense_hash_map>


class SingleAgentICBS
{
public:
	// define handles to the heaps and hashmap (allow up to quickly update a node in the heap)
	typedef boost::heap::fibonacci_heap< Node*, boost::heap::compare<Node::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< Node*, boost::heap::compare<Node::secondary_compare_node> > heap_focal_t;
	typedef google::dense_hash_map<Node*, Node*, Node::NodeHasher, Node::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;

	// used in hash table and would be deleted from the d'tor
	Node* empty_node;
	Node* deleted_node;



	std::vector<pathEntry> path;  // a path that takes the agent from initial to goal location satisying all constraints

	int path_cost;
	int start_location;
	int goal_location;

	const int* my_heuristic;  // this is the precomputed heuristic for this agent
	const int* my_gvals_lb;  // this is the precomputed lower bound of g_vals
	const bool* my_map;
	int map_size;
	const int* moves_offset;
	const int* actions_offset;
	int num_expanded;
	int num_generated;
	double lower_bound;  // FOCAL's lower bound ( = e_weight * min_f_val)
	int min_f_val;  // min f-val seen so far

	int agent_size;
	int num_col;

	


	const std::vector<pathEntry>* getPath() { return &path; }  // return a pointer to the path found;


	

	// Checks if a vaild path found (wrt my_map and constraints)
	// Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
	inline bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons) const;

	bool validMove(int curr, int next) const; // whetehr curr->next is a valid move

	// Returns true if a collision free path found (with cost up to f_weight * f-min) while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	bool findPath(double f_weight, const std::vector < std::list< std::pair<int, int> > >* constraints, bool* res_table, int max_plan_len);



public:
	SingleAgentICBS(int start_location, int goal_location, 
		const int* my_heuristic, const int* my_gvals_lb, const bool* my_map, int map_size, const int* moves_offset,
		int size, int num_col);
	~SingleAgentICBS();
private:
	// Updates the path datamember (vector<int>).
	// After update it will contain the sequence of locations found from the goal to the start.
	void updatePath(Node* goal);

	// Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
	// Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
	int numOfConflictsForStep(int curr_id, int next_id, int next_timestep, bool* res_table, int max_plan_len);

	// Iterate over OPEN and adds to FOCAL all nodes with: 1) f-val > old_min_f_val ; and 2) f-val * f_weight < new_lower_bound.
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);

	// returns the minimal plan length for the agent (that is, extract the latest timestep which
	// has a constraint invloving this agent's goal location).
	int extractLastGoalTimestep(int goal_location, const std::vector< std::list< std::pair<int, int> > >* cons) const;

	inline void releaseClosedListNodes(hashtable_t* allNodes_table);
	
};

