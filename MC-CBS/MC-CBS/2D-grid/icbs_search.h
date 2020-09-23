#pragma once

#include "icbs_node.h"
#include "single_agent_icbs.h"
#include "agents_loader.h"

enum constraint_strategy { BASIC, SYMMETRIC, ASYMMETRIC, MAX, MAXpH, STRATEGY_COUNT };

class ICBSSearch
{
public:
	bool solution_found;

	// solution quality
	int solution_cost;
	int min_f_val;
	double focal_list_threshold;
	ICBSNode* dummy_start;

	//algorithm efficiency
	double runtime = 0;
	int HL_num_expanded = 0;
	int HL_num_generated = 0;
	int LL_num_expanded = 0;
	int LL_num_generated = 0;

	ICBSSearch(int TIME_LIMIT, const MapLoader& ml, const AgentsLoader& al, double f_w, constraint_strategy c, int lookahead);
	~ICBSSearch();

	// Runs the algorithm until the problem is solved or time is exhausted
	bool runICBSSearch();

private:
	// input parameters
	const int TIME_LIMIT;
	const constraint_strategy cons_strategy;
	const int lookahead;
	const double focal_w = 1.0;
	int map_size;
	int num_of_agents;
	int num_col;
	AgentsLoader al;


	// intermidiate results
	std::vector < std::vector<pathEntry>* > paths;  // agents paths
	std::vector < std::vector<pathEntry>* > paths_found_initially;  // contain initial paths found
	vector <int> paths_costs;
	std::vector <int> paths_costs_found_initially;

	// helper
	std::vector < SingleAgentICBS* > search_engines;  // used to find (single) agents' paths
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;
	ICBSNode* empty_node; // used for hash_map
	ICBSNode* deleted_node; // used for hash_map

	

	//update information
	inline bool updateICBSNode(ICBSNode* leaf_node, ICBSNode* root_node);
	inline void updatePaths(ICBSNode* curr, ICBSNode* root_node);
	void updateReservationTable(bool* res_table, size_t max_plan_len, int exclude_agentconst);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);
	inline void releaseClosedListNodes();
	
	// detect conflicts
	inline bool overlaped(int loc1, int loc2, int size1, int size2) const;
	inline bool switchedLocations(int agent1_id, int agent2_id, size_t timestep) const;
	void findConflicts(ICBSNode& curr);
	void findConflicts(ICBSNode& curr, int a1, int a2) const;
	Conflict* chooseConflict(ICBSNode &parent);

	int computeHeuristics(const ICBSNode& curr);
	
	void generateChild(ICBSNode* n1, const ICBSNode* curr);
	
	void buildMDD(ICBSNode& curr, int id, int lookahead);
	
	// print
	void printStrategy(constraint_strategy s) const;
	void printStatistics() const;

	// helper tools
	void copyConflicts(const list<Conflict*>& conflicts, list<Conflict*>& copy, int excluded_agent) const;
	int minimumWVC(std::vector<std::vector<int>>& w, std::vector<std::vector<bool>>& x, int row, int col) const;
	size_t getPathsMaxLength() const;
	inline int getAgentLocation(int agent_id, size_t timestep) const;
	std::vector < std::list< std::pair<int, int> > >* collectConstraints(ICBSNode* curr, int agent_id);
};

