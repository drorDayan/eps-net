#pragma once

#include "ICBSNode.h"
#include "single_agent_ecbs.h"
#include "searchgraph.h"

class ICBSSearch
{
public:
	bool useh;
	const int TIME_LIMIT;
	constraint_strategy cons_strategy;
	double runtime = 0;
	const searchGraph_t G;
	double focal_list_threshold;
	double HL_w; // suboptimal w for high level
	double min_sum_f_vals;
	vector < vector<pathEntry>* > paths;  // agents paths
	vector < vector<pathEntry>* > paths_found_initially;  // contain initial paths found
	int num_of_agents;

	bool solution_found;
	double solution_cost;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	vector < SingleAgentECBS* > search_engines;  // used to find (single) agents' paths
	vector <double> ll_min_f_vals_found_initially;  // contains initial ll_min_f_vals found
	vector <double> ll_min_f_vals;  // each entry [i] represent the lower bound found for agent[i]
	vector <double> paths_costs_found_initially;
	vector <double> paths_costs;

	int lookahead;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	typedef google::dense_hash_map<ICBSNode*, ICBSNode*, ICBSNode::ICBSNodeHasher, ICBSNode::ecbs_eqnode> hashtable_t;

	heap_open_t open_list;
	heap_focal_t focal_list;

	list<ICBSNode*> allNodes_table;

	ICBSNode* dummy_start;

	int admissibleH(vector<vector<int>>& w, vector<vector<bool>>& x, int row, int col);
	void compact(ICBSNode& node);

	bool runICBSSearch();
	bool isCardinal(const list<tuple<vertex_t, vertex_t, int>>& cons, const list<MDDNode*>& level);
	void collectConstraints(ICBSNode* curr, int agent_id, vector < list< pair<vertex_t, vertex_t> > >& cons_vec);
	void findConflicts(ICBSNode& curr, int a1, int a2);
	void findConflicts(ICBSNode& curr);
	Conflict* chooseConflict(ICBSNode &parent);
	bool detectCollision(vertex_t v1, vertex_t v2);
	bool detectCollision(vertex_t v1, vertex_t v2, vertex_t v1_to, vertex_t v2_to);
	int getHLHeuristic(const ICBSNode& curr);
	inline bool updateICBSNode(ICBSNode* leaf_node, ICBSNode* root_node);
	inline void updatePaths(ICBSNode* curr, ICBSNode* root_node);
	void generateChild(ICBSNode* n1, const ICBSNode* curr);

	void buildMDD(ICBSNode& curr, int id, int lookahead);
	void updateFocalList(double old_lower_bound, double new_lower_bound);
	void computeHeuristics(vector<double>& heuristics, vertex_t goal, const searchGraph_t& G);
	inline void releaseClosedListNodes();
	ICBSSearch(int TIME_LIMIT, const searchGraph_t& G, const std::vector<vertex_t>& starts, const std::vector<vertex_t>& goals, int num_of_agents, constraint_strategy c, double HL_w = 1, int lookahead = 0, bool useh = false);
	~ICBSSearch();

	void copyConflicts(const list<Conflict*>& conflicts, list<Conflict*>& copy, int excluded_agent) const;
	void findConflicts(ICBSNode& curr, int a1, int a2) const;
};

