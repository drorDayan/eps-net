#pragma once


#include "compute_heuristic.h"
#include "epea_node.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;
using google::dense_hash_map;
using namespace std;

class EPEASearch
{

public:
	double runtime = 0;
	bool solution_found = false;
	double solution_cost = -1;
	double solution_depth = 0;

	int num_expanded = 0;
	int num_generated = 0;

	vector < vector<int> > paths;  // agents paths

	bool runEPEASearch();
	EPEASearch(int TIME_LIMIT, const MapLoader& ml, const AgentsLoader& al);
	~EPEASearch();

private:
	const int TIME_LIMIT;
	const int maxCost = INT_MAX;
	int start_time;
	typedef fibonacci_heap< EPEANode*, compare<EPEANode::compare_node> > heap_open_t;
	typedef dense_hash_map<EPEANode*, EPEANode*, EPEANode::EPEANodeHasher, EPEANode::epea_eqnode> hashtable_t;
	heap_open_t open_list;
	hashtable_t allNodes_table;

	int num_of_agents;
	const int* moves_offset;
	int num_col;
	int num_row;
	AgentsLoader al;

	// used in hash table
	EPEANode* empty_node;
	EPEANode* deleted_node;

	vector<int*> my_heuristics;  // this is the precomputed heuristic for this agent

	list<EPEANode*> ExpandOneAgent(list<EPEANode*>& intermediateNodes, int agentIndex);
	void Expand(EPEANode& node);
	void Clear();
};

