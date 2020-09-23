#pragma once
#include "ecbs_node.h"
#include "MDD.h"
#include "conflict.h"



class ICBSNode :
	public ECBSNode
{
public:
	typedef boost::heap::fibonacci_heap< ICBSNode*, compare<compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, compare<secondary_compare_node> >::handle_type focal_handle_t;
	typedef boost::heap::fibonacci_heap< MDDNode*, boost::heap::compare<MDDNode::compare_node> > mdd_open_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	// The following is used by googledensehash for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specilized template inside std namespace
	struct ICBSNodeHasher {
		std::size_t operator()(const ICBSNode* n) const {
			size_t agent_id_hash = std::hash<int>()(n->agent_id);
			size_t time_generated_hash = std::hash<int>()(n->time_generated);
			return (agent_id_hash ^ (time_generated_hash << 1));
		}
	};

	list<Conflict*> cardinalConf;
	list<Conflict*> semiConf;
	list<Conflict*> nonConf;
	list<Conflict*> unknownConf;
	ICBSNode* parent;
	vector<MDD*> mdds;
	Conflict* conflict;
	int h_val;

	ICBSNode();
	ICBSNode(int agent_id, int numAgents, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals);
	ICBSNode(int agent_id, ICBSNode* parent, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals);

	~ICBSNode();
};

