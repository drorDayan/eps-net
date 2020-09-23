// CT node in ICBS

#pragma once
#include "mdd.h"
#include "conflict.h"

class ICBSNode
{
public:
	int agent_id;
	std::tuple<int, int, int> constraint;  // <int loc1, int loc2, int timestep> NOTE loc2 = rightbottom_location for vertex constraint; loc2 = - loation2 - 1 for Edge Constraint
	ICBSNode* parent;
	std::vector<pathEntry> path;
	int g_val;  // sum of path costs
	int h_val;
	int f_val;  // g_val + h_val
	int num_of_collisions;
	int time_expanded;
	int time_generated;
	//double ll_min_f_val;  // saves this agent's low-level min-f-val (as reported by the search that found the path stored)
	int path_cost;  // saves this agent's low-level path-cost

					   // the following is used to comapre nodes in the OPEN list
	struct compare_node {
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const {
			return n1->f_val >= n2->f_val;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

		// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node {
		bool operator()(const ICBSNode* n1, const ICBSNode* n2) const {
			if (n1->num_of_collisions == n2->num_of_collisions)
				return n1->g_val >= n2->g_val;  // break ties towards shorter (overall) solutions
			return n1->num_of_collisions >= n2->num_of_collisions;
		}
	};  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)


	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<secondary_compare_node> >::handle_type focal_handle_t;
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

	std::list<Conflict*> cardinalConf;
	std::list<Conflict*> semiConf;
	std::list<Conflict*> nonConf;
	std::list<Conflict*> unknownConf;
	//std::vector<int> constraintCost;

	std::vector<MDD*> mdds;
	Conflict* conflict;
	
	//inline int getFVal() const {return g_val + h_val; }
	//bool buildMDD(const std::vector < std::list< std::pair<int, int> > >& constraints, int numOfLevels);
	//bool updateMDD(const std::tuple<int, int, int> &constraint);
	//bool findPathByMDD(bool* res_table);

	ICBSNode(): agent_id(-1), g_val(0), h_val(0), conflict(NULL), num_of_collisions(0), time_expanded(-1), f_val(-1) {}
	~ICBSNode();
};

//std::ostream& operator<<(std::ostream& os, const ICBSNode& n);