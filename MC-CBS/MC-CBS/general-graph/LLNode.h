#pragma once
#include <boost/heap/fibonacci_heap.hpp>
#include <vector>
#include <functional>  // for std::hash (c++11 and above)
#include "searchGraph.h"
using boost::heap::fibonacci_heap;
using boost::heap::compare;

struct pathEntry
{
	vertex_t vertex;
	pathEntry(){}
	pathEntry(vertex_t& vertex)
	{ 
		vertex = vertex; 
	}
};

class LLNode
{
public:
	vertex_t vertex;
	double g_val;
	double h_val = 0;
	LLNode* parent;
	int timestep = 0;
	int num_internal_conf = 0;
	bool in_openlist = false;

	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const 
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

		// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node
	{
		// returns true if n1 > n2
		bool operator()(const LLNode* n1, const LLNode* n2) const 
		{
			if (n1->num_internal_conf == n2->num_internal_conf) 
			{
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			}
			return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


		// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef boost::heap::fibonacci_heap< LLNode*, compare<LLNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< LLNode*, compare<LLNode::secondary_compare_node> >::handle_type focal_handle_t; 

	open_handle_t open_handle;
	focal_handle_t focal_handle;


	LLNode();
	LLNode(const LLNode& other);
	LLNode(const vertex_t& vertex, double g_val, double h_val,
		LLNode* parent, int timestep,
		int num_internal_conf = 0, bool in_openlist = false);
	inline double getFVal() const { return g_val + h_val; }
	~LLNode(){}

	// The following is used by googledensehash for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestep
	struct eqnode 
	{
		bool operator()(const LLNode* s1, const LLNode* s2) const 
		{
			return (s1 == s2) || (s1 && s2 &&
				s1->vertex == s2->vertex &&
				s1->timestep == s2->timestep);
		}
	};

	// The following is used by googledensehash for generating the hash value of a nodes
	struct NodeHasher 
	{
		std::size_t operator()(const LLNode* n) const 
		{
			size_t loc_hash = std::hash<int>()((int) n->vertex);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			return (loc_hash ^ (timestep_hash << 1));
		}
	};

};

