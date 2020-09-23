// Represents 2D-Nodes
#pragma once

#include <boost/heap/fibonacci_heap.hpp>
#include <functional>  // for std::hash (c++11 and above)
#include "map_loader.h"

struct pathEntry
{
  int location;
};

class Node 
{
public:
	// the following is used by googledensehash for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestep
	struct eqnode
	{
		bool operator()(const Node* s1, const Node* s2) const
		{
			return (s1 == s2) || (s1 && s2 && s1->loc == s2->loc && s1->timestep == s2->timestep);
		}
	};

	// the following is used by googledensehash for generating the hash value of a nodes
	struct NodeHasher 
	{
		std::size_t operator()(const Node* n) const 
		{
			size_t loc_hash = std::hash<int>()(n->loc);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			return (loc_hash ^ (timestep_hash << 1));
		}
	};

	// the following is used to comapre nodes in the OPEN list (top of the heap has min f_vals)
	struct compare_node
	{
		bool operator()(const Node* n1, const Node* n2) const
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val; // n1 > n2 if it has larger f_val
		}
	};

	// the following is used to comapre nodes in the FOCAL list (top of the heap has min number-of-conflicts)
	struct secondary_compare_node
	{
		bool operator()(const Node* n1, const Node* n2) const
		{
			if (n1->num_internal_conf == n2->num_internal_conf)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
					return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
			}
			return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
		}
	};


	
	typedef boost::heap::fibonacci_heap< Node*, boost::heap::compare<compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< Node*, boost::heap::compare<secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;

	inline double getFVal() const { return g_val + h_val; }

	int loc;
	int g_val;
	int h_val = 0;
	Node* parent;
	int timestep = 0;
	int num_internal_conf = 0;
	bool in_openlist = false;

	Node();
	Node(const Node& other);
	Node(int loc, int g_val, int h_val, Node* parent, int timestep, int num_internal_conf = 0, bool in_openlist = false);
	~Node(){}	
};

