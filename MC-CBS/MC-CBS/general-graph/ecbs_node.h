#pragma once

#include <string>
#include <vector>
#include <list>
#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>
#include "LLNode.h"

using boost::heap::fibonacci_heap;
using boost::heap::compare;

using namespace std;

class ECBSNode 
{
 public:
  int agent_id;
  list<tuple<vertex_t, vertex_t, int>> constraints; 
  ECBSNode* parent;
  vector<pathEntry> path;
  int g_val;  // (total cost)
  int num_of_collisions;  // (number of collisions)
  int time_expanded;
  int time_generated;
  int sum_min_f_vals;  // saves the overall sum of min f-vals.
  int ll_min_f_val;  // saves this agent's low-level min-f-val (as reported by the search that found the path stored)
  int path_cost;  // saves this agent's low-level path-cost

  // the following is used to comapre nodes in the OPEN list
  struct compare_node
  {
    bool operator()(const ECBSNode* n1, const ECBSNode* n2) const 
	{
      return n1->sum_min_f_vals >= n2->sum_min_f_vals;
    }
  };  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

  // the following is used to comapre nodes in the FOCAL list
  struct secondary_compare_node {
    bool operator()(const ECBSNode* n1, const ECBSNode* n2) const 
	{
      if (n1->num_of_collisions == n2->num_of_collisions)
        return n1->g_val >= n2->g_val;  // break ties towards shorter (overall) solutions
      return n1->num_of_collisions >= n2->num_of_collisions;
    }
  };  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

  typedef boost::heap::fibonacci_heap< ECBSNode* , compare<compare_node> >::handle_type open_handle_t;
  typedef boost::heap::fibonacci_heap< ECBSNode* , compare<secondary_compare_node> >::handle_type focal_handle_t;

  open_handle_t open_handle;
  focal_handle_t focal_handle;


  ECBSNode();  // for efficiency, reserve num_of_agents in all_constraints vector
  ECBSNode(int agent_id, ECBSNode* parent, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals);
  ~ECBSNode(){}


  // The following is used by googledensehash for checking whether two nodes are equal
  // we say that two nodes, s1 and s2, are equal if
  // both are non-NULL and have the same time_expanded (unique)
  struct ecbs_eqnode 
  {
    bool operator()(const ECBSNode* s1, const ECBSNode* s2) const
	{
      return (s1 == s2) || (s1 && s2 && s1->time_generated == s2->time_generated);
    }
  };

  // The following is used by googledensehash for generating the hash value of a nodes
  // this is needed because otherwise we'll have to define the specilized template inside std namespace
  struct ECBSNodeHasher 
  {
    std::size_t operator()(const ECBSNode* n) const 
	{
      size_t agent_id_hash = std::hash<int>()(n->agent_id);
      size_t time_generated_hash = std::hash<int>()(n->time_generated);
      return ( agent_id_hash ^ (time_generated_hash << 1) );
    }
  };
};
