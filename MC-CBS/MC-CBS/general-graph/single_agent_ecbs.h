#pragma once

#include <vector>
#include <list>
#include <google/dense_hash_map>
#include "LLNode.h"


using std::cout;

class SingleAgentECBS
{
 public:
  // define typedefs (will also be used in ecbs_search)
  typedef boost::heap::fibonacci_heap< LLNode* , boost::heap::compare<LLNode::compare_node> > heap_open_t;
  typedef boost::heap::fibonacci_heap< LLNode* , boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t;

  typedef google::dense_hash_map<LLNode*, LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
  // note -- hash_map (key is a node pointer, data is a node handler,
  //                   NodeHasher is the hash function to be used,
  //                   eqnode is used to break ties when hash values are equal)

 std:: vector<pathEntry> path;  // a path that takes the agent from initial to goal location satisying all constraints
  // consider changing path from vector to deque (efficient front insertion)
  int path_cost;
  const vertex_t startV;
  const vertex_t goalV;
  const int agent_id;
  const std::vector<int> my_heuristic;  // this is the precomputed heuristic for this agent

  const searchGraph_t G;

  uint64_t num_expanded;
  uint64_t num_generated;

  double lower_bound;  // FOCAL's lower bound ( = e_weight * min_f_val)
  int min_f_val;  // min f-val seen so far

  // note -- handle typedefs is defined inside the class (hence, include node.h is not enough).
  heap_open_t open_list;
  heap_focal_t focal_list;

  hashtable_t allNodes_table;

  // used in hash table and would be deleted from the d'tor
  LLNode* empty_node;
  LLNode* deleted_node;


  SingleAgentECBS(vertex_t start, vertex_t goal, const std::vector<int>& my_heuristic,
					const searchGraph_t& G, int agent_id);


  const std::vector<pathEntry>* getPath() {return &path;}  // return a pointer to the path found;


  // returns the minimal plan length for the agent (that is, extract the latest timestep which
  //   has a constraint invloving this agent's goal location).
  int extractLastGoalTimestep(const std::vector< std::list< std::pair<vertex_t, vertex_t> > >& cons);

  inline void releaseClosedListNodes(hashtable_t* allNodes_table);

  // Checks if a vaild path found (wrt my_map and constraints)
  // Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
  inline bool isConstrained(vertex_t currV, vertex_t nextV, int next_timestep, const std::vector< std::list< std::pair<vertex_t, vertex_t> > >& cons) const;

  //Updates the path datamember (vector<int>).
 //After update it will contain the sequence of locations found from the goal to the start.
  void updatePath(LLNode* goal);  // $$$ make inline?

  // Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
  // Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
  int numOfConflictsForStep(vertex_t currV, vertex_t nextV, int next_timestep, const std::vector<std::vector<pathEntry>*>& paths);

  // Iterate over OPEN and adds to FOCAL all nodes with: 1) f-val > old_min_f_val ; and 2) f-val * f_weight < new_lower_bound.
  void updateFocalList(int old_lower_bound, int new_lower_bound);

  // Returns true if a collision free path found (with cost up to f_weight * f-min) while
  //   minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
  bool findPath(const std::vector < std::list< std::pair<vertex_t, vertex_t> > >& constraints, const std::vector<std::vector<pathEntry>*>& paths, double w = 1);

  ~SingleAgentECBS();
};

