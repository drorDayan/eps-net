#include "ecbs_node.h"
#include <vector>

ECBSNode::ECBSNode() 
{
  agent_id = -1;
  g_val = 0;
  num_of_collisions = 0;
  time_expanded = -1;
  sum_min_f_vals = -1;
  path_cost = -1;
}

ECBSNode::ECBSNode(int agent_id, ECBSNode* parent, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals):
    agent_id(agent_id), parent(parent), g_val(g_val), num_of_collisions(num_of_collisions), time_expanded(time_expanded), sum_min_f_vals(sum_min_f_vals) {}






