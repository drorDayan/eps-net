#include "icbs_node.h"


//ICBSNode::ICBSNode(int agent_id, int numAgents, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals)
//{
//	this->agent_id = agent_id;
//	this->g_val = g_val;
//	this->h_val = 0;
//	conflict = NULL;
//	this->num_of_collisions = num_of_collisions;
//	this->time_expanded = time_expanded;
//	this->sum_min_f_vals = sum_min_f_vals;
//	this->mdds.resize(numAgents, NULL);
//	this->parent = NULL;
//	this->constraintCost.resize(numAgents, 0);
//}
//
//ICBSNode::ICBSNode(int agent_id, ICBSNode* parent, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals)
//	:parent(parent)
//{
//	this->agent_id = agent_id;
//	this->g_val = g_val;
//	this->h_val = 0;
//	conflict = NULL;
//	this->num_of_collisions = num_of_collisions;
//	this->time_expanded = time_expanded;
//	this->sum_min_f_vals = sum_min_f_vals;
//	mdds.resize(parent->mdds.size());
//	for (int i = 0; i < mdds.size(); i++)
//	{
//		mdds[i] = parent->mdds[i];
//		if (mdds[i] != NULL)
//			mdds[i]->numPointers++;
//	}
//	this->constraintCost.resize(parent->constraintCost.size());
//	constraintCost.assign(parent->constraintCost.begin(), parent->constraintCost.end());
//}



ICBSNode::~ICBSNode()
{
	if(mdds.empty())
		return;
	for(int i = 0; i < mdds.size(); i++)
		if(mdds[i] != NULL)
			delete mdds[i];
}


//bool ICBSNode::isEqual(const ICBSNode* n1, const ICBSNode* n2) {
//	return (n1->parent == n2->parent &&
//		n1->agent_id == n2->agent_id &&
//		n1->constraint == n2->constraint);
//}




//std::ostream& operator<<(std::ostream& os, const ICBSNode& n) {
//	os << "THIS NODE HAS: g_val=" << n.g_val << " and num_of_collisions=" << n.num_of_collisions << ". It constrains agent " << n.agent_id <<
//		" on loc1[" << std::get<0>(n.constraint) << "] to loc2[" << std::get<1>(n.constraint) << "] at time[" << std::get<2>(n.constraint) << "]" <<
//		" ; Path found has LB=" << n.ll_min_f_val << " ; And sum_min_f_vals=" << n.sum_min_f_vals << std::endl;
//	return os;
//}
