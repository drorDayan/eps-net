#include "ICBSNode.h"



ICBSNode::ICBSNode()
{
	agent_id = -1;
	g_val = 0;
	h_val = 0;
	conflict = NULL;
	num_of_collisions = 0;
	time_expanded = -1;
	sum_min_f_vals = -1;
}

ICBSNode::ICBSNode(int agent_id, int numAgents, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals)
{
	this->agent_id = agent_id;
	this->g_val = g_val;
	this->h_val = 0;
	conflict = NULL;
	this->num_of_collisions = num_of_collisions;
	this->time_expanded = time_expanded;
	this->sum_min_f_vals = sum_min_f_vals;
	this->mdds.resize(numAgents, NULL);
	this->parent = NULL;
}

ICBSNode::ICBSNode(int agent_id, ICBSNode* parent, double g_val, int num_of_collisions, int time_expanded, double sum_min_f_vals)
	:parent(parent)
{
	this->agent_id = agent_id;
	this->g_val = g_val;
	this->h_val = 0;
	conflict = NULL;
	this->num_of_collisions = num_of_collisions;
	this->time_expanded = time_expanded;
	this->sum_min_f_vals = sum_min_f_vals;
	mdds.resize(parent->mdds.size());
	for (int i = 0; i < mdds.size(); i++)
	{
		mdds[i] = parent->mdds[i];
		if (mdds[i] != NULL)
			mdds[i]->numPointers++;
	}
}



ICBSNode::~ICBSNode()
{
	if(mdds.empty())
		return;
	for(int i = 0; i < mdds.size(); i++)
		if(mdds[i] != NULL)
			delete mdds[i];
}
