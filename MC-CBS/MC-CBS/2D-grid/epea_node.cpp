#include "epea_node.h"


// Check if the move is valid, i.e. not colliding into walls or other agents
 bool EPEANode::IsValid(int agent_id, int curr_loc, int next_loc)
{
	 if (0 > next_loc || next_loc >= num_of_cols * num_of_rows 
		|| my_heuristics[agent_id][next_loc] >= num_of_cols * num_of_rows)
		return false;
	 for (list<pair<int, int>>::iterator it = edge_cons.begin(); it != edge_cons.end(); ++it)
		 if (it->first == next_loc  && it->second == curr_loc)
			 return false;
	 for (list<pair<int, int>>::iterator it = vertex_cons.begin(); it != vertex_cons.end(); ++it)
		if(	it->first / num_of_cols <= next_loc / num_of_cols + agent_sizes[agent_id] - 1	&& next_loc / num_of_cols <= it->second / num_of_cols &&
			it->first % num_of_cols <= next_loc % num_of_cols + agent_sizes[agent_id] - 1 && next_loc % num_of_cols <= it->second % num_of_cols)
			return false; 
	return true;
}

void EPEANode::calcSingleAgentDeltaFs()
{
	// Init
	singleAgentDeltaFs = new vector<list<pair<int16_t, int16_t>>>(num_of_agents);

	int hBefore, hAfter;

	maxDeltaF = 0;

	// Set values
	for (int i = 0; i < num_of_agents; i++)
	{
		hBefore = (int)round(my_heuristics[i][locs[i]]);

		int singleAgentMaxLegalDeltaF = -1;

		for(int16_t j = 0; j < MapLoader::MOVE_COUNT; j++)
		{
			if (IsValid(i, locs[i], locs[i] + move_offset[j]))
			{
				hAfter = (int)round(my_heuristics[i][locs[i] + move_offset[j]]);
				int16_t deltaF  = 0;
				if (hBefore != 0)
					deltaF = (int16_t)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
				else if (hAfter != 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference
					deltaF = (int16_t)(hAfter - hBefore + makespan - arrival_time[i] + 1);
				
				list< pair<int16_t, int16_t> >::iterator it = singleAgentDeltaFs->at(i).begin();
				for(; it != singleAgentDeltaFs->at(i).end() && it->second < deltaF; ++it)
					continue;
				singleAgentDeltaFs->at(i).insert(it, make_pair(j, deltaF));

				singleAgentMaxLegalDeltaF = singleAgentMaxLegalDeltaF > deltaF ? singleAgentMaxLegalDeltaF: deltaF;
			}
		}

		if (singleAgentMaxLegalDeltaF == -1) // No legal action for this agent, so no legal children exist for this node
		{
			this->maxDeltaF = 0; // Can't make it negative without widening the field.
			break;
		}

		this->maxDeltaF += singleAgentMaxLegalDeltaF;
	}
}

void EPEANode::Clear()
{
	//Save some memory
	this->arrival_time.clear();
	this->vertex_cons.clear();
	this->edge_cons.clear();
	this->fLookup.clear();
	this->my_heuristics.clear();
	if (singleAgentDeltaFs != NULL)
		this->singleAgentDeltaFs->clear();
	this->alreadyExpanded = false; // Enables reopening
	this->targetDeltaF = 0;
	this->remainingDeltaF = 0;
}

void EPEANode::ClearConstraintTable() // Clear constraint table
{
	vertex_cons.clear();
	edge_cons.clear();
}


// Recursive func. Kind of dynamic programming as it updates the lookup table as it goes to refrain from computing answers twice.
bool EPEANode::existsChildForF(int agentNum, int remainingTargetDeltaF)
{
	return true;
	// Stopping conditions:
	if (agentNum == this->num_of_agents)
	{
		if (remainingTargetDeltaF == 0)
			return true;
		return false;
	}
	if(fLookup.empty())
		fLookup.resize(this->num_of_agents);
	if(fLookup[agentNum].empty())
		fLookup.resize(this->maxDeltaF + 1);
	if (fLookup[agentNum][remainingTargetDeltaF] != 0) // Answer known (arrays are initialized to zero).
	{
		return fLookup[agentNum][remainingTargetDeltaF] == 1; // Return known answer. 
	}

	// Recursive actions:
	for (list< pair<int16_t, int16_t> >::iterator it = singleAgentDeltaFs->at(agentNum).begin(); it != singleAgentDeltaFs->at(agentNum).end() 
			&& it->second <= remainingTargetDeltaF; ++it)
	{
		if (existsChildForF(agentNum + 1, (int16_t)(remainingTargetDeltaF - it->second)))
		{
			fLookup[agentNum][remainingTargetDeltaF] = 1;
			return true;
		}
	}
	fLookup[agentNum][remainingTargetDeltaF] = 2;
	return false;
}

void EPEANode::MoveTo(int agent_id, int next_loc)
{
	// Update constraint table
	int x_max = min(next_loc / num_of_cols + agent_sizes[agent_id] - 1, num_of_rows - 1);
	int y_max = min(next_loc % num_of_cols + agent_sizes[agent_id] - 1, num_of_cols - 1);
	vertex_cons.push_back(make_pair(next_loc, x_max * num_of_cols + y_max));
	edge_cons.push_back(make_pair(locs[agent_id], next_loc));

	// Update stats
	int hBefore = my_heuristics[agent_id][locs[agent_id]];
	int hAfter = my_heuristics[agent_id][next_loc];
	if(makespan == 0 && hBefore == 0)
		arrival_time[agent_id] = 0;
	else if(hBefore > 0 && hAfter == 0) // arriving at the goal
		arrival_time[agent_id] = makespan + 1;
	if (hBefore > 0)
		this->g++;
	else if (hAfter > 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference
		this->g += makespan + 1 - arrival_time[agent_id];
	this->h += hAfter - hBefore;

	// Update location
	locs[agent_id] = next_loc;
	
}

void EPEANode::Update(const EPEANode &cpy)
{
	this->g = cpy.g;
	this->h = cpy.h;
	this->maxDeltaF = cpy.maxDeltaF;
	this->remainingDeltaF = cpy.remainingDeltaF;
	this->targetDeltaF = cpy.targetDeltaF;
	this->parent = cpy.parent;
	this->alreadyExpanded = cpy.alreadyExpanded;

	// Deep copy
	this->arrival_time.resize(num_of_agents);
	this->arrival_time.assign(cpy.arrival_time.begin(), cpy.arrival_time.end());
	this->vertex_cons.resize(cpy.vertex_cons.size());
	this->vertex_cons.assign(cpy.vertex_cons.begin(), cpy.vertex_cons.end());
	this->edge_cons.resize(cpy.edge_cons.size());
	this->edge_cons.assign(cpy.edge_cons.begin(), cpy.edge_cons.end());
}


EPEANode::EPEANode(const MapLoader &ml, const AgentsLoader &al, const vector<int*>& my_heuristics)
{
	this->num_of_agents = al.num_of_agents;
	this->num_of_cols = ml.cols;
	this->num_of_rows = ml.rows;
	this->move_offset = ml.moves_offset;
	this->agent_sizes = al.sizes;
	this->my_heuristics = my_heuristics;
	this->g = 0;
	this->h = 0;
	locs.resize(al.num_of_agents);
	for (int i = 0; i < locs.size(); i++)
	{
		locs[i] =num_of_cols * al.initial_locations[i].first + al.initial_locations[i].second;
		this->h += my_heuristics[i][locs[i]];
	}
	arrival_time.resize(al.num_of_agents, -1);
}

vector<vector<int>> EPEANode::GetPlan()
{
	vector<vector<int>> paths(num_of_agents);
	for(int i = 0; i < num_of_agents; i++)
		paths[i].resize(arrival_time[i] + 1);
	EPEANode *node = this;
	for (int t = makespan; t >= 0; t--)
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			if(t <= arrival_time[i])
				paths[i][t] = node->locs[i];
		}
		node = node->parent;
	}
	return paths;
}

void EPEANode::deep_copy(const EPEANode &cpy)
{
	//Constants
	this->num_of_cols = cpy.num_of_cols;
	this->num_of_rows = cpy.num_of_rows;
	this->num_of_agents = cpy.num_of_agents;
	this->move_offset = cpy.move_offset;
	this->my_heuristics = cpy.my_heuristics;
	this->agent_sizes = cpy.agent_sizes;

	//Stats
	this->makespan = cpy.makespan;
	this->g = cpy.g;
	this->h = cpy.h;
	this->parent = cpy.parent;

	// Deep copy
	this->locs.resize(num_of_agents);
	this->locs.assign(cpy.locs.begin(), cpy.locs.end());
	this->arrival_time.resize(num_of_agents);
	this->arrival_time.assign(cpy.arrival_time.begin(), cpy.arrival_time.end());
	this->vertex_cons.resize(cpy.vertex_cons.size());
	this->vertex_cons .assign(cpy.vertex_cons.begin(), cpy.vertex_cons.end());
	this->edge_cons.resize(cpy.edge_cons.size());
	this->edge_cons.assign(cpy.edge_cons.begin(), cpy.edge_cons.end());
	this->singleAgentDeltaFs = new vector<list<pair<int16_t, int16_t>>>(cpy.singleAgentDeltaFs->size());
	for (int i = 0; i < this->singleAgentDeltaFs->size(); i++)
	{
		this->singleAgentDeltaFs->at(i).resize(cpy.singleAgentDeltaFs->at(i).size());
		this->singleAgentDeltaFs->at(i).assign(cpy.singleAgentDeltaFs->at(i).begin(), cpy.singleAgentDeltaFs->at(i).end());
	}
	
	this->fLookup.clear(); 

	alreadyExpanded = false;  // Creating a new unexpanded node from cpy

	// For intermediate nodes created during expansion (fully expanded nodes have these fields recalculated when they're expanded)
	targetDeltaF = cpy.targetDeltaF;
	remainingDeltaF = cpy.remainingDeltaF;
	
						   
	maxDeltaF = cpy.maxDeltaF; // Not necessarily achievable after some of the agents moved.
							   // The above is OK because we won't be using data for agents that already moved.
}

EPEANode::EPEANode():
	singleAgentDeltaFs(NULL)
{
	this->makespan = -1;
	locs.resize(1);
}

EPEANode::~EPEANode()
{
	Clear();
}
