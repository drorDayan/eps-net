#include "MDD.h"

bool MDD::buildMDD(const searchGraph_t& G, const std::vector < std::list< std::pair<vertex_t, vertex_t> > >& constraints, int current_cost, int lookahead, const SingleAgentECBS& solver)
{
	int numOfLevels = current_cost + lookahead;
	MDDNode* root = new MDDNode(solver.startV, NULL); // Root
	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	while (!open.empty())
	{
		MDDNode* node = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		if (node->level == numOfLevels - 1)
		{
			node->cost = lookahead;
			levels[numOfLevels - 1].push_back(node);
			break;
		}
		int heuristicBound = numOfLevels - node->level - 2; // We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the _children_.

		//Move action
		auto neighbours = boost::adjacent_vertices(node->vertex, G);
		for (auto newVert : make_iterator_range(neighbours))// Try every possible move. We only add backward edges in this step.
		{
			if (solver.my_heuristic[newVert] <= heuristicBound && !solver.isConstrained(node->vertex, newVert, node->level + 1, constraints)) // valid move
			{
				std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if ((*child)->vertex == newVert) // If the child node exists
					{
						(*child)->parents.push_back(node); // then add corresponding parent link and child link
						find = true;
						break;
					}
				if (!find) // Else generate a new mdd node
				{
					MDDNode* childNode = new MDDNode(newVert, node);
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
		
		//Wait action
		if (solver.my_heuristic[node->vertex] <= heuristicBound && !solver.isConstrained(node->vertex, node->vertex, node->level + 1, constraints)) // valid move
		{
			std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
			bool find = false;
			for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
				if ((*child)->vertex == node->vertex) // If the child node exists
				{
					(*child)->parents.push_back(node); // then add corresponding parent link and child link
					find = true;
					break;
				}
			if (!find) // Else generate a new mdd node
			{
				MDDNode* childNode = new MDDNode(node->vertex, node);
				open.push(childNode);
				closed.push_back(childNode);
			}
		}
	}
	// Backward
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
		{
			for (std::list<MDDNode*>::iterator parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
			{
				if ((*parent)->children.empty()) // a new node
				{
					levels[t - 1].push_back(*parent);
				}
				if ((*parent)->vertex == solver.goalV //goal location
					&& (*it)->cost == t - current_cost + 1) // agent can keep staying at goal location
					(*parent)->cost = (*it)->cost - 1;
				else if((*parent)->cost < 0)
					(*parent)->cost = (*it)->cost;
				else
					(*parent)->cost = std::min((*parent)->cost, (*it)->cost);
				(*parent)->children.push_back(*it); // add forward edge	
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (std::list<MDDNode*>::iterator it = closed.begin(); it != closed.end(); ++it)
		if ((*it)->children.empty() && (*it)->level < numOfLevels - 1)
			delete (*it);
	return true;
}

bool MDD::updateMDD(const std::tuple<vertex_t, vertex_t, int> &constraint, int num_col)
{
	vertex_t loc1 = get<0>(constraint), loc2 = get<1>(constraint); 
	int t = get<2>(constraint);

	if (loc2 != NULL) // Edge constraint
	{
		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
			if ((*it)->vertex == loc1)
				for (std::list<MDDNode*>::iterator child = (*it)->children.begin(); child != (*it)->children.end(); ++child)
					if ((*child)->vertex == loc2)
					{
						(*it)->children.erase(child);
						(*child)->parents.remove(*it);
						if ((*it)->children.empty())
							deleteNode(*it);
						if ((*child)->parents.empty())
							deleteNode(*child);
						return true;
					}
	}
	else // Vertex constraint
	{
		std::list<MDDNode*> ToDelete;
		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
			if (loc1 == (*it)->vertex)
				ToDelete.push_back(*it);
		for (std::list<MDDNode*>::iterator it = ToDelete.begin(); it !=ToDelete.end(); ++it)
			deleteNode(*it);
		return true;
	}
	return false;
}




void MDD::deleteNode(MDDNode* node)
{
	levels[node->level].remove(node);
	for (std::list<MDDNode*>::iterator child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if((*child)->parents.empty())
			deleteNode(*child);
	}
	for (std::list<MDDNode*>::iterator parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent);
	}
}

void MDD::clear()
{
	if(levels.empty())
		return;
	for (int i = 0; i < levels.size(); i++)
	{
		for (std::list<MDDNode*>::iterator it = levels[i].begin(); it != levels[i].end(); ++it)
			delete (*it);
	}
}

MDDNode* MDD::find(const vertex_t& V, int level) 
{
	if(level < levels.size())
		for (std::list<MDDNode*>::iterator it = levels[level].begin(); it != levels[level].end(); ++it)
			if((*it)->vertex == V)
				return (*it);
	return NULL;
}

MDD::MDD(MDD & cpy) // deep copy
{
	levels.resize(cpy.levels.size());
	MDDNode* root = new MDDNode(cpy.levels[0].front()->vertex, NULL);
	levels[0].push_back(root);
	for(int t = 0; t < levels.size() - 1; t++)
	{
		for (std::list<MDDNode*>::iterator node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->vertex, (*node)->level);
			for (std::list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = find((*cpyChild)->vertex, (*cpyChild)->level);
				if (child == NULL)
				{
					child = new MDDNode((*cpyChild)->vertex, (*node));
					levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}
		
	}
}

MDD::~MDD()
{
	clear();
}
