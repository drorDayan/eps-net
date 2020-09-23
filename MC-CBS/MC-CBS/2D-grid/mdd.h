#pragma once
#include <list>
#include <vector>
#include <stdio.h>

#include "node.h"
#include "single_agent_icbs.h"

class MDDNode
{
public:
	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if(parent == NULL)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
		num_internal_conf = 0;
		cost  = -1;
		parent = NULL;
	}
	int location;
	int level;
	int cost; // the minimal cost of path that traverses this node - current cost
	int num_internal_conf; // used for bypass choosing
	 // the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const MDDNode* n1, const MDDNode* n2) const 
		{
			return n1->num_internal_conf >= n2->num_internal_conf;
		}
	};
	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}


	std::list<MDDNode*> children;
	std::list<MDDNode*> parents;
	MDDNode* parent;
};

class MDD
{
public:
	std::vector<std::list<MDDNode*>> levels;

	int numPointers; //used to count how many pointers pointed to this
	bool buildMDD(const std::vector < std::list< std::pair<int, int> > >& constraints, int current_cost, int lookahead, const SingleAgentICBS & solver);
	bool updateMDD(const std::tuple<int, int, int> &constraint, int num_col);
	MDDNode* find(int location, int level);
	void deleteNode(MDDNode* node);
	void clear();

	

	MDD(){};
	MDD(MDD & cpy);
	~MDD();
};

