#pragma once
#include "mdd.h"
enum conflict_type { UNKNOWN, CARDINAL, SEMI, GEQSEMI, NON, TYPE_COUNT };

class Conflict
{
public:
	int timestep;
	int a1;
	int a2;
	int loc1;
	int loc2; //if loc2 < 0, then it is an edge conflict between locations loc1 and -loc2-1
	int cons1[2]; // rectangle constraint for a1 [top left corner, bottom right corner]
	int cons2[2];// rectangle constraint for a2 [top left corner, bottom right corner]
	int weight1; // increased cost of a1
	int weight2; // increased cost of a2
	conflict_type type;
	
	Conflict(int time, int agent1, int agent2, int location1, int location2, conflict_type contype = UNKNOWN):
		timestep(time), a1(agent1), a2(agent2), loc1(location1), loc2(location2), weight1(0), weight2(0), type(contype) {}
	Conflict(const Conflict& cpy);
	~Conflict() {}
	void swap(); // swap indices of a1 and a2

	void ComputeAsymConstraints(int num_col, int num_row, int size1, int size2); // compute asymmetric constraint
	void ComputeSymConstraints(int num_col, int num_row, int s1, int s2); // compute symmetric constraint
	bool disjunctive(const std::pair<int, int>& min1, const std::pair<int, int>& max1,
		const std::pair<int, int>& min2, const std::pair<int, int>& max2, int size1, int size2) const;
	void getMinimalRectangle(std::vector<int>& mdd_x_min, std::vector<int>& mdd_x_max,
		std::vector<int>& mdd_y_min, std::vector<int>& mdd_y_max, 
		int lookahead, const std::list<MDDNode*>& MDDNodes, int num_col) const;
	void computeMaximalDisjunctiveConstraint2(const std::pair<int, int>& min1, const std::pair<int, int>& max1,
		int size1, int size2, int num_col, int num_row);
	bool hasBetterWeight(int w1, int w2) const;
	bool hasBetterWeight(const Conflict* conflict) const {return hasBetterWeight(conflict->weight1, conflict->weight2); }
	void computeConstraintsForEdgeConflict(const std::vector<std::list<MDDNode*>>& mdd1,
		const std::vector<std::list<MDDNode*>>& mdd2);
};