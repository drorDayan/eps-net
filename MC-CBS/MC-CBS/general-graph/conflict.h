#pragma once
#include "MDD.h"
#include "ecbs_node.h"

enum conflict_type { UNKNOWN, CARDINAL, SEMI, GEQSEMI, NON, TYPE_COUNT };
enum constraint_strategy { ASYM, MAX, ICBS, STRATEGY_COUNT };

class Conflict
{
public:
	int timestep;
	int a1;
	int a2;
	vertex_t v1;
	vertex_t v2;
	vertex_t v1_from;
	vertex_t v2_from;
	bool vertexConflict;
	list<tuple<vertex_t, vertex_t, int>> con1;
	list<tuple<vertex_t, vertex_t, int>> con2;
	int cost1; // delta cost of child 1
	int cost2; // delta cost of child 2
	conflict_type type;


	Conflict(int timestep, int a1, int a2, vertex_t v1, vertex_t v2, conflict_type type = UNKNOWN) :
		timestep(timestep), a1(a1), a2(a2), v1(v1), v2(v2), type(type), vertexConflict(true) {}
	Conflict(int timestep, int a1, int a2, vertex_t v1_from, vertex_t v2_from, vertex_t v1, vertex_t v2, conflict_type type = UNKNOWN) :
		timestep(timestep), a1(a1), a2(a2), v1(v1), v2(v2), v1_from(v1_from), v2_from(v2_from), type(type), vertexConflict(false) {}
	Conflict(const Conflict& cpy);
	~Conflict() {}

	void ComputeICBSConstraint();

	void ComputeAsymConstraints(const searchGraph_t& G, bool swap);

	void ComputeSymConstraints(const searchGraph_t& G);

	bool isConflicting(const searchGraph_t& G, vertex_t v, const list<vertex_t>& V);

	bool isConflicting(const searchGraph_t& G, vertex_t e_from, vertex_t e_to, const list<pair<vertex_t, vertex_t>>& E);

	int getReward(const list<vertex_t> &V, const list<MDDNode*>& mdd, int lookahead);

	int getReward(const list<pair<vertex_t, vertex_t>> &E, const list<MDDNode*>& mdd, int lookahead);

	int getReward(const list<tuple<vertex_t, vertex_t, int>>& cons, const list<MDDNode*>& level, int lookahead);

	void ComputeMaxRewardConstraints(const searchGraph_t& G, const list<MDDNode*>& mdd1, 
		const list<MDDNode*>& mdd2, int lookahead);
};
