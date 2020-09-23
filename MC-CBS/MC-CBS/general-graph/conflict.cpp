#include "conflict.h"

Conflict::Conflict(const Conflict& cpy)
{
	timestep = cpy.timestep;
	a1 = cpy.a1;
	a2 = cpy.a2;
	v1 = cpy.v1;
	v2 = cpy.v2;
	v1_from = cpy.v1_from;
	v2_from = cpy.v2_from;
	type = cpy.type;
	vertexConflict = cpy.vertexConflict;
	cost1 = cpy.cost1;
	cost2 = cpy.cost2;
	con1 = cpy.con1;
	con2 = cpy.con2;
}

void Conflict::ComputeICBSConstraint()
{
	if (vertexConflict)
	{
		con1.push_back(make_tuple(v1, SIZE_MAX, timestep));
		con2.push_back(make_tuple(v2, SIZE_MAX, timestep));
	}
	else
	{
		con1.push_back(make_tuple(v1_from, v1, timestep));
		con2.push_back(make_tuple(v2_from, v2, timestep));
	}
}


void Conflict::ComputeAsymConstraints(const searchGraph_t& G, bool swap = true)
{
	if (vertexConflict)
	{
		if (v1 == v2 || !swap || G[v1].generalizedVertexConflicts.size() >= G[v2].generalizedVertexConflicts.size())
		{
			con1.push_back(make_tuple(v1, SIZE_MAX, timestep));
			con2.push_back(make_tuple(v1, SIZE_MAX, timestep));
			for (auto v : G[v1].generalizedVertexConflicts)
			{
				con2.push_back(make_tuple(v, SIZE_MAX, timestep));
			}
			return;
		}
		else
		{
			con2.push_back(make_tuple(v2, SIZE_MAX, timestep));
			con1.push_back(make_tuple(v2, SIZE_MAX, timestep));
			for (auto v : G[v2].generalizedVertexConflicts)
			{
				con1.push_back(make_tuple(v, SIZE_MAX, timestep));
			}
			return;
		}
	}
	else
	{
		if (v1 == v1_from) //a1 waits
		{
			con1.push_back(make_tuple(v1, v1, timestep));
			con2.push_back(make_tuple(v1, v1, timestep));
			for (auto e : G[v1].generalizedVertexEdgeConflicts)
				con2.push_back(make_tuple(e.m_source, e.m_target, timestep));
			return;
		}
		else if (v2 == v2_from) //a2 waits
		{
			con2.push_back(make_tuple(v2, v2, timestep));
			con1.push_back(make_tuple(v2, v2, timestep));
			for (auto e : G[v2].generalizedVertexEdgeConflicts)
				con1.push_back(make_tuple(e.m_source, e.m_target, timestep));
			return;
		}
		else if (!swap || G[v1].generalizedVertexConflicts.size() >= G[v2].generalizedVertexConflicts.size())
		{
			con1.push_back(make_tuple(v1_from, v1, timestep));
			con2.push_back(make_tuple(v1_from, v1, timestep));
			edge_t e1 = (boost::edge(v1_from, v1, G)).first;
			for (auto e : G[e1].generalizedEdgeConflicts)
				con2.push_back(make_tuple(e.m_source, e.m_target, timestep));
			for (auto v : G[e1].generalizedEdgeVertexConflicts)
				con2.push_back(make_tuple(v, v, timestep));
			return;
		}
		else
		{
			con2.push_back(make_tuple(v2_from, v2, timestep));
			con1.push_back(make_tuple(v2_from, v2, timestep));
			edge_t e2 = (boost::edge(v2_from, v2, G)).first;
			for (auto e : G[e2].generalizedEdgeConflicts)
				con1.push_back(make_tuple(e.m_source, e.m_target, timestep));
			for (auto v : G[e2].generalizedEdgeVertexConflicts)
				con1.push_back(make_tuple(v, v, timestep));
			return;
		}
	}
}

void Conflict::ComputeSymConstraints(const searchGraph_t& G)
{
	if (vertexConflict)
	{
		if (v1 == v2) // occupy the same vertex
		{
			for (auto v : G[v1].generalizedVertexConflicts)
			{
			}
			return;
		}
		position_t mid = (G[v1].pos + G[v2].pos) / 2;
	}
	else
	{

	}
}

bool Conflict::isConflicting(const searchGraph_t& G, vertex_t v, const list<vertex_t>& V)
{
	for (auto u : V)
	{
		if (u == v)
			continue;
		//DROR: It seems like it is possible to get a vertex from its vertex_t using the searchGraph_t
		if (G[v].generalizedVertexConflicts.find(u) == G[v].generalizedVertexConflicts.end())
			return false;
	}
	return true;
}

bool Conflict::isConflicting(const searchGraph_t& G, vertex_t e_from, vertex_t e_to, const list<pair<vertex_t, vertex_t>>& E)
{
	if (e_from == e_to) // wait action
	{
		for (auto vv : E)
		{
			if (vv.first == e_from && vv.second == e_to)
				continue;
			bool blocked = false;
			for (auto e : G[e_from].generalizedVertexEdgeConflicts)
				if ((e.m_source == vv.first && e.m_target == vv.second) || (e.m_source == vv.second && e.m_target == vv.first))
				{
					blocked = true;
					break;
				}
			if (!blocked)
				return false;
		}
	}
	else // move action
	{
		edge_t e = (boost::edge(e_from, e_to, G)).first;
		for (auto vv : E)
		{
			if ((vv.first == e_from && vv.second == e_to) || (vv.first == e_to && vv.second == e_from))
				continue;
			bool blocked = false;
			if (vv.first == vv.second)
			{
				for (auto v : G[e].generalizedEdgeVertexConflicts)
				{
					if (vv.first == v)
					{
						blocked = true;
						break;
					}
				}
			}
			else
			{
				for (auto e2 : G[e].generalizedEdgeConflicts)
				{
					if ((e2.m_source == vv.first && e2.m_target == vv.second) || (e2.m_source == vv.second && e2.m_target == vv.first))
					{
						blocked = true;
						break;
					}
				}
			}
			if (!blocked)
				return false;

		}
	}
	return true;
}

int Conflict::getReward(const list<vertex_t> &V, const list<MDDNode*>& mdd, int lookahead)
{
	int r = lookahead + 1;
	for (auto node : mdd)
	{
		if (node->cost >= r)
			continue;
		bool blocked = false;
		for (auto v : V)
		{
			if (node->vertex == v)
			{
				blocked = true;
				break;
			}
		}
		if (!blocked)
			r = min(r, node->cost);
	}
	return r;
}

int Conflict::getReward(const list<pair<vertex_t, vertex_t>> &E, const list<MDDNode*>& mdd, int lookahead)
{
	int r = lookahead + 1;
	for (auto node : mdd)
	{
		if (node->cost >= r)
			continue;
		for (auto parent : node->parents)
		{
			bool blocked = false;
			for (auto e : E)
			{
				if ((node->vertex == e.first && parent->vertex == e.second) || (node->vertex == e.second && parent->vertex == e.first))
				{
					blocked = true;
					break;
				}
			}
			if (!blocked)
			{
				r = min(r, node->cost);
				break;
			}
		}
	}
	return r;
}

int Conflict::getReward(const list<tuple<vertex_t, vertex_t, int>>& cons, const list<MDDNode*>& level, int lookahead)
{
	list<MDDNode> copy;
	for (auto node : level)
		copy.push_back(MDDNode(*node));
	for (auto con : cons)
	{
		if (get<1>(con) == SIZE_MAX) // vertex constraint
		{
			for (list<MDDNode>::iterator it = copy.begin(); it != copy.end(); it++)
				if (it->vertex == get<0>(con))
				{
					copy.erase(it);
					break;
				}
		}
		else // edge constraint
		{
			for (list<MDDNode>::iterator it = copy.begin(); it != copy.end(); it++)
			{
				if (it->vertex == get<1>(con))
				{
					for (list<MDDNode*>::iterator it2 = it->parents.begin(); it2 != it->parents.end(); it2++)
					{
						if ((*it2)->vertex == get<0>(con))
						{
							it->parents.erase(it2);
							if (it->parents.empty())
								copy.erase(it);
							break;
						}
					}
					break;
				}
			}
		}
		if (copy.empty())
			return lookahead + 1;
	}
	int r = lookahead + 1;
	for (auto node : copy)
	{
		r = min(r, node.cost);
	}
	return r;
}

void Conflict::ComputeMaxRewardConstraints(const searchGraph_t& G, const list<MDDNode*>& mdd1, 
	const list<MDDNode*>& mdd2, int lookahead)
{
	if (vertexConflict)
	{
		list<vertex_t> V1_bestsofar, V2_bestsofar;
		V1_bestsofar.push_back(v1);
		V2_bestsofar.push_back(v1);
		for (auto v : G[v1].generalizedVertexConflicts)
			V2_bestsofar.push_back(v);
		int r1_bestsofar = 0;
		int r2_bestsofar = getReward(V2_bestsofar, mdd2, lookahead);
		for (int r1 = 1; r1 <= lookahead + 1; r1++)
		{
			list<vertex_t> V1, V2;
			V2.push_back(v2);
			bool exist = true;
			for (auto node : mdd1)
			{
				if (node->cost < r1)
				{
					if (isConflicting(G, node->vertex, V2))
					{
						V1.push_back(node->vertex);
					}
					else
					{
						exist = false;
						break;
					}
				}
			}
			if (!exist)
				break;
			for (auto v : G[v1].generalizedVertexConflicts)
			{
				if (isConflicting(G, v, V1))
					V2.push_back(v);
			}
			if (V2.empty())
				break;
			int r2 = getReward(V2, mdd2, lookahead);

			if (min(r1, r2) > min(r1_bestsofar, r2_bestsofar) ||
				(min(r1, r2) == min(r1_bestsofar, r2_bestsofar) && r1 + r2 > r1_bestsofar + r2_bestsofar) ||
				(r1 + r2 == r1_bestsofar + r2_bestsofar && min(r1, r2) == min(r1_bestsofar, r2_bestsofar) && V1.size() + V2.size() > V1_bestsofar.size() + V2_bestsofar.size()))
			{
				V1_bestsofar = V1;
				V2_bestsofar = V2;
				r1_bestsofar = r1;
				r2_bestsofar = r2;
			}

		}
		for (auto v : V1_bestsofar)
		{
			con1.push_back(make_tuple(v, SIZE_MAX, timestep));
		}
		for (auto v : V2_bestsofar)
		{
			con2.push_back(make_tuple(v, SIZE_MAX, timestep));
		}
		cost1 = r1_bestsofar;
		cost2 = r2_bestsofar;
	}
	else
	{
		list<pair<vertex_t, vertex_t>> E1_bestsofar, E2_bestsofar;
		E1_bestsofar.push_back(make_pair(v1_from, v1));
		E2_bestsofar.push_back(make_pair(v1_from, v1));
		edge_t e1 = (boost::edge(v1_from, v1, G)).first;
		if (v1_from == v1) //wait action
			for (auto e : G[v1].generalizedVertexEdgeConflicts)
				E2_bestsofar.push_back(make_pair(e.m_source, e.m_target));
		else // move action
		{
			for (auto e : G[e1].generalizedEdgeConflicts)
				E2_bestsofar.push_back(make_pair(e.m_source, e.m_target));
			for (auto v : G[e1].generalizedEdgeVertexConflicts)
				E2_bestsofar.push_back(make_pair(v, v));
		}
		int r1_bestsofar = 0;
		int r2_bestsofar = getReward(E2_bestsofar, mdd2, lookahead);
		for (int r1 = 1; r1 <= lookahead + 1; r1++)
		{
			list<pair<vertex_t, vertex_t>> E1, E2;
			E2.push_back(make_pair(v2_from, v2));
			bool exist = true;
			for (auto node : mdd1)
			{
				if (node->cost < r1)
				{
					for (auto parent : node->parents)
					{
						if (isConflicting(G, node->vertex, parent->vertex, E2))
							E1.push_back(make_pair(node->vertex, parent->vertex));
						else
						{
							exist = false;
							break;
						}
					}
					if (!exist)
						break;
				}
			}
			if (!exist)
				break;
			if (v1_from == v1) //wait action
			{
				for (auto e : G[v1].generalizedVertexEdgeConflicts)
				{
					if (isConflicting(G, e.m_source, e.m_target, E1))
						E2.push_back(make_pair(e.m_source, e.m_target));
				}
			}
			else
			{
				for (auto e : G[e1].generalizedEdgeConflicts)
				{
					if (isConflicting(G, e.m_source, e.m_target, E1))
						E2.push_back(make_pair(e.m_source, e.m_target));
				}
				for (auto v : G[e1].generalizedEdgeVertexConflicts)
				{
					if (isConflicting(G, v, v, E1))
						E2.push_back(make_pair(v, v));
				}
			}

			if (E2.empty())
				break;
			int r2 = getReward(E2, mdd2, lookahead);
			
			if (min(r1, r2) > min(r1_bestsofar, r2_bestsofar) ||
				(min(r1, r2) == min(r1_bestsofar, r2_bestsofar) && r1 + r2 > r1_bestsofar + r2_bestsofar))
			{
				E1_bestsofar = E1;
				E2_bestsofar = E2;
				r1_bestsofar = r1;
				r2_bestsofar = r2;
			}

		}
		for (auto e : E1_bestsofar)
		{
			con1.push_back(make_tuple(e.first, e.second, timestep));
		}
		for (auto e : E2_bestsofar)
		{
			con2.push_back(make_tuple(e.first, e.second, timestep));
		}
		cost1 = r1_bestsofar;
		cost2 = r2_bestsofar;
	}
}