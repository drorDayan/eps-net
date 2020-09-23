#include "conflict.h"
#include <utility>
#include <vector>
#include <list>

Conflict::Conflict(const Conflict& cpy)
{
	timestep = cpy.timestep;
	a1 = cpy.a1;
	a2 = cpy.a2;
	loc1 = cpy.loc1;
	loc2 = cpy.loc2;
	weight1 = cpy.weight1;
	weight2 = cpy.weight2;
	type = cpy.type;
	cons1[0] = cpy.cons1[0];
	cons1[1] = cpy.cons1[1];
	cons2[0] = cpy.cons2[0];
	cons2[1] = cpy.cons2[1];
}

void Conflict::swap() // swap indices of a1 and a2
{
	int temp = a1; a1 = a2; a2 = temp;
	temp = loc1; loc1 = loc2; loc2 = temp;
	temp = weight1; weight1 = weight2; weight2 = temp;
	temp = cons1[0]; cons1[0] = cons2[0]; cons2[0] = temp;
	temp = cons1[1]; cons1[1] = cons2[1]; cons2[1] = temp;
}

void Conflict::ComputeAsymConstraints(int num_col, int num_row, int size1, int size2) // compute asymmetric constraint
{
	cons1[0] = loc1;
	cons1[1] = loc1;
	int min_x = loc1 / num_col - (size2 - 1);
	int min_y = loc1 % num_col - (size2 - 1);
	int max_x = loc1 / num_col + size1 - 1;
	int max_y = max_y = loc1 % num_col + size1 - 1;
	min_x = min_x > 0 ? min_x : 0;
	min_y = min_y > 0 ? min_y : 0;
	max_x = max_x < num_row - 1 ? max_x : num_row - 1;
	max_y = max_y < num_col - 1 ? max_y : num_col - 1;
	cons2[0] = min_x * num_col + min_y;
	cons2[1] = max_x * num_col + max_y;
}

void Conflict::ComputeSymConstraints(int num_col, int num_row, int s1, int s2)
{
	int x1 = loc1 / num_col;
	int x2 = loc2 / num_col;
	int y1 = loc1 % num_col;
	int y2 = loc2 % num_col;
	int i = x1 > x2 ? x1 : x2;
	int j = x1 + s1 - 1 < x2 + s2 - 1 ? x1 + s1 - 1 : x2 + s2 - 1;
	int x0 = (i + j) / 2;
	i = y1 > y2 ? y1 : y2;
	j = y1 + s1 - 1 < y2 + s2 - 1 ? y1 + s1 - 1 : y2 + s2 - 1;
	int y0 = (i + j) / 2;
	int min_x1 = 0, min_y1 = 0, max_x1 = x0, max_y1 = y0;
	int min_x2 = 0, min_y2 = 0, max_x2 = x0, max_y2 = y0;
	if (x0 > s1 - 1)
		min_x1 = x0 - (s1 - 1);
	if (x0 > s2 - 1)
		min_x2 = x0 - (s2 - 1);
	if (y0 > s1 - 1)
		min_y1 = y0 - (s1 - 1);
	if (y0 > s2 - 1)
		min_y2 = y0 - (s2 - 1);
	cons1[0] = min_x1 * num_col + min_y1;
	cons1[1] = max_x1 * num_col + max_y1;
	cons2[0] = min_x2 * num_col + min_y2;
	cons2[1] = max_x2 * num_col + max_y2;
}

bool Conflict::disjunctive(const std::pair<int, int>& min1, const std::pair<int, int>& max1,
	const std::pair<int, int>& min2, const std::pair<int, int>& max2, int size1, int size2) const
{
	return max1.first - min2.first < size2 && max2.first - min1.first < size1 &&
		max1.second - min2.second < size2 && max2.second - min1.second < size1;
}

void Conflict::getMinimalRectangle(std::vector<int>& mdd_x_min, std::vector<int>& mdd_x_max,
	std::vector<int>& mdd_y_min, std::vector<int>& mdd_y_max, 
	int lookahead, const std::list<MDDNode*>& MDDNodes, int num_col) const
{
	// the rectangle should first cover MDD nodes with the same weight
	for (std::list<MDDNode*>::const_iterator it = MDDNodes.begin(); it != MDDNodes.end(); ++it)
	{
		mdd_x_min[(*it)->cost] = std::min(mdd_x_min[(*it)->cost], (*it)->location / num_col);
		mdd_x_max[(*it)->cost] = std::max(mdd_x_max[(*it)->cost], (*it)->location / num_col);
		mdd_y_min[(*it)->cost] = std::min(mdd_y_min[(*it)->cost], (*it)->location % num_col);
		mdd_y_max[(*it)->cost] = std::max(mdd_y_max[(*it)->cost], (*it)->location % num_col);
	}
	// the rectangle should also cover the rectangle with smaller weight
	for (int i = 0; i < lookahead; i++)
	{
		mdd_x_min[i + 1] = std::min(mdd_x_min[i], mdd_x_min[i + 1]);
		mdd_x_max[i + 1] = std::max(mdd_x_max[i], mdd_x_max[i + 1]);
		mdd_y_min[i + 1] = std::min(mdd_y_min[i], mdd_y_min[i + 1]);
		mdd_y_max[i + 1] = std::max(mdd_y_max[i], mdd_y_max[i + 1]);
	}
}

void Conflict::computeMaximalDisjunctiveConstraint2(const std::pair<int, int>& min1, const std::pair<int, int>& max1, 
	int size1, int size2, int num_col, int num_row)
{
	cons1[0] = min1.first * num_col + min1.second;
	cons1[1] = max1.first * num_col + max1.second;
	int min_x2 = max1.first - size2 + 1;
	int max_x2 = min1.first + size1 - 1;
	int min_y2 = max1.second - size2 + 1;
	int max_y2 = min1.second + size1 - 1;
	min_x2 = min_x2 > 0 ? min_x2 : 0;
	min_y2 = min_y2 > 0 ? min_y2 : 0;
	max_x2 = max_x2 < num_row - 1 ? max_x2 : num_row - 1;
	max_y2 = max_y2 < num_col - 1 ? max_y2 : num_col - 1;
	cons2[0] = min_x2 * num_col + min_y2;
	cons2[1] = max_x2 * num_col + max_y2;
}

bool Conflict::hasBetterWeight(int w1, int w2) const
{
	return std::min(weight1,weight2) > std::min(w1, w2) ||
		(std::min(weight1, weight2) == std::min(w1, w2) && weight1 + weight2 > w1 + w2);
}

void Conflict::computeConstraintsForEdgeConflict(const std::vector<std::list<MDDNode*>>& mdd1,
	const std::vector<std::list<MDDNode*>>& mdd2)
{
	cons1[0] = loc1;
	cons1[1] = loc2;
	cons2[0] = -loc2 - 1;
	cons2[1] = -loc1 - 1;
	weight1 = 0;
	weight2 = 0;
	if (mdd1[timestep].size() == 1 && mdd1[timestep + 1].size() == 1) // The only edge for a1
	{
		weight1++;
	}
	if (mdd2[timestep].size() == 1 && mdd2[timestep + 1].size() == 1) // The only edge for a2
	{
		weight2++;
	}
	if (weight1 > 0 && weight2 > 0)
	{
		type = conflict_type::CARDINAL;
	}
	else if (weight1 > 0)
	{
		type = conflict_type::SEMI;
	}
	else if (weight2 > 0)
	{
		type = conflict_type::SEMI;
		swap();
	}
	else
	{
		type = conflict_type::NON;
	}
}