#pragma once

class ComputeHeuristic 
{
public:
	ComputeHeuristic(int start_location, int goal_location, int size, 
		const bool* my_map, int map_rows,int map_cols, const int* moves_offset):
		my_map(my_map), map_rows(map_rows), map_cols(map_cols), 
		start_location(start_location), goal_location(goal_location), agent_size(size), moves_offset(moves_offset) {}
  
	int* getHVals() {return getShortestPathVals(goal_location); } // return a (deep) copy of h_vals;
	int* getEstimatedGVals() {return getShortestPathVals(start_location);} // return a lower bound of g_vals;
	~ComputeHeuristic();

private:
	int start_location;
	int goal_location;
	int agent_size;
	const bool* my_map;
	const int* moves_offset;
	int map_rows;
	int map_cols;
	int* getShortestPathVals(int root);
	bool validMove(int curr, int next) const;
};
