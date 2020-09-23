// Load a 2D map.
// First line: ROWS,COLS
// Second line and onward, "." represent unblocked cell (otherwise, blocked)

#pragma once

#include <string>

#define TIME_LIMIT_DEFAULT 3600000
#define H_MAX INT_MAX

class MapLoader 
{
public:
	bool* my_map;
	int rows;
	int cols;
	enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size
	int* moves_offset;
  
	MapLoader(std::string fname); // load map from file
	bool* get_map() const; // return a deep-copy of my_map

	inline bool is_blocked (int row, int col) const { return my_map[row * this->cols + col]; }
	inline bool is_blocked (int loc) const { return my_map[loc]; }
	inline size_t map_size() const { return rows * cols; }	
	inline int linearize_coordinate(int row, int col) const { return ( this->cols * row + col); }
	inline int row_coordinate(int id) const { return id / this->cols; }
	inline int col_coordinate(int id) const { return id % this->cols; }

	~MapLoader();
};

