#include "map_loader.h"
#include <iostream>
#include <fstream>
#include<boost/tokenizer.hpp>

// load map from file
// first line is the number of rows and the number of cols
// "." represents empty cell
// other chars represent blocked cells
MapLoader::MapLoader(std::string fname)
{
	std::string line;
	std::ifstream myfile (fname.c_str());
	if (myfile.is_open()) 
	{
		getline (myfile,line);
		boost::char_separator<char> sep(",");
		boost::tokenizer< boost::char_separator<char> > tok(line, sep);
		boost::tokenizer< boost::char_separator<char> >::iterator beg=tok.begin();
		rows = atoi ( (*beg).c_str() ); // read number of rows
		beg++;
		cols = atoi ( (*beg).c_str() ); // read number of cols
		my_map= new bool[rows*cols];
		for (int i=0; i<rows*cols; i++)
		my_map[i] = false;
		// read map (and start/goal locations)
		for (int i=0; i<rows; i++) 
		{
			getline (myfile, line);
			for (int j=0; j<cols; j++) 
				my_map[cols*i + j] = (line[j] != '.');
		}
		myfile.close();

		// initialize moves_offset array
		moves_offset = new int[5];
		moves_offset = new int[MapLoader::MOVE_COUNT];
		moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
		moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
		moves_offset[MapLoader::valid_moves_t::EAST] = 1;
		moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
		moves_offset[MapLoader::valid_moves_t::WEST] = -1;
	}
	else
	{
		std::cerr << "Map file \"" << fname << "\" not found." << std::endl;
		exit(10);
	}
}


bool* MapLoader::get_map() const 
{
	  bool* retVal = new bool [ this->rows * this->cols ];
	  memcpy (retVal, this->my_map, sizeof(bool)* this->rows * this->cols );
	  return retVal;
}

MapLoader::~MapLoader() 
{
	  delete[] this->my_map;
	  delete[] this->moves_offset;
}

