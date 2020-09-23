#include <boost/heap/fibonacci_heap.hpp>
#include "compute_heuristic.h"
#include <google/dense_hash_map>
#include "node.h"

bool ComputeHeuristic::validMove(int curr, int next) const
{
	if (next < 0 && next >= map_rows * map_cols)
		return false;
	int curr_x = curr / map_cols;
	int curr_y = curr % map_cols;
	int next_x = next / map_cols;
	int next_y = next % map_cols;
	return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}


int* ComputeHeuristic::getShortestPathVals(int root_location)
{
	int* res = new int[map_rows * map_cols];
	for (int i = 0; i < map_rows * map_cols; i++)
		res[i] = H_MAX;
	// generate a heap that can save nodes (and a open_handle) and a hash_map
	boost::heap::fibonacci_heap< Node*, boost::heap::compare<Node::compare_node> > heap;
	google::dense_hash_map<Node*, boost::heap::fibonacci_heap<Node*, boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode> nodes;
	nodes.set_empty_key(NULL);
	google::dense_hash_map<Node*, boost::heap::fibonacci_heap<Node*, boost::heap::compare<Node::compare_node> >::handle_type, Node::NodeHasher, Node::eqnode>::iterator it; // will be used for find()

	Node* root = new Node(root_location, 0, 0, NULL, 0, false);
	root->open_handle = heap.push(root);  // add root to heap
	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	while (!heap.empty()) 
	{
		Node* curr = heap.top();
		heap.pop();
		for (int direction = 0; direction < 5; direction++) 
		{
			int next_loc = curr->loc + moves_offset[direction];
			bool unblocked = true;
			if (validMove(curr->loc, next_loc))
			{
				for (int i = 0; i < agent_size && unblocked; i++)
				{
					for (int j = 0; j < agent_size && unblocked; j++)
					{
						if (my_map[next_loc + i * map_cols + j])
							unblocked = false;
					}
				}
				if (unblocked)   // if the body of the agent does not overlap any blocked cells
				{
					// compute cost to next_loc via curr node
					int next_g_val = curr->g_val + 1;
					Node* next = new Node(next_loc, next_g_val, 0, NULL, 0, false);
					it = nodes.find(next);
					if (it == nodes.end())  // add the newly generated node to heap and hash table
					{ 
						next->open_handle = heap.push(next);
						nodes[next] = next->open_handle;
					}
					else // update existing node's g_val if needed (only in the heap)
					{  
						delete(next);  // not needed anymore -- we already generated it before
						Node* existing_next = (*it).first;
						if (existing_next->g_val > next_g_val) 
						{
							existing_next->g_val = next_g_val;
							heap.update((*it).second);
						}
					}
				}
			}
		}
	}
	// iterate over all nodes and populate the shortest distance
	for (it = nodes.begin(); it != nodes.end(); it++) {
		Node* s = (*it).first;
		res[s->loc] = s->g_val;
		delete (s);
	}
	nodes.clear();
	heap.clear();
	return res;
}

ComputeHeuristic::~ComputeHeuristic() 
{
  delete[] my_map;
}
