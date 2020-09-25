#pragma once

#include <set>
#include <unordered_map>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <Eigen/Core>


typedef boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::undirectedS > searchGraphTraits_t;
typedef searchGraphTraits_t::vertex_descriptor vertex_t;
typedef searchGraphTraits_t::edge_descriptor edge_t;

#define TIME_LIMIT_DEFAULT 3600000

//DROR: In 2D this 3 should probably be 2
typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> position_t;

//DROR: This class has the name and pos of each vertex
struct Vertex
{
  std::string name;
  position_t pos;
  std::set<vertex_t> generalizedVertexConflicts;
  std::set<edge_t> generalizedVertexEdgeConflicts;
};

struct Edge
{
  std::string name;
  std::set<edge_t> generalizedEdgeConflicts;
  std::set<vertex_t> generalizedEdgeVertexConflicts;
  float length;
  bool isHighway;
};

typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::undirectedS,
        Vertex, Edge>
        searchGraph_t;

void loadSearchGraph(
  searchGraph_t& searchGraph,
  std::unordered_map<std::string, vertex_t>& vNameToV,
  std::unordered_map<std::string, edge_t>& eNameToE,
	std::vector<vertex_t>& starts, std::vector<vertex_t>& goals,
  const std::string& fileName);

void saveSearchGraph(
  const searchGraph_t& searchGraph,
  const std::string& fileName);

void getGraphInfo(const searchGraph_t& G);

// boost graph helpers
// http://stackoverflow.com/questions/13453350/replace-bgl-iterate-over-vertexes-with-pure-c11-alternative

#include <boost/range/iterator_range.hpp>

template<class It>
boost::iterator_range<It> pair_range(std::pair<It, It> const& p){
  return boost::make_iterator_range(p.first, p.second);
}