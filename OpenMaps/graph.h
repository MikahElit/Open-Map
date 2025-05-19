//  @file graph.h
//  @author Muhammad Hakim
//  @date CS 251, Spring 2023.
//  @brief Project 6 - Open Map, University of Illinois Chicago
//  @description - The project involves implementing Dijkstra's algorithm to find the shortest weighted path in a graph that is constructed from a map data file.
//  The map data file contains information about the vertices and edges of the graph, including their weights.
//  The program reads the map data file, constructs the graph, and then applies Dijkstra's algorithm to find the shortest path between two specified vertices 
//
// Adam T Koehler, PhD
// University of Illinois Chicago
// CS 251, Spring 2023
//
// Project Original Variartion By:
// Joe Hummel, PhD
// University of Illinois at Chicago
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <vector>
#include <set>
#include <unordered_map>

using namespace std;

template<typename VertexT, typename WeightT>
class graph {
 private:
     unordered_map<VertexT, unordered_map<VertexT, WeightT>> adjList;

     // helper function to check if vertex exists in the graph
     bool vertexExists(VertexT v) const {
         return adjList.count(v) != 0;
     };


public:
    //
    // default constructors
    //
    graph() {}

    void clear() {
        adjList.clear();
    }

    graph& operator=(const graph& other) {
        adjList = other.adjList;
    }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
      return static_cast<int>(adjList.size());
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
      int count = 0;
      for (auto& adj : adjList) {
          count += adj.second.size();
      }
      return count;
  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
      if (vertexExists(v)) {
          return false; // Vertex already exists in the graph
      }
      unordered_map<VertexT, WeightT> map;
      adjList[v] = map;
      return true;
  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
      if (!vertexExists(from) || !vertexExists(to)) {
          return false;
      }
      unordered_map<VertexT, WeightT> map = adjList[from];
      map[to] = weight;
      adjList[from] = map;
      return true;
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
      if (!vertexExists(from)) {
          return false;
      }
      const auto& map = adjList.at(from);
      const auto iter = map.find(to);
      if (iter == map.end()) {
          return false;
      }
      weight = iter->second;
      return true;
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {
      set<VertexT> neigh;
      auto iter = adjList.find(v);
      if (iter != adjList.end()) {
          for (const auto& neighbor : iter->second) {
              neigh.insert(neighbor.first);
          }
      }
      return neigh;
  }


  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {
      vector<VertexT> vertices;
      for (auto& adj : adjList) {
          vertices.push_back(adj.first);
      }

      return vertices;
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;
    int count = 0;
    for (const auto& vertex : adjList) {
        output << " " << count << ". " << vertex.first << endl;
        count++;
    }

    output << endl;
    output << "**Edges:" << endl;

    for (const auto& vertex : adjList) {
        output << " " << vertex.first << ": ";
        for (const auto& neighbor : vertex.second) {
            output << "(" << vertex.first << "," << neighbor.first << "," << neighbor.second << ") ";
        }
        output << endl;
    }
    output << "**************************************************" << endl;
  }
};
