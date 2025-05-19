//  @file application.cpp
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
// 
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <iostream>
#include <iomanip>  /*setprecision*/
#include <string>
#include <vector>
#include <map>
#include <cstring>
#include <cassert>
#include <map>
#include <vector>
#include <set>
#include <stack>
#include <queue>
#include <limits>
#include "tinyxml2.h"
#include "dist.h"
#include "graph.h"
#include "osm.h"
#include <fstream>


using namespace std;
using namespace tinyxml2;
double INF = numeric_limits<double>::max(); // infinity

// looks if a given building name exists in the buildings vector
// returns false if it does not exists and trur if it does.
bool findBuilding(vector<BuildingInfo>& Buildings, BuildingInfo& Buildings1,  string buildingName) {
    for (auto i : Buildings) {
        if (i.Abbrev == buildingName) {
            Buildings1 = i;
            return true;
        }
        else if (i.Fullname.find(buildingName) != string::npos) {
            Buildings1 = i;
            return true;
        }
    }

    return false;
}
void printBuildingInfo(const BuildingInfo& building) {
    cout << " " << building.Fullname << endl;
    cout << " (" << building.Coords.Lat << ", ";
    cout << building.Coords.Lon << ")" << endl;
}

vector<long long> getPath(long long source, long long dest, const map<long long, double>& predV) {
    vector<long long> path;
    long long i = dest;
    while (i != source) {
        path.push_back(i);
        i = predV.at(i);
    }
    path.push_back(source);
    reverse(path.begin(), path.end());
    return path;
}


vector<long long> dijkstra(graph<long long, double>& G, long long start_vertex,
    map<long long, double>& distances, map<long long, double>& predecessors);


//
// Implement your standard application here
//
void application(map<long long, Coordinates>& Nodes, vector<FootwayInfo>& Footways, vector<BuildingInfo>& Buildings, graph<long long, double>& G) {
    string person1Building, person2Building;
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);

    while (person1Building != "#") {
        cout << "Enter person 2's building (partial name or abbreviation)> ";
        getline(cin, person2Building);


        BuildingInfo sbuild, ebuild, fbuild;
        bool sbuildFound = false;
        bool ebuildFound = false;

        sbuildFound = findBuilding(Buildings, sbuild, person1Building);

        if (sbuildFound == true) {
            ebuildFound = findBuilding(Buildings, ebuild, person2Building);
        }
        if (sbuildFound == false) {
            cout << "Person 1's building not found" << endl;
        }
        else if (ebuildFound == false) {
            cout << "Person 2's building not found" << endl;
        }
        if (sbuildFound && ebuildFound) {
            cout << endl;
            // get the midpoint between the two buildings
            Coordinates midpoint = centerBetween2Points(sbuild.Coords.Lat, sbuild.Coords.Lon, ebuild.Coords.Lat, ebuild.Coords.Lon);
            // check the building closest to the midpoint
            double min = INF;
            double MinDistFromMid;
            for (auto i : Buildings) {
                MinDistFromMid = distBetween2Points(midpoint.Lat, midpoint.Lon, i.Coords.Lat, i.Coords.Lon);
                if (MinDistFromMid < min) {
                    min = MinDistFromMid;
                    fbuild = i;
                }
            }
            cout << "Person 1's point:" << endl;
            printBuildingInfo(sbuild);
            cout << "Person 2's point:" << endl;
            printBuildingInfo(ebuild);
            cout << "Destination Building:" << endl;
            printBuildingInfo(fbuild);



            // Define a pair to hold the latitude and longitude of the nearest node.
            pair<double, double> nearest;

            // Define maps to hold the distances between the start building, end building,
            // and destination building to the nearest node.
            map<double, long long> slength, elength, fdist;

            // Define variables to hold the distances between the start building, end building,
            // and destination building to the nearest node.
            double p1_nearest_dist, p2_nearest_dist, dest_nearest_dist;

            // Loop through all the footways.
            for (auto i : Footways) {
                // Loop through all the nodes of the current footway.
                for (auto j : i.Nodes) {
                    // Set the latitude and longitude of the current node as the nearest.
                    nearest.first = Nodes[j].Lat;
                    nearest.second = Nodes[j].Lon;
                    // Calculate the distance between the start building and the nearest node.
                    p1_nearest_dist = distBetween2Points(sbuild.Coords.Lat,
                        sbuild.Coords.Lon, nearest.first, nearest.second);
                    // Add the distance to the map.
                    slength[p1_nearest_dist] = j;

                    // Calculate the distance between the end building and the nearest node.
                    p2_nearest_dist = distBetween2Points(ebuild.Coords.Lat,
                        ebuild.Coords.Lon, nearest.first, nearest.second);
                    // Add the distance to the map.
                    elength[p2_nearest_dist] = j;

                    // Calculate the distance between the destination building and the nearest node.
                    dest_nearest_dist = distBetween2Points(fbuild.Coords.Lat,
                        fbuild.Coords.Lon, nearest.first, nearest.second);
                    // Add the distance to the map.
                    fdist[dest_nearest_dist] = j;
                }
            }




            // Create maps for distances and predecessors, vectors for visited nodes, and stacks for paths
            map<long long, double> distances1, distances2, prev1, prev2;
            vector<long long> p1visited, p2visited;
            stack<long long> path1, path2;

            // Find nearest nodes to starting points and destination
            long long p1ID = slength.begin()->second, p2ID = elength.begin()->second, destID = fdist.begin()->second;
            pair<double, double> p1Nearest{ Nodes[p1ID].Lat, Nodes[p1ID].Lon }, p2Nearest{ Nodes[p2ID].Lat, Nodes[p2ID].Lon }, destNearest{ Nodes[destID].Lat, Nodes[destID].Lon };

            // Print out information about nearest nodes
            cout << "Nearest P1 node:\n " << p1ID << "\n (" << p1Nearest.first << ", " << p1Nearest.second << ")\n";
            cout << "Nearest P2 node:\n " << p2ID << "\n (" << p2Nearest.first << ", " << p2Nearest.second << ")\n";
            cout << "Nearest destination node:\n " << destID << "\n (" << destNearest.first << ", " << destNearest.second << ")\n\n";



            p1visited = dijkstra(G, p1ID, distances1, prev1);
            p2visited = dijkstra(G, p2ID, distances2, prev2);

            if (distances1[p2ID] >= INF) {
                cout << "Sorry, destination unreachable." << endl;
            }
            else if (distances1[destID] >= INF || distances2[destID] >= INF) {
                set<string> unreachableB{ fbuild.Fullname };
                while (true) {
                    double minDist = INF;
                    for (const auto& i : Buildings) {
                        double distFromMid = distBetween2Points(midpoint.Lat, midpoint.Lon, i.Coords.Lat, i.Coords.Lon);
                        if (distFromMid < minDist && unreachableB.count(i.Fullname) == 0) {
                            minDist = distFromMid;
                            fbuild = i;
                        }
                    }
                    map<double, long long> newDestDist;
                    for (const auto& i : Footways) {
                        for (const auto& j : i.Nodes) {
                            double destDist = distBetween2Points(fbuild.Coords.Lat, fbuild.Coords.Lon, Nodes[j].Lat, Nodes[j].Lon);
                            newDestDist[destDist] = j;
                        }
                    }
                    destID = newDestDist.begin()->second;
                    destNearest = { Nodes[destID].Lat, Nodes[destID].Lon };
                    if (distances1[destID] != INF && distances2[destID] != INF) {
                        break;
                    }
                    else {
                        unreachableB.insert(fbuild.Fullname);
                        cout << "At least one person was unable to reach the destination building. Finding next closest building..." << endl;
                    }
                }
            }
            if (distances1[destID] != INF) {
                cout << "Person 1's distance to dest: " << distances1[destID] << " miles" << endl;
                cout << "Path: ";
                vector<long long> path = getPath(p1ID, destID, prev1);
                for (auto id : path) {
                    cout << id << "->";
                }
                cout << endl;
            }

            if (distances2[destID] != INF) {
                cout << "Person 2's distance to dest: " << distances2[destID] << " miles" << endl;
                cout << "Path: ";
                vector<long long> path = getPath(p2ID, destID, prev2);
                for (auto id : path) {
                    cout << id << "->";
                }
                cout << endl;
            }
        }


        cout << endl;
        cout << "Enter person 1's building (partial name or abbreviation), or #> ";
        getline(cin, person1Building);
    }
}

int main() {

  // maps a Node ID to it's coordinates (lat, lon)
  map<long long, Coordinates>  Nodes;
  // info about each footway, in no particular order
  vector<FootwayInfo>          Footways;
  // info about each building, in no particular order
  vector<BuildingInfo>         Buildings;
  XMLDocument                  xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;


  graph<long long, double> G;

  // add vertex from Nodes to graph
  for (auto& e : Nodes) {
      G.addVertex(e.first);
  }

  double distance;
  pair<double, double> c1, c2;

  // add edges for each vertex
  for (auto i : Footways) {
      for (unsigned int j = 0; j < i.Nodes.size() - 1; j++) {
          c1.first = Nodes[i.Nodes[j]].Lat;
          c2.first = Nodes[i.Nodes[j + 1]].Lat;
          c1.second = Nodes[i.Nodes[j]].Lon;
          c2.second = Nodes[i.Nodes[j + 1]].Lon;
          distance = distBetween2Points(c1.first, c1.second, c2.first, c2.second);
          G.addEdge(i.Nodes[j], i.Nodes[j + 1], distance);
          G.addEdge(i.Nodes[j + 1], i.Nodes[j], distance);
      }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  // Execute Application
  application(Nodes, Footways, Buildings, G);

  //
  // done:
  //
  cout << "** Done **" << endl;
  return 0;
}



// Dijkstra's algorithm to find the shortest path between vertices in a graph
// Parameters:
// G: graph object
// start_vertex: the vertex to start the search from
// distances: a map to keep track of the distance from start_vertex to each vertex
// predecessors: a map to keep track of the previous vertex visited to reach each vertex
// Returns:
// a vector of visited vertices in the order they were visited
vector<long long> dijkstra(graph<long long, double>& G, long long start_vertex,
    map<long long, double>& distances, map<long long, double>& predecessors) {

    class ShortestDistanceComparator {
    public:
        bool operator()(const pair<long long, double>& p1,
            const pair<long long, double>& p2) const {
            // Compare by distance
            if (p1.second != p2.second) {
                return p1.second > p2.second;
            }
            // If distances are equal, break ties by vertex ID
            return p1.first > p2.first;
        }
    };



    // Priority queue of unvisited vertices, sorted by their distances from the start vertex
    priority_queue<pair<long long, double>, vector<pair<long long, double>>, ShortestDistanceComparator> unvisited_vertices;

    // Set all distances to infinity except for start_vertex, which is set to 0
    for (auto vertex : G.getVertices()) {
        distances[vertex] = INF;
    }
    distances[start_vertex] = 0;

    // Set of visited vertices
    set<long long> visited_vertices;

    // Vector of visited vertices in the order they were visited
    vector<long long> visited_vertices_order;

    // Add the start vertex to the queue
    unvisited_vertices.push(make_pair(start_vertex, 0));

    // While there are unvisited vertices in the queue
    while (!unvisited_vertices.empty()) {

        // Get the vertex with the shortest distance from the queue
        long long current_vertex = unvisited_vertices.top().first;
        unvisited_vertices.pop();

        // If the current vertex has already been visited, continue to the next vertex
        if (visited_vertices.count(current_vertex) == 1) {
            continue;
        }

        // Mark the current vertex as visited
        visited_vertices.insert(current_vertex);
        visited_vertices_order.push_back(current_vertex);

        // Get the set of neighbors of the current vertex
        set<long long> neighbors = G.neighbors(current_vertex);

        // For each neighbor of the current vertex
        for (auto neighbor : neighbors) {
            double edge_weight;
            G.getWeight(current_vertex, neighbor, edge_weight);

            // Calculate the distance to the neighbor through the current vertex
            double distance_through_current_vertex = distances[current_vertex] + edge_weight;

            // If the new distance to the neighbor is less than the current distance, update the distance
            if (distance_through_current_vertex < distances[neighbor]) {
                distances[neighbor] = distance_through_current_vertex;
                predecessors[neighbor] = current_vertex;

                // Add the neighbor to the queue with its updated distance
                unvisited_vertices.push(make_pair(neighbor, distances[neighbor]));
            }
        }
    }

    // Return the vector of visited vertices in the order they were visited
    return visited_vertices_order;
}
