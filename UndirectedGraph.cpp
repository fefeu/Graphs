#include "UndirectedGraph.hpp"
#include "Edge.hpp"
#include "Vertex.hpp"

#include <vector>
#include <queue>
#include <utility>
#include <iostream>
#include <limits>

using namespace std;

UndirectedGraph::UndirectedGraph() {}

// Delete all vertices from heap.
UndirectedGraph::~UndirectedGraph() {
  for(auto it = vertices.begin(); it != vertices.end(); it++) {
    delete it->second;
  }
}

// Adds an undirected edge between from and to vertices.
void UndirectedGraph::addEdge(const string &from, const string &to,
    unsigned int cost, unsigned int length) {
  Vertex* fromV;
  Vertex* toV;
  unordered_map<string, Vertex*>::const_iterator it = vertices.find(from);

  // If the from vertex doesn't exist in the graph yet.
  if(it == vertices.end()) {
    fromV = new Vertex(from);
    pair<string, Vertex*> vertex (from, fromV);
    
    // Insert this new vertex to the graph.
    vertices.insert(vertex);  
  }

  // We found the from vertex!
  else {
    fromV = it->second;       
  }

  it = vertices.find(to);

  // If the to vertex doesn't exist in the graph yet.
  if(it == vertices.end()) {
    toV = new Vertex(to);
    pair<string, Vertex*> vertex (to, toV);
    
    // Insert this new vertex to the graph.
    vertices.insert(vertex);  
  } 
 
  // We found the to vertex!
  else {
    toV = it->second;         
  }

  // Adds edges from "from" to "to" as well as from "to" to "from"
  fromV->addEdge(toV, cost, length);
  toV->addEdge(fromV, cost, length);
}

// Calculates the total edge cost of the graph.
unsigned int UndirectedGraph::totalEdgeCost() const {
  unsigned int total = 0;

  // Loop through all the vertices and their edges, adding the edge costs.
  for(auto it = vertices.begin(); it != vertices.end(); it++) {
    unordered_map<string, Edge> edges = it->second->edges;
    for(auto itE = edges.begin(); itE != edges.end(); itE++ ) {
      total += itE->second.getCost();
    }
  }

  // Above loop adds an edge's cost twice due to the graph being undirected.
  return total / 2;
}

// Constructs the MST of the graph using Prim's Algorithm.
void UndirectedGraph::minSpanningTree() {
  priority_queue <Edge> pq;

  // Initialize all vertices' visited to false. 
  for(auto it = vertices.begin(); it != vertices.end(); it++) {
    it->second->setVisited(false);
  }

  Vertex* start = vertices.begin()->second;
  start->setVisited(true);

  // Push all adjacent vertices of the start vertex to the priority queue.
  for(auto it = start->edges.begin(); it != start->edges.end(); it++) {
    pq.push(it->second);
  }

  // Construct the MST starting at the start vertex.
  while(!pq.empty()) {
    Edge edge = pq.top();
    pq.pop();

    Vertex* cur = edge.getTo();

    // If the popped edge's to-vertex had already been visited, delete the edge.
    if(cur->wasVisited()) {
      cur->edges.erase(edge.getFrom()->getName());
      edge.getFrom()->edges.erase(cur->getName());
      continue;
    }

    cur->setVisited(true);

    // Push all adjacent vertices to the pq if they had not been visited already.
    for(auto it = cur->edges.begin(); it != cur->edges.end(); it++) {
      if(!it->second.getTo()->wasVisited()) { 
        pq.push(it->second);
      }
    }
  }  
}

/*
 * Calculate the minimum total latency it takes to send packets from "from" 
 * vertex to every other vertex using Dijkstra's Algorithm.
 */
unsigned int UndirectedGraph::totalDistance(const string &from) {
  // Priority queue of pairs of vertices and their scores.
  priority_queue <pair<Vertex*, unsigned int>, vector<pair<Vertex*,unsigned int>>, 
                 DijkstraVertexComparator> pq;

  unsigned int total = 0;   // Total distance to be returned.

  // Initialization: set all vertices' distance to "infinity".
  for(auto it = vertices.begin(); it != vertices.end(); it++) {
    it->second->setDistance(numeric_limits<int>::max());
    it->second->setVisited(false);
  }

  // Starting vertex
  Vertex* start = vertices.find(from)->second;  
  start->setDistance(0);

  pair<Vertex*, unsigned int> startPair(start, 0);
  pq.push(startPair);
  
  // Dijkstra's shortest path algorithm.
  while(!pq.empty()) {
    Vertex* cur = pq.top().first;
    pq.pop();

    // Skips already-visited vertices
    if(cur->wasVisited()) 
      continue;

    /* Found the shortest path to cur */
    cur->setVisited(true);
    total += cur->getDistance();

    // Loop through all edges to determine the shortest path.
    for(auto it = cur->edges.begin(); it != cur->edges.end(); it++) {
      Vertex* neighbor = it->second.getTo();
      
      if(neighbor->wasVisited()) continue;
     
      // Calculate the total distance of taking this path.
      unsigned int distance = cur->getDistance() + it->second.getLength(); 

      // If a shorter path to this vertex is found, update its distance and
      // push it to the priority queue.
      if(distance < neighbor->getDistance()) {
        neighbor->setDistance(distance);
        pair<Vertex*, unsigned int> newPair(neighbor, distance);
        pq.push(newPair);
      }
    }
  }

  // Return max possible distance if the graph is not connected.
  for(auto it = vertices.begin(); it != vertices.end(); it++) {
    if(!it->second->wasVisited()) {
      return numeric_limits<int>::max();
    }
  }

  return total;
}

/*
 * Calculates the minimum total latency it takes to send packets from every
 * vertex to every other vertex.
 */
unsigned int UndirectedGraph::totalDistance() { 
  unsigned int sum = 0;

  // Add the total distance from every vertex to every other vertex. 
  for(auto it = vertices.begin(); it != vertices.end(); it++) {
    unsigned int total = totalDistance(it->first);

    // If the tree is disconnected, return max possible integer.
    if(total == numeric_limits<int>::max()) {
      return total;
    }

    sum += total;
  }

  return sum;
}

/*
 * Comparator function to organize the priority queue used for Dijkstra's
 * Algorithm. Returns true if the left pair's second element is larger
 * than the right pair's.
 */
bool UndirectedGraph::DijkstraVertexComparator::operator()(const pair<Vertex*, unsigned int> left,
                                         const pair<Vertex*, unsigned int> right) {
  return left.second > right.second;
}
