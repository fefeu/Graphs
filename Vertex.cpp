#include "Vertex.hpp"

using namespace std;

Vertex::Vertex(const string &name) {
  this->name = name;
}

// Adds an edge to the to-vertex with given cost and length.
bool Vertex::addEdge(Vertex *to, unsigned int cost, unsigned int length) {
  unordered_map<string,Edge>::iterator it = edges.find (to->getName());

  // If there isn't already an edge to the vertex.
  if(it == edges.end()) {
    Edge newEdge(this, to, cost, length);
    pair<string, Edge> newPair(to->getName(), newEdge);
    edges.insert(newPair);
  } 
  // If the edge is already there, update its cost and length.
  else {
    it->second.setCost(cost);
    it->second.setLength(length);
  }

  return true;
}

const string& Vertex::getName() const {
  return name;
}

unsigned int Vertex::getDistance() const {
  return distance;
}

void Vertex::setDistance(unsigned int distance) {
  this->distance = distance;
}

bool Vertex::wasVisited() const {
  return visited;
}

void Vertex::setVisited(bool visited) {
  this->visited = visited;
}

void Vertex::clearEdges() { 
  edges.clear();
}

unsigned int Vertex::totalEdgeCost() const {
  unsigned int sum = 0;
  for(auto it = edges.begin(); it != edges.end(); it++) {
    sum += it->second.getCost();
  }
  return sum;
}

const unordered_map<string, Edge>& Vertex::getEdges() const {
  return edges;
}
// Method implementations here
