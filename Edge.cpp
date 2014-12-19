#include "Edge.hpp"

Edge::Edge(Vertex *from, Vertex *to, unsigned int cost, unsigned int length) {
  this->from = from;
  this->to = to;
  this->cost = cost;
  this->length = length;
}

Vertex* Edge::getFrom() const {
  return from;
}

Vertex* Edge::getTo() const {
  return to;
}

void Edge::setCost(unsigned int cost) {
  this->cost = cost;
}

unsigned int Edge::getCost() const {
  return cost;
}

void Edge::setLength(unsigned int length) {
  this->length = length;
}

unsigned int Edge::getLength() const {
  return length;
}

bool Edge::operator<(const Edge &right) const {
  return cost > right.cost;
}

// Method implementations here
