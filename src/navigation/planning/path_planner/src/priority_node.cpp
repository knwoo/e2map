#include "priority_node.h"

Vertex::Vertex(std::pair<int, int> pos) : pos(pos) {}

const std::pair<int, int>& Vertex::getPos() const {return pos;}

const std::unordered_map<std::pair<int, int>, double, pair_hash>& Vertex::getEdgesAndCosts() const {return edgesAndCosts;}

void Vertex::addEdgeWithCost(std::pair<int, int> succ, double cost) {
    if (succ != pos) {
        edgesAndCosts[succ] = cost;
    }
}

bool Vertex::operator==(const Vertex& other) const {
        return pos == other.pos;
}

Vertices::Vertices() : list(std::vector<Vertex>()) {}

Vertices::Vertices(const Vertices& other) : list(other.list) {}

void Vertices::addVertex(const Vertex& v) {
    list.push_back(v);
}

std::vector<Vertex> Vertices::getVertices() const {
    return list;
}

void Vertices::clearVertices() {
    list.clear();
}

Priority::Priority(double k1, double k2)
    : k1(k1), k2(k2) {}

bool Priority::operator<(const Priority& other) const {
    return k1 < other.k1 || (k1 == other.k1 && k2 < other.k2);
}

bool Priority::operator<=(const Priority& other) const {
    return k1 < other.k1 || (k1 == other.k1 && k2 <= other.k2);
}

PriorityNode::PriorityNode(Priority priority, Vertex vertex)
    : priority(priority), vertex(vertex) {}

bool PriorityNode::operator<=(const PriorityNode& other) const {
    return priority <= other.priority;
}

bool PriorityNode::operator<(const PriorityNode& other) const {
    return priority < other.priority;
}
