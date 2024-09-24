#pragma once

#include <vector>
#include <unordered_map>
#include <functional>
#include <algorithm>

template <class T> inline void hash_combine(size_t &seed, T const &v) {
    seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct pair_hash {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2> &p) const {
        size_t seed = 0;
        hash_combine(seed, p.first);
        hash_combine(seed, p.second);
        return seed;
    }
};

class Vertex {
    friend class DStar;

private:
    std::pair<int, int> pos;
    std::unordered_map<std::pair<int, int>, double, pair_hash> edgesAndCosts;

public:
    Vertex(std::pair<int, int> pos);

    const std::pair<int, int>& getPos() const;

    const std::unordered_map<std::pair<int, int>, double, pair_hash>& getEdgesAndCosts() const;

    void addEdgeWithCost(std::pair<int, int> succ, double cost);

    bool operator==(const Vertex& other) const;
};

class Vertices {
private:
    std::vector<Vertex> list;

public:
    Vertices();
    Vertices(const Vertices& other);

    void addVertex(const Vertex& v);

    std::vector<Vertex> getVertices() const;

    void clearVertices();
};

class Priority {
public:
    double k1;
    double k2;

public:
    Priority(double k1, double k2);

    bool operator<(const Priority& other) const;

    bool operator<=(const Priority& other) const;
};

class PriorityNode {
public:
    Priority priority;
    Vertex vertex;

public:
    PriorityNode(Priority priority, Vertex vertex);
    bool operator<=(const PriorityNode& other) const;

    bool operator<(const PriorityNode& other) const;
};