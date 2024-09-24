#pragma once

#include "priority_node.h"
#include <vector>

class PriorityQueue {
public:
    std::vector<PriorityNode> heap;
    std::vector<Vertex> vertices_in_heap;

    void _siftDown(int startpos, int pos);

    void _siftUp(int pos);

public:
    PriorityNode getTopNode();

    Priority getTopKey();

    PriorityNode popNode();

    void insertNode(Vertex vertex, Priority priority);

    void removeNode(Vertex vertex);

    void updateNode(Vertex vertex, Priority priority);

    void buildHeap();
};