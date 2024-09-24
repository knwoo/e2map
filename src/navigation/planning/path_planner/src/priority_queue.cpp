#include "priority_queue.h"
#include <algorithm>

void PriorityQueue::_siftDown(int startpos, int pos) {
    PriorityNode newitem = heap[pos];

    while (pos > startpos) {
        int parentpos = (pos - 1) >> 1;
        PriorityNode parent = heap[parentpos];

        if (newitem < parent) {
            heap[pos] = parent;
            pos = parentpos;
            continue;
        }
        break;
    }

    heap[pos] = newitem;
}

void PriorityQueue::_siftUp(int pos) {
    int endpos = heap.size();
    int startpos = pos;
    PriorityNode newitem = heap[pos];
    int childpos = 2 * pos + 1;

    while (childpos < endpos) {
        int rightpos = childpos + 1;
        if (rightpos < endpos && !(heap[childpos] < heap[rightpos])) {
            childpos = rightpos;
        }

        heap[pos] = heap[childpos];
        pos = childpos;
        childpos = 2 * pos + 1;
    }

    heap[pos] = newitem;
    _siftDown(startpos, pos);
}

PriorityNode PriorityQueue::getTopNode() {
    return heap[0];
}

Priority PriorityQueue::getTopKey() {
    if (heap.empty()) return Priority(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    return heap[0].priority;
}

PriorityNode PriorityQueue::popNode() {
    PriorityNode returnitem = heap[0];
    vertices_in_heap.erase(std::remove(vertices_in_heap.begin(), vertices_in_heap.end(), returnitem.vertex), vertices_in_heap.end());

    PriorityNode lastelt = heap.back();
    heap.pop_back();

    if (!heap.empty()) {
        heap[0] = lastelt;
        _siftUp(0);
    }

    return returnitem;
}

void PriorityQueue::insertNode(Vertex vertex, Priority priority) {
    PriorityNode item(priority, vertex);
    vertices_in_heap.push_back(vertex);

    heap.push_back(item);
    _siftDown(0, heap.size() - 1);
}

void PriorityQueue::removeNode(Vertex vertex) {
    vertices_in_heap.erase(std::remove(vertices_in_heap.begin(), vertices_in_heap.end(), vertex), vertices_in_heap.end());

    for (size_t index = 0; index < heap.size(); ++index) {
        if (heap[index].vertex == vertex) {
            heap[index] = heap.back();
            heap.pop_back();
            break;
        }
    }

    buildHeap();
}

void PriorityQueue::updateNode(Vertex vertex, Priority priority) {
    for (size_t index = 0; index < heap.size(); ++index) {
        if (heap[index].vertex == vertex) {
            heap[index].priority = priority;
            break;
        }
    }

    buildHeap();
}

void PriorityQueue::buildHeap() {
    int n = heap.size();

    for (int i = n / 2 - 1; i >= 0; --i) {
        _siftUp(i);
    }
}