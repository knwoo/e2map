#pragma once

#include <ros/console.h>
#include <vector>
#include "priority_queue.h"
#include "occupancy_grid_map.h"
#include "utils.h"
#include <iostream>
#include <tuple>
#include <algorithm>
#include <cassert>

class DStar {
    friend class Planner;
private:
    float _cost_co;
    int k_m;
    PriorityQueue Q;
    std::pair<int, int> s_start;
    std::pair<int, int> s_goal;
    std::pair<int, int> s_last;
    std::vector<std::vector<double>> rhs;
    std::vector<std::vector<double>> g;
    OccupancyGridMap sensed_map;
    Vertices new_edges_and_old_costs;

    std::vector<std::pair<int, int>> prev_path;

public:
    DStar(const OccupancyGridMap& map, std::pair<int, int> s_start_val, std::pair<int, int> s_goal_val, float cost_co);

    double calCost(std::pair<int, int> p1, std::pair<int, int> p2);

    double calHeuristic(std::pair<int, int> p1, const std::pair<int, int> p2);

    Priority calKey(std::pair<int, int> p);

    void updateVertex(std::pair<int, int> p);

    bool isContain(std::pair<int, int> p);

    void computeShortestPath();

    std::vector<std::pair<int, int>> plan(std::pair<int, int> robot_pos);

    Vertices getUpdateCosts();
};
