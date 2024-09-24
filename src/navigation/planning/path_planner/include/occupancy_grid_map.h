#pragma once

#include <vector>
#include <unordered_map>
#include "utils.h"
#include "priority_node.h"
#include "nav_msgs/OccupancyGrid.h"

static constexpr int UNOCCUPIED = 0;
static constexpr int OBSTACLE = 100;
static constexpr int OBSTACLE_THRESH = 10;

class OccupancyGridMap {
public:
    int margin;
    int x_dim;
    int y_dim;
    std::pair<int,int> goal;
    std::pair<int,int> start;
    std::vector<std::vector<int>> occupancy_map;

public:
    OccupancyGridMap();

    OccupancyGridMap(int x_dim, int y_dim);

    OccupancyGridMap(const OccupancyGridMap& other);

    OccupancyGridMap(const nav_msgs::OccupancyGrid::ConstPtr& map);

    std::vector<std::vector<int>> getMap() const;

    void setMargin(const int _margin); 

    void setMap(const std::vector<std::vector<int>>& new_ogrid);

    void setMap(const OccupancyGridMap& _map);

    bool isEmpty(const std::pair<int, int> p);

    bool inBounds(const std::pair<int, int> p);

    void setObstacle(const std::pair<int, int> p);

    void removeObstacle(const std::pair<int, int> p);

    std::vector<std::pair<int, int>> filter(const std::vector<std::pair<int, int>>& neighbors);

    std::vector<std::pair<int, int>> getSuccessors(const std::pair<int, int> p);
};

class ScanMap {
private:
    OccupancyGridMap groundTruthMap;
    OccupancyGridMap scanedMap;
    int view_range;
    double _cost_co;

public:
    ScanMap(const OccupancyGridMap& map, int view_range, double cost_co);

    void setGroundTruthMap(const OccupancyGridMap& gt_map);

    void setScanMapMargin(const int& margin);

    OccupancyGridMap getScanMap();

    double calCost(const std::pair<int, int> p1, const std::pair<int, int> p2);

    double calHeuristic(const std::pair<int, int> p1, const std::pair<int, int> p2);

    Vertices scanLocalMap(const std::pair<int, int> local_position);

    std::unordered_map<std::pair<int, int>, int, pair_hash> getLocalMap(const std::pair<int, int> local_position);

    Vertices updateChangedCosts(const std::unordered_map<std::pair<int, int>, int, pair_hash>& local_grid);
};
