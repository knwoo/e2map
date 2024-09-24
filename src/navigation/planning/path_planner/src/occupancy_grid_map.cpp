#include "occupancy_grid_map.h"
OccupancyGridMap::OccupancyGridMap() : margin(0) {}

OccupancyGridMap::OccupancyGridMap(int x_dim, int y_dim) : x_dim(x_dim), y_dim(y_dim) {
    occupancy_map.resize(x_dim, std::vector<int>(y_dim, 0));
    margin = 0;
}

OccupancyGridMap::OccupancyGridMap(const OccupancyGridMap& other) {
    margin = other.margin;
    x_dim = other.x_dim;
    y_dim = other.y_dim;
    goal = other.goal;
    start = other.start;
    occupancy_map = other.occupancy_map; 
}

OccupancyGridMap::OccupancyGridMap(const nav_msgs::OccupancyGrid::ConstPtr& map) : x_dim(map->info.width), y_dim(map->info.height) {
    occupancy_map.resize(x_dim, std::vector<int>(y_dim, 0));
}

std::vector<std::vector<int>> OccupancyGridMap::getMap() const {
    return occupancy_map;
}

void OccupancyGridMap::setMargin(const int _margin) {
    margin = _margin;
}

void OccupancyGridMap::setMap(const std::vector<std::vector<int>>& new_ogrid) {
    occupancy_map = new_ogrid;
}

void OccupancyGridMap::setMap(const OccupancyGridMap& _map) {
    occupancy_map = _map.occupancy_map;
}

bool OccupancyGridMap::isEmpty(const std::pair<int, int> p) {
    return occupancy_map[p.first][p.second] < OBSTACLE_THRESH;
}

bool OccupancyGridMap::inBounds(const std::pair<int, int> p) {
    return p.first >= 0 && p.first < x_dim && p.second >= 0 && p.second < y_dim;
}

void OccupancyGridMap::setObstacle(const std::pair<int, int> p) {
    occupancy_map[p.first][p.second] = OBSTACLE;
}

void OccupancyGridMap::removeObstacle(const std::pair<int, int> p) {
    occupancy_map[p.first][p.second] = UNOCCUPIED;
}

std::vector<std::pair<int, int>> OccupancyGridMap::filter(const std::vector<std::pair<int, int>>& neighbors) {
    std::vector<std::pair<int, int>> filtered_neighbors;
    for (std::pair<int,int> node : neighbors) {
        if (inBounds(node) && isEmpty(node)) {
            filtered_neighbors.push_back(node);
        }
    }
    return filtered_neighbors;
}


std::vector<std::pair<int, int>> OccupancyGridMap::getSuccessors(const std::pair<int, int> p) {
    std::vector<std::pair<int, int>> movements = getMovementsDirection8(p.first, p.second);
    std::vector<std::pair<int, int>> filtered;
    for (std::pair<int, int> node : movements) {
        filtered = filter(movements);
        }
    return filtered;
}

ScanMap::ScanMap(const OccupancyGridMap& map, int view_range, double cost_co)
    : groundTruthMap(map), view_range(view_range), scanedMap(map), _cost_co(cost_co) {}

void ScanMap::setGroundTruthMap(const OccupancyGridMap& gt_map) {
    groundTruthMap = gt_map;
}

void ScanMap::setScanMapMargin(const int& margin) {
    scanedMap.margin = margin;
    groundTruthMap.margin = margin;
}

OccupancyGridMap ScanMap::getScanMap() {
    return scanedMap;
}

double ScanMap::calCost(const std::pair<int, int> p1, const std::pair<int, int> p2) {
    if (!scanedMap.isEmpty(p1) || !scanedMap.isEmpty(p2)) {
        return std::numeric_limits<double>::infinity();
    }
    else {
        return calHeuristic(p1, p2);
    }
}

double ScanMap::calHeuristic(const std::pair<int, int> p1, const std::pair<int, int> p2) {
    int x1 = std::get<0>(p1);
    int y1 = std::get<1>(p1);
    int x2 = std::get<0>(p2);
    int y2 = std::get<1>(p2);
    
    int cost_sum = groundTruthMap.occupancy_map[x1][y1];
    int i = 1;
    float gamma = 0.99;

    while (x1 != x2 || y1 != y2) {
        if (x1 != x2) {
            x1 += (x2 > x1) ? 1 : -1;
        }
        if (y1 != y2) {
            y1 += (y2 > y1) ? 1 : -1;
        }
        i += 1;
        cost_sum += groundTruthMap.occupancy_map[x1][y1];
    }
    double heuristic_distance = std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
    double heuristic_cost = _cost_co * cost_sum;

    return heuristic_distance + std::pow(gamma, i) * heuristic_cost;
}


Vertices ScanMap::scanLocalMap(const std::pair<int, int> local_position) {
    std::unordered_map<std::pair<int, int>, int, pair_hash> local_observation = getLocalMap(local_position);

    Vertices vertices = updateChangedCosts(local_observation);
    return vertices;
}

std::unordered_map<std::pair<int, int>, int, pair_hash> ScanMap::getLocalMap(const std::pair<int, int> local_position) {
    int px = local_position.first;
    int py = local_position.second;
    std::unordered_map<std::pair<int, int>, int, pair_hash> observation;
    for (int x = px - view_range; x <= px + view_range; x++) {
        for (int y = py - view_range; y <= py + view_range; y++) {
            std::pair<int, int> node = std::make_pair(x, y);
            if (groundTruthMap.inBounds(node)){
                observation[node] = groundTruthMap.occupancy_map[x][y];
            }
        }
    }
    return observation;
}

Vertices ScanMap::updateChangedCosts(const std::unordered_map<std::pair<int, int>, int, pair_hash>& local_grid) {
    Vertices vertices;
    for (auto& entry : local_grid) {
        std::pair<int, int> node = entry.first;
        int value = entry.second;
        if (scanedMap.occupancy_map[node.first][node.second] != value) {
            scanedMap.occupancy_map[node.first][node.second] = value;

            Vertex v(node);
            auto succ = scanedMap.getSuccessors(node);
            for (auto& u : succ) {
                v.addEdgeWithCost(u, calCost(u, node));
            }
            vertices.addVertex(v);
        }
    }
    return vertices;
}