#include "dstar.h"

DStar::DStar(const OccupancyGridMap& map, std::pair<int, int> s_start_val, std::pair<int, int> s_goal_val, float cost_co)
    : s_start(s_start_val), s_goal(s_goal_val), s_last(s_start_val), k_m(0), sensed_map(map), _cost_co(cost_co) {
    rhs.resize(map.x_dim, std::vector<double>(map.y_dim, std::numeric_limits<double>::infinity()));
    g = rhs;
    rhs[s_goal.first][s_goal.second] = 0;
    Q.insertNode(std::pair<int, int>(s_goal.first, s_goal.second), calKey(s_goal));

    std::cout << "start: " << s_start.first << ", " << s_start.second << std::endl;
    std::cout << "goal : " << s_goal.first << ", " << s_goal.second << std::endl;
}

double DStar::calCost(std::pair<int, int> p1, std::pair<int, int> p2) {
    if (!sensed_map.isEmpty(p1) || !sensed_map.isEmpty(p2))
        return std::numeric_limits<double>::infinity();
    else
        return calHeuristic(p1, p2);
}

double DStar::calHeuristic(const std::pair<int, int> p1, const std::pair<int, int> p2) {
    int x1 = std::get<0>(p1);
    int y1 = std::get<1>(p1);
    int x2 = std::get<0>(p2);
    int y2 = std::get<1>(p2);
    
    int cost_sum = sensed_map.occupancy_map[x1][y1];
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
        cost_sum += sensed_map.occupancy_map[x1][y1];
    }
    double heuristic_distance = std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
    double heuristic_cost = _cost_co * cost_sum;
    
    return heuristic_distance + std::pow(gamma, i) * heuristic_cost;
}

Priority DStar::calKey(std::pair<int, int> p) {
    double k1 = std::min(g[p.first][p.second], rhs[p.first][p.second]) + calHeuristic(s_start, p) + k_m;
    double k2 = std::min(g[p.first][p.second], rhs[p.first][p.second]);
    return Priority(k1, k2);
}

void DStar::updateVertex(std::pair<int, int> p) {
    if (g[p.first][p.second] != rhs[p.first][p.second] && isContain(p)) {
        Q.updateNode(p, calKey(p));
    }
    else if (g[p.first][p.second] != rhs[p.first][p.second] && !isContain(p)) {
        Q.insertNode(p, calKey(p));
    }
    else if (g[p.first][p.second] == rhs[p.first][p.second] && isContain(p)) {
        Q.removeNode(p);
    }
}

bool DStar::isContain(std::pair<int, int> p) {
    return std::find(Q.vertices_in_heap.begin(), Q.vertices_in_heap.end(), p) != Q.vertices_in_heap.end();
}

void DStar::computeShortestPath() {
    while (Q.getTopKey() < calKey(s_start) || rhs[s_start.first][s_start.second] != g[s_start.first][s_start.second]) {
        PriorityNode u = Q.getTopNode();
        Priority k_old = Q.getTopKey();
        Priority k_new = calKey(u.vertex.pos);
        if (k_old < k_new) {
            Q.updateNode(u.vertex, k_new);
        }
        else if (g[u.vertex.pos.first][u.vertex.pos.second] > rhs[u.vertex.pos.first][u.vertex.pos.second]) {
            g[u.vertex.pos.first][u.vertex.pos.second] = rhs[u.vertex.pos.first][u.vertex.pos.second];
            Q.removeNode(u.vertex);
            std::vector<std::pair<int, int>> pred = sensed_map.getSuccessors(u.vertex.pos);
            for (std::pair<int, int> s : pred) {
                if (s != s_goal) {
                    rhs[s.first][s.second] = std::min(rhs[s.first][s.second], calCost(s, u.vertex.pos) + g[u.vertex.pos.first][u.vertex.pos.second]);
                }
                updateVertex(s);
            }
        }
        else {
            double g_old = g[u.vertex.pos.first][u.vertex.pos.second];
            g[u.vertex.pos.first][u.vertex.pos.second] = std::numeric_limits<double>::infinity();
            std::vector<std::pair<int, int>> pred = sensed_map.getSuccessors(u.vertex.pos);
            pred.push_back(u.vertex.pos);
            for (std::pair<int, int>& s : pred) {
                if (rhs[s.first][s.second] == calCost(s, u.vertex.pos) + g_old || s == u.vertex.pos) {
                    if (s != s_goal) {
                        double min_s = std::numeric_limits<double>::infinity();
                        std::vector<std::pair<int, int>> succ = sensed_map.getSuccessors(s);
                        for (const std::pair<int, int>& ss : succ) {
                            double temp = calCost(s, ss) + g[ss.first][ss.second];
                            min_s = std::min(min_s, temp);
                        }
                        rhs[s.first][s.second] = min_s;
                    }
                }
                updateVertex(u.vertex.pos);
            }
        }
    }
}

std::vector<std::pair<int, int>> DStar::plan(std::pair<int, int> robot_pos) {
    std::vector<std::pair<int, int>> path;

    try {
        path.push_back(robot_pos);
        sensed_map.goal = s_goal;
        sensed_map.start = s_start;
        s_start = robot_pos;
        s_last = s_start;
        computeShortestPath();

        while (s_start != s_goal) {
            if (rhs[s_start.first][s_start.second] == std::numeric_limits<double>::infinity()) {
                throw std::runtime_error("No valid path found: rhs value is infinity");
            }

            std::vector<std::pair<int, int>> succ = sensed_map.getSuccessors(s_start);

            if (succ.empty()) {
                std::cerr << "No valid successors found for current position." << std::endl;
                path.push_back(s_start);
                break;
            } else {
                double min_s = std::numeric_limits<double>::infinity();
                std::pair<int, int> arg_min = succ[0];
                for (const std::pair<int, int>& ss : succ) {
                    double temp = calCost(s_start, ss) + g[ss.first][ss.second];
                    if (temp < min_s) {
                        min_s = temp;
                        arg_min = ss;
                    }
                }
                s_start = arg_min;
                path.push_back(s_start);
            }

            Vertices changed_edges_with_old_cost = getUpdateCosts();

            if (!changed_edges_with_old_cost.getVertices().empty()) {
                
                std::cout << "update costs : re";

                k_m += calHeuristic(s_last, s_start);
                s_last = s_start;

                std::vector<Vertex> vertices = changed_edges_with_old_cost.getVertices();
                for (const Vertex& vertex : vertices) {
                    std::pair<int, int> v = vertex.pos;
                    auto& succ_v = vertex.edgesAndCosts;
                    for (const auto& pair : succ_v) {
                        std::pair<int, int> u = pair.first;
                        double c_old = pair.second;
                        double c_new = calCost(u, v);
                        if (c_old > c_new) {
                            if (u != s_goal) {
                                rhs[u.first][u.second] = std::min(rhs[u.first][u.second], calCost(u, v) + g[v.first][v.second]);
                            }
                        }
                        else if (rhs[u.first][u.second] == c_old + g[v.first][v.second]) {
                            if (u != s_goal) {
                                double min_s = std::numeric_limits<double>::infinity();
                                std::vector<std::pair<int, int>> succ_u = sensed_map.getSuccessors(u);
                                for (std::pair<int, int> ss : succ_u) {
                                    double temp = calCost(u, ss) + g[ss.first][ss.second];
                                    if (min_s > temp) {
                                        min_s = temp;
                                    }
                                }
                                rhs[u.first][u.second] = min_s;
                            }
                        }               

                        updateVertex(u);
                    }
                }
            }
            computeShortestPath();
        }
        prev_path = path;
        std::cout << "path found!" << std::endl;
    } catch (const std::exception& e) {
        if (prev_path.size() > 0) {
            path = prev_path;
        }
        ROS_INFO_STREAM("Exception in DStar::plan: " << e.what());
    }    
    return path;
}

Vertices DStar::getUpdateCosts() {
    Vertices new_edges_and_old_costs_temp;
    std::swap(new_edges_and_old_costs, new_edges_and_old_costs_temp);
    return new_edges_and_old_costs_temp;
}