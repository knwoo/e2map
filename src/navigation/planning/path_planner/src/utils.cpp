#include "utils.h"

std::vector<std::pair<int, int>> getMovementsDirection4(const int x, const int y) {
    return {
        std::make_pair(x + 1, y + 0),  
        std::make_pair(x + 0, y + 1),  
        std::make_pair(x - 1, y + 0),   
        std::make_pair(x + 0, y - 1)    
    };
}

std::vector<std::pair<int, int>> getMovementsDirection8(const int x, const int y) {
    return {
        std::make_pair(x + 1, y + 0),  
        std::make_pair(x + 0, y + 1),  
        std::make_pair(x - 1, y + 0),  
        std::make_pair(x + 0, y - 1),
        std::make_pair(x + 1, y + 1),  
        std::make_pair(x - 1, y + 1),  
        std::make_pair(x - 1, y - 1),  
        std::make_pair(x + 1, y - 1)
    };
}

double euclideanDistance(const std::pair<int, int> p1, const std::pair<int, int> p2) {
    double dx = p1.first - p2.first;
    double dy = p1.second - p2.second;
    return std::sqrt(dx * dx + dy * dy);
}