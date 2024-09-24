#pragma once
#include <cmath>
#include <vector>
#include <tuple>

std::vector<std::pair<int, int>> getMovementsDirection4(const int x, const int y);

std::vector<std::pair<int, int>> getMovementsDirection8(const int x, const int y);

double euclideanDistance(const std::pair<int, int> p1, const std::pair<int, int> p2);