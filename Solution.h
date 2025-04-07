#pragma once
#include "Route.h"
#include <vector>

struct Solution {
    std::vector<Route> routes;
    int total_distance = 0;
    Solution();

    void add_route(const Route &route);
};