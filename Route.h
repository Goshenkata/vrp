#pragma once
#include <vector>

struct  Route {
    std::vector<int> path;
    int total_distance = 0;
    int capacity_used = 0;
    Route();

    void add_location(int location_id);
};