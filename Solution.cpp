#include "Solution.h"

Solution::Solution() : total_distance(0) {}

void Solution::add_route(const Route& route) {
    routes.push_back(route);
    total_distance += route.total_distance;
}
