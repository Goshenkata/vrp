#include "euclidean_solver.h"
#include <set>
#include <stack>
#include <unordered_map>
#include <cmath>
#include <omp.h>
#include <limits>

using namespace std;

// Modified solve method with support for multiple vehicles
vector<vector<int>> EuclideanSolver::solve(const vector<Point>& locations, const map<int, int>& demand,
                                             int capacity, int max_stops, int& bestCost, int num_vehicles) {
    // Generate all valid routes (each route starts and ends at the depot)
    vector<vector<int>> routes = GenerateAllCombinations(locations, demand, capacity, max_stops);
    vector<vector<int>> bestCombination;
    bestCost = numeric_limits<int>::max();

    int num_threads = 24;
    omp_set_num_threads(num_threads);

    // Use OpenMP tasks to try different starting routes in parallel
    #pragma omp parallel
    {
        #pragma omp single nowait
        {
            for (size_t i = 0; i < routes.size(); ++i) {
                #pragma omp task firstprivate(i)
                {
                    vector<vector<int>> currentCombination;
                    // Begin recursive search from index i
                    FindBestCombination(routes, currentCombination, i, locations, bestCost, bestCombination, num_vehicles);
                }
            }
        }
    }

    return bestCombination;
}

// Recursive backtracking function with an extra parameter for num_vehicles.
// This version checks for overlapping non-depot locations before adding a route.
void EuclideanSolver::FindBestCombination(const vector<vector<int>>& routes,
                                            vector<vector<int>>& currentCombination,
                                            size_t index, const vector<Point>& locations,
                                            int& bestCost, vector<vector<int>>& bestCombination,
                                            int num_vehicles) {
    // If we've considered all routes or already used the available vehicles, check if the combination covers all locations.
    if (index >= routes.size() || currentCombination.size() == (size_t)num_vehicles) {
        if (coversAllLocations(currentCombination, locations)) {
            int totalCost = CalculateTotalCost(currentCombination, locations);
            #pragma omp critical
            {
                if (totalCost < bestCost) {
                    bestCost = totalCost;
                    bestCombination = currentCombination;
                }
            }
        }
        return;
    }

    // Option 1: Try adding the current route if it does not cause overlap.
    bool overlap = false;
    // Check each non-depot location in the candidate route.
    for (int loc : routes[index]) {
        if (loc == 0) continue; // Skip depot
        for (const auto& route : currentCombination) {
            for (int existingLoc : route) {
                if (existingLoc == 0) continue;
                if (existingLoc == loc) {
                    overlap = true;
                    break;
                }
            }
            if (overlap) break;
        }
        if (overlap) break;
    }

    if (!overlap) {
        currentCombination.push_back(routes[index]);
        FindBestCombination(routes, currentCombination, index + 1, locations, bestCost, bestCombination, num_vehicles);
        currentCombination.pop_back();
    }

    // Option 2: Skip the current route and move to the next.
    FindBestCombination(routes, currentCombination, index + 1, locations, bestCost, bestCombination, num_vehicles);
}

// Other methods remain unchanged

double EuclideanSolver::calculateDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

double EuclideanSolver::calculateRouteCost(const vector<int>& route, const vector<Point>& locations) {
    if (route.size() <= 1) {
        return 0;
    }
    double totalCost = 0;
    // Helper lambda to find a point by its ID
    auto findPoint = [&locations](int id) -> const Point& {
        for (const auto& point : locations) {
            if (point.id == id) {
                return point;
            }
        }
        static Point dummy = {-1, 0, 0};
        return dummy;
    };

    for (size_t i = 0; i < route.size() - 1; i++) {
        const Point& p1 = findPoint(route[i]);
        const Point& p2 = findPoint(route[i + 1]);
        totalCost += calculateDistance(p1, p2);
    }
    return totalCost;
}

bool EuclideanSolver::verifyValidRoute(const vector<int>& route) {
    // All routes are valid with Euclidean distance
    return true;
}

vector<vector<int>> EuclideanSolver::GenerateAllCombinations(const vector<Point>& locations,
                                                               const map<int, int>& demand,
                                                               int capacity, int max_stops) {
    vector<vector<int>> routes;
    int n = locations.size();
    vector<vector<int>> all_routes(1 << n);

    #pragma omp parallel for
    for (int i = 1; i < (1 << n); i++) {
        vector<int> route;
        int total_demand = 0;
        unordered_map<int, int> place_count;
        bool invalid = false;

        for (int j = 0; j < n; j++) {
            if (i & (1 << j)) {
                int place_id = locations[j].id;
                route.push_back(place_id);

                auto it = demand.find(place_id);
                if (it != demand.end()) {
                    total_demand += it->second;
                }

                place_count[place_id]++;

                if (total_demand > capacity || place_count[place_id] > 1) {
                    invalid = true;
                    break;
                }
            }
        }

        // Add depot as first and last point if not already there
        if (!invalid && !route.empty() && route.size() <= (size_t)max_stops) {
            if (route[0] != 0) {
                route.insert(route.begin(), 0);
            }
            if (route.back() != 0) {
                route.push_back(0);
            }
            all_routes[i] = route;
        }
    }

    for (int i = 1; i < (1 << n); i++) {
        if (!all_routes[i].empty()) {
            routes.push_back(all_routes[i]);
        }
    }
    return routes;
}

int EuclideanSolver::CalculateTotalCost(const vector<vector<int>>& routes, const vector<Point>& locations) {
    double totalCost = 0;
    #pragma omp parallel for reduction(+:totalCost)
    for (size_t i = 0; i < routes.size(); ++i) {
        totalCost += calculateRouteCost(routes[i], locations);
    }
    return static_cast<int>(totalCost);
}

// Check that every non-depot location appears in at least one route of the combination.
bool EuclideanSolver::coversAllLocations(const vector<vector<int>>& combination, const vector<Point>& locations) {
    set<int> uncoveredLocations;
    // Add all non-depot location IDs
    for (const auto& point : locations) {
        if (point.id != 0) {
            uncoveredLocations.insert(point.id);
        }
    }
    for (const auto& route : combination) {
        for (int loc_id : route) {
            uncoveredLocations.erase(loc_id);
        }
    }
    return uncoveredLocations.empty();
}
