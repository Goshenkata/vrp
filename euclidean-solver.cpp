#include <algorithm>

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

    // Calculate and store the cost of each route
    vector<int> routeCosts(routes.size());
    #pragma omp parallel for
    for (size_t i = 0; i < routes.size(); ++i) {
        routeCosts[i] = static_cast<int>(calculateRouteCost(routes[i], locations));
    }

    // Sort routes by cost (ascending) for better pruning
    vector<size_t> routeIndices(routes.size());
    for (size_t i = 0; i < routes.size(); ++i) {
        routeIndices[i] = i;
    }
    ranges::sort(routeIndices,
                 [&routeCosts](size_t a, size_t b) { return routeCosts[a] < routeCosts[b]; });

    int num_threads = 24;
    omp_set_num_threads(num_threads);

    // Use OpenMP tasks to try different starting routes in parallel
    #pragma omp parallel
    {
        #pragma omp single nowait
        {
            for (size_t idx = 0; idx < routes.size(); ++idx) {
                size_t i = routeIndices[idx];
                EuclideanSolver* solver = this;
                #pragma omp task firstprivate(i, solver)
                {
                    vector<vector<int>> currentCombination;
                    set<int> coveredLocations;
                    coveredLocations.insert(0); // Depot is always covered
                    int currentCost = 0;

                    // Begin recursive search from index i
                    solver->FindBestCombination(routes, routeCosts, currentCombination, i,
                                       locations, bestCost, bestCombination,
                                       num_vehicles, coveredLocations, currentCost);
                }
            }
        }
    }

    return bestCombination;
}

// Calculate a lower bound for the remaining uncovered locations
int EuclideanSolver::calculateLowerBound(const set<int>& coveredLocations,
                                         const vector<Point>& locations) {
    if (coveredLocations.size() >= locations.size()) {
        return 0; // All locations are covered
    }

    // Find the depot
    Point depot;
    for (const auto& p : locations) {
        if (p.id == 0) {
            depot = p;
            break;
        }
    }

    double minLowerBound = 0;

    // For each uncovered location, find minimum distance to depot
    for (const auto& point : locations) {
        if (coveredLocations.find(point.id) == coveredLocations.end()) {
            // This location is not covered - we need at least two edges (to and from depot)
            double distToDepot = calculateDistance(point, depot);
            minLowerBound += 2 * distToDepot; // Round trip to depot (minimum possible)
        }
    }

    // This is an optimistic lower bound - in reality, routes will be longer
    return static_cast<int>(minLowerBound);
}

// Recursive backtracking function with early pruning
void EuclideanSolver::FindBestCombination(const vector<vector<int>>& routes,
                                         const vector<int>& routeCosts,
                                         vector<vector<int>>& currentCombination,
                                         size_t index, const vector<Point>& locations,
                                         int& bestCost, vector<vector<int>>& bestCombination,
                                         int num_vehicles, set<int>& coveredLocations,
                                         int currentCost) {
    // Early pruning: if current cost already exceeds best cost, stop exploring this branch
    if (currentCost >= bestCost) {
        return;
    }

    // Calculate lower bound for remaining locations
    int lowerBound = calculateLowerBound(coveredLocations, locations);

    // If current cost plus lower bound exceeds best cost, prune this branch
    if (currentCost + lowerBound >= bestCost) {
        return;
    }

    // If we've considered all routes or already used the available vehicles, check if the combination covers all locations.
    if (index >= routes.size() || currentCombination.size() == (size_t)num_vehicles) {
        if (coveredLocations.size() == locations.size()) {
            #pragma omp critical
            {
                if (currentCost < bestCost) {
                    bestCost = currentCost;
                    bestCombination = currentCombination;
                }
            }
        }
        return;
    }

    // Option 1: Try adding the current route if it does not cause overlap.
    bool overlap = false;
    set<int> newLocations;

    // Check each non-depot location in the candidate route.
    for (int loc : routes[index]) {
        if (loc == 0) continue; // Skip depot
        if (coveredLocations.find(loc) != coveredLocations.end()) {
            overlap = true;
            break;
        }
        newLocations.insert(loc);
    }

    if (!overlap) {
        // Add this route
        currentCombination.push_back(routes[index]);

        // Update covered locations
        for (int loc : newLocations) {
            coveredLocations.insert(loc);
        }

        // Update current cost
        int newCost = currentCost + routeCosts[index];

        // Continue search
        FindBestCombination(routes, routeCosts, currentCombination, index + 1,
                           locations, bestCost, bestCombination, num_vehicles,
                           coveredLocations, newCost);

        // Backtrack: remove locations from coverage
        for (int loc : newLocations) {
            coveredLocations.erase(loc);
        }

        currentCombination.pop_back();
    }

    // Option 2: Skip the current route and move to the next.
    FindBestCombination(routes, routeCosts, currentCombination, index + 1,
                       locations, bestCost, bestCombination, num_vehicles,
                       coveredLocations, currentCost);
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

// We don't need this function anymore since we're tracking covered locations directly
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