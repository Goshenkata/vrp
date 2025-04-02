#include <algorithm>
#include <climits>
#include <set>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <omp.h>

#include "euclidean_solver.h"

using namespace std;

vector<vector<int>> EuclideanSolver::solve(const vector<Point> &locations, const map<int, int> &demand,
                                          int capacity, int max_stops, int &bestCost, int num_vehicles) {
    // Check for valid input
    if (locations.empty() || demand.empty()) {
        cerr << "Error: Empty locations or demand." << endl;
        return {};
    }

    // Initialize variables
    vector<vector<int>> bestCombination;
    bestCost = INT_MAX;
    set<int> coveredLocations;

    // Generate all valid combinations of routes
    vector<vector<int>> allCombinations = GenerateAllCombinations(locations, demand, capacity, max_stops);
    if (allCombinations.empty()) {
        cerr << "Error: No valid combinations generated." << endl;
        return {};
    }

    // Calculate route costs
    vector<int> routeCosts(allCombinations.size());
    for (size_t i = 0; i < allCombinations.size(); ++i) {
        routeCosts[i] = calculateRouteCost(allCombinations[i], locations);
    }

    // Find the best combination of routes
    vector<vector<int>> tempCombination;
    FindBestCombination(allCombinations, routeCosts, tempCombination, 0, locations,
                       bestCost, bestCombination, num_vehicles, coveredLocations, 0);

    return bestCombination;
}

// Calculate a lower bound for the remaining uncovered locations
int EuclideanSolver::calculateLowerBound(const set<int>& coveredLocations, const vector<Point>& locations) {
    if (coveredLocations.size() >= locations.size() - 1) { // Subtract 1 for depot
        return 0; // All locations are covered
    }

    // Find the depot
    Point depot = {0, 0, 0};
    for (const auto& p : locations) {
        if (p.id == 0) {
            depot = p;
            break;
        }
    }

    double minLowerBound = 0;

    // For each uncovered location, find minimum distance to depot
    for (const auto& point : locations) {
        if (point.id != 0 && coveredLocations.find(point.id) == coveredLocations.end()) {
            // This location is not covered - we need at least two edges (to and from depot)
            double distToDepot = calculateDistance(point, depot);
            minLowerBound += 2 * distToDepot; // Round trip to depot (minimum possible)
        }
    }

    return static_cast<int>(minLowerBound);
}

// Recursive backtracking function with early pruning
void EuclideanSolver::FindBestCombination(const vector<vector<int>>& routes,
                                        const vector<int>& routeCosts,
                                        vector<vector<int>>& currentCombination,
                                        unsigned int index, const vector<Point>& locations,
                                        int& bestCost, vector<vector<int>>& bestCombination,
                                        int num_vehicles, set<int>& coveredLocations,
                                        int currentCost) {
    // Early pruning conditions
    if (currentCost >= bestCost) return;

    int lowerBound = calculateLowerBound(coveredLocations, locations);
    if (currentCost + lowerBound >= bestCost) return;

    // Check if we've reached the end of our search
    if (index >= routes.size() || currentCombination.size() == static_cast<unsigned int>(num_vehicles)) {
        // Check if we've covered all non-depot locations
        int nonDepotLocations = 0;
        for (const auto& point : locations) {
            if (point.id != 0) nonDepotLocations++;
        }

        if (coveredLocations.size() == nonDepotLocations) {
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

    // Option 1: Try adding the current route if it does not cause overlap
    bool overlap = false;
    set<int> newLocations;

    for (int loc : routes[index]) {
        if (loc == 0) continue; // Skip depot
        if (coveredLocations.find(loc) != coveredLocations.end()) {
            overlap = true;
            break;
        }
        newLocations.insert(loc);
    }

    if (!overlap) {
        // Add this route and continue search
        currentCombination.push_back(routes[index]);
        for (int loc : newLocations) coveredLocations.insert(loc);

        FindBestCombination(routes, routeCosts, currentCombination, index + 1,
                          locations, bestCost, bestCombination, num_vehicles,
                          coveredLocations, currentCost + routeCosts[index]);

        // Backtrack
        for (int loc : newLocations) coveredLocations.erase(loc);
        currentCombination.pop_back();
    }

    // Option 2: Skip the current route and move to the next
    FindBestCombination(routes, routeCosts, currentCombination, index + 1,
                      locations, bestCost, bestCombination, num_vehicles,
                      coveredLocations, currentCost);
}

double EuclideanSolver::calculateDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

double EuclideanSolver::calculateRouteCost(const vector<int>& route, const vector<Point>& locations) {
    if (route.size() <= 1) return 0;

    double totalCost = 0;
    // Helper lambda to find a point by its ID
    auto findPoint = [&locations](int id) -> const Point& {
        for (const auto& point : locations) {
            if (point.id == id) return point;
        }
        static Point dummy = {-1, 0, 0};
        return dummy;
    };

    for (unsigned int i = 0; i < route.size() - 1; i++) {
        const Point& p1 = findPoint(route[i]);
        const Point& p2 = findPoint(route[i + 1]);
        totalCost += calculateDistance(p1, p2);
    }
    return totalCost;
}

vector<vector<int>> EuclideanSolver::GenerateAllCombinations(const vector<Point>& locations,
                                                          const map<int, int>& demand,
                                                          int capacity, int max_stops) {
    vector<vector<int>> routes;
    unsigned long n = locations.size();
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
        if (!invalid && !route.empty() && route.size() <= (unsigned int)max_stops) {
            if (route[0] != 0) route.insert(route.begin(), 0);
            if (route.back() != 0) route.push_back(0);
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