#include "VRPSolver.h"

#include <cmath>
#include <algorithm>
#include <omp.h>
#include <random>
// Calculate the Euclidean distance between two locations
int calculateDistance(const Location& loc1, const Location& loc2) {
    int dx = loc1.x - loc2.x;
    int dy = loc1.y - loc2.y;
    return static_cast<int>(std::sqrt(dx * dx + dy * dy));
}

VRPSolver::VRPSolver(const std::vector<Location>& locations, int numVehicles, int capacity, int maxStops)
    : locations_(locations), numVehicles_(numVehicles), capacity_(capacity), maxStops_(maxStops), rng_(std::random_device{}()) {}

// Parallelize distance matrix computation
std::vector<std::vector<int>> VRPSolver::computeDistanceMatrix() {
    int n = locations_.size();
    std::vector<std::vector<int>> matrix(n, std::vector<int>(n));

    #pragma omp parallel for collapse(2) // Parallelize nested loops
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            matrix[i][j] = calculateDistance(locations_[i], locations_[j]);
        }
    }
    return matrix;
}

// Compute the total distance for a given route using the distance matrix
int VRPSolver::computeRouteDistance(const Route& route, const std::vector<std::vector<int>>& distMatrix) {
    int dist = 0;
    for (size_t i = 1; i < route.path.size(); i++) {
        dist += distMatrix[route.path[i - 1]][route.path[i]];
    }
    return dist;
}

// Compute the total distance for the entire solution
int VRPSolver::computeSolutionDistance(const Solution& sol) {
    int total = 0;
    for (const auto& r : sol.routes) {
        total += r.total_distance;
    }
    return total;
}

// Greedy initial solution: assign customers sequentially, start a new route when capacity or stops limit is hit.
Solution VRPSolver::generateInitialSolution(const std::vector<std::vector<int>>& distMatrix) {
    Solution sol;
    Route current;
    current.path.push_back(0); // Start at depot
    current.capacity_used = 0;
    current.total_distance = 0;

    for (size_t i = 1; i < locations_.size(); i++) {
        if ((current.path.size() - 1 >= static_cast<size_t>(maxStops_)) ||
            (current.capacity_used + locations_[i].demand > capacity_)) {
            current.path.push_back(0); // Return to depot
            current.total_distance = computeRouteDistance(current, distMatrix);
            sol.routes.push_back(current);
            current = Route();
            current.path.push_back(0);
            current.capacity_used = 0;
            current.total_distance = 0;
        }
        current.path.push_back(i);
        current.capacity_used += locations_[i].demand;
    }
    current.path.push_back(0);
    current.total_distance = computeRouteDistance(current, distMatrix);
    sol.routes.push_back(current);
    sol.total_distance = computeSolutionDistance(sol);
    return sol;
}

// Check if all routes satisfy capacity and max stops constraints
bool VRPSolver::isFeasible(const Solution& sol, const std::vector<std::vector<int>>& distMatrix) {
    for (const auto& route : sol.routes) {
        if (route.path.size() - 2 > static_cast<size_t>(maxStops_)) {
            return false;
        }
        int cap = 0;
        for (size_t i = 1; i < route.path.size() - 1; i++) {
            cap += locations_[route.path[i]].demand;
        }
        if (cap > capacity_) {
            return false;
        }
    }
    return true;
}

// Create a neighbor solution using either an intra-route swap or moving a customer between routes.
Solution VRPSolver::getNeighbor(const Solution& sol, const std::vector<std::vector<int>>& distMatrix) {
    Solution neighbor = sol;
    std::uniform_int_distribution<> routeDist(0, neighbor.routes.size() - 1);
    int routeIdx = routeDist(rng_);
    Route& route = neighbor.routes[routeIdx];
    std::uniform_int_distribution<> moveDist(0, 1);
    int moveType = moveDist(rng_);

    if (moveType == 0 && route.path.size() > 3) {
        // Intra-route swap
        std::uniform_int_distribution<> posDist(1, route.path.size() - 2);
        int i = posDist(rng_), j = posDist(rng_);
        if (i != j) {
            std::swap(route.path[i], route.path[j]);
        }
        route.total_distance = computeRouteDistance(route, distMatrix);
    }
    else if (neighbor.routes.size() > 1) {
        // Move a customer from one route to another
        #pragma omp parallel for
        for (int i = 0; i < neighbor.routes.size(); i++) {
            int src = i;
            int dst = (src + 1) % neighbor.routes.size();
            Route& srcRoute = neighbor.routes[src];
            Route& dstRoute = neighbor.routes[dst];
            if (srcRoute.path.size() > 3) {
                std::uniform_int_distribution<> posDist(1, srcRoute.path.size() - 2);
                int pos = posDist(rng_);
                int customer = srcRoute.path[pos];
                srcRoute.path.erase(srcRoute.path.begin() + pos);
                srcRoute.total_distance = computeRouteDistance(srcRoute, distMatrix);
                std::uniform_int_distribution<> dstPosDist(1, dstRoute.path.size() - 1);
                int dstPos = dstPosDist(rng_);
                dstRoute.path.insert(dstRoute.path.begin() + dstPos, customer);
                dstRoute.total_distance = computeRouteDistance(dstRoute, distMatrix);
            }
        }
    }

    #pragma omp parallel for
    for (auto& r : neighbor.routes) {
        neighbor.total_distance += r.total_distance;
    }

    if (!isFeasible(neighbor, distMatrix)) {
        return sol; // Return original solution if neighbor is infeasible
    }
    return neighbor;
}

// Simulated Annealing algorithm for VRP
Solution VRPSolver::simulatedAnnealing(const std::vector<std::vector<int>>& distMatrix,
                                                  double initialTemp, double finalTemp,
                                                  double coolingRate, int iterations) {
    Solution current = generateInitialSolution(distMatrix);
    Solution best = current;
    double temp = initialTemp;
    std::uniform_real_distribution<> realDist(0.0, 1.0);

    for (int iter = 0; iter < iterations && temp > finalTemp; iter++) {
        Solution neighbor = getNeighbor(current, distMatrix);
        int delta = neighbor.total_distance - current.total_distance;
        if (delta < 0 || realDist(rng_) < std::exp(-delta / temp)) {
            current = neighbor;
            if (current.total_distance < best.total_distance) {
                best = current;
            }
        }
        temp *= coolingRate;
    }

    return best;
}
