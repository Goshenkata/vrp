#ifndef VRP_SOLVER_H
#define VRP_SOLVER_H

#include <vector>
#include <random>
#include <cmath>
#include <omp.h>
#include <Route.h>
#include <Solution.h>

#include "Location.h"

class VRPSolver {
public:
    VRPSolver(const std::vector<Location>& locations, int numVehicles, int capacity, int maxStops);

    std::vector<std::vector<int>> computeDistanceMatrix();
    int computeRouteDistance(const Route& route, const std::vector<std::vector<int>>& distMatrix);
    int computeSolutionDistance(const Solution& sol);
    Solution generateInitialSolution(const std::vector<std::vector<int>>& distMatrix);
    bool isFeasible(const Solution& sol, const std::vector<std::vector<int>>& distMatrix);
    Solution getNeighbor(const Solution& sol, const std::vector<std::vector<int>>& distMatrix);
    Solution simulatedAnnealing(const std::vector<std::vector<int>>& distMatrix, double initialTemp, double finalTemp, double coolingRate, int iterations);

private:
    std::vector<Location> locations_;
    int numVehicles_;
    int capacity_;
    int maxStops_;
    std::mt19937 rng_;
};

#endif // VRP_SOLVER_H
