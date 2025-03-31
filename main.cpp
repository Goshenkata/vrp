#include "euclidean_solver.h"
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include <climits>

using namespace std;
using namespace std::chrono;

int main() {
    // Hardcoded VRP data
    vector<Point> locations = {
        {0, 0, 0},  // Depot
        {1, 5, 5},
        {2, 10, 10},
        {3, 15, 5},
        {4, 20, 10},
        {5, 25, 5},
        {6, 30, 10},
        {7, 35, 15},
        {8, 40, 10},
        {9, 45, 5},
        {10, 50, 10},
        {11, 55, 15},
        {12, 60, 10},
        {13, 65, 5},
    };

    map<int, int> demand = {
        {0, 0},    // Depot has no demand
        {1, 10},
        {2, 15},
        {3, 20},
        {4, 10},
        {5, 15},
        {6, 20},
        {7, 25},
        {8, 10},
        {9, 15},
        {10, 20},
        {11, 25},
        {12, 10},
        {13, 15},
    };


    // Set capacity, max stops, and number of vehicles
    int capacity = 500;
    int max_stops = 10000;
    int num_vehicles = 3;  // Number of vehicles available

    cout << "VRP Problem Information:" << endl;
    cout << "Number of locations: " << locations.size() << endl;
    cout << "Vehicle capacity: " << capacity << endl;
    cout << "Maximum stops: " << max_stops << endl;
    cout << "Number of vehicles: " << num_vehicles << endl;

    // Print locations
    cout << "\nLocations:" << endl;
    for (const auto& point : locations) {
        cout << "ID: " << point.id << ", Coordinates: (" << point.x << ", " << point.y
             << "), Demand: " << demand[point.id] << endl;
    }

    // Solve VRP and measure execution time
    int bestCost = INT_MAX;
    auto start = high_resolution_clock::now();

    EuclideanSolver *solver = new EuclideanSolver();
    vector<vector<int>> solution = solver->solve(locations, demand, capacity, max_stops, bestCost, num_vehicles);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    // Print results
    cout << "\nSolution found in " << duration.count() << " milliseconds." << endl;
    cout << "Best cost: " << bestCost << endl;
    cout << "Routes:" << endl;

    for (size_t i = 0; i < solution.size(); i++) {
        cout << "Route " << (i+1) << ": ";
        for (size_t j = 0; j < solution[i].size(); j++) {
            cout << solution[i][j];
            if (j < solution[i].size() - 1) cout << " -> ";
        }

        // Calculate and display route cost
        double routeCost = EuclideanSolver::calculateRouteCost(solution[i], locations);
        cout << " (Cost: " << routeCost << ")" << endl;

        // Calculate and display total demand for this route
        int routeDemand = 0;
        for (int loc : solution[i]) {
            routeDemand += demand[loc];
        }
        cout << "   Total demand: " << routeDemand << " / " << capacity << endl;
    }

    return 0;
}
