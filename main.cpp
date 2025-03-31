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
        {0, 50.0, 50.0},  // Depot
        {1, 20.5, 30.2},
        {2, 75.8, 40.3},
        {3, 45.2, 60.8},
        {4, 88.5, 25.3},
        {5, 35.6, 75.2},
        {6, 60.3, 85.7},
        {7, 15.8, 65.4}
    };

    map<int, int> demand = {
        {0, 0},    // Depot has no demand
        {1, 15},
        {2, 22},
        {3, 18},
        {4, 12},
        {5, 25},
        {6, 20},
        {7, 10}
    };

    // Set capacity, max stops, and number of vehicles
    int capacity = 100;
    int max_stops = 5;
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

    vector<vector<int>> solution = EuclideanSolver::solve(locations, demand, capacity, max_stops, bestCost, num_vehicles);

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
