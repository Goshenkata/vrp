#include "euclidean_solver.h"
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include <climits>

using namespace std;
using namespace std::chrono;

int main() {
    // Define locations with depot at index 0 (ID: 0)
    vector<Point> locations = {
        {0, 0, 0},     // Depot
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
        {14, 103, 30}
        // Removed duplicate location (id 15 was at same coordinates as id 4)
    };

    // Define demand for each location (depot has no demand)
    map<int, int> demand = {
        {0, 0},     // Depot has no demand
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
        {14, 20}
    };

    // Set constraints
    int capacity = 300;
    int max_stops = 10;
    int num_vehicles = 3;

    // Print problem info
    cout << "VRP Problem Information:" << endl;
    cout << "Number of locations: " << locations.size() << endl;
    cout << "Vehicle capacity: " << capacity << endl;
    cout << "Maximum stops: " << max_stops << endl;
    cout << "Number of vehicles: " << num_vehicles << endl;

    // Print locations and their demand
    cout << "\nLocations:" << endl;
    for (const auto& point : locations) {
        cout << "ID: " << point.id << ", Coordinates: (" << point.x << ", " << point.y
             << "), Demand: " << demand[point.id] << endl;
    }

    // Solve VRP and measure execution time
    int bestCost = INT_MAX;
    auto start = high_resolution_clock::now();

    EuclideanSolver solver = EuclideanSolver();
    vector<vector<int>> solution = solver.solve(locations, demand, capacity, max_stops, bestCost, num_vehicles);

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

        // Calculate and display route cost and demand
        double routeCost = solver.calculateRouteCost(solution[i], locations);
        cout << " (Cost: " << routeCost << ")" << endl;

        // Calculate route demand
        int routeDemand = 0;
        for (int loc : solution[i]) {
            routeDemand += demand[loc];
        }
        cout << "   Total demand: " << routeDemand << " / " << capacity << endl;
    }

    return 0;
}