#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <chrono>
#include <omp.h>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>

struct Location {
    int x, y, demand;
};

struct Route {
    std::vector<int> path;
    int total_distance;
    int capacity_used;
};

struct Solution {
    std::vector<Route> routes;
    int total_distance;
};

// Euclidean distance between two locations
int calculateDistance(const Location& a, const Location& b) {
    return static_cast<int>(std::sqrt((a.x - b.x)*(a.x - b.x) +
                                        (a.y - b.y)*(a.y - b.y)));
}

// Build distance matrix for all locations
std::vector<std::vector<int>> computeDistanceMatrix(const std::vector<Location>& locations) {
    int n = locations.size();
    std::vector<std::vector<int>> matrix(n, std::vector<int>(n));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            matrix[i][j] = calculateDistance(locations[i], locations[j]);
    return matrix;
}

// Compute total distance for a given route using the distance matrix
int computeRouteDistance(const Route& route, const std::vector<std::vector<int>>& distMatrix) {
    int dist = 0;
    for (size_t i = 1; i < route.path.size(); i++)
        dist += distMatrix[route.path[i-1]][route.path[i]];
    return dist;
}

// Compute the total distance for the entire solution
int computeSolutionDistance(const Solution& sol) {
    int total = 0;
    for (const auto& r : sol.routes)
        total += r.total_distance;
    return total;
}

// Greedy initial solution: assign customers sequentially, start a new route when capacity or stops limit is hit.
Solution generateInitialSolution(const std::vector<Location>& locations, int numVehicles,
                                 int capacity, int maxStops,
                                 const std::vector<std::vector<int>>& distMatrix) {
    Solution sol;
    Route current;
    current.path.push_back(0); // start at depot
    current.capacity_used = 0;
    current.total_distance = 0;

    for (size_t i = 1; i < locations.size(); i++) {
        // If adding a customer violates max stops or capacity, finish current route.
        if ((current.path.size() - 1 >= static_cast<size_t>(maxStops)) ||
            (current.capacity_used + locations[i].demand > capacity)) {
            current.path.push_back(0); // return to depot
            current.total_distance = computeRouteDistance(current, distMatrix);
            sol.routes.push_back(current);
            current = Route();
            current.path.push_back(0);
            current.capacity_used = 0;
            current.total_distance = 0;
        }
        current.path.push_back(i);
        current.capacity_used += locations[i].demand;
    }
    // Finalize the last route.
    current.path.push_back(0);
    current.total_distance = computeRouteDistance(current, distMatrix);
    sol.routes.push_back(current);
    sol.total_distance = computeSolutionDistance(sol);
    return sol;
}

// Check if all routes satisfy capacity and max stops constraints.
bool isFeasible(const Solution& sol, int capacity, int maxStops,
                const std::vector<Location>& locations) {
    for (const auto& route : sol.routes) {
        // Exclude depots (first and last)
        if (route.path.size() - 2 > static_cast<size_t>(maxStops))
            return false;
        int cap = 0;
        for (size_t i = 1; i < route.path.size()-1; i++)
            cap += locations[route.path[i]].demand;
        if (cap > capacity)
            return false;
    }
    return true;
}

// Create a neighbor solution using either an intra-route swap or moving a customer between routes.
Solution getNeighbor(const Solution& sol, const std::vector<Location>& locations,
                     int capacity, int maxStops,
                     const std::vector<std::vector<int>>& distMatrix,
                     std::mt19937& rng) {
    Solution neighbor = sol;
    std::uniform_int_distribution<> routeDist(0, neighbor.routes.size() - 1);
    int routeIdx = routeDist(rng);
    Route& route = neighbor.routes[routeIdx];
    std::uniform_int_distribution<> moveDist(0, 1);
    int moveType = moveDist(rng);

    if (moveType == 0 && route.path.size() > 3) {
        // Intra-route swap (avoid depot positions)
        std::uniform_int_distribution<> posDist(1, route.path.size() - 2);
        int i = posDist(rng), j = posDist(rng);
        if (i != j)
            std::swap(route.path[i], route.path[j]);
        route.total_distance = computeRouteDistance(route, distMatrix);
    }
    else if (neighbor.routes.size() > 1) {
        // Move a customer from one route to another.
        std::uniform_int_distribution<> srcDist(0, neighbor.routes.size() - 1);
        int src = srcDist(rng);
        int dst = src;
        while (dst == src)
            dst = srcDist(rng);
        Route& srcRoute = neighbor.routes[src];
        Route& dstRoute = neighbor.routes[dst];
        if (srcRoute.path.size() > 3) {
            std::uniform_int_distribution<> posDist(1, srcRoute.path.size() - 2);
            int pos = posDist(rng);
            int customer = srcRoute.path[pos];
            srcRoute.path.erase(srcRoute.path.begin() + pos);
            srcRoute.total_distance = computeRouteDistance(srcRoute, distMatrix);
            std::uniform_int_distribution<> dstPosDist(1, dstRoute.path.size() - 1);
            int dstPos = dstPosDist(rng);
            dstRoute.path.insert(dstRoute.path.begin() + dstPos, customer);
            dstRoute.total_distance = computeRouteDistance(dstRoute, distMatrix);
        }
    }
    // Recalculate overall distance.
    neighbor.total_distance = 0;
    for (auto& r : neighbor.routes)
        neighbor.total_distance += r.total_distance;
    if (!isFeasible(neighbor, capacity, maxStops, locations))
        return sol; // return original solution if neighbor is infeasible.
    return neighbor;
}

// Simulated Annealing algorithm for VRP.
Solution simulatedAnnealing(const std::vector<Location>& locations, int numVehicles,
                            int capacity, int maxStops,
                            const std::vector<std::vector<int>>& distMatrix,
                            double initialTemp, double finalTemp,
                            double coolingRate, int iterations) {
    std::random_device rd;
    std::mt19937 rng(rd());
    Solution current = generateInitialSolution(locations, numVehicles, capacity, maxStops, distMatrix);
    Solution best = current;
    double temp = initialTemp;
    std::uniform_real_distribution<> realDist(0.0, 1.0);

    for (int iter = 0; iter < iterations && temp > finalTemp; iter++) {
        Solution neighbor = getNeighbor(current, locations, capacity, maxStops, distMatrix, rng);
        int delta = neighbor.total_distance - current.total_distance;
        if (delta < 0 || realDist(rng) < std::exp(-delta / temp)) {
            current = neighbor;
            if (current.total_distance < best.total_distance)
                best = current;
        }
        temp *= coolingRate;
    }
    return best;
}

// Simple function to print a solution.
void printSolution(const Solution& sol) {
    std::cout << "Total Distance: " << sol.total_distance << "\n";
    for (size_t i = 0; i < sol.routes.size(); i++) {
        std::cout << "Vehicle " << i + 1 << ": ";
        for (size_t j = 0; j < sol.routes[i].path.size(); j++) {
            std::cout << sol.routes[i].path[j];
            if (j < sol.routes[i].path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << " (Distance: " << sol.routes[i].total_distance << ")\n";
    }
}

class FileLoader {
public:
    struct VRPConfig {
        int numVehicles;
        int maxStops;
        int capacity;
        std::vector<Location> locations;
    };

    static VRPConfig loadFromFile(const std::string& filePath) {
        VRPConfig config;
        std::ifstream file(filePath);

        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filePath);
        }

        // Read the first three lines for numVehicles, maxStops, and capacity
        std::string line;
        if (std::getline(file, line)) {
            config.numVehicles = std::stoi(line);
        } else {
            throw std::runtime_error("Failed to read numVehicles from file");
        }

        if (std::getline(file, line)) {
            config.maxStops = std::stoi(line);
        } else {
            throw std::runtime_error("Failed to read maxStops from file");
        }

        if (std::getline(file, line)) {
            config.capacity = std::stoi(line);
        } else {
            throw std::runtime_error("Failed to read capacity from file");
        }

        // Read location data (x, y, demand)
        int locationIndex = 0;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            int x, y, demand;

            if (iss >> x >> y >> demand) {
                config.locations.push_back({x, y, demand});
            } else {
                std::cerr << "Warning: Invalid location format at line " << (locationIndex + 4) << std::endl;
            }

            locationIndex++;
        }

        if (config.locations.empty()) {
            throw std::runtime_error("No locations found in file");
        }

        std::cout << "Loaded configuration: " << std::endl;
        std::cout << "  - Number of vehicles: " << config.numVehicles << std::endl;
        std::cout << "  - Max stops per vehicle: " << config.maxStops << std::endl;
        std::cout << "  - Vehicle capacity: " << config.capacity << std::endl;
        std::cout << "  - Number of locations: " << config.locations.size() << std::endl;

        return config;
    }
};

int main() {
    // Define locations (first entry is depot)
    std::vector<Location> locations = {
        {0, 0, 0}, // Depot
        {10, 15, 5}, {25, 35, 7}, {35, 12, 3}, {48, 33, 6}, {56, 47, 4},
        {32, 52, 8}, {20, 38, 5}, {42, 25, 9}, {15, 45, 4}, {65, 28, 7},
        {29, 17, 6}, {77, 33, 8}, {38, 66, 5}, {52, 15, 9}, {28, 75, 7},
        {85, 42, 4}, {44, 62, 6}, {36, 23, 3}, {57, 54, 5}, {12, 33, 6},
        {32, 52, 8}, {20, 38, 5}, {42, 25, 9}, {15, 45, 4}, {65, 28, 7},
        {29, 17, 6}, {77, 33, 8}, {38, 66, 5}, {52, 15, 9}, {28, 75, 7},
        {85, 42, 4}, {44, 62, 6}, {36, 23, 3}, {57, 54, 5}, {12, 33, 6},
        {24, 48, 4}, {66, 18, 7}, {31, 43, 5}, {49, 49, 9}, {18, 59, 4},
        {41, 37, 8}, {53, 63, 6}, {30, 21, 3}, {76, 45, 5}, {22, 66, 4},
        {24, 48, 4}, {66, 18, 7}, {31, 43, 5}, {49, 49, 9}, {18, 59, 4},
        {41, 37, 8}, {53, 63, 6}, {30, 21, 3}, {76, 45, 5}, {22, 66, 4},
        {37, 12, 6}, {61, 26, 8}, {70, 55, 4}, {33, 28, 7}, {14, 50, 5},
        {60, 39, 6}, {27, 60, 4}, {43, 33, 9}, {19, 24, 6}, {55, 72, 5},
        {48, 14, 7}, {34, 69, 3}, {66, 62, 8}, {23, 31, 4}, {39, 48, 7},
        {50, 58, 6}, {26, 36, 5}, {62, 34, 4}, {35, 41, 9}, {17, 53, 6},
        {45, 21, 7}, {40, 61, 5}, {13, 27, 4}, {54, 50, 8}, {47, 29, 3},
        {58, 65, 6}, {36, 18, 5}, {20, 42, 7}, {72, 37, 4}, {25, 55, 6},
        {58, 65, 6}, {36, 18, 5}, {20, 42, 7}, {72, 37, 4}, {25, 55, 6},
        {31, 67, 3}, {68, 49, 5}, {21, 29, 4}, {63, 41, 6}, {28, 39, 8},
        {29, 17, 6}, {77, 33, 8}, {38, 66, 5}, {52, 15, 9}, {28, 75, 7},
        {29, 17, 6}, {77, 33, 8}, {38, 66, 5}, {52, 15, 9}, {28, 75, 7},
        {85, 42, 4}, {44, 62, 6}, {36, 23, 3}, {57, 54, 5}, {12, 33, 6},
        {31, 67, 3}, {68, 49, 5}, {21, 29, 4}, {63, 41, 6}, {28, 39, 8},
        {29, 17, 6}, {77, 33, 8}, {38, 66, 5}, {52, 15, 9}, {28, 75, 7},
        {29, 17, 6}, {77, 33, 8}, {38, 66, 5}, {52, 15, 9}, {28, 75, 7},
        {85, 42, 4}, {44, 62, 6}, {36, 23, 3}, {57, 54, 5}, {12, 33, 6},
        {24, 48, 4}, {66, 18, 7}, {31, 43, 5}, {49, 49, 9}, {18, 59, 4},
        {41, 37, 8}, {53, 63, 6}, {30, 21, 3}, {76, 45, 5}, {22, 66, 4},
        {24, 48, 4}, {66, 18, 7}, {31, 43, 5}, {49, 49, 9}, {18, 59, 4},
        {29, 17, 6}, {77, 33, 8}, {38, 66, 5}, {52, 15, 9}, {28, 75, 7},
        {85, 42, 4}, {44, 62, 6}, {36, 23, 3}, {57, 54, 5}, {12, 33, 6},
        {24, 48, 4}, {66, 18, 7}, {31, 43, 5}, {49, 49, 9}, {18, 59, 4},
        {41, 37, 8}, {53, 63, 6}, {30, 21, 3}, {76, 45, 5}, {22, 66, 4},
        {37, 12, 6}, {61, 26, 8}, {70, 55, 4}, {33, 28, 7}, {14, 50, 5},
        {60, 39, 6}, {27, 60, 4}, {43, 33, 9}, {19, 24, 6}, {55, 72, 5},
        {48, 14, 7}, {34, 69, 3}, {66, 62, 8}, {23, 31, 4}, {39, 48, 7},
        {85, 42, 4}, {44, 62, 6}, {36, 23, 3}, {57, 54, 5}, {12, 33, 6},
        {24, 48, 4}, {66, 18, 7}, {31, 43, 5}, {49, 49, 9}, {18, 59, 4},
        {41, 37, 8}, {53, 63, 6}, {30, 21, 3}, {76, 45, 5}, {22, 66, 4},
        {37, 12, 6}, {61, 26, 8}, {70, 55, 4}, {33, 28, 7}, {14, 50, 5},
        {60, 39, 6}, {27, 60, 4}, {43, 33, 9}, {19, 24, 6}, {55, 72, 5},
        {48, 14, 7}, {34, 69, 3}, {66, 62, 8}, {23, 31, 4}, {39, 48, 7},
        {50, 58, 6}, {26, 36, 5}, {62, 34, 4}, {35, 41, 9}, {17, 53, 6},
        {45, 21, 7}, {40, 61, 5}, {13, 27, 4}, {54, 50, 8}, {47, 29, 3},
        {58, 65, 6}, {36, 18, 5}, {20, 42, 7}, {72, 37, 4}, {25, 55, 6},
        {31, 67, 3}, {68, 49, 5}, {21, 29, 4}, {63, 41, 6}, {28, 39, 8},
        {46, 65, 3}, {60, 24, 7}, {32, 58, 4}, {42, 20, 6}, {15, 35, 5},
        {51, 46, 7}, {34, 32, 4}, {56, 60, 3}, {30, 50, 6}, {19, 40, 5},
        {46, 65, 3}, {60, 24, 7}, {32, 58, 4}, {42, 20, 6}, {15, 35, 5},
        {51, 46, 7}, {34, 32, 4}, {56, 60, 3}, {30, 50, 6}, {19, 40, 5},
        {75, 40, 7}, {44, 44, 8}, {52, 28, 3}, {38, 53, 4}, {61, 59, 6},
        {29, 26, 5}, {67, 33, 7}, {26, 44, 3}, {59, 36, 8}, {16, 47, 4},
        {71, 31, 6}, {33, 55, 5}, {64, 20, 7}, {24, 62, 3}, {73, 46, 6},
        {27, 34, 4}, {49, 56, 5}, {39, 30, 8}, {20, 52, 3}, {57, 38, 6},
        {12, 36, 5}, {70, 29, 7}, {18, 57, 4}, {46, 35, 6}, {35, 63, 5},
        {62, 46, 7}, {21, 48, 4}, {50, 42, 6}, {25, 26, 3}, {58, 44, 8},
        {43, 51, 5}, {16, 60, 6}, {47, 39, 4}, {66, 30, 7}, {36, 25, 3},
        {55, 33, 6}, {14, 43, 5}, {68, 43, 4}, {30, 61, 8}, {72, 50, 5},
        {40, 27, 6}, {19, 30, 7}, {60, 52, 4}, {22, 46, 3}, {53, 48, 6},
        {28, 29, 4}, {45, 53, 5}, {31, 36, 7}, {65, 37, 3}, {23, 57, 6},
        {69, 39, 5}, {26, 42, 4}, {41, 29, 8}, {17, 55, 6}, {59, 31, 7},
        {13, 49, 3}, {63, 53, 5}, {37, 45, 6}, {33, 40, 4}, {48, 31, 8},
        {75, 40, 7}, {44, 44, 8}, {52, 28, 3}, {38, 53, 4}, {61, 59, 6},
        {29, 26, 5}, {67, 33, 7}, {26, 44, 3}, {59, 36, 8}, {16, 47, 4},
        {55, 33, 6}, {14, 43, 5}, {68, 43, 4}, {30, 61, 8}, {72, 50, 5},
        {40, 27, 6}, {19, 30, 7}, {60, 52, 4}, {22, 46, 3}, {53, 48, 6},
        {28, 29, 4}, {45, 53, 5}, {31, 36, 7}, {65, 37, 3}, {23, 57, 6},
        {69, 39, 5}, {26, 42, 4}, {41, 29, 8}, {17, 55, 6}, {59, 31, 7},
        {13, 49, 3}, {63, 53, 5}, {37, 45, 6}, {33, 40, 4}, {48, 31, 8},
        {75, 40, 7}, {44, 44, 8}, {52, 28, 3}, {38, 53, 4}, {61, 59, 6},
        {29, 26, 5}, {67, 33, 7}, {26, 44, 3}, {59, 36, 8}, {16, 47, 4},
        {58, 65, 6}, {36, 18, 5}, {20, 42, 7}, {72, 37, 4}, {25, 55, 6},
        {31, 67, 3}, {68, 49, 5}, {21, 29, 4}, {63, 41, 6}, {28, 39, 8},
        {75, 40, 7}, {44, 44, 8}, {52, 28, 3}, {38, 53, 4}, {61, 59, 6},
        {29, 26, 5}, {67, 33, 7}, {26, 44, 3}, {59, 36, 8}, {16, 47, 4},
        {55, 33, 6}, {14, 43, 5}, {68, 43, 4}, {30, 61, 8}, {72, 50, 5},
        {75, 40, 7}, {44, 44, 8}, {52, 28, 3}, {38, 53, 4}, {61, 59, 6},
        {29, 26, 5}, {67, 33, 7}, {26, 44, 3}, {59, 36, 8}, {16, 47, 4},
        {55, 33, 6}, {14, 43, 5}, {68, 43, 4}, {30, 61, 8}, {72, 50, 5},
        {40, 27, 6}, {19, 30, 7}, {60, 52, 4}, {22, 46, 3}, {53, 48, 6},
        {28, 29, 4}, {45, 53, 5}, {31, 36, 7}, {65, 37, 3}, {23, 57, 6},
        {69, 39, 5}, {26, 42, 4}, {41, 29, 8}, {17, 55, 6}, {59, 31, 7},
        {13, 49, 3}, {63, 53, 5}, {37, 45, 6}, {33, 40, 4}, {48, 31, 8},
        {75, 40, 7}, {44, 44, 8}, {52, 28, 3}, {38, 53, 4}, {61, 59, 6},
        {29, 26, 5}, {67, 33, 7}, {26, 44, 3}, {59, 36, 8}, {16, 47, 4},
        {58, 65, 6}, {36, 18, 5}, {20, 42, 7}, {72, 37, 4}, {25, 55, 6},
        {31, 67, 3}, {68, 49, 5}, {21, 29, 4}, {63, 41, 6}, {28, 39, 8},
        {75, 40, 7}, {44, 44, 8}, {52, 28, 3}, {38, 53, 4}, {61, 59, 6},
    };

    int numVehicles = 8;
    int maxStops    = 10;
    int capacity    = 35;
    omp_set_num_threads(24);

    auto distMatrix = computeDistanceMatrix(locations);

    // Simulated Annealing parameters.
    int iterations   = 10000;
    double initTemp  = 1000.0;
    double finalTemp = 1.0;
    double cooling   = 0.995;

    int numRuns = 8;  // number of independent SA runs
    Solution bestSolution;
    bestSolution.total_distance = std::numeric_limits<int>::max();

    auto startTime = std::chrono::high_resolution_clock::now();

    // Run multiple SA processes in parallel using OpenMP.
    #pragma omp parallel for
    for (int i = 0; i < numRuns; i++) {
        Solution sol = simulatedAnnealing(locations, numVehicles, capacity, maxStops,
                                          distMatrix, initTemp, finalTemp, cooling, iterations);
        #pragma omp critical
        {
            if (sol.total_distance < bestSolution.total_distance)
                bestSolution = sol;
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    std::cout << "Best solution found in " << elapsed.count() << " seconds.\n";
    printSolution(bestSolution);

    return 0;
}
