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

int main(int argc, char* argv[]) {
    // Check if we have the required command line arguments
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <numThreads> <filePath>" << std::endl;
        return 1;
    }

    // Parse command line arguments
    int numThreads = std::stoi(argv[1]);
    std::string filePath = argv[2];

    try {
        // Set the number of threads for OpenMP
        omp_set_num_threads(numThreads);
        std::cout << "Using " << numThreads << " threads" << std::endl;

        // Load configuration and locations from file
        auto config = FileLoader::loadFromFile(filePath);

        // Compute distance matrix
        auto distMatrix = computeDistanceMatrix(config.locations);

        // Simulated Annealing parameters
        int iterations = 10000;
        double initTemp = 1000.0;
        double finalTemp = 1.0;
        double cooling = 0.995;

        int numRuns = numThreads;  // Set number of independent SA runs to match thread count
        Solution bestSolution;
        bestSolution.total_distance = std::numeric_limits<int>::max();

        auto startTime = std::chrono::high_resolution_clock::now();

        // Run multiple SA processes in parallel using OpenMP
        #pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < numRuns; i++) {
            Solution sol = simulatedAnnealing(
                config.locations,
                config.numVehicles,
                config.capacity,
                config.maxStops,
                distMatrix,
                initTemp,
                finalTemp,
                cooling,
                iterations
            );

            #pragma omp critical
            {
                if (sol.total_distance < bestSolution.total_distance)
                    bestSolution = sol;
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = endTime - startTime;

        std::cout << "Best solution found in " << elapsed.count() << " seconds." << std::endl;
        printSolution(bestSolution);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}