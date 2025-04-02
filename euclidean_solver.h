#ifndef EUCLIDEAN_SOLVER_H
#define EUCLIDEAN_SOLVER_H

#include <vector>
#include <map>
#include <set>

using namespace std;

struct Point {
    int id;
    double x;
    double y;
};

class EuclideanSolver {
public:
    // Main solving method for the Vehicle Routing Problem
    vector<vector<int>> solve(const vector<Point> &locations, const map<int, int> &demand,
                            int capacity, int max_stops, int &bestCost, int num_vehicles);

    // Helper methods
    static double calculateDistance(const Point &p1, const Point &p2);
    static double calculateRouteCost(const vector<int> &route, const vector<Point> &locations);

private:
    // Calculate lower bound for branch and bound optimization
    int calculateLowerBound(const set<int>& coveredLocations, const vector<Point>& locations);

    // Recursive backtracking search for optimal solution
    void FindBestCombination(const vector<vector<int>> &routes, const vector<int> &routeCosts,
                           vector<vector<int>> &currentCombination, unsigned int index,
                           const vector<Point> &locations, int &bestCost,
                           vector<vector<int>> &bestCombination, int num_vehicles,
                           set<int> &coveredLocations, int currentCost);

    // Generate all valid route combinations considering constraints
    static vector<vector<int>> GenerateAllCombinations(const vector<Point> &locations,
                                                     const map<int, int> &demand,
                                                     int capacity, int max_stops);
};

#endif // EUCLIDEAN_SOLVER_H