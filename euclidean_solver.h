#ifndef EUCLIDEAN_SOLVER_H
#define EUCLIDEAN_SOLVER_H

#include <vector>
#include <map>
#include <cmath>
#include <set>

using namespace std;

struct Point {
    int id;
    double x;
    double y;
};

class EuclideanSolver {
public:

       // New method declarations
    int calculateLowerBound(const set<int>& coveredLocations,
                           const vector<Point>& locations);

    void FindBestCombination(const vector<vector<int>>& routes,
                           const vector<int>& routeCosts,
                           vector<vector<int>>& currentCombination,
                           size_t index, const vector<Point>& locations,
                           int& bestCost, vector<vector<int>>& bestCombination,
                           int num_vehicles, set<int>& coveredLocations,
                           int currentCost);
    // Method to solve the Vehicle Routing Problem for multiple vehicles
    vector<vector<int> > solve(const vector<Point> &locations, const map<int, int> &demand,
                                      int capacity, int max_stops, int &bestCost, int num_vehicles);

    // Method to calculate the Euclidean distance between two points
    static double calculateDistance(const Point &p1, const Point &p2);

    // Method to calculate the total cost of a route
    static double calculateRouteCost(const vector<int> &route, const vector<Point> &locations);

    // Method to check if a given route is valid
    static bool verifyValidRoute(const vector<int> &route);

private:
    // Method to generate all valid combinations of routes considering capacity and maximum stops
    static vector<vector<int> > GenerateAllCombinations(const vector<Point> &locations,
                                                        const map<int, int> &demand,
                                                        int capacity, int max_stops);

    // Method to calculate the total cost of all routes in a solution
    static int CalculateTotalCost(const vector<vector<int> > &routes, const vector<Point> &locations);

    // Method to check if the combination of routes covers all locations (except depot)
    static bool coversAllLocations(const vector<vector<int> > &combination, const vector<Point> &locations);

    // Recursive method to find the best combination of routes
    static void FindBestCombination(const vector<vector<int> > &routes,
                                    vector<vector<int> > &currentCombination,
                                    size_t index, const vector<Point> &locations,
                                    int &bestCost, vector<vector<int> > &bestCombination,
                                    int num_vehicles);
};

#endif // EUCLIDEAN_SOLVER_H
