#pragma once
#include "Location.h"
#include <vector>
#include <string>

class DataLoader {
public:
    static bool load(const std::string& filename, int& numVehicles, int& maxStops, int& capacity, std::vector<Location>& locations);
};