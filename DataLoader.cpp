#include "DataLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>

bool DataLoader::load(const std::string &filename, int &numVehicles, int &maxStops, int &capacity,
    std::vector<Location> &locations) {

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return false;
    }

    file >> numVehicles >> maxStops >> capacity;

    Location location;
    int counter = 0;
    while (file >> location.x >> location.y >> location.demand) {
        location.id = counter++;
        counter++;
        locations.push_back(location);
    }

    return true;
}
