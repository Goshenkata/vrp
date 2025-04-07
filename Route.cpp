#include "Route.h"

Route::Route() : total_distance(0), capacity_used(0) {}

void Route::add_location(int location_id) {
    path.push_back(location_id);
}
