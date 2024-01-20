#include "main.h"

/**
 * @brief Population Map as RTree from random sample points over raster. 
 * 
 * Geojson made by QGIS. 
*/
void plan_env_pop_rtree()
{
    auto planner = PlanningSetup();
    std::array<double, 3> start_point = { 8.657507656252882, 49.86462268679067, 100 };
    std::array<double, 3> goal_point = { 8.68757388575945, 50.107998827159896, 100 };
    planner.set_start_goal(start_point, goal_point);

    // Use alternatively "/home/samtal/risk_path/ressources/new.geojson" for the new population map.
    const char* filename = "/home/samtal/risk_path/ressources/ghs_pop_random_sample_points.geojson";
    planner.set_rtree_objective(filename);

    const char* path = "/home/samtal/risk_path/ressources/test.geojson";
    planner.set_validity_checker(path);
    
    std::array<double, 3> lower = {5.868, 49.0, 100.0};
    std::array<double, 3> higher = {9.0, 51.0, 100.0};
    planner.set_boundaries(lower, higher);

    planner.solve();
}

/**
 * @brief Population Map as raster. 
 * 
*/
void plan_env_1()
{
    auto planner = PlanningSetup();
    std::array<double, 3> start_point = { 8.657507656252882, 49.86462268679067, 100 };
    std::array<double, 3> goal_point = { 8.68757388575945, 50.107998827159896, 100 };
    planner.set_start_goal(start_point, goal_point);
    
    auto pop_map = std::make_shared<PopulationMap>();
    planner.set_objective(pop_map);

    const char* path = "/home/samtal/risk_path/ressources/test.geojson";
    planner.set_validity_checker(path);

    std::array<double, 3> lower = { 5.868, 49.0, 100.0 };
    std::array<double, 3> higher = { 9.0, 51.0, 100.0 };
    planner.set_boundaries(lower, higher);

    planner.solve();
}

int main(int /*argc*/, char** /*argv*/)
{
    std::cout << "Hello, from risk_path!\n";
    plan_env_pop_rtree();
}