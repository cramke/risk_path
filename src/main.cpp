#include "main.h"

void plan_env_pop_rtree()
{
    // Population Map as RTree from random sample points over raster. Geojson made by QGIS. 

    PlanningSetup planner = PlanningSetup();
    std::array<double, 3> start_point = { 8.657507656252882, 49.86462268679067, 100 };
    std::array<double, 3> goal_point = { 8.68757388575945, 50.107998827159896, 100 };
    planner.set_start_goal(start_point, goal_point);

    const char* filename = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/data/ghs_pop_random_sample_points.geojson";
    // const char* filename = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/data/new.geojson";
    planner.set_rtree_objective(filename);

    const char* path = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/data/test.geojson";
    planner.set_validity_checker(path);
    
    std::array<double, 3> lower = {5.868, 49.0, 100.0};
    std::array<double, 3> higher = {9.0, 51.0, 100.0};
    planner.set_boundaries(lower, higher);

    planner.solve();
}

void plan_env_1()
{
    PlanningSetup planner = PlanningSetup();
    std::array<double, 3> start_point = { 8.657507656252882, 49.86462268679067, 100 };
    std::array<double, 3> goal_point = { 8.68757388575945, 50.107998827159896, 100 };
    planner.set_start_goal(start_point, goal_point);
    
    auto pop_map = std::make_shared<PopulationMap>();
    planner.set_objective(pop_map);

    const char* path = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/data/test.geojson";
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