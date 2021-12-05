#pragma once
#include <iostream>
#include "risk_path.h"


void plan_env_pop_rtree()
{
    // Population Map as RTree from random sample points over raster. Geojson made by QGIS. 

    PlanningSetup planner = PlanningSetup();
    std::array<double, 3> start_point = { 49.86462268679067, 8.657507656252882, 100 };
    std::array<double, 3> goal_point = { 50.107998827159896, 8.68757388575945, 100 };
    planner.set_start_goal(start_point, goal_point);

    const char* filename = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/data/ghs_pop_random_sample_points.geojson";
    GeoJsonReader reader = GeoJsonReader(filename);
    auto points = reader.get_points();
    std::shared_ptr<RTreeBox> rtree = std::make_shared<RTreeBox>(points);
    planner.set_rtee_objective(rtree);

    const char* path = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/data/test.geojson";
    planner.set_validity_checker(path);
    
    planner.set_boundaries();
    planner.solve();
}

void plan_env_1()
{
    PlanningSetup planner = PlanningSetup();
    std::array<double, 3> start_point = { 49.86462268679067, 8.657507656252882, 100 };
    std::array<double, 3> goal_point = { 50.107998827159896, 8.68757388575945, 100 };
    planner.set_start_goal(start_point, goal_point);
    
    auto pop_map = std::make_shared<PopulationMap>();
    planner.set_objective(pop_map);

    const char* path = "C:/Users/carst/OneDrive/Projekte/risk-path/risk_path/data/test.geojson";
    planner.set_validity_checker(path);

    planner.set_boundaries();
    planner.solve();
}

int main(int /*argc*/, char** /*argv*/)
{
    plan_env_pop_rtree();
}