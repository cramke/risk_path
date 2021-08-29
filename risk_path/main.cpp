#pragma once
#include <iostream>
#include "risk_path.cpp"

void plan_env_1()
{
    std::array<double, 3> start_point = { 49.86462268679067, 8.657507656252882, 100 };
    std::array<double, 3> goal_point = { 50.107998827159896, 8.68757388575945, 100 };
    auto pop_map = std::make_shared<PopulationMap>();

    PlanningSetup planner = PlanningSetup();
    planner.set_start_goal(start_point, goal_point);
    planner.set_boundaries();
    planner.set_validity_checker(start_point, goal_point);
    planner.set_objective(pop_map);
    planner.solve();
}

int main(int /*argc*/, char** /*argv*/)
{
    GeoJsonReader reader = GeoJsonReader();
    auto polys = reader.get_polygons();
    RTree rtree = RTree(polys);

    box query_box(point(0, 0), point(60, 60));
    std::vector<value> result;
    rtree.rtree.query(bg::index::contains(point(8.660146140, 49.884646188)), std::back_inserter(result));
    if (result.empty())
    {
        std::cout << "Valid State" << std::endl;
    }
    else
    {
        std::cout << "Invalid State" << std::endl;
        std::cout << "Intersections with: " << result.size() << std::endl;
    }
}