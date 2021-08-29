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
    planner.set_validity_checker();
    planner.set_objective(pop_map);
    planner.solve();
}

int main(int /*argc*/, char** /*argv*/)
{
    plan_env_1();
}