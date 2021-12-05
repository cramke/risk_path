#include "risk_path.h"

ProjectValidityChecker::ProjectValidityChecker(const ob::SpaceInformationPtr& si, std::shared_ptr<RTree> rtree_ptr) : ob::StateValidityChecker(si)
{
    rtree = rtree_ptr;
}

bool ProjectValidityChecker::isValid(const ob::State* state) const
{
    const double* pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
    bool is_point_valid = rtree->check_point(pos[0], pos[1]);
    if (is_point_valid) return true;
    else return false;
}

PlanningSetup::PlanningSetup()
{
    space = std::make_shared<ob::RealVectorStateSpace>(3);
    space->setLongestValidSegmentFraction(0.005);
    ss = std::make_shared<og::SimpleSetup>(space);
    si = ss->getSpaceInformation();
    ss->setPlanner(std::make_shared<og::PRMstar>(si));
}

void PlanningSetup::set_validity_checker(const char* path)
{
    GeoJsonReader reader = GeoJsonReader(path);
    auto polys = reader.get_polygons();
    std::shared_ptr<RTree> rtree = std::make_shared<RTree>(polys);
    ss->setStateValidityChecker(std::make_shared<ProjectValidityChecker>(si, rtree));
}

void PlanningSetup::set_objective(std::shared_ptr<PopulationMap> map)
{
    auto population_objective = std::make_shared<CustomOptimizationObjective>(si, map);
    ss->setOptimizationObjective(population_objective);
}

void PlanningSetup::set_rtee_objective(std::shared_ptr<RTree> rtree)
{
    auto population_objective = std::make_shared<RTreeOptimizationObjective>(si, rtree);
    ss->setOptimizationObjective(population_objective);
}

void PlanningSetup::set_boundaries()
{
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, 49);
    bounds.setHigh(0, 51);
    bounds.setLow(1, 5.868);
    bounds.setHigh(1, 9);
    bounds.setLow(2, 100);
    bounds.setHigh(2, 100);
    space->setBounds(bounds);
}

void PlanningSetup::set_start_goal(std::array<double, 3> start_coords, std::array<double, 3> goal_coords)
{
    ob::ScopedState<> start_state(space);
    start_state[0] = start_coords.at(0);
    start_state[1] = start_coords.at(1);
    start_state[2] = start_coords.at(2);
    ob::ScopedState<> goal_state(space);
    goal_state[0] = goal_coords.at(0);
    goal_state[1] = goal_coords.at(1);
    goal_state[2] = goal_coords.at(2);
    ss->setStartAndGoalStates(start_state, goal_state);
}

void PlanningSetup::solve()
{
    // this call is optional, but we put it in to get more output information
    ss->setup();
    ss->print();

    auto planner = ss->getPlanner();
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
    double long_segment = space->getLongestValidSegmentLength();
    double long_fraction = space->getLongestValidSegmentFraction();

    const int PLANNING_TIME = 3;
    ob::PlannerStatus solved = ss->solve(PLANNING_TIME);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss->getSolutionPath().print(std::cout);
    }
    else std::cout << "No solution found" << std::endl;
}