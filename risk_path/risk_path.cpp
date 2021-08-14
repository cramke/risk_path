#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>

#include <ompl/config.h>
#include <iostream>
#include "Population.cpp"
#include "risk_path.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ProjectValidityChecker : public ob::StateValidityChecker
{
private:
    PopulationMap map = PopulationMap();

public:
    ProjectValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si)
    {    
    }

    bool isValid(const ob::State* state) const override
    {
        const double* pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
        double lat = pos[0];
        double lon = pos[1];
        Coordinates point = Coordinates(lat, lon, map);
        return map.check_map_bounds(lat, lon);
    }
};

class CustomOptimizationObjective : ob::OptimizationObjective
{
    CustomOptimizationObjective(ob::SpaceInformationPtr& si) : ob::OptimizationObjective(si)
    {}

    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override
    {
        throw "CustomOptimizationObjective.motionCost(): Not yet implemented!";
    }
};

void planWithSimpleSetup()
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,49);
    bounds.setHigh(0,51);
    bounds.setLow(1,5.86);
    bounds.setHigh(1,9);
    bounds.setLow(2,100);
    bounds.setHigh(2,100);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);
    ob::SpaceInformationPtr si = ss.getSpaceInformation();

    // create a random start / goal state
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 49.86462268679067;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 8.657507656252882;
    start[2] = 100;
    ob::ScopedState<> goal(space);
    goal[0] = 50.107998827159896;
    goal[1] = 8.68757388575945;
    goal[2] = 100;
    ss.setStartAndGoalStates(start, goal);

    ss.setStateValidityChecker(std::make_shared<ProjectValidityChecker>(si));
    ss.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    ss.setPlanner(std::make_shared<og::PRMstar>(si));

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    const int PLANNING_TIME = 5;
    ob::PlannerStatus solved = ss.solve(PLANNING_TIME);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    std::cout << std::endl << std::endl;
    planWithSimpleSetup();
}