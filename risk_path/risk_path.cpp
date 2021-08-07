#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

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
        return point.check_map_bounds();
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}


void planWithSimpleSetup()
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,47);
    bounds.setHigh(0,55);
    bounds.setLow(1,5.86);
    bounds.setHigh(1,15);
    bounds.setLow(2,100);
    bounds.setHigh(2,100);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(std::make_shared<ProjectValidityChecker>(si));

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

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    const int PLANNING_TIME = 1;
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