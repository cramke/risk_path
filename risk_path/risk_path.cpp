#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include "Population.cpp"
#include "risk_path.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State* state)
{
    const double* pos = state->as<ob::RealVectorStateSpace::StateType>()->values;

    PopulationMap map = PopulationMap();
    Coordinates* point = new Coordinates(12, 8, map.transform);

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return pos[2] == 100 && pos[1] != pos[0];
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
    ss.setStateValidityChecker([](const ob::State* state) { return isStateValid(state); });

    // create a random start / goal state
    ob::ScopedState<> start(space);
    start[0] = 49.86462268679067;
    start[1] = 8.657507656252882;
    start[2] = 100;
    ob::ScopedState<> goal(space);
    goal[0] = 50.107998827159896;
    goal[1] = 8.68757388575945;
    goal[2] = 100;
    ss.setStartAndGoalStates(start, goal);

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