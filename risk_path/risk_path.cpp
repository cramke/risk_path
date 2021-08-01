#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include "Population.cpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State* state)
{
    // cast the abstract state type to the type we expect
    const auto* se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    // const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    const ob::SO3StateSpace::StateType& rot = se3state->rotation();

    Coordinates* point = new Coordinates(se3state->getX(), se3state->getY());

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return se3state->getX() != se3state->getY();
}


void planWithSimpleSetup()
{
    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,-1);
    bounds.setHigh(0,1);
    bounds.setLow(1,-1);
    bounds.setHigh(1,1);
    bounds.setLow(2,0);
    bounds.setHigh(2,100);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State* state) { return isStateValid(state); });

    // create a random start / goal state
    ob::ScopedState<> start(space);
    start.random();
    ob::ScopedState<> goal(space);
    goal.random();
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    const int PLANNING_TIME = 0.1;
    ob::PlannerStatus solved = ss.solve(PLANNING_TIME);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
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