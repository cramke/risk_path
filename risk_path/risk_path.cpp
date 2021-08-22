#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>


#include <ompl/config.h>
#include <iostream>

#include "Population.h"
#include "Vector.h"



namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bg = boost::geometry;

class ProjectValidityChecker : public ob::StateValidityChecker
{
private:
    std::shared_ptr<Vector> boundaries;

public:

    ProjectValidityChecker(const ob::SpaceInformationPtr& si, std::shared_ptr<Vector> bounds) : ob::StateValidityChecker(si)
    {
        boundaries = bounds;
    }

    bool isValid(const ob::State* state) const override
    {
        const double* pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
        bool valid = boundaries->within(pos[0], pos[1]);
        return valid;
    }
};

class CustomOptimizationObjective : public ob::OptimizationObjective
{
private:
    std::shared_ptr<PopulationMap> map;

public:
    CustomOptimizationObjective(ob::SpaceInformationPtr& si, std::shared_ptr<PopulationMap> map_given) : ob::OptimizationObjective(si)
    {
        map = map_given;
    }

    ob::Cost stateCost(const ob::State* state) const override
    {
        const double* pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
        double lat = pos[0];
        double lon = pos[1];
        Coordinates point = Coordinates(lat, lon, map);

        double cost_value = map->read_population_from_indexes(point.x, point.y);
        return ob::Cost(cost_value);
    }

    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override
    {
        return ob::Cost(1.0);
    }
};


class PlanningSetup 
{

public:
    std::shared_ptr<ob::RealVectorStateSpace> space;
    std::shared_ptr<og::SimpleSetup> ss;
    ob::SpaceInformationPtr si;

    PlanningSetup(std::shared_ptr<PopulationMap> map)
    {
        space = std::make_shared<ob::RealVectorStateSpace>(3);
        ss = std::make_shared<og::SimpleSetup>(space);
        si = ss->getSpaceInformation();

        
        auto population_objective = std::make_shared<CustomOptimizationObjective>(si, map);
        ss->setOptimizationObjective(population_objective);
        ss->setPlanner(std::make_shared<og::PRMstar>(si));
    }

    void set_validity_checker(std::array<double, 3> start_coords, std::array<double, 3> goal_coords)
    {
        std::shared_ptr<Vector> boundaries = std::make_shared<Vector>(start_coords[0] - 0.001, start_coords[1] - 0.001, goal_coords[0] + 0.001, goal_coords[1] + 0.001);
        ss->setStateValidityChecker(std::make_shared<ProjectValidityChecker>(si, boundaries));
    }

    void set_boundaries() 
    {
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, 49);
        bounds.setHigh(0, 51);
        bounds.setLow(1, 5.86);
        bounds.setHigh(1, 9);
        bounds.setLow(2, 100);
        bounds.setHigh(2, 100);
        space->setBounds(bounds);
    }

    void set_start_goal(std::array<double, 3> start_coords, std::array<double, 3> goal_coords)
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

    void solve()
    {
        // this call is optional, but we put it in to get more output information
        ss->setup();
        ss->print();
        const int PLANNING_TIME = 1;
        ob::PlannerStatus solved = ss->solve(PLANNING_TIME);
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            ss->getSolutionPath().print(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }
};



int main(int /*argc*/, char** /*argv*/)
{
    std::array<double, 3> start_point = { 49.86462268679067, 8.657507656252882, 100 };
    std::array<double, 3> goal_point = { 50.107998827159896, 8.68757388575945, 100 };
    auto pop_map = std::make_shared<PopulationMap>();

    PlanningSetup planner = PlanningSetup(pop_map);
    planner.set_start_goal(start_point, goal_point);
    planner.set_boundaries();
    planner.set_validity_checker(start_point, goal_point);
    planner.solve();
}