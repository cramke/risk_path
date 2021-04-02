#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>
#include <fstream>


#include <iostream>
/* For geometry operations */
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Geometry.h>
/* For WKT read/write */
#include <geos/io/WKTReader.h>
#include <geos/io/WKTWriter.h>

/* Geometry/GeometryFactory */
using namespace geos::geom;
/* WKTReader/WKTWriter */
using namespace geos::io;

namespace ob = ompl::base;
namespace og = ompl::geometric;


class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
private:
    GeometryFactory::Ptr factory = GeometryFactory::create();
    WKTReader reader = *factory;
    std::string wkt_a = "POLYGON((0.5 0.5, 1 0.5, 1 1, 0.5 1, 0.5 0.5))";
    std::unique_ptr<Geometry> geom_a = reader.read(wkt_a);

    bool isValid(const ob::State* state) const override
    {
        const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        std::string wkt_b = "POINT(" + std::to_string(x) + " " + std::to_string(y) + ")";

        WKTReader reader = *factory;
        std::unique_ptr<Geometry> geom_b = reader.read(wkt_b);

        /* Calculate intersection */
        bool inter = geom_a->intersects(geom_b.get());
        return not inter;
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
ob::OptimizationObjectivePtr getCustomObjective(const ob::SpaceInformationPtr& si);
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

void plan(double runTime, const std::string& outputFile)
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    space->setBounds(0, 2);
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    si->setup();

    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 2.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 2.0;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(getBalancedObjective1(si));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = std::make_shared<og::RRTstar>(si);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);

    if (solved)
    {
        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        if (!outputFile.empty())
        {
            std::ofstream outFile(outputFile.c_str());
            std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                printAsMatrix(outFile);
            outFile.close();
        }
    }
    else
        std::cout << "No solution found." << std::endl;
}

int main()
{
    double runTime = 1;
    std::string outputFile = "solution_path.txt";

    plan(runTime, outputFile);
    return 0;
}




class CustomObjective : public ob::StateCostIntegralObjective
{
public:
    CustomObjective(const ob::SpaceInformationPtr& si) : ob::StateCostIntegralObjective(si, true)
    {
        int i = 1;
    }
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(10.65);
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

ob::OptimizationObjectivePtr getCustomObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<CustomObjective>(si);
}

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto customObj(std::make_shared<CustomObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    opt->addObjective(lengthObj, 1.0);
    opt->addObjective(customObj, 10.0);

    return ob::OptimizationObjectivePtr(opt);
}

