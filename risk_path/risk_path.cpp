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


namespace ob = ompl::base;
namespace og = ompl::geometric;


class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const override
    {
        const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];
        
        /* New factory with default (float) precision model */
        GeometryFactory::Ptr factory = GeometryFactory::create();

        /*
        * Reader requires a factory to bind the geometry to
        * for shared resources like the PrecisionModel
        */
        WKTReader reader(*factory);

        /* Input WKT strings */
        std::string wkt_a("POLYGON((0.5 0.5, 1 0.5, 1 1, 0.5 1, 0.5 0.5))");
        std::string a = "POINT(";
        std::string b = std::to_string(x);
        std::string c = " ";
        std::string d = std::to_string(y);
        std::string e = ")";

        std::string wkt_b(a+b+c+d+e);
        std::cout << b << std::endl;

        /* Convert WKT to Geometry */
        std::unique_ptr<Geometry> geom_a(reader.read(wkt_a));
        std::unique_ptr<Geometry> geom_b(reader.read(wkt_b));

        /* Calculate intersection */
        bool inter = geom_a->intersects(geom_b.get());

        std::cout << inter << std::endl;
        return not inter;

        if (x > -0.1 && x < 0.1 && y > -0.1 && y < 0.1)
        {
            std::cout << x << std::endl;
            std::cout << y << std::endl;
            return false;
        }
        else
        {
            return true;
        }
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

void plan(double runTime, const std::string& outputFile)
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    space->setBounds(-1.0, 1.0);
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    si->setup();

    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -1.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = -1.0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(getPathLengthObjective(si));

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

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}


ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    opt->addObjective(lengthObj, 10.0);

    return ob::OptimizationObjectivePtr(opt);
}