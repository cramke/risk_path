#pragma once
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
#include <iostream>

#include "Population.h"
#include "Vector.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ProjectValidityChecker : public ob::StateValidityChecker
{
public: 
	std::shared_ptr<RTree> rtree;
	ProjectValidityChecker(const ob::SpaceInformationPtr& si, std::shared_ptr<RTree> rtree_ptr);
	bool isValid(const ob::State* state) const override;
};

class CustomOptimizationObjective : public ob::OptimizationObjective
{
private:
	std::shared_ptr<PopulationMap> map;
public:
	CustomOptimizationObjective(ob::SpaceInformationPtr& si, std::shared_ptr<PopulationMap> map_given);
	ob::Cost stateCost(const ob::State* state) const override;
	ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;
};

class RTreeOptimizationObjective : public ob::OptimizationObjective
{
    std::shared_ptr<RTree> rtree;
public:
    RTreeOptimizationObjective(ob::SpaceInformationPtr& si, std::shared_ptr<RTree> rtree);
    ob::Cost stateCost(const ob::State* state) const override;
    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;
};

class PlanningSetup
{
public:
    std::shared_ptr<ob::RealVectorStateSpace> space;
    std::shared_ptr<og::SimpleSetup> ss;
    ob::SpaceInformationPtr si;

    PlanningSetup();
    void set_validity_checker(const char*);
    void set_objective(std::shared_ptr<PopulationMap> map);
    void set_rtee_objective(std::shared_ptr<RTree> rtree);
    void set_boundaries();
    void set_start_goal(std::array<double, 3> start_coords, std::array<double, 3> goal_coords);
    void solve();
};