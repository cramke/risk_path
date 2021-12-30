#include "risk_path.h"

CustomOptimizationObjective::CustomOptimizationObjective(ob::SpaceInformationPtr& si, std::shared_ptr<PopulationMap> map_given) : ob::OptimizationObjective(si), map(map_given)
{
}

ob::Cost CustomOptimizationObjective::stateCost(const ob::State* state) const 
{
    const double* pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
    Coordinates point = Coordinates(pos[0], pos[1], map);
    double cost_value = map->read_population_from_indexes(point.x, point.y);
    return ob::Cost(cost_value);        
}

ob::Cost CustomOptimizationObjective::motionCost(const ob::State* s1, const ob::State* s2) const
{
    const double* pos = s1->as<ob::RealVectorStateSpace::StateType>()->values;
    Coordinates point1 = Coordinates(pos[0], pos[1], map);
    double cost_value1 = map->read_population_from_indexes(point1.x, point1.y);

    const double* pos2 = s2->as<ob::RealVectorStateSpace::StateType>()->values;
    Coordinates point2 = Coordinates(pos2[0], pos[1], map);
    double cost_value2 = map->read_population_from_indexes(point2.x, point2.y);

    return ob::Cost(cost_value1 + cost_value2);
}

RTreeOptimizationObjective::RTreeOptimizationObjective(ob::SpaceInformationPtr& si, std::shared_ptr<RTreePoint> rtree_given) : ob::OptimizationObjective(si)
{
    rtree = rtree_given;
}

ob::Cost RTreeOptimizationObjective::stateCost(const ob::State* state) const
{
    const double* pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
    // double state_cost = rtree->nearest_point_cost(pos[0], pos[1]);

    double state_cost = rtree->buffered_point_cost(pos);
    return ob::Cost(state_cost);
}

ob::Cost RTreeOptimizationObjective::motionCost(const ob::State* s1, const ob::State* s2) const
{
    const double* pos1 = s1->as<ob::RealVectorStateSpace::StateType>()->values;
    const double* pos2 = s2->as<ob::RealVectorStateSpace::StateType>()->values;
    double cost = rtree->buffered_line_cost(pos1, pos2);
    return ob::Cost(cost);
}