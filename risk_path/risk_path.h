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