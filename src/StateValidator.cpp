#include "StateValidator.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
bool StateValidator::isValid(const ompl::base::State *state) const
{   
    const ob::RealVectorStateSpace::StateType *state3D =
        state->as<ob::RealVectorStateSpace::StateType>();
    const Eigen::Vector3d pos(state3D->values[0], state3D->values[1], state3D->values[2]);
    return world->checkPointValidity(pos, canPassGate);
}