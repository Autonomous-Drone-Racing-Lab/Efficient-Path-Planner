#include "MotionValidator.h"
#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;

bool MotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{   
    const ob::RealVectorStateSpace::StateType *state1 = s1->as<ob::RealVectorStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *state2 = s2->as<ob::RealVectorStateSpace::StateType>();

    Eigen::Vector3f start(state1->values[0], state1->values[1], state1->values[2]);
    Eigen::Vector3f end(state2->values[0], state2->values[1], state2->values[2]);

    return world->checkRayValid(start, end);
}

bool MotionValidator::checkMotion(const ob::State *s1, const ob::State *s2, std::pair< ob::State *, double > &lastValid) const
{ 
    const ob::RealVectorStateSpace::StateType *state1 = s1->as<ob::RealVectorStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *state2 = s2->as<ob::RealVectorStateSpace::StateType>();

    Eigen::Vector3f start(state1->values[0], state1->values[1], state1->values[2]);
    Eigen::Vector3f end(state2->values[0], state2->values[1], state2->values[2]);

    // run across ray in steps of delta
    const float distance_delta = 0.1;
    Eigen::Vector3f direction = (end - start).normalized();
    Eigen::Vector3f current = start;
    const int steps = (end - start).norm() / distance_delta;
    Eigen::Vector3f lastValidPoint = start;
    for (int i = 0; i < steps; i++)
    {
        current = start + direction * i * distance_delta;
        if (!world->checkPointValidity(current))
        {
           const float percentage = i / steps;
           lastValid.second = percentage;

           // converte lastValidPoint to state
            auto lastValidState = lastValid.first->as<ob::RealVectorStateSpace::StateType>();
            lastValidState->values[0] = lastValidPoint.x();
            lastValidState->values[1] = lastValidPoint.y();
            lastValidState->values[2] = lastValidPoint.z();
            
            
            return false;

        }
        lastValidPoint = current;
    }


    return true;
}