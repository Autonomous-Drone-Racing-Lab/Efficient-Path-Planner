#pragma once
#include <ompl/base/MotionValidator.h>
#include "Types.h"
#include <memory>
#include "World.h"

class MotionValidator: public ompl::base::MotionValidator
{
    public:
    MotionValidator(const ompl::base::SpaceInformationPtr &si, const std::shared_ptr<World> &world, const bool canPassGate): ompl::base::MotionValidator(si), world(world), canPassGate(canPassGate) {};

    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    virtual bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2, std::pair< ompl::base::State *, double > &lastValid) const override;

    private:
    std::shared_ptr<World> world;
    const bool canPassGate;
};