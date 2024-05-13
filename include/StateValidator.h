#pragma once

#include "Types.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <memory>
#include "World.h"

class StateValidator : public ompl::base::StateValidityChecker
{
    public:
    StateValidator(const ompl::base::SpaceInformationPtr &si, const std::shared_ptr<World> &world): ompl::base::StateValidityChecker(si), world(world){};

    virtual bool isValid(const ompl::base::State *state) const override;
    
    private:
    std::shared_ptr<World> world;
};