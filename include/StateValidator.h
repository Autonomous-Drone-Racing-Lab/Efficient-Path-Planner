#pragma once

#include "Types.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <memory>
#include "World.h"

/**
 * @class StateValidator
 * @brief A class that validates the state of the drone in the world.
 *
 * This class is a subclass of `ompl::base::StateValidityChecker` and provides state validation for our drone flight scenario.
 */
class StateValidator : public ompl::base::StateValidityChecker
{
public:
    /**
     * Construct State Validator object.
     *
     * @param si The space information for which motion is to be validated.
     * @param world A reference to the world (i.e. the drone flight environment)
     * @param canPassGate We are constructing straight line segmenest from behind a gate to befor the next one. This flag indiates whether the drone can
     * pass any gate son such a segment. Example do a sharp turn behind a gate and fly back through the same one
     */
    StateValidator(const ompl::base::SpaceInformationPtr &si, const std::shared_ptr<World> &world, const bool canPassGate) : ompl::base::StateValidityChecker(si), world(world), canPassGate(canPassGate){};

    /**
     * Check if the given state is valid.
     */
    virtual bool isValid(const ompl::base::State *state) const override;

private:
    std::shared_ptr<World> world;
    bool canPassGate;
};