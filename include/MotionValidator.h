#pragma once
#include <ompl/base/MotionValidator.h>
#include "Types.h"
#include <memory>
#include "World.h"

class MotionValidator: public ompl::base::MotionValidator
{
    public:
    /**
     * @class MotionValidator
     * @brief A class that validates motion between states in a given space.
     *
     * This class is a subclass of `ompl::base::MotionValidator` and provides motion validation for our 
     * drone flight scenario.
     * 
     * @param si The space information for which motion is to be validated.
     * @param world A reference to the world (i.e. the drone flight environment)
     * @param canPassGate We are constructing straight line segmenest from behind a gate to befor the next one. This flag indiates whether the drone can 
     * pass any gate son such a segment. Example do a sharp turn behind a gate and fly back through the same one
     */
    MotionValidator(const ompl::base::SpaceInformationPtr &si, const std::shared_ptr<World> &world, const bool canPassGate): ompl::base::MotionValidator(si), world(world), canPassGate(canPassGate) {};

    virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    virtual bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2, std::pair< ompl::base::State *, double > &lastValid) const override;

    private:
    std::shared_ptr<World> world;
    const bool canPassGate;
};