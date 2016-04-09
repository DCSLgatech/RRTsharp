//
// Created by Kamil Saigol on 4/9/16.
//

#ifndef RRTSHARP_RRTSHARP_H
#define RRTSHARP_RRTSHARP_H

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/OptimizationObjective.h"
#include <deque>
#include <utility>
#include <vector>

namespace ompl {

class RRTsharp: public base::Planner
{
public:
    RRTsharp(const base::SpaceInformationPtr &si);
    virtual ~RRTsharp();

    virtual void getPlannerData(base::PlannerData &data) const;
    virtual void setup();
    virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
    virtual void clear();

    template<template<typename T> class NN>
    void setNearestNeighbors()
    {
        nn_.reset(new NN<Motion*>());
    }

    void setGoalBias(double goalBias)
    {
        goalBias_ = goalBias;
    }

    double getGoalBias() const
    {
        return goalBias_;
    }

    void setRange(double distance)
    {
        maxDistance_ = distance;
    }

    double getRange() const
    {
        return maxDistance_;
    }

    unsigned int numIterations() const
    {
        return iterations_;
    }

    ompl::base::Cost bestCost() const
    {
        return bestCost_;
    }

    void setRadiusMultiplier(const double radiusMultiplier)
    {
        if (radiusMultiplier <= 0.0)
            throw Exception("Radius multiplier must be greater than zero");
        radiusMultiplier_ = radiusMultiplier;
    }

    double getRadiusMultiplier() const
    {
        return radiusMultiplier_;
    }

protected:
    typedef std::pair<ompl::base::Cost, ompl::base::Cost> key_type;

    class Motion
    {
    public:
        Motion()
                : state(NULL), parent(NULL)
        {
        }

        Motion(const base::SpaceInformationPtr &si)
                : state(si->allocState()), parent(NULL)
        {
        }

        ~Motion()
        {
        }

        base::State             *state;
        Motion                  *parent;
        base::Cost              c;
        base::Cost              g;
        base::Cost              lmc;
        std::vector<Motion*>    children;
        key_type                key;
    };

    struct key_compare
    {
        key_compare(ompl::base::OptimizationObjectivePtr &opt)
                : opt(opt) {}
        bool operator()(const key_type &k1, const key_type &k2) const
        {
            return opt->isCostBetterThan(k1.first, k2.first) ||
                   (opt->isCostEquivalentTo(k1.first, k2.first) &&
                    (opt->isCostBetterThan(k1.second, k2.second) || opt->isCostEquivalentTo(k1.second, k2.second)));
        }

        ompl::base::OptimizationObjectivePtr &opt;
    };

    struct motion_key_compare
    {
        motion_key_compare(ompl::base::OptimizationObjectivePtr &opt)
                : kc(opt) {}
        key_compare kc;
        bool operator()(const Motion* m1, const Motion * m2) const
        {
            return kc(m1->key, m2->key);
        }
    };

    void initialize(Motion *x, Motion *x_pr);
    void updateQueue(Motion *x);
    key_type key(Motion *x) const;

    ompl::base::Cost heuristicValue(Motion *v) const
    {
        return opt_->costToGo(v->state, pdef_->getGoal().get());
    }

    void freeMemory();
    double calculateUnitBallVolume(const unsigned int dimension) const;
    double calculateRadius(const unsigned int dimension, const unsigned int n) const;
    double distanceFunction(const Motion *a, const Motion *b) const
    {
        return si_->distance(a->state, b->state);
    }

    base::StateSamplerPtr                          sampler_;
    boost::shared_ptr< NearestNeighbors<Motion*> > nn_;
    double                                         goalBias_;
    double                                         maxDistance_;
    RNG                                            rng_;
    Motion                                         *lastGoalMotion_;
    Motion                                         *bestMotion_;
    unsigned int                                   iterations_;
    base::Cost                                     bestCost_;
    ompl::base::OptimizationObjectivePtr           opt_;
    std::deque<Motion*>                            q_;
    double                                         radiusMultiplier_;
    double                                         freeSpaceVolume_;

    std::string numIterationsProperty() const
    {
        return boost::lexical_cast<std::string>(numIterations());
    }

    std::string bestCostProperty() const
    {
        return boost::lexical_cast<std::string>(bestCost());
    }
};

}

#endif //RRTSHARP_RRTSHARP_H
