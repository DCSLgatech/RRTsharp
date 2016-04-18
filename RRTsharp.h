//
// Created by Kamil Saigol on 4/9/16.
//

#ifndef RRTSHARP_RRTSHARP_H
#define RRTSHARP_RRTSHARP_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/OptimizationObjective.h>

#include <boost/heap/fibonacci_heap.hpp>

#include <utility>
#include <vector>

namespace ompl {

class RRTsharp: public base::Planner
{
    friend class motion_compare;
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

    base::Cost bestCost() const
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
    // Key type
    typedef std::pair<base::Cost, base::Cost> key_type;

    // Key comparison functor
    struct key_compare
    {
        key_compare(base::OptimizationObjectivePtr &opt) : opt(opt) {}
        bool operator()(const key_type &k1, const key_type &k2) const
        {
            return opt->isCostBetterThan(k1.first, k2.first) ||
                   (opt->isCostEquivalentTo(k1.first, k2.first) &&
                    (opt->isCostBetterThan(k1.second, k2.second) || opt->isCostEquivalentTo(k1.second, k2.second)));
        }

        base::OptimizationObjectivePtr &opt;
    };

    struct Motion;
    struct motion_compare
    {
        motion_compare(base::OptimizationObjectivePtr &opt) : kc(opt) {}
        inline bool operator()(const Motion *m1, const Motion *m2) const;
        key_compare kc;
    };

    struct Motion
    {
        Motion() : state(NULL), parent(NULL) { handle.node_ = NULL; }
        Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL) { handle.node_ = NULL; }
        ~Motion() {}

        base::State             *state;
        Motion                  *parent;
        std::vector<Motion*>    children;
        base::Cost              c;
        base::Cost              g;
        base::Cost              lmc;
        key_type                key;
        boost::heap::fibonacci_heap< Motion*, boost::heap::compare<motion_compare> >::handle_type handle;
    };

    void initialize(Motion *x, Motion *x_pr);
    void updateQueue(Motion *x);
    key_type key(Motion *x) const;
    base::Cost heuristicValue(Motion *v) const;
    double calculateUnitBallVolume(const unsigned int dimension) const;
    double calculateRadius(const unsigned int dimension, const unsigned int n) const;

    double distanceFunction(const Motion *a, const Motion *b) const;
    void freeMemory();

    base::StateSamplerPtr                           sampler_;
    boost::shared_ptr< NearestNeighbors<Motion*> >  nn_;
    double                                          goalBias_;
    double                                          maxDistance_;
    RNG                                             rng_;
    Motion                                          *lastGoalMotion_;
    Motion                                          *bestMotion_;
    unsigned int                                    iterations_;
    base::Cost                                      bestCost_;
    base::OptimizationObjectivePtr                  opt_;
    double                                          radiusMultiplier_;
    double                                          freeSpaceVolume_;
    boost::heap::fibonacci_heap< Motion*, boost::heap::compare<motion_compare> > q_;

    std::string numIterationsProperty() const
    {
        return boost::lexical_cast<std::string>(numIterations());
    }

    std::string bestCostProperty() const
    {
        return boost::lexical_cast<std::string>(bestCost());
    }
};

bool RRTsharp::motion_compare::operator()(const RRTsharp::Motion *m1, const RRTsharp::Motion *m2) const
{
    return !kc(m1->key, m2->key);
}

}

#endif //RRTSHARP_RRTSHARP_H
