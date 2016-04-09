//
// Created by Kamil Saigol on 3/03/16.
//

#include "RRTsharp.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/MagicConstants.h"

#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

ompl::RRTsharp::RRTsharp(const base::SpaceInformationPtr &si)
: base::Planner(si, "RRTsharp"),
  goalBias_(0.05),
  maxDistance_(0.0),
  lastGoalMotion_(NULL),
  radiusMultiplier_(1.1)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    specs_.optimizingPaths = true;

    freeSpaceVolume_ = si_->getStateSpace()->getMeasure();

    base::Planner::declareParam<double>("range", this, &RRTsharp::setRange, &RRTsharp::getRange, "0.:1.:10000.");
    base::Planner::declareParam<double>("goal_bias", this, &RRTsharp::setGoalBias, &RRTsharp::getGoalBias, "0.:.05:1.");
    base::Planner::declareParam<double>("radius_multiplier", this, &RRTsharp::setRadiusMultiplier, &RRTsharp::getRadiusMultiplier, "0.1:0.05:50.");

    addPlannerProgressProperty("iterations INTEGER", boost::bind(&RRTsharp::numIterationsProperty, this));
    addPlannerProgressProperty("best cost REAL", boost::bind(&RRTsharp::bestCostProperty, this));
}

ompl::RRTsharp::~RRTsharp()
{
    freeMemory();
}

void ompl::RRTsharp::getPlannerData(base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
}

void ompl::RRTsharp::setup()
{
    base::Planner::setup();
    tools::SelfConfig sc(si_, getName());

    if (!pdef_ || !pdef_->hasOptimizationObjective())
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.", getName().c_str());
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
    else
        opt_ = pdef_->getOptimizationObjective();

    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
        sc.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    }

    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    }

    nn_->setDistanceFunction(boost::bind(&RRTsharp::distanceFunction, this, _1, _2));

    bestCost_ = opt_->infiniteCost();
    bestMotion_ = NULL;
    iterations_ = 0;
}

ompl::base::PlannerStatus
ompl::RRTsharp::solve(const base::PlannerTerminationCondition &ptc)
{
    // Error checking
    checkValidity();

    // Goal information
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goalRegion = dynamic_cast<base::GoalSampleableRegion*>(goal);

    // Loop through valid input states and add to tree
    while (const base::State *state = pis_.nextStart())
    {
        // Allocate memory for a new start state motion based on the "space-information"-size
        Motion *motion = new Motion(si_);

        // Copy destination <= source
        si_->copyState(motion->state, state);

        // Set cost for this start state
        motion->g = opt_->initialCost(motion->state);

        if (nn_->size() == 0)  // do not overwrite best from previous call to solve
        {
            bestCost_ = motion->g;
            bestMotion_ = motion;
        }

        // Add start motion to the tree
        nn_->add(motion);
    }

    // Check that input states exist
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Create state sampler if this is the first run
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // Output start information
    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    // solution
    Motion *solution  = NULL;

    // approximate solution, returned if no exact solution found
    Motion *approxSolution = NULL;

    // distance from goal to closest solution yet found
    double approxDifference = std::numeric_limits<double>::infinity();

    // distance between states - the intial state and the interpolated state (may be the same)
    double rDistance;

    // Create random motion and a pointer (for optimization) to its state
    Motion *mrand   = new Motion(si_);
    Motion *mnearest;

    // STATES

    // Random state
    base::State *xrand = mrand->state;

    // The new state that is generated between states *to* and *from*
    base::State *interpolatedState = si_->allocState(); // Allocates "space information"-sized memory for a state

    // The chosen state by steering
    base::State *xnew;

    // Begin sampling --------------------------------------------------------------------------------------
    while (ptc() == false)
    {
        iterations_++;
        // I. xrand = Sample(k)

        // Sample random state (with goal biasing probability)
        if (goalRegion && rng_.uniform01() < goalBias_ && goalRegion->canSample())
        {
            // Bias sample towards goal
            goalRegion->sampleGoal(xrand);
        }
        else
        {
            // Uniformly Sample
            sampler_->sampleUniform(xrand);
        }

        /** EXTEND **/

        // II. xnearest = Nearest(xrand)

        // Find closest state in the tree
        mnearest = nn_->nearest(mrand);

        // III. xnew = Steer(xnearest, xrand)

        // Distance from near state q_n to a random state
        rDistance = si_->distance(mnearest->state, xrand);

        // Check if the rand_state is too far away
        if (rDistance > maxDistance_)
        {
            // Computes the state that lies at time t in [0, 1] on the segment that connects *from* state to *to* state.
            // The memory location of *state* is not required to be different from the memory of either *from* or *to*.
            si_->getStateSpace()->interpolate(mnearest->state, xrand,
                                              maxDistance_ / rDistance, interpolatedState);

            // Update the distance between near and new with the interpolated_state
            rDistance = si_->distance(mnearest->state, interpolatedState);

            // Use the interpolated state as the new state
            xnew = interpolatedState;
        }
        else  // Random state is close enough
        {
            xnew = xrand;
        }

        // IV. If ObstacleFree(xnearest, xnew)
        if (!si_->checkMotion(mnearest->state, xnew))
            continue; // try a new sample

        // V. Initialize(xnew, xnearest)
        // first create a motion
        Motion *mnew = new Motion(si_);
        si_->copyState(mnew->state, xnew);

        initialize(mnew, mnearest);

        // VI. Xnear = Near(xnew, |V|)

        // storage for a set of nearest neighbors
        std::vector<Motion*> Xnear;

        // radius within which to fetch neighbors
        double radius = calculateRadius(si_->getStateDimension(), nn_->size());

        // nearest neighbors in radius
        nn_->nearestR(mnew, radius, Xnear);

        // VII. foreach(xnear in Xnear)
        for(int xnear = 0; xnear < Xnear.size(); ++xnear)
        {
            Motion *mnear = Xnear.at(xnear);

            // if ObstacleFree(xnear, xnew)
            if (!si_->checkMotion(mnear->state, mnew->state))
                continue;

            // calculate c and g values
            mnew->c = opt_->motionCost(mnear->state, mnew->state);
            mnew->g = opt_->combineCosts(mnear->g, mnew->c);

            // if(lmc(xnew) > g(xnear) + c(xnew, xnear)
            if(opt_->isCostBetterThan(mnew->g, mnew->lmc))
            {
                mnew->lmc = mnew->g;
                mnew->parent = mnear;
            }

            // Cycle
            mnear->children.push_back(mnew);
            mnew->children.push_back(mnear);
        }

        // VIII. Add motion
        nn_->add(mnew);

        // IX. UpdateQueue(xnew)
        updateQueue(mnew);

        // X. Update the current best cost and motion if necessary
        if (opt_->isCostBetterThan(mnew->lmc, bestCost_))
        {
            bestCost_ = mnew->lmc;
            bestMotion_ = mnew;
        }

        /** REPLAN **/
        motion_key_compare mc(opt_);
        key_compare kc(opt_);

        // XI. while(q.findmin() < Key(v*)
        while(1)
        {
            std::deque<Motion*>::iterator min = std::min_element(q_.begin(), q_.end(), mc);

            if(!kc((*min)->key, bestMotion_->key))
                break;

            Motion *x = (*min);
            x->g = x->lmc;
            q_.erase(min);
            for(int i = 0; i < x->children.size(); ++i)
            {
                Motion *s = x->children.at(i);
                s->c = opt_->motionCost(x->state, s->state);
                s->g = opt_->combineCosts(x->g, s->c);

                if(opt_->isCostBetterThan(s->g, s->lmc))
                {
                    s->lmc = s->g;
                    s->parent = x;
                    // s is already a child of x
                    updateQueue(s);
                }
            }
        }

        // Check if this motion is the goal
        double distToGoal = 0.0;
        bool isSatisfied = goal->isSatisfied(mnew->state, &distToGoal);
        if (isSatisfied)
        {
            approxDifference = distToGoal; // the tolerated error distance between state and goal
            solution = mnew; // set the final solution
            break;
        }

        // Is this the closest solution we've found so far?
        if (distToGoal < approxDifference)
        {
            approxDifference = distToGoal;
            approxSolution = mnew;
        }
    }

    // Finish solution processing --------------------------------------------------------------------

    bool solved = false;
    bool approximate = false;

    // Substitute an empty solution with the best approximation
    if (solution == NULL)
    {
        solution = approxSolution;
        approximate = true;
    }

    // Generate solution path for real/approx solution
    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        geometric::PathGeometric *path = new geometric::PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            path->append(mpath[i]->state);

        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxDifference, getName());
        solved = true;
    }

    // Clean up ---------------------------------------------------------------------------------------

    si_->freeState(interpolatedState);
    if (mrand->state)
        si_->freeState(mrand->state);
    delete mrand;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::RRTsharp::clear()
{
    base::Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
    {
        nn_->clear();
    }
    lastGoalMotion_ = NULL;

    if (opt_)
    {
        bestCost_ = opt_->infiniteCost();
        bestMotion_ = NULL;
    }
    iterations_ = 0;
}

void ompl::RRTsharp::initialize(Motion *x, Motion *x_pr)
{
    x->g = opt_->infiniteCost();
    if(!x_pr)
    {
        x->lmc = opt_->infiniteCost();
        x->parent = NULL;
    }
    else
    {
        x->c = opt_->motionCost(x_pr->state, x->state);
        x->lmc = opt_->combineCosts(x_pr->g, x->c);
        x->parent = x_pr;
        x_pr->children.push_back(x);
    }
}

void ompl::RRTsharp::updateQueue(Motion *x)
{
    bool equal_cost = opt_->isCostEquivalentTo(x->g, x->lmc);

    std::deque<Motion*>::iterator it = std::find(q_.begin(), q_.end(), x);

    if(!equal_cost)
    {
        if(it != q_.end())
        {
            (*it)->key = key(x);
        }
        else
        {
            x->key = key(x);
            q_.push_back(x);
        }
    }
    else // equal_cost is true
    {
        if(it != q_.end())
        {
            q_.erase(it);
        }
    }
}

ompl::RRTsharp::key_type ompl::RRTsharp::key(Motion *x) const
{
    base::Cost k1 = opt_->combineCosts(x->lmc, heuristicValue(x));
    return std::make_pair(k1, x->lmc);
}

double ompl::RRTsharp::calculateUnitBallVolume(const unsigned int dimension) const
{
    if (dimension == 0)
        return 1.0;
    else if (dimension == 1)
        return 2.0;
    return 2.0 * boost::math::constants::pi<double>() / dimension
           * calculateUnitBallVolume(dimension - 2);
}

double ompl::RRTsharp::calculateRadius(const unsigned int dimension, const unsigned int n) const
{
    double a = 1.0 / (double)dimension;
    double unitBallVolume = calculateUnitBallVolume(dimension);

    return radiusMultiplier_ * 2.0 * std::pow(a, a) * std::pow(freeSpaceVolume_ / unitBallVolume, a) * std::pow(std::log((double)n) / (double)n, a);
}

void ompl::RRTsharp::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0; i < motions.size(); ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}