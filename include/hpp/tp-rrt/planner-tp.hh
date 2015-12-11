//
// Copyright (c) 2015 CNRS
// Authors: Steve Tonneau
//
// This file is part of tp_rtt
// tp_rtt is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_PLANNER_TP_HH
# define HPP_PLANNER_TP_HH

#include <hpp/core/config-validations.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/collision-validation.hh>

#include <hpp/tp-rrt/shooter-tp.hh>
#include <boost/tuple/tuple.hpp>

namespace hpp
{
namespace tp_rrt
{

/// \addtogroup configuration_sampling
/// \{

/// Uniformly sample with bounds of degrees of freedom.
class PlannerTP : public core::PathPlanner
{
    typedef boost::tuple <NodePtr_t, ConfigurationPtr_t, PathPtr_t> DelayedEdge_t;
    typedef std::vector <DelayedEdge_t> DelayedEdges_t;

    public:
    /// Create an instance and return a shared pointer to the instance
    static PlannerTPPtr_t create (const core::Problem& problem,
    const core::RoadmapPtr_t& roadmap)
    {
        PlannerTP* ptr = new PlannerTP (problem, roadmap);
        PlannerTPPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
    }

    /// One step of extension.
    ///
    /// This method implements one step of your algorithm. The method
    /// will be called iteratively until one goal configuration is accessible
    /// from the initial configuration.
    virtual void oneStep ();

    /// Tries to extend the graph towards a configuration
    ///
    /// \param target a randomly generated configuration
    /// \return whether the heuristic has been added. False is returned if a heuristic with that name already exists.
    PathPtr_t extend (const NodePtr_t nearest, const ConfigurationPtr_t& target,
                                 DelayedEdges_t& delayedEdges, Nodes_t& newNodes);

    protected:
    /// Protected constructor
    /// Users need to call Planner::create in order to create instances.
    PlannerTP (const core::Problem& problem, const core::RoadmapPtr_t& roadmap)
        : core::PathPlanner (problem, roadmap)
        , shooter_ (tp_rrt::ShooterTP::create (problem.robot ()))
        , collisionValidation_(CollisionValidation::create(problem.robot()))
    {
        for(core::ObjectVector_t::const_iterator cit = problem.collisionObstacles().begin();
            cit != problem.collisionObstacles().end(); ++cit)
        {
            collisionValidation_->addObstacle(*cit);
        }
    }

    /// Store weak pointer to itself
    void init (const PlannerTPWkPtr_t& weak)
    {
        core::PathPlanner::init (weak);
        weakPtr_ = weak;
    }
    private:
    /// Configuration shooter to uniformly shoot random configurations
    ShooterTPPtr_t shooter_;
    /// weak pointer to itself
    PlannerTPWkPtr_t weakPtr_;
    core::CollisionValidationPtr_t collisionValidation_;
}; // class PlannerTP
/// \}
} //   namespace tp_rrt
} // namespace hpp

#endif // HPP_SHOOTER_TP_HH
