// Copyright (c) 2015, LAAS-CNRS
// Authors: Steve Tonneau (steve.tonneau@laas.fr)
//
// This file is part of tp_rtt.
// hpp-rbprm is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-rbprm is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-rbprm. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/tp-rrt/planner-tp.hh>
#include <hpp/core/path-validation-report.hh>

using namespace  hpp::core;
using namespace  hpp::tp_rrt;


bool belongs (const ConfigurationPtr_t& q, const Nodes_t& nodes)
{
    for (Nodes_t::const_iterator itNode = nodes.begin ();
    itNode != nodes.end (); ++itNode)
    {
        if (*((*itNode)->configuration ()) == *q) return true;
    }
    return false;
}

/// Compute the largest valid interval starting from the path beginning
///
/// \param path the path to check for validity,
/// \param reverse if true check from the end,
/// \retval validPart the extracted valid part of the path,
///         pointer to path if path is valid.
/// \retval validationReport information about the validation process:
///         which objects have been detected in collision and at which
///         parameter along the path.
/// \param collisionValidation object used to perform collision detection
/// \return whether the whole path is valid.
bool validate (const PathPtr_t& path,
           PathPtr_t& validPart,
           PathValidationReportPtr_t& report, CollisionValidationPtr_t collisionValidation)
{
    value_type tmin = path->timeRange ().first;
    value_type tmax = path->timeRange ().second;
    value_type lastValidTime = tmin;
    value_type t = tmin;
    value_type stepSize_ = 0.1;
    bool valid = true;
    bool success;
    ValidationReportPtr_t cReport(new CollisionValidationReport);
    unsigned finished = 0;
    while (finished < 2 && valid)
    {
        Configuration_t q = (*path) (t, success);
        if (!collisionValidation->validate (q, cReport))
        {
            report->parameter = t;
            valid = false;
        }
        else
        {
            lastValidTime = t;
            t += stepSize_;
        }
        if (t > tmax)
        {
            t = tmax;
            finished ++;
        }
    }
    if (valid)
    {
        validPart = path;
        return true;
    }
    else
    {
        validPart = path->extract (std::make_pair (tmin, lastValidTime));
        return false;
    }
}

// Vous devez implémenter cette méthode
/// One step of extension.
///
/// This method implements one step of your RRT algorithm. The method
/// will be called iteratively until one goal configuration is accessible
/// from the initial configuration.
void PlannerTP::oneStep ()
{
    // Retrieve roadmap of the path planner
    core::RoadmapPtr_t r (roadmap ());
    // Temporary storing of the new edges
    // that can't be added ruding the iteration on connex components
    DelayedEdges_t delayedEdges;
    Nodes_t newNodes;

    // shoot a valid random configuration
    // and iterate over connected component to
    // try to connect it
    ConfigurationPtr_t qrand = shooter_->shoot();
    for(ConnectedComponents_t::const_iterator cit = r->connectedComponents().begin();
        cit != r->connectedComponents().end(); ++cit)
    {
        const ConnectedComponentPtr_t cc = *cit;
        double distance;
        extend(r->nearestNode(qrand, cc, distance), qrand, delayedEdges, newNodes);
    }

    // Insert new nodes,
    // and delayed edges into the roadmap
    // Add edges in both directions
    // using the method extract of a Path
    for (DelayedEdges_t::const_iterator itEdge = delayedEdges.begin ();
         itEdge != delayedEdges.end (); ++itEdge)
    {
        const NodePtr_t& near = itEdge-> get <0> ();
        const ConfigurationPtr_t& q_new = itEdge-> get <1> ();
        const PathPtr_t& validPath = itEdge-> get <2> ();
        NodePtr_t newNode = roadmap ()->addNode (q_new);
        roadmap ()->addEdge (near, newNode, validPath);
        interval_t timeRange = validPath->timeRange ();
        roadmap ()->addEdge (newNode, near, validPath->extract
                   (interval_t (timeRange.second ,
                        timeRange.first)));
    }

    //
    // Second, try to connect new nodes together
    //
    PathPtr_t path, validPath;
    core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
    core::SteeringMethodPtr_t sm (problem ().steeringMethod ());
    for (Nodes_t::const_iterator itn1 = newNodes.begin ();
     itn1 != newNodes.end (); ++itn1)
    {
        for (Nodes_t::const_iterator itn2 = boost::next (itn1);
           itn2 != newNodes.end (); ++itn2)
        {
            ConfigurationPtr_t q1 ((*itn1)->configuration ());
            ConfigurationPtr_t q2 ((*itn2)->configuration ());
            assert (*q1 != *q2);
            path = (*sm) (*q1, *q2);
            PathValidationReportPtr_t report(new PathValidationReport);
          /*  if (path && pathValidation->validate
                    (path, false, validPath, report))*/
            if(path && validate(path,validPath,report, collisionValidation_))
            {
                roadmap ()->addEdge (*itn1, *itn2, path);
                interval_t timeRange = path->timeRange ();
                roadmap ()->addEdge (*itn2, *itn1, path->extract
                       (interval_t (timeRange.second,
                                timeRange.first)));
            }
        }
    }
}

PathPtr_t PlannerTP::extend (const NodePtr_t nearest, const ConfigurationPtr_t& target,
                             DelayedEdges_t& delayedEdges, Nodes_t& newNodes)
{
    PathPtr_t path, validPath;
    // Retrieve the path validation algorithm associated to the problem
    core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
    // Retrieve the steering method and use it to compute
    // a path between the two configurations
    core::SteeringMethodPtr_t sm (problem ().steeringMethod ());
    path = (*sm) (*(nearest->configuration ()), *target);
    if (path)
    {
        // If a path is defined
        // try to validate it
        PathValidationReportPtr_t report;
        bool pathValid = pathValidation->validate (path, false, validPath, report);
        // If the path is only partially validated
        // Insert the end of the valid path q_near to the set of newNodes
        // and add path to q_near in the roadmap
        // else add the new edge
        value_type t_final = validPath->timeRange ().second;
        if (t_final != path->timeRange ().first)
        {
            ConfigurationPtr_t q_new (new Configuration_t (validPath->end ()));
            if (!pathValid || !belongs (q_new, newNodes))
            {
                newNodes.push_back (roadmap ()->addNodeAndEdges(nearest, q_new, validPath));
            }
            else
            {
                delayedEdges.push_back (DelayedEdge_t (nearest, q_new, validPath));
            }
        }
    }
    return validPath;
}

