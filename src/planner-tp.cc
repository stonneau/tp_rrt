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

using namespace  hpp::core;
using namespace  hpp::tp_rrt;

namespace
{
    // Vous devez implémenter cette méthode
    /// Uniformly sample a random value for a joint of the robot
    /// \param min_value: minimum value that can be taken by the current joint
    /// \param max_value: maximum value that can be taken by the current joint
    /// \return the sampled value
    double sample(double /*min_value*/, double /*max_value*/)
    {
        std::cout << "TODO: implémenter SHOOT !" << std::endl;
        return 0.;
    }
}

// Vous devez implémenter cette méthode
/// One step of extension.
///
/// This method implements one step of your algorithm. The method
/// will be called iteratively until one goal configuration is accessible
/// from the initial configuration.
///
/// We will see how to implement a basic PRM algorithm.
void PlannerTP::oneStep ()
{
    // Retrieve the path validation algorithm associated to the problem
    core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
    // Retrieve configuration validation methods associated to the problem
    core::ConfigValidationsPtr_t configValidations (problem ().configValidations ());
    // Retrieve the steering method
    core::SteeringMethodPtr_t sm (problem ().steeringMethod ());
    // Retrieve roadmap of the path planner
    core::RoadmapPtr_t r (roadmap ());
    // shoot a valid random configuration
    core::ConfigurationPtr_t qrand;
    do
    {
        qrand = shooter_->shoot ();
    } while (!configValidations->validate (*qrand));
    // Add qrand as a new node
    core::NodePtr_t newNode = r->addNode (qrand);
    // try to connect the random configuration to each connected component
    // of the roadmap.
    for (core::ConnectedComponents_t::const_iterator itcc =
    r->connectedComponents ().begin ();
    itcc != r->connectedComponents ().end (); ++itcc)
    {
        core::ConnectedComponentPtr_t cc = *itcc;
        // except its own connected component of course
        if (cc != newNode->connectedComponent ())
        {
            double d;
            // Get nearest node to qrand in connected component
            core::NodePtr_t nearest = r->nearestNode (qrand, cc, d);
            core::ConfigurationPtr_t qnear = nearest->configuration ();
            // Create local path between qnear and qrand
            core::PathPtr_t localPath = (*sm) (*qnear, *qrand);
            // validate local path
            core::PathPtr_t validPart;
            if (pathValidation->validate (localPath, false, validPart))
            {
                // Create node and edges with qrand and the local path
                r->addEdge (nearest, newNode, localPath);
                r->addEdge (newNode, nearest, localPath->reverse ());
            }
        }
    }
}
