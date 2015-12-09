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

#include <hpp/tp-rrt/shooter-tp.hh>
#include <hpp/model/joint-configuration.hh>

using namespace  hpp::core;
using namespace  hpp::tp_rrt;

namespace
{
    typedef std::vector<double> d_vector;
    // Vous devez implémenter cette méthode
    /// Uniformly sample a random value for a joint of the robot
    /// \param min_value: minimum value that can be taken by the current joint
    /// \param max_value: maximum value that can be taken by the current joint
    /// \return the sampled value
    double sample(double min_value, double max_value)
    {
        //std::cout << "TODO: implémenter SHOOT !" << std::endl;
        // SOLUTION
        return ((double) rand() / (RAND_MAX)) * (max_value-min_value+1) + min_value;
        //return 0.;
    }
}

ConfigurationPtr_t ShooterTP::shoot() const
{
    JointVector_t jv = robot_->getJointVector ();
    ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
    for (JointVector_t::const_iterator itJoint = jv.begin ();
     itJoint != jv.end (); itJoint++)
    {
        std::size_t rank = (*itJoint)->rankInConfiguration ();
        const hpp::model::JointConfiguration* jc = (*itJoint)->configuration ();
        if(!jc->isBounded(rank))
        {
         //   throw std::runtime_error("Joint not bounded. You must bound the joints to sample a joint configuration.");
        }
        //(*config)[rank] = value_type(sample(jc->lowerBound(rank), jc->upperBound(rank)));
        (*itJoint)->configuration ()->uniformlySample (rank, *config);
    }
    return config;
}
