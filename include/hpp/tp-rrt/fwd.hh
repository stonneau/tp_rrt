//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux
//
// This file is part of tp_rrt
// tp_rrt is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// tp_rrt is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// tp_rrt  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_TP_RRT_FWD_HH
# define HPP_TP_RRT_FWD_HH

# include <hpp/util/pointer.hh>
# include <hpp/core/fwd.hh>

namespace hpp {
  namespace tp_rrt {
    HPP_PREDEF_CLASS (FlatPath);
    typedef boost::shared_ptr <FlatPath> FlatPathPtr_t;
    HPP_PREDEF_CLASS (SteeringMethod);
    typedef boost::shared_ptr <SteeringMethod> SteeringMethodPtr_t;
    HPP_PREDEF_CLASS (ShooterTP);
    typedef boost::shared_ptr <ShooterTP> ShooterTPPtr_t;
    HPP_PREDEF_CLASS (PlannerTP);
    typedef boost::shared_ptr <PlannerTP> PlannerTPPtr_t;
    typedef core::value_type value_type;
    typedef model::Device Device_t;
    typedef model::DevicePtr_t DevicePtr_t;
    typedef model::DeviceWkPtr_t DeviceWkPtr_t;
    typedef model::Configuration_t Configuration_t;
    typedef model::ConfigurationIn_t ConfigurationIn_t;
    typedef model::ConfigurationOut_t ConfigurationOut_t;
    typedef core::ConstraintSet ConstraintSet;
    typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
    typedef core::Path Path;
    typedef core::PathPtr_t PathPtr_t;
    typedef core::interval_t interval_t;

    typedef Eigen::Matrix <value_type, 2, 1> vector2_t;
    typedef model::vector3_t Vector3_t;
    typedef model::Transform3f Transform3f;
  } // tp_rrt
} // namespace hpp
#endif // HPP_TP_RRT_FWD_HH
