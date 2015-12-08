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

# include <hpp/util/pointers.hh>

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
  } // tp_rrt
} // namespace hpp
#endif // HPP_TP_RRT_FWD_HH
