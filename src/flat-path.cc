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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/tp-rrt/flat-path.hh>

namespace hpp {
  namespace tp_rrt {
    FlatPath::FlatPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				value_type distanceBetweenAxes) :
      parent_t (interval_t (0, 1.), device->configSize (),
		device->numberDof ()),
      device_ (device), initial_ (init), end_ (end)
    {
      assert (device);
      assert (distanceBetweenAxes >= 0);
      assert (!constraints ());
    }

    FlatPath::FlatPath (const DevicePtr_t& device,
				ConfigurationIn_t init,
				ConfigurationIn_t end,
				value_type distanceBetweenAxes,
				ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, distanceBetweenAxes), device->configSize (),
		device->numberDof (), constraints),
      device_ (device), initial_ (init), end_ (end)
    {
      assert (device);
      assert (distanceBetweenAxes >= 0);
      assert (!constraints || constraints->isSatisfied (initial_));
      if (constraints && !constraints->isSatisfied (end_)) {
	hppDout (error, *constraints);
	hppDout (error, end_.transpose ());
	abort ();
      }
    }

    FlatPath::FlatPath (const FlatPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_)
    {
    }

    FlatPath::FlatPath (const FlatPath& path,
				const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_)
    {
      assert (constraints->apply (initial_));
      assert (constraints->apply (end_));
      assert (constraints->isSatisfied (initial_));
      assert (constraints->isSatisfied (end_));
    }

    bool FlatPath::impl_compute (ConfigurationOut_t result,
				     value_type param) const
    {
      if (param == timeRange ().first || timeRange ().second == 0) {
	result = initial_;
	return true;
      }
      if (param == timeRange ().second) {
	result = end_;
	return true;
      }
      return true;
    }

    DevicePtr_t FlatPath::device () const
    {
      return device_;
    }
  } //   namespace tp_rrt
} // namespace hpp

