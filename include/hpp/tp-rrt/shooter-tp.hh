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

#ifndef HPP_SHOOTER_TP_HH
# define HPP_SHOOTER_TP_HH

# include <hpp/model/device.hh>
# include <hpp/model/joint.hh>
# include <hpp/model/joint-configuration.hh>
# include <hpp/core/configuration-shooter.hh>
# include <hpp/tp-rrt/fwd.hh>

namespace hpp {
namespace tp_rrt {

/// \addtogroup configuration_sampling
/// \{

/// Uniformly sample with bounds of degrees of freedom.
class ShooterTP : public core::ConfigurationShooter
{
    public:
    static ShooterTPPtr_t create (const core::DevicePtr_t& robot)
    {
        ShooterTP* ptr = new ShooterTP (robot);
        ShooterTPPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
    }

    /// Uniformly sample configuration space
    ///
    /// Note that translation joints have to be bounded.
    virtual core::ConfigurationPtr_t shoot () const;

    protected:
    ShooterTP (const core::DevicePtr_t& robot) : robot_ (robot) {}

    void init (const ShooterTPPtr_t& self)
    {
        core::ConfigurationShooter::init (self);
        weak_ = self;
    }

    private:
    const core::DevicePtr_t& robot_;
    ShooterTPWkPtr_t weak_;
}; // class ShooterTP
/// \}
} //   namespace tp_rrt
} // namespace hpp

#endif // HPP_SHOOTER_TP_HH
