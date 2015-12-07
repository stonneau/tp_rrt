//
// Copyright (c) 2015 CNRS
// Authors: Steve Tonneau
//
// This file is part of hpp_tutorial
// hpp_tutorial is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp_tutorial is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp_tutorial  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/tp-rrt/planner-tp.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>

// main function of the corba server
int main (int argc, const char* argv[])
{
  // create a ProblemSolver instance.
  // This class is a container that does the interface between hpp-core library
  // and component to be running in a middleware like CORBA or ROS.
  hpp::core::ProblemSolverPtr_t problemSolver =
    hpp::core::ProblemSolver::create ();
  // Add the new planner type in order to be able to select it from python
  // client.
  problemSolver->addPathPlannerType ("PlannerTP", hpp::tp_rrt::PlannerTP::create);
  // Create the CORBA server.
  hpp::corbaServer::Server server (problemSolver, argc, argv, true);
  // Start the CORBA server.
  server.startCorbaServer ();
  // Wait for CORBA requests.
  server.processRequest (true);
}
