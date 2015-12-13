#
# Copyright (c) 2015 CNRS
# Authors: Steve Tonneau
#
#
# This file is part of tp_rtt
# tp_rtt is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_tutorial is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_tutorial  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

ADD_EXECUTABLE (hpp-tp-rrt
  shooter-tp.cc ${PROJECT_SOURCE_DIR}/include/hpp/tp-rrt/shooter-tp.hh
  planner-tp.cc ${PROJECT_SOURCE_DIR}/include/hpp/tp-rrt/planner-tp.hh
  flat-path.cc ${PROJECT_SOURCE_DIR}/include/hpp/tp-rrt/flat-path.hh
  ${PROJECT_SOURCE_DIR}/include/hpp/tp-rrt/steering-method.hh
  ${PROJECT_SOURCE_DIR}/include/hpp/tp-rrt/fwd.hh
  main.cc
)
# Link executable with hpp-corbaserver library
PKG_CONFIG_USE_DEPENDENCY (hpp-tp-rrt hpp-corbaserver)
# Install executable
INSTALL (TARGETS hpp-tp-rrt DESTINATION ${CMAKE_INSTALL_BINDIR})
