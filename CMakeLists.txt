#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp_tutorial
# hpp_tutorial is free software: you can redistribute it
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

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME tp-rrt)
SET(PROJECT_DESCRIPTION
  "Tutorial for humanoid path planner platform."
)
SET(PROJECT_URL "")

FINDPYTHON()

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

ADD_DOC_DEPENDENCY("hpp-core >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-util")

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
    include/hpp/tp-rrt/shooter-tp.hh
    include/hpp/tp-rrt/planner-tp.hh
    include/hpp/tp-rrt/steering-method.hh
  )

ADD_SUBDIRECTORY(src)

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  data/package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  data/urdf/buggy.urdf
  data/urdf/scene.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  data/srdf/buggy.srdf
  data/srdf/scene.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
  data/meshes/buggy.dae
  data/meshes/buggyBB.dae
  data/meshes/scene.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  )
install (FILES
  src/hpp/tp_rrt/robot.py
  src/hpp/tp_rrt/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/tp_rrt)

install(FILES
  script/tp-rrt.py
  DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html/script
  )
SETUP_PROJECT_FINALIZE()
