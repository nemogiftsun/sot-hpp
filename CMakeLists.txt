#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-core
# hpp-core is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-core  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME sot-hpp)
SET(PROJECT_DESCRIPTION "Bridge between hpp and sot")
SET(PROJECT_URL "")

SETUP_PROJECT()

ADD_REQUIRED_DEPENDENCY ("hpp-core >= 3")
ADD_REQUIRED_DEPENDENCY ("hpp-model-urdf >= 3")
ADD_REQUIRED_DEPENDENCY ("dynamic-graph-python >= 2")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
