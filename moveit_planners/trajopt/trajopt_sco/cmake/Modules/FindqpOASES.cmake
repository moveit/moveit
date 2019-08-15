#.rst:
# FindqpOASES
# -----------
#
# Try to find the qpOASES library.
# Once done this will define the following variables::
#
#  qpOASES_FOUND         - System has qpOASES
#  qpOASES_INCLUDE_DIRS  - qpOASES include directory
#  qpOASES_LIBRARIES     - qpOASES libraries
#
# qpOASES does not have an "install" step, and the includes are in the source
# tree, while the libraries are in the build tree.
# Therefore the environment and cmake variables `qpOASES_SOURCE_DIR` and
# `qpOASES_BINARY_DIR` will be used to locate the includes and libraries.

#=============================================================================
# Copyright 2014 iCub Facility, Istituto Italiano di Tecnologia
#   Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of YCM, substitute the full
#  License text for the above reference.)


include(FindPackageHandleStandardArgs)

find_path(qpOASES_INCLUDEDIR
          NAMES qpOASES.hpp
          HINTS "${qpOASES_SOURCE_DIR}"
                ENV qpOASES_SOURCE_DIR
          PATH_SUFFIXES include)
find_library(qpOASES_LIB
             NAMES qpOASES
             HINTS "${qpOASES_BINARY_DIR}"
                   ENV qpOASES_BINARY_DIR
             PATH_SUFFIXES lib
                           libs)

set(qpOASES_INCLUDE_DIRS ${qpOASES_INCLUDEDIR})
set(qpOASES_LIBRARIES ${qpOASES_LIB})

find_package_handle_standard_args(qpOASES DEFAULT_MSG qpOASES_LIBRARIES
                                                      qpOASES_INCLUDE_DIRS)
set(qpOASES_FOUND ${QPOASES_FOUND})
