# - Try to find GUROBI
#
# This modules reads hints about search locations from the following
# environment variables:
# GUROBI_HOME
# 
# e.g. GUROBI_HOME=/opt/gurobobi810/linux64
#
# The module will define:
# GUROBI_FOUND - If we found Gurobi in the system
# GUROBI_INCLUDE_DIR - Gurobi include directories
# GUROBI_LIBRARIES - libraries needed to use Gurobi
#
# This module will NOT enforce a specific GUROBI version, rather it will
# only work with latest GUROBI version (8.1.0)

find_path(GUROBI_INCLUDE_DIR NAMES gurobi_c++.h
          HINTS
          ENV GUROBI_HOME
          PATH_SUFFIXES include
          )

find_library(GUROBI_LIBRARY NAMES gurobi81
             HINTS
             ENV GUROBI_HOME
             PATH_SUFFIXES lib
             )

find_library(GUROBI_CXX_LIBRARY NAMES gurobi_c++
             HINTS
             ENV GUROBI_HOME
             PATH_SUFFIXES lib
             )

set(GUROBI_LIBRARIES  ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set GUROBI_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(GUROBI  DEFAULT_MSG
                                  GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_INCLUDE_DIR)

mark_as_advanced(GUROBI_INCLUDE_DIR GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_LIBRARIES)
