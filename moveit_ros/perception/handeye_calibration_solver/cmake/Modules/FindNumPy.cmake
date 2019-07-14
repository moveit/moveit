# COPYRIGHT

# All contributions by the University of California:
# Copyright (c) 2014-2017 The Regents of the University of California (Regents)
# All rights reserved.

# All other contributions:
# Copyright (c) 2014-2017, the respective contributors
# All rights reserved.

# Caffe uses a shared copyright model: each contributor holds copyright over
# their contributions to Caffe. The project versioning records all such
# contribution and copyright details. If a contributor wants to further mark
# their specific copyright on a particular contribution, they should indicate
# their copyright solely in the commit message of the change when it is
# committed.

# LICENSE

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# - Find the NumPy libraries
# This module finds if NumPy is installed, and sets the following variables
# indicating where it is.
#
# TODO: Update to provide the libraries and paths for linking npymath lib.
#
#  NUMPY_FOUND               - was NumPy found
#  NUMPY_VERSION             - the version of NumPy found as a string
#  NUMPY_VERSION_MAJOR       - the major version number of NumPy
#  NUMPY_VERSION_MINOR       - the minor version number of NumPy
#  NUMPY_VERSION_PATCH       - the patch version number of NumPy
#  NUMPY_VERSION_DECIMAL     - e.g. version 1.6.1 is 10601
#  NUMPY_INCLUDE_DIR         - path to the NumPy include files

unset(NUMPY_VERSION)
unset(NUMPY_INCLUDE_DIR)

if(PYTHONINTERP_FOUND)
  execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
    "import numpy as n; print(n.__version__); print(n.get_include());"
    RESULT_VARIABLE __result
    OUTPUT_VARIABLE __output
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if(__result MATCHES 0)
    string(REGEX REPLACE ";" "\\\\;" __values ${__output})
    string(REGEX REPLACE "\r?\n" ";"    __values ${__values})
    list(GET __values 0 NUMPY_VERSION)
    list(GET __values 1 NUMPY_INCLUDE_DIR)

    string(REGEX MATCH "^([0-9])+\\.([0-9])+\\.([0-9])+" __ver_check "${NUMPY_VERSION}")
    if(NOT "${__ver_check}" STREQUAL "")
      set(NUMPY_VERSION_MAJOR ${CMAKE_MATCH_1})
      set(NUMPY_VERSION_MINOR ${CMAKE_MATCH_2})
      set(NUMPY_VERSION_PATCH ${CMAKE_MATCH_3})
      math(EXPR NUMPY_VERSION_DECIMAL
        "(${NUMPY_VERSION_MAJOR} * 10000) + (${NUMPY_VERSION_MINOR} * 100) + ${NUMPY_VERSION_PATCH}")
      string(REGEX REPLACE "\\\\" "/"  NUMPY_INCLUDE_DIR ${NUMPY_INCLUDE_DIR})
    else()
     unset(NUMPY_VERSION)
     unset(NUMPY_INCLUDE_DIR)
     message(STATUS "Requested NumPy version and include path, but got instead:\n${__output}\n")
    endif()
  endif()
else()
  message(STATUS "Could not find NumPy. To find it, a Python interpreter needs to be found first.")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NumPy REQUIRED_VARS NUMPY_INCLUDE_DIR NUMPY_VERSION
                                        VERSION_VAR   NUMPY_VERSION)

if(NUMPY_FOUND)
  message(STATUS "NumPy ver. ${NUMPY_VERSION} found (include: ${NUMPY_INCLUDE_DIR})")
endif()
