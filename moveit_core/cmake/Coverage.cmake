#
# CMake module that detects if the compilers support coverage
#
# If this is the case, the build type Coverage is set up for coverage, and COVERAGE_SUPPORTED is set to true.
# By default the Coverage build type is added to CMAKE_CONFIGURATION_TYPES on multi-config generators.
# This can be controlled through the COVERAGE_IN_CONFIGURATION_TYPES option.
#

# License:
#
# Copyright (C) 2017 Lectem <lectem@gmail.com>
#
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files
# (the 'Software') deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.3)

include(CMakeDependentOption)

set(COVERAGE_COMPILER_FLAGS  "-g -O0 --coverage" CACHE INTERNAL "")
set(COVERAGE_LINKER_FLAGS    "--coverage"        CACHE INTERNAL "")

get_property(ENABLED_LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)

foreach(_LANG IN LISTS ENABLED_LANGUAGES)
	include(Check${_LANG}CompilerFlag OPTIONAL)
	set(CMAKE_REQUIRED_LIBRARIES ${COVERAGE_LINKER_FLAGS}) # This is ugly, but better than rewriting/fixing check_<LANG>_compiler_flag

	if(_LANG STREQUAL "C")
		check_c_compiler_flag("--coverage" ${_LANG}_COVERAGE_SUPPORTED)
	elseif(_LANG STREQUAL "CXX")
		check_cxx_compiler_flag("--coverage" ${_LANG}_COVERAGE_SUPPORTED)
	else()
		if(DEFINED ${_LANG}_COVERAGE_SUPPORTED)
			message(STATUS "Skipping ${_LANG}, not supported by Coverage.cmake script")
		endif()
		set(${_LANG}_COVERAGE_SUPPORTED FALSE)
		continue()
	endif()
	if(${_LANG}_COVERAGE_SUPPORTED)
		set(CMAKE_${_LANG}_FLAGS_COVERAGE
			${COVERAGE_COMPILER_FLAGS}
			CACHE STRING "Flags used by the ${_LANG} compiler during coverage builds."
		)
		mark_as_advanced(CMAKE_${_LANG}_FLAGS_COVERAGE)
		set(COVERAGE_SUPPORTED TRUE CACHE INTERNAL "Whether or not coverage is supported by at least one compiler.")
	endif()
endforeach()

if(COVERAGE_SUPPORTED)
	set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
		"${COVERAGE_LINKER_FLAGS}"
		CACHE STRING "Flags used for linking binaries during coverage builds."
	)
	set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE
		"${COVERAGE_LINKER_FLAGS}"
		CACHE STRING "Flags used by the shared libraries linker during coverage builds."
	)
	mark_as_advanced(
		CMAKE_EXE_LINKER_FLAGS_COVERAGE
		CMAKE_SHARED_LINKER_FLAGS_COVERAGE
	)
endif()


cmake_dependent_option(COVERAGE_IN_CONFIGURATION_TYPES
    "Should the Coverage target be in the CMAKE_CONFIGURATION_TYPES list if supported ?" ON
    "CMAKE_CONFIGURATION_TYPES;COVERAGE_SUPPORTED" OFF # No need for this option if we are not using a multi-config generator
)

if(COVERAGE_IN_CONFIGURATION_TYPES)
	# Modify this only if using a multi-config generator, some modules rely on this variable to detect those generators.
	if(CMAKE_CONFIGURATION_TYPES AND COVERAGE_SUPPORTED)
		list(APPEND CMAKE_CONFIGURATION_TYPES Coverage)
		list(REMOVE_DUPLICATES CMAKE_CONFIGURATION_TYPES)
		set(CMAKE_CONFIGURATION_TYPES
			"${CMAKE_CONFIGURATION_TYPES}"
			CACHE STRING
			"Semicolon separated list of supported configuration types, only supports ${CMAKE_CONFIGURATION_TYPES} anything else will be ignored."
			FORCE
		)
	endif()
else()
	if(Coverage IN_LIST CMAKE_CONFIGURATION_TYPES)
		message(STATUS "Removing Coverage configuration type (COVERAGE_IN_CONFIGURATION_TYPES is OFF)")
		list(REMOVE_ITEM CMAKE_CONFIGURATION_TYPES Coverage)
		list(REMOVE_DUPLICATES CMAKE_CONFIGURATION_TYPES)
		set(CMAKE_CONFIGURATION_TYPES
			"${CMAKE_CONFIGURATION_TYPES}"
			CACHE STRING
			"Semicolon separated list of supported configuration types, only supports ${CMAKE_CONFIGURATION_TYPES} anything else will be ignored."
			FORCE
		)
	endif()
endif()
