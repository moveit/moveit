/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

/** The following macros allow to temporarily disable specific warnings,
 *  which in turn allows us to compile the MoveIt code base with rather strict
 *  compiler warnings, and still avoid corresponding issues in external sources.
 *  Just wrap an offending include with:
 *
 *  DIAGNOSTIC_PUSH
 *  SILENT_UNUSED_PARAM
 *  #include <offender.h>
 *  DIAGNOSTIC_POP
 *
 *  The push and pop operations ensure that the old status is restored afterwards.
 */

#if defined(_MSC_VER)
#define DIAGNOSTIC_PUSH __pragma(warning(push))
#define DIAGNOSTIC_POP __pragma(warning(pop))
#define SILENT_UNUSED_PARAM __pragma(warning(disable : 4100))

#elif defined(__GNUC__) || defined(__clang__)
#define DO_PRAGMA(X) _Pragma(#X)
#define DIAGNOSTIC_PUSH DO_PRAGMA(GCC diagnostic push)
#define DIAGNOSTIC_POP DO_PRAGMA(GCC diagnostic pop)

#if defined(__clang__)
#define SILENT_UNUSED_PARAM DO_PRAGMA(GCC diagnostic ignored "-Wunused-parameter")
#else
#define SILENT_UNUSED_PARAM                                                                                            \
  DO_PRAGMA(GCC diagnostic ignored "-Wunused-parameter")                                                               \
  DO_PRAGMA(GCC diagnostic ignored "-Wunused-but-set-parameter")
#endif

#else
#define DIAGNOSTIC_PUSH
#define DIAGNOSTIC_POP
#define SILENT_UNUSED_PARAM

#endif
