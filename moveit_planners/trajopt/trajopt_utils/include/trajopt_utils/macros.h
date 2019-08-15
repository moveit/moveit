#pragma once
#include <iostream>

#define TRAJOPT_IGNORE_WARNINGS_PUSH                                                                                   \
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wall\"")                                           \
      _Pragma("GCC diagnostic ignored \"-Wint-to-pointer-cast\"")                                                      \
          _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")                                                     \
              _Pragma("GCC diagnostic ignored \"-Wsuggest-override\"")                                                 \
                  _Pragma("GCC diagnostic ignored \"-Wconversion\"")                                                   \
                      _Pragma("GCC diagnostic ignored \"-Wfloat-conversion\"")                                         \
                          _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")

#define TRAJOPT_IGNORE_WARNINGS_POP _Pragma("GCC diagnostic pop")

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define TRAJOPT_HELPER_DLL_IMPORT __declspec(dllimport)
#define TRAJOPT_HELPER_DLL_EXPORT __declspec(dllexport)
#define TRAJOPT_HELPER_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define TRAJOPT_HELPER_DLL_IMPORT __attribute__((visibility("default")))
#define TRAJOPT_HELPER_DLL_EXPORT __attribute__((visibility("default")))
#define TRAJOPT_HELPER_DLL_LOCAL __attribute__((visibility("hidden")))
#else
#define TRAJOPT_HELPER_DLL_IMPORT
#define TRAJOPT_HELPER_DLL_EXPORT
#define TRAJOPT_HELPER_DLL_LOCAL
#endif
#endif

// Now we use the generic helper definitions above to define TRAJOPT_API and TRAJOPT_LOCAL.
// TRAJOPT_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static
// build)
// TRAJOPT_LOCAL is used for non-api symbols.

#define TRAJOPT_DLL

#ifdef TRAJOPT_DLL          // defined if TRAJOPT is compiled as a DLL
#ifdef TRAJOPT_DLL_EXPORTS  // defined if we are building the TRAJOPT DLL (instead of using it)
#define TRAJOPT_API TRAJOPT_HELPER_DLL_EXPORT
#else
#define TRAJOPT_API TRAJOPT_HELPER_DLL_IMPORT
#endif  // TRAJOPT_DLL_EXPORTS
#define TRAJOPT_LOCAL TRAJOPT_HELPER_DLL_LOCAL
#else  // TRAJOPT_DLL is not defined: this means TRAJOPT is a static lib.
#define TRAJOPT_API
#define TRAJOPT_LOCAL
#endif  // TRAJOPT_DLL

#define PRINT_AND_THROW(s)                                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    std::cerr << "\033[1;31mERROR " << s << "\033[0m\n";                                                               \
    std::cerr << "at " << __FILE__ << ":" << __LINE__ << std::endl;                                                    \
    std::stringstream ss;                                                                                              \
    ss << s;                                                                                                           \
    throw std::runtime_error(ss.str());                                                                                \
  } while (0)
#define FAIL_IF_FALSE(expr)                                                                                            \
  if (!(expr))                                                                                                         \
  {                                                                                                                    \
    PRINT_AND_THROW("expected true: " #expr);                                                                          \
  }

#define ALWAYS_ASSERT(exp)                                                                                             \
  if (!(exp))                                                                                                          \
  {                                                                                                                    \
    printf("%s failed in file %s at line %i\n", #exp, __FILE__, __LINE__);                                             \
    abort();                                                                                                           \
  }
