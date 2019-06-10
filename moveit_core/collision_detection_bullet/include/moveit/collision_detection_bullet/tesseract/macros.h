/**
 * @file macros.h
 * @brief The tesseract_core package macros.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_CORE_MACROS_H
#define TESSERACT_CORE_MACROS_H
#define TESSERACT_IGNORE_WARNINGS_PUSH                                                                                 \
  _Pragma("GCC diagnostic push") _Pragma("GCC diagnostic ignored \"-Wall\"")                                           \
      _Pragma("GCC diagnostic ignored \"-Wint-to-pointer-cast\"")                                                      \
          _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"")                                                     \
              _Pragma("GCC diagnostic ignored \"-Wsuggest-override\"")                                                 \
                  _Pragma("GCC diagnostic ignored \"-Wconversion\"")                                                   \
                      _Pragma("GCC diagnostic ignored \"-Wfloat-conversion\"")                                         \
                          _Pragma("GCC diagnostic ignored \"-Wsign-conversion\"")

#define TESSERACT_IGNORE_WARNINGS_POP _Pragma("GCC diagnostic pop")

#endif  // TESSERACT_CORE_MACROS_H
