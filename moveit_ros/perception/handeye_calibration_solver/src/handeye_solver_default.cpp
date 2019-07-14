/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Yu Yan */

#include <moveit/handeye_calibration_solver/handeye_solver_default.h>

namespace moveit_handeye_calibration
{
const std::string LOGNAME = "handeye_solver_default";

void HandEyeSolverDefault::initialize()
{
  solver_names_ = { "Daniilidis1999", "ParkBryan1994", "TsaiLenz1989" };
  camera_robot_pose_ = Eigen::Isometry3d::Identity();
}

const std::vector<std::string>& HandEyeSolverDefault::getSolverNames() const
{
  return solver_names_;
}

const Eigen::Isometry3d& HandEyeSolverDefault::getCameraRobotPose() const
{
  return camera_robot_pose_;
}

bool HandEyeSolverDefault::solve(const std::vector<Eigen::Isometry3d>& effector_wrt_world,
                                 const std::vector<Eigen::Isometry3d>& object_wrt_sensor, SensorMountType setup,
                                 const std::string& solver_name)
{
  // Check the size of the two sets of pose sample equal
  if (effector_wrt_world.size() != object_wrt_sensor.size())
  {
    ROS_ERROR_STREAM_NAMED(
        LOGNAME, "The sizes of the two input pose sample vectors are not equal: effector_wrt_world.size() = "
                     << effector_wrt_world.size() << " and object_wrt_sensor.size() == " << object_wrt_sensor.size());
    return false;
  }

  auto it = std::find(solver_names_.begin(), solver_names_.end(), solver_name);
  if (it == solver_names_.end())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Unknown handeye solver name: " << solver_name);
    return false;
  }

  char program_name[7] = "python";
  Py_SetProgramName(program_name);
  static bool numpy_loaded{ false };
  if (!numpy_loaded)  // Py_Initialize() can only be called once when numpy is loaded, otherwise will segfault
  {
    Py_Initialize();
    atexit(Py_Finalize);
    numpy_loaded = true;
  }
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Python C API start");

  // Load numpy c api
  if (_import_array() < 0)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error importing numpy: ");
    return false;
  }

  PyObject *python_name, *python_module, *python_class, *python_instance, *python_func_add_sample, *python_func_solve;
  PyObject *python_args, *python_value;

  // Import handeye.calibrator python module
  python_name = PyString_FromString("handeye.calibrator");
  python_module = PyImport_Import(python_name);
  Py_DECREF(python_name);
  if (!python_module)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load python module: "
                                        << "handeye.calibrator");
    PyErr_Print();
    return false;
  }

  // Find handeye.calibrator.HandEyeCalibrator class
  python_class = PyObject_GetAttrString(python_module, "HandEyeCalibrator");
  Py_DECREF(python_module);
  if (!python_class || !PyCallable_Check(python_class))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find \"HandEyeCalibrator\" python class");
    PyErr_Print();
    return false;
  }

  // Parse sensor mount type
  python_value = PyString_FromString("");
  if (setup == EYE_TO_HAND)
    python_value = PyString_FromString("Fixed");
  else if (setup == EYE_IN_HAND)
    python_value = PyString_FromString("Moving");
  if (!python_value)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't creat sensor mount type python value");
    Py_DECREF(python_class);
    PyErr_Print();
    return false;
  }

  // Create handeye.calibrator.HandEyeCalibrator instance
  python_args = PyTuple_New(1);
  PyTuple_SetItem(python_args, 0, python_value);
  if (!python_args)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't build python arguments");
    Py_DECREF(python_class);
    PyErr_Print();
    return false;
  }
  python_instance = PyEval_CallObject(python_class, python_args);
  Py_DECREF(python_args);
  Py_DECREF(python_class);
  if (!python_instance)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create \"HandEyeCalibrator\" python instance");
    PyErr_Print();
    return false;
  }

  // Find handeye.calibrator.HandEyeCalibrator.add_sample method
  python_func_add_sample = PyObject_GetAttrString(python_instance, "add_sample");
  if (!python_func_add_sample || !PyCallable_Check(python_func_add_sample))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find 'add_sample' method");
    Py_DECREF(python_instance);
    PyErr_Print();
    return false;
  }

  // Add sample poses to handeye.calibrator.HandEyeCalibrator instance
  size_t number_of_poses = effector_wrt_world.size();
  PyArrayObject *numpy_arg_eef_wrt_base[number_of_poses], *numpy_arg_obj_wrt_sensor[number_of_poses];
  PyObject *python_array_eef_wrt_base[number_of_poses], *python_array_obj_wrt_sensor[number_of_poses];
  PyObject* python_args_sample[number_of_poses];
  npy_intp dims[2]{ TRANSFORM_MATRIX_DIMENSION, TRANSFORM_MATRIX_DIMENSION };
  const int number_of_dims{ 2 };
  // Using C array to store the pyarray data, which will be automatically freed
  double c_arr_eef_wrt_world[number_of_poses][TRANSFORM_MATRIX_DIMENSION][TRANSFORM_MATRIX_DIMENSION];
  double c_arr_obj_wrt_sensor[number_of_poses][TRANSFORM_MATRIX_DIMENSION][TRANSFORM_MATRIX_DIMENSION];
  for (size_t i = 0; i < number_of_poses; ++i)
  {
    // Convert effector_wrt_world[i] from Eigen::isometry3d to C array
    if (!toCArray(effector_wrt_world[i], c_arr_eef_wrt_world[i]))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error converting Eigen::isometry3d to C array");
      Py_DECREF(python_func_add_sample);
      Py_DECREF(python_instance);
      PyErr_Print();
      return false;
    }

    // From C array to PyArrayObject
    python_array_eef_wrt_base[i] =
        PyArray_SimpleNewFromData(number_of_dims, dims, NPY_DOUBLE, (void*)(c_arr_eef_wrt_world[i]));
    if (!python_array_eef_wrt_base[i])
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error creating PyArray object");
      Py_DECREF(python_func_add_sample);
      Py_DECREF(python_instance);
      PyErr_Print();
      return false;
    }
    numpy_arg_eef_wrt_base[i] = (PyArrayObject*)(python_array_eef_wrt_base[i]);
    if (PyArray_NDIM(numpy_arg_eef_wrt_base[i]) == 2)  // Check PyArrayObject dims are 4x4
    {
      npy_intp* py_array_dims = PyArray_DIMS(numpy_arg_eef_wrt_base[i]);
      if (py_array_dims[0] != 4 || py_array_dims[1] != 4)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error PyArrayObject dims: " << py_array_dims[0] << "x" << py_array_dims[1]);
        Py_DECREF(numpy_arg_eef_wrt_base[i]);
        Py_DECREF(python_func_add_sample);
        Py_DECREF(python_instance);
        return false;
      }
    }

    // Convert object_wrt_sensor[i] from Eigen::isometry3d to C array
    if (!toCArray(object_wrt_sensor[i], c_arr_obj_wrt_sensor[i]))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error converting Eigen::isometry3d to C array");
      Py_DECREF(python_func_add_sample);
      Py_DECREF(python_instance);
      PyErr_Print();
      return false;
    }

    // From C array to PyArrayObject
    python_array_obj_wrt_sensor[i] =
        PyArray_SimpleNewFromData(number_of_dims, dims, NPY_DOUBLE, (void*)(c_arr_obj_wrt_sensor[i]));
    if (!python_array_obj_wrt_sensor[i])
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error creating PyArray object");
      Py_DECREF(python_func_add_sample);
      Py_DECREF(python_instance);
      PyErr_Print();
      return false;
    }
    numpy_arg_obj_wrt_sensor[i] = (PyArrayObject*)(python_array_obj_wrt_sensor[i]);
    if (PyArray_NDIM(numpy_arg_obj_wrt_sensor[i]) == 2)  // Check PyArrayObject dims are 4x4
    {
      npy_intp* py_array_dims = PyArray_DIMS(numpy_arg_obj_wrt_sensor[i]);
      if (py_array_dims[0] != 4 || py_array_dims[1] != 4)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Error PyArrayObject dims: " << py_array_dims[0] << "x" << py_array_dims[1]);
        Py_DECREF(numpy_arg_obj_wrt_sensor[i]);
        Py_DECREF(python_func_add_sample);
        Py_DECREF(python_instance);
        return false;
      }
    }

    // Assign sample poses to 'HandEyeCalibrator' instance
    python_args_sample[i] = Py_BuildValue("OO", numpy_arg_eef_wrt_base[i], numpy_arg_obj_wrt_sensor[i]);
    if (!python_args_sample[i])
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create argument tuple for 'add_sample' method");
      Py_DECREF(python_func_add_sample);
      Py_DECREF(python_instance);
      PyErr_Print();
      return false;
    }
    python_value = PyEval_CallObject(python_func_add_sample, python_args_sample[i]);
    if (!python_value)
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Error calling 'add_sample' method");
      Py_DECREF(python_func_add_sample);
      Py_DECREF(python_instance);
      PyErr_Print();
      return false;
    }
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "num_samples: " << PyInt_AsLong(python_value));
    Py_DECREF(python_value);
  }
  Py_DECREF(python_func_add_sample);

  // print the pair of transforms as python arguments
  for (size_t i = 0; i < number_of_poses; i++)
  {
    std::stringstream ss;
    ss << "\nnp_arg_eef_wrt_base";
    for (size_t m = 0; m < TRANSFORM_MATRIX_DIMENSION; m++)
    {
      ss << "\n";
      for (size_t n = 0; n < TRANSFORM_MATRIX_DIMENSION; n++)
        ss << *(double*)PyArray_GETPTR2(numpy_arg_eef_wrt_base[i], m, n) << " ";
    }
    ss << "\nnp_arg_obj_wrt_sensor";
    for (size_t m = 0; m < TRANSFORM_MATRIX_DIMENSION; m++)
    {
      ss << "\n";
      for (size_t n = 0; n < TRANSFORM_MATRIX_DIMENSION; n++)
        ss << *(double*)PyArray_GETPTR2(numpy_arg_obj_wrt_sensor[i], m, n) << " ";
    }
    ROS_DEBUG_STREAM_NAMED(LOGNAME, ss.str());
  }

  // Import handeye.solver python module
  python_name = PyString_FromString("handeye.solver");
  python_module = PyImport_Import(python_name);
  Py_DECREF(python_name);
  if (!python_module)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load python module: "
                                        << "handeye.solver");
    Py_DECREF(python_instance);
    PyErr_Print();
    return false;
  }

  // Find handeye.solver.solver_name class
  python_class = PyObject_GetAttrString(python_module, solver_name.c_str());
  Py_DECREF(python_module);
  if (!python_class || !PyCallable_Check(python_class))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find \"" << solver_name << "\" python class");
    Py_DECREF(python_instance);
    PyErr_Print();
    return false;
  }

  // Find handeye.calibrator.HandEyeCalibrator.solve method
  python_func_solve = PyObject_GetAttrString(python_instance, "solve");
  if (!python_func_solve || !PyCallable_Check(python_func_solve))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't find 'solve' method");
    Py_DECREF(python_class);
    Py_DECREF(python_instance);
    PyErr_Print();
    return false;
  }

  // Create argument list for 'solve' method
  python_args = Py_BuildValue("{s:O}", "method", python_class);
  Py_DECREF(python_class);
  if (!python_args)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Can't create argument tuple for 'solve' method");
    Py_DECREF(python_instance);
    PyErr_Print();
    return false;
  }

  // Call 'solve' method to solve AX=XB problem
  python_value = PyEval_CallObjectWithKeywords(python_func_solve, nullptr, python_args);
  Py_DECREF(python_args);
  Py_DECREF(python_func_solve);
  for (size_t i = 0; i < number_of_poses; ++i)
    Py_DECREF(python_args_sample[i]);
  Py_DECREF(python_instance);
  if (!python_value)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error calling 'solve' method");
    PyErr_Print();
    return false;
  }
  PyArrayObject* np_ret = (PyArrayObject*)python_value;
  if (!PyArray_Check(python_value) || PyArray_NDIM(np_ret) != 2 || PyArray_NBYTES(np_ret) != sizeof(double) * 16)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Did not return a valid array");
    Py_DECREF(python_value);
    PyErr_Print();
    return false;
  }

  std::stringstream ss;
  ss << "\n Result camera-robot pose";
  for (size_t m = 0; m < TRANSFORM_MATRIX_DIMENSION; m++)
  {
    ss << "\n";
    for (size_t n = 0; n < TRANSFORM_MATRIX_DIMENSION; n++)
    {
      double item = *(double*)PyArray_GETPTR2(np_ret, m, n);
      camera_robot_pose_(m, n) = item;
      ss << item << " ";
    }
  }
  ROS_DEBUG_STREAM_NAMED(LOGNAME, ss.str());

  Py_DECREF(python_value);
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Python C API end");
  return true;
}

bool HandEyeSolverDefault::toCArray(const Eigen::Isometry3d& pose, double (*c_arr)[TRANSFORM_MATRIX_DIMENSION]) const
{
  const Eigen::MatrixXd& mat = pose.matrix();

  if (mat.rows() != TRANSFORM_MATRIX_DIMENSION || mat.cols() != TRANSFORM_MATRIX_DIMENSION)
  {
    ROS_ERROR_NAMED(LOGNAME, "Error matrix dims: %zux%zu, required %dx%d", mat.rows(), mat.cols(),
                    TRANSFORM_MATRIX_DIMENSION, TRANSFORM_MATRIX_DIMENSION);
    return false;
  }

  for (size_t i = 0; i < TRANSFORM_MATRIX_DIMENSION; ++i)
    for (size_t j = 0; j < TRANSFORM_MATRIX_DIMENSION; ++j)
      c_arr[i][j] = mat(i, j);
  return true;
}

}  // namespace moveit_handeye_calibration
