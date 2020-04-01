/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, RWTH Aachen University.
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
 *   * Neither the name of RWTH Aachen University nor the names of its
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

/* Author: Bjarne von Horn */

#include <Python.h>
#include <boost/python.hpp>
#include <gtest/gtest.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <geometry_msgs/Vector3.h>
#include <cstring>

namespace bp = boost::python;
using moveit::py_bindings_tools::ByteString;

// Helper class to be exposed to Python
class ByteStringTestHelper
{
  // Helper to test whether a vector of unsigned chars has the same content as a bytes Object
  bool doCompare(const std::vector<unsigned char>& data, PyObject* obj)
  {
    const char* py_data = PyBytes_AsString(obj);
    if (!py_data)
      return false;
    Py_ssize_t size = PyBytes_GET_SIZE(obj);
    if (size < 0 || std::vector<unsigned char>::size_type(size) != data.size())
      return false;
    return std::memcmp(py_data, &data[0], size) == 0;
  }

public:
  bool compareBytes(const ByteString& s)
  {
    const std::vector<unsigned char> testdata{ 0xff, 0xef, 0x00, 0x10 };
    return doCompare(testdata, s.ptr());
  }
  bool compareTuple(const bp::tuple& t)
  {
    const ByteString s(t[0]);
    const std::vector<unsigned char> testdata{ 'm', 'n', 'o' };
    return doCompare(testdata, s.ptr());
  }

  ByteString getBytesPChar()
  {
    return ByteString("abcdef");
  }
  ByteString getBytesStdString()
  {
    std::string s;
    s.push_back('\xff');
    s.push_back('\xfe');
    s.push_back('\x10');
    s.push_back('\x00');
    s.push_back('\x00');
    return ByteString(s);
  }
  ByteString getDefaultBytes()
  {
    return ByteString();
  }
  bp::tuple getTuple()
  {
    return bp::make_tuple(ByteString("abcdef"));
  }
  ByteString getVector()
  {
    geometry_msgs::Vector3 v;
    v.x = 1.0;
    v.y = -2.0;
    v.z = 0.25;
    return ByteString(v);
  }
  bool compareVector(const ByteString& s)
  {
    geometry_msgs::Vector3 v;
    s.deserialize(v);
    return v.x == 1.0 && v.y == -2.0 && v.z == 0.25;
  }
  bool compareVectorTuple(const bp::tuple& t)
  {
    const ByteString s(t[0]);
    geometry_msgs::Vector3 v;
    s.deserialize(v);
    return v.x == 1.0 && v.y == -2.0 && v.z == 0.25;
  }

  static void setup(bp::object& main_ns)
  {
    bp::class_<ByteStringTestHelper> cls("ByteStringTestHelper");
    cls.def("compareBytes", &ByteStringTestHelper::compareBytes);
    cls.def("compareTuple", &ByteStringTestHelper::compareTuple);
    cls.def("compareVectorTuple", &ByteStringTestHelper::compareVectorTuple);
    cls.def("getBytesPChar", &ByteStringTestHelper::getBytesPChar);
    cls.def("getBytesStdString", &ByteStringTestHelper::getBytesStdString);
    cls.def("getDefaultBytes", &ByteStringTestHelper::getDefaultBytes);
    cls.def("getTuple", &ByteStringTestHelper::getTuple);
    cls.def("getVector", &ByteStringTestHelper::getVector);
    cls.def("compareVector", &ByteStringTestHelper::compareVector);
    cls.def("compareVectorTuple", &ByteStringTestHelper::compareVectorTuple);
    main_ns["ByteStringTestHelper"] = cls;
  }
};

struct ByteStringTest : public ::testing::Test
{
  static bp::object main_namespace_;
  static void SetUpTestCase()
  {
    try
    {
      bp::object main_module(bp::handle<>(bp::borrowed(PyImport_AddModule("__main__"))));
      main_namespace_ = main_module.attr("__dict__");
      ByteStringTestHelper::setup(main_namespace_);
      bp::exec("helper = ByteStringTestHelper()", main_namespace_);
      bp::object msg_module(bp::handle<>(bp::borrowed(PyImport_ImportModule("geometry_msgs.msg"))));
      main_namespace_["msgs"] = msg_module;
// StringIO in Python 2.7, BytesIO in >= 3.0
#if PY_VERSION_HEX >= 0x03000000
      bp::object io_module(bp::handle<>(bp::borrowed(PyImport_ImportModule("io"))));
      main_namespace_["StringIO"] = io_module.attr("BytesIO");
#else
      bp::object io_module(bp::handle<>(bp::borrowed(PyImport_ImportModule("StringIO"))));
      main_namespace_["StringIO"] = io_module.attr("StringIO");
#endif
    }
    catch (const bp::error_already_set&)
    {
      failAddPythonErrorToGTest(__LINE__);
    }
  }
  static void TearDownTestCase()
  {
    main_namespace_ = bp::object();
  }
  void TearDown() override
  {
    // reset used variables
    main_namespace_["result"] = bp::object();
    main_namespace_["tmp"] = bp::object();
  }
  // add python exception message to gtest output and reset it
  static void failAddPythonErrorToGTest(int linenumber) noexcept
  {
    PyObject *exc, *val, *tb;
    PyErr_Fetch(&exc, &val, &tb);
    PyErr_NormalizeException(&exc, &val, &tb);
    // force Unicode in Python 2.7
    PyObject* err_str =
#if PY_VERSION_HEX >= 0x03000000
        PyObject_Str(val);
#else
        PyObject_Unicode(val);
#endif
    if (err_str)
    {
      PyObject* ascii = PyUnicode_AsASCIIString(err_str);
      if (ascii)
      {
        ADD_FAILURE_AT(__FILE__, linenumber) << "Python error: " << PyBytes_AsString(ascii);
        Py_DECREF(ascii);
      }
      else
      {
        ADD_FAILURE() << "Unknown Python Error";
      }
      Py_DECREF(err_str);
    }
    else
    {
      ADD_FAILURE() << "Unknown Python Error";
    }
    // drop Exception
    Py_XDECREF(exc);
    Py_XDECREF(val);
    Py_XDECREF(tb);
    PyErr_Clear();
  }
};

bp::object ByteStringTest::main_namespace_;

// sending Python bytes instance to C++
TEST_F(ByteStringTest, PyBytesToCxx)
{
  bp::exec("result = helper.compareBytes(b'\\xff\\xef\\x00\\x10')", main_namespace_);
  ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
}

// sending ByteString C++ instance to Python
TEST_F(ByteStringTest, CxxBytesToPy)
{
  bp::exec("result = helper.getBytesPChar() == b'abcdef'", main_namespace_);
  ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
}

// sending std::string with embedded NULL to Python
TEST_F(ByteStringTest, CxxStringToPy)
{
  bp::exec("result = helper.getBytesStdString() == b'\\xff\\xfe\\x10\\x00\\x00'", main_namespace_);
  ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
}

// testing default constructor of ByteString
TEST_F(ByteStringTest, DefaultByteString)
{
  bp::exec("result = helper.getDefaultBytes() == b''", main_namespace_);
  ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
}

// sending a tuple of ByteString from C++ to Python
TEST_F(ByteStringTest, CxxTupleToPy)
{
  bp::exec("result = helper.getTuple()[0] == b'abcdef'", main_namespace_);
  ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
}

// sending a tuple of bytes to C++
TEST_F(ByteStringTest, PyTupleToCxx)
{
  bp::exec("result = helper.compareTuple((b'mno',))", main_namespace_);
  ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
}

// sending a geometry_msgs::Vector3 from C++ to Python
TEST_F(ByteStringTest, CxxMsgToPy)
{
  try
  {
    bp::exec("tmp = msgs.Vector3()\n"
             "tmp.deserialize(helper.getVector())\n"
             "result = tmp == msgs.Vector3(1.0, -2.0, 0.25)\n",
             main_namespace_);
    ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
  }
  catch (const bp::error_already_set&)
  {
    failAddPythonErrorToGTest(__LINE__);
  }
}

// sending a geometry_msgs::Vector3 from Python to C++
TEST_F(ByteStringTest, PyMsgToCxx)
{
  try
  {
    bp::exec("tmp = StringIO()\n"
             "msgs.Vector3(x=1.0, y=-2.0, z=0.25).serialize(tmp)\n"
             "result = helper.compareVector(tmp.getvalue())\n",
             main_namespace_);
    ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
  }
  catch (const bp::error_already_set&)
  {
    failAddPythonErrorToGTest(__LINE__);
  }
}

// Test whether an unicode instance is rejected from C++ at deserializing (important for Python 3)
TEST_F(ByteStringTest, Unicode)
{
  try
  {
    bp::exec("result=False\n"
             "try:\n"
             "   helper.compareBytes(u'kdasd')\n"
             "except Exception as e:\n"
             "   if e.__class__.__name__ == 'ArgumentError' and str(e).startswith('Python argument types in'):\n"
             "      result=True\n"
             "   else:\n"
             "      raise\n",
             main_namespace_);
    ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
  }
  catch (const bp::error_already_set&)
  {
    failAddPythonErrorToGTest(__LINE__);
  }
}

// Test whether an unicode instance is rejected from C++ at deserializing in a tuple (important for Python 3)
// Unicode is convertible to Bytes in Python 2.7, so this testcase does not trigger for Python 2
#if PY_VERSION_HEX >= 0x03000000
TEST_F(ByteStringTest, UnicodeTuple)
#else
TEST_F(ByteStringTest, DISABLED_UnicodeTuple)
#endif
{
  try
  {
    bp::exec("result=False\n"
             "try:\n"
             "   helper.compareVectorTuple((u'kdasd',))\n"
             "except RuntimeError as e:\n"
             "   if str(e) == 'Underlying python object is not a Bytes/String instance':\n"
             "      result=True\n"
             "   else:\n"
             "       raise\n",
             main_namespace_);
    ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
  }
  catch (const bp::error_already_set&)
  {
    failAddPythonErrorToGTest(__LINE__);
  }
}

// Test whether an int instance is rejected from C++ at deserializing
TEST_F(ByteStringTest, Int)
{
  try
  {
    bp::exec("result=False\n"
             "try:\n"
             "   helper.compareBytes(4711)\n"
             "except Exception as e:\n"
             "   if e.__class__.__name__ == 'ArgumentError' and str(e).startswith('Python argument types in'):\n"
             "      result=True\n"
             "   else:\n"
             "      raise\n",
             main_namespace_);
    ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
  }
  catch (const bp::error_already_set&)
  {
    failAddPythonErrorToGTest(__LINE__);
  }
}

// Test whether an int instance is rejected from C++ at deserializing in a tuple
TEST_F(ByteStringTest, IntTuple)
{
  try
  {
    bp::exec("result=False\n"
             "try:\n"
             "   helper.compareVectorTuple((4711,))\n"
             "except RuntimeError as e:\n"
             "   if str(e) == 'Underlying python object is not a Bytes/String instance':\n"
             "      result=True\n"
             "   else:\n"
             "       raise\n",
             main_namespace_);
    ASSERT_TRUE(bp::extract<bool>(main_namespace_["result"]));
  }
  catch (const bp::error_already_set&)
  {
    failAddPythonErrorToGTest(__LINE__);
  }
}

int main(int argc, char** argv)
{
  ::Py_InitializeEx(0);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
