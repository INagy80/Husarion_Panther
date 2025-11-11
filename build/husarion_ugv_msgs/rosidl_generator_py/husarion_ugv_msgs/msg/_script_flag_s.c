// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__struct.h"
#include "husarion_ugv_msgs/msg/detail/script_flag__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool husarion_ugv_msgs__msg__script_flag__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("husarion_ugv_msgs.msg._script_flag.ScriptFlag", full_classname_dest, 45) == 0);
  }
  husarion_ugv_msgs__msg__ScriptFlag * ros_message = _ros_message;
  {  // loop_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "loop_error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->loop_error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // encoder_disconnected
    PyObject * field = PyObject_GetAttrString(_pymsg, "encoder_disconnected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->encoder_disconnected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // amp_limiter
    PyObject * field = PyObject_GetAttrString(_pymsg, "amp_limiter");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->amp_limiter = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * husarion_ugv_msgs__msg__script_flag__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ScriptFlag */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("husarion_ugv_msgs.msg._script_flag");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ScriptFlag");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  husarion_ugv_msgs__msg__ScriptFlag * ros_message = (husarion_ugv_msgs__msg__ScriptFlag *)raw_ros_message;
  {  // loop_error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->loop_error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "loop_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // encoder_disconnected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->encoder_disconnected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "encoder_disconnected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // amp_limiter
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->amp_limiter ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "amp_limiter", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
