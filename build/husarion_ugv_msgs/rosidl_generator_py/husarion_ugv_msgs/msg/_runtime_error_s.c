// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
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
#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.h"
#include "husarion_ugv_msgs/msg/detail/runtime_error__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool husarion_ugv_msgs__msg__runtime_error__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
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
    assert(strncmp("husarion_ugv_msgs.msg._runtime_error.RuntimeError", full_classname_dest, 49) == 0);
  }
  husarion_ugv_msgs__msg__RuntimeError * ros_message = _ros_message;
  {  // amps_limit_active
    PyObject * field = PyObject_GetAttrString(_pymsg, "amps_limit_active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->amps_limit_active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // motor_stall
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_stall");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->motor_stall = (Py_True == field);
    Py_DECREF(field);
  }
  {  // loop_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "loop_error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->loop_error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // safety_stop_active
    PyObject * field = PyObject_GetAttrString(_pymsg, "safety_stop_active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->safety_stop_active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // forward_limit_triggered
    PyObject * field = PyObject_GetAttrString(_pymsg, "forward_limit_triggered");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->forward_limit_triggered = (Py_True == field);
    Py_DECREF(field);
  }
  {  // reverse_limit_triggered
    PyObject * field = PyObject_GetAttrString(_pymsg, "reverse_limit_triggered");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->reverse_limit_triggered = (Py_True == field);
    Py_DECREF(field);
  }
  {  // amps_trigger_activated
    PyObject * field = PyObject_GetAttrString(_pymsg, "amps_trigger_activated");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->amps_trigger_activated = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * husarion_ugv_msgs__msg__runtime_error__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RuntimeError */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("husarion_ugv_msgs.msg._runtime_error");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RuntimeError");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  husarion_ugv_msgs__msg__RuntimeError * ros_message = (husarion_ugv_msgs__msg__RuntimeError *)raw_ros_message;
  {  // amps_limit_active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->amps_limit_active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "amps_limit_active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_stall
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->motor_stall ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_stall", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
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
  {  // safety_stop_active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->safety_stop_active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "safety_stop_active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // forward_limit_triggered
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->forward_limit_triggered ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "forward_limit_triggered", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reverse_limit_triggered
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->reverse_limit_triggered ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reverse_limit_triggered", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // amps_trigger_activated
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->amps_trigger_activated ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "amps_trigger_activated", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
