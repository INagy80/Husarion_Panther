// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from husarion_ugv_msgs:msg/IOState.idl
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
#include "husarion_ugv_msgs/msg/detail/io_state__struct.h"
#include "husarion_ugv_msgs/msg/detail/io_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool husarion_ugv_msgs__msg__io_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("husarion_ugv_msgs.msg._io_state.IOState", full_classname_dest, 39) == 0);
  }
  husarion_ugv_msgs__msg__IOState * ros_message = _ros_message;
  {  // aux_power
    PyObject * field = PyObject_GetAttrString(_pymsg, "aux_power");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->aux_power = (Py_True == field);
    Py_DECREF(field);
  }
  {  // charger_connected
    PyObject * field = PyObject_GetAttrString(_pymsg, "charger_connected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->charger_connected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // charger_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "charger_enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->charger_enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // digital_power
    PyObject * field = PyObject_GetAttrString(_pymsg, "digital_power");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->digital_power = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fan
    PyObject * field = PyObject_GetAttrString(_pymsg, "fan");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fan = (Py_True == field);
    Py_DECREF(field);
  }
  {  // led_control
    PyObject * field = PyObject_GetAttrString(_pymsg, "led_control");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->led_control = (Py_True == field);
    Py_DECREF(field);
  }
  {  // motor_on
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_on");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->motor_on = (Py_True == field);
    Py_DECREF(field);
  }
  {  // power_button
    PyObject * field = PyObject_GetAttrString(_pymsg, "power_button");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->power_button = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * husarion_ugv_msgs__msg__io_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IOState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("husarion_ugv_msgs.msg._io_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IOState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  husarion_ugv_msgs__msg__IOState * ros_message = (husarion_ugv_msgs__msg__IOState *)raw_ros_message;
  {  // aux_power
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->aux_power ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aux_power", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // charger_connected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->charger_connected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "charger_connected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // charger_enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->charger_enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "charger_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // digital_power
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->digital_power ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "digital_power", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fan
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fan ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fan", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // led_control
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->led_control ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "led_control", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_on
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->motor_on ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_on", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // power_button
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->power_button ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "power_button", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
