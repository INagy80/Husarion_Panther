// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
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
#include "husarion_ugv_msgs/msg/detail/driver_state__struct.h"
#include "husarion_ugv_msgs/msg/detail/driver_state__functions.h"

bool husarion_ugv_msgs__msg__fault_flag__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * husarion_ugv_msgs__msg__fault_flag__convert_to_py(void * raw_ros_message);
bool husarion_ugv_msgs__msg__script_flag__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * husarion_ugv_msgs__msg__script_flag__convert_to_py(void * raw_ros_message);
bool husarion_ugv_msgs__msg__runtime_error__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * husarion_ugv_msgs__msg__runtime_error__convert_to_py(void * raw_ros_message);
bool husarion_ugv_msgs__msg__runtime_error__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * husarion_ugv_msgs__msg__runtime_error__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool husarion_ugv_msgs__msg__driver_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("husarion_ugv_msgs.msg._driver_state.DriverState", full_classname_dest, 47) == 0);
  }
  husarion_ugv_msgs__msg__DriverState * ros_message = _ros_message;
  {  // voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current
    PyObject * field = PyObject_GetAttrString(_pymsg, "current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // heatsink_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "heatsink_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heatsink_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fault_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "fault_flag");
    if (!field) {
      return false;
    }
    if (!husarion_ugv_msgs__msg__fault_flag__convert_from_py(field, &ros_message->fault_flag)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // script_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "script_flag");
    if (!field) {
      return false;
    }
    if (!husarion_ugv_msgs__msg__script_flag__convert_from_py(field, &ros_message->script_flag)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // channel_1_motor_runtime_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_1_motor_runtime_error");
    if (!field) {
      return false;
    }
    if (!husarion_ugv_msgs__msg__runtime_error__convert_from_py(field, &ros_message->channel_1_motor_runtime_error)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // channel_2_motor_runtime_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_2_motor_runtime_error");
    if (!field) {
      return false;
    }
    if (!husarion_ugv_msgs__msg__runtime_error__convert_from_py(field, &ros_message->channel_2_motor_runtime_error)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // motor_states_data_timed_out
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_states_data_timed_out");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->motor_states_data_timed_out = (Py_True == field);
    Py_DECREF(field);
  }
  {  // driver_state_data_timed_out
    PyObject * field = PyObject_GetAttrString(_pymsg, "driver_state_data_timed_out");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->driver_state_data_timed_out = (Py_True == field);
    Py_DECREF(field);
  }
  {  // can_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "can_error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->can_error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // heartbeat_timeout
    PyObject * field = PyObject_GetAttrString(_pymsg, "heartbeat_timeout");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->heartbeat_timeout = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * husarion_ugv_msgs__msg__driver_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of DriverState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("husarion_ugv_msgs.msg._driver_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "DriverState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  husarion_ugv_msgs__msg__DriverState * ros_message = (husarion_ugv_msgs__msg__DriverState *)raw_ros_message;
  {  // voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heatsink_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heatsink_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heatsink_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fault_flag
    PyObject * field = NULL;
    field = husarion_ugv_msgs__msg__fault_flag__convert_to_py(&ros_message->fault_flag);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "fault_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // script_flag
    PyObject * field = NULL;
    field = husarion_ugv_msgs__msg__script_flag__convert_to_py(&ros_message->script_flag);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "script_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_1_motor_runtime_error
    PyObject * field = NULL;
    field = husarion_ugv_msgs__msg__runtime_error__convert_to_py(&ros_message->channel_1_motor_runtime_error);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_1_motor_runtime_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_2_motor_runtime_error
    PyObject * field = NULL;
    field = husarion_ugv_msgs__msg__runtime_error__convert_to_py(&ros_message->channel_2_motor_runtime_error);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_2_motor_runtime_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_states_data_timed_out
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->motor_states_data_timed_out ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_states_data_timed_out", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // driver_state_data_timed_out
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->driver_state_data_timed_out ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "driver_state_data_timed_out", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // can_error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->can_error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "can_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heartbeat_timeout
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->heartbeat_timeout ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heartbeat_timeout", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
