// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
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
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__struct.h"
#include "husarion_ugv_msgs/msg/detail/robot_driver_state__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "husarion_ugv_msgs/msg/detail/driver_state_named__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool husarion_ugv_msgs__msg__driver_state_named__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * husarion_ugv_msgs__msg__driver_state_named__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool husarion_ugv_msgs__msg__robot_driver_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[59];
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
    assert(strncmp("husarion_ugv_msgs.msg._robot_driver_state.RobotDriverState", full_classname_dest, 58) == 0);
  }
  husarion_ugv_msgs__msg__RobotDriverState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // driver_states
    PyObject * field = PyObject_GetAttrString(_pymsg, "driver_states");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'driver_states'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!husarion_ugv_msgs__msg__DriverStateNamed__Sequence__init(&(ros_message->driver_states), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create husarion_ugv_msgs__msg__DriverStateNamed__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    husarion_ugv_msgs__msg__DriverStateNamed * dest = ros_message->driver_states.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!husarion_ugv_msgs__msg__driver_state_named__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // error
    PyObject * field = PyObject_GetAttrString(_pymsg, "error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // write_pdo_cmds_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "write_pdo_cmds_error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->write_pdo_cmds_error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // read_pdo_motor_states_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "read_pdo_motor_states_error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->read_pdo_motor_states_error = (Py_True == field);
    Py_DECREF(field);
  }
  {  // read_pdo_driver_state_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "read_pdo_driver_state_error");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->read_pdo_driver_state_error = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * husarion_ugv_msgs__msg__robot_driver_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobotDriverState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("husarion_ugv_msgs.msg._robot_driver_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobotDriverState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  husarion_ugv_msgs__msg__RobotDriverState * ros_message = (husarion_ugv_msgs__msg__RobotDriverState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // driver_states
    PyObject * field = NULL;
    size_t size = ros_message->driver_states.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    husarion_ugv_msgs__msg__DriverStateNamed * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->driver_states.data[i]);
      PyObject * pyitem = husarion_ugv_msgs__msg__driver_state_named__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "driver_states", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // write_pdo_cmds_error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->write_pdo_cmds_error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "write_pdo_cmds_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // read_pdo_motor_states_error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->read_pdo_motor_states_error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "read_pdo_motor_states_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // read_pdo_driver_state_error
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->read_pdo_driver_state_error ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "read_pdo_driver_state_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
