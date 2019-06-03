// generated from rosidl_generator_py/resource/_msg_support.c.em
// generated code does not contain a copyright notice

#include <Python.h>
#include <stdbool.h>

#include <rosidl_generator_c/visibility_control.h>

#include <ultrasonic_sensor/msg/ultrasonic_msg__struct.h>
#include <ultrasonic_sensor/msg/ultrasonic_msg__functions.h>


ROSIDL_GENERATOR_C_EXPORT
bool ultrasonic_sensor__msg__ultrasonic_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
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
    assert(strncmp(
        "ultrasonic_sensor.msg._ultrasonic_msg.UltrasonicMsg",
        full_classname_dest, 51) == 0);
  }
  ultrasonic_sensor__msg__UltrasonicMsg * ros_message = _ros_message;
  {  // z_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ultrasonic_sensor__msg__ultrasonic_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of UltrasonicMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ultrasonic_sensor.msg._ultrasonic_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "UltrasonicMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ultrasonic_sensor__msg__UltrasonicMsg * ros_message = (ultrasonic_sensor__msg__UltrasonicMsg *)raw_ros_message;
  {  // z_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
