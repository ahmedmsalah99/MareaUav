// generated from rosidl_generator_py/resource/_msg_pkg_typesupport_entry_point.c.em
// generated code does not contain a copyright notice

#include <Python.h>
#include <stdbool.h>
#include <stdint.h>

#include <rosidl_generator_c/message_type_support_struct.h>
#include <rosidl_generator_c/visibility_control.h>

#include <ultrasonic_sensor/msg/ultrasonic_msg__functions.h>
#include <ultrasonic_sensor/msg/ultrasonic_msg__struct.h>
#include <ultrasonic_sensor/msg/ultrasonic_msg__type_support.h>


static void * ultrasonic_sensor__msg__ultrasonic_msg__create_ros_message(void)
{
  return ultrasonic_sensor__msg__UltrasonicMsg__create();
}

static void ultrasonic_sensor__msg__ultrasonic_msg__destroy_ros_message(void * raw_ros_message)
{
  ultrasonic_sensor__msg__UltrasonicMsg * ros_message = (ultrasonic_sensor__msg__UltrasonicMsg *)raw_ros_message;
  ultrasonic_sensor__msg__UltrasonicMsg__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool ultrasonic_sensor__msg__ultrasonic_msg__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * ultrasonic_sensor__msg__ultrasonic_msg__convert_to_py(void * raw_ros_message);

static PyMethodDef ultrasonic_sensor__methods[] = {
  {NULL, NULL, 0, NULL}  /* sentinel */
};

static struct PyModuleDef ultrasonic_sensor__module = {
  PyModuleDef_HEAD_INIT,
  "_ultrasonic_sensor_support",
  "_ultrasonic_sensor_doc",
  -1,  /* -1 means that the module keeps state in global variables */
  ultrasonic_sensor__methods,
  NULL,
  NULL,
  NULL,
  NULL,
};

ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(ultrasonic_sensor, msg, UltrasonicMsg);

int8_t
_register_msg_type__msg__ultrasonic_msg(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&ultrasonic_sensor__msg__ultrasonic_msg__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__msg_ultrasonic_msg",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&ultrasonic_sensor__msg__ultrasonic_msg__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__msg_ultrasonic_msg",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&ultrasonic_sensor__msg__ultrasonic_msg__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__msg_ultrasonic_msg",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&ultrasonic_sensor__msg__ultrasonic_msg__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__msg_ultrasonic_msg",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(ultrasonic_sensor, msg, UltrasonicMsg),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__msg_ultrasonic_msg",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

PyMODINIT_FUNC
PyInit_ultrasonic_sensor_s__rosidl_typesupport_introspection_c(void)
{
  PyObject * pymodule = NULL;
  pymodule = PyModule_Create(&ultrasonic_sensor__module);
  if (!pymodule) {
    return NULL;
  }
  int8_t err;
  err = _register_msg_type__msg__ultrasonic_msg(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  return pymodule;
}
