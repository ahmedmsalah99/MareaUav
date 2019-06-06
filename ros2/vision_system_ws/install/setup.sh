# generated from colcon_core/shell/template/prefix_chain.sh.em

# This script extends the environment with the environment of other prefix
# paths which were sourced when this file was generated as well as all packages
# contained in this prefix path.

# since a plain shell script can't determine its own path when being sourced
# either use the provided COLCON_CURRENT_PREFIX
# or fall back to the build time prefix (if it exists)
_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=/home/pi/mar_frankenrotor_ros/ros2/vision_system_ws/install
if [ ! -z "$COLCON_CURRENT_PREFIX" ]; then
  _colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX="$COLCON_CURRENT_PREFIX"
elif [ ! -d "$_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX" ]; then
  echo "The build time path \"$_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX\" doesn't exist. Either source a script for a different shell or set the environment variable \"COLCON_CURRENT_PREFIX\" explicitly." 1>&2
  unset _colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX
  return 1
fi

# function to source another script with conditional trace output
# first argument: the path of the script
_colcon_prefix_chain_sh_source_script() {
  if [ -f "$1" ]; then
    if [ -n "$COLCON_TRACE" ]; then
      echo ". \"$1\""
    fi
    . "$1"
  else
    echo "not found: \"$1\"" 1>&2
  fi
}

# source chained prefixes
# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_clang_format"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_copyright"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cppcheck"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cpplint"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_flake8"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_index_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_index_python"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_lint_cmake"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_package"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_core"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_export_definitions"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_export_include_directories"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_export_libraries"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_export_interfaces"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_export_link_flags"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_include_directories"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_libraries"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_export_dependencies"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_python"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_target_dependencies"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_test"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_auto"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_clang_format"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_copyright"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_cppcheck"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_cpplint"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_flake8"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_lint_cmake"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_nose"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_pytest"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_ros"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_lint_auto"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_pclint"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_pclint"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_pep257"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_pep257"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_pep8"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_pep8"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_pyflakes"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_pyflakes"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_xmllint"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_xmllint"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/connext_cmake_module"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/console_bridge_vendor"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/fastrtps_cmake_module"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_gtest"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_gmock"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/libcurl_vendor"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/libyaml_vendor"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/message_filters"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/opensplice_cmake_module"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/osrf_pycommon"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/launch"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/launch_testing"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/class_loader"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/python_cmake_module"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rcl_logging_noop"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rcl_yaml_param_parser"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rcutils"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/resource_retriever"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_connext_shared_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_implementation_cmake"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros_environment"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_adapter"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_parser"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_actions"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_cmake"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_generator_dds_idl"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_interface"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_generator_c"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_fastrtps_shared_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_generator_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_connext_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_connext_c"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_connext_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_fastrtps_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_fastrtps_c"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_fastrtps_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_introspection_c"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_introspection_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_fastrtps_dynamic_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_opensplice_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_opensplice_c"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_opensplice_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rmw_implementation"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_c"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_generator_py"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_typesupport_cpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_default_generators"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosidl_default_runtime"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/builtin_interfaces"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/lifecycle_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rcl_interfaces"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rcl"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rcl_lifecycle"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rclpy"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/launch_ros"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2cli"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2msg"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2multicast"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2node"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2param"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2pkg"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2launch"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2run"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2srv"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2topic"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2service"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ros2lifecycle"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rosgraph_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rclcpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rclcpp_lifecycle"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/sros2"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/std_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/actionlib_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/demo_nodes_cpp_native"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/geometry_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/diagnostic_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/nav_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/dummy_map_server"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/sensor_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/dummy_sensors"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/dummy_robot_bringup"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/image_tools"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/shape_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/std_srvs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/stereo_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/test_cli"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/test_cli_remapping"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/test_communication"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/test_launch_ros"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/test_rclcpp"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/test_security"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/tf2"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/tf2_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/tf2_ros"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/tf2_eigen"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/tf2_sensor_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/pluginlib"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/tlsf"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/trajectory_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/uncrustify_vendor"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_uncrustify"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_cmake_uncrustify"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/ament_lint_common"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/unique_identifier_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/action_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rcl_action"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/rclcpp_action"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/test_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/urdf"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/visualization_msgs"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/common_interfaces"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install/yaml_cpp_vendor"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="/home/pi/ros2_ws/install"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"


# source this prefix
# setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
COLCON_CURRENT_PREFIX="$_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX"
_colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"

unset _colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX
unset _colcon_prefix_chain_sh_source_script
unset COLCON_CURRENT_PREFIX
