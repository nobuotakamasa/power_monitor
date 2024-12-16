#!/bin/bash

# set name
PACKAGE_NAME="power_consumption_monitor"
EXECUTABLE_NAME="power_consumption_monitor"

# set workspace
WORKSPACE_DIRECTORY="."

# get executable path
EXECUTABLE_PATH="${WORKSPACE_DIRECTORY}/build/${PACKAGE_NAME}/${EXECUTABLE_NAME}"

# change root
sudo chown root:root ${EXECUTABLE_PATH}
sudo chmod u+s ${EXECUTABLE_PATH}

# symbolic link ros2 libraries
sudo ln -s /opt/ros/humble/lib/liblibstatistics_collector.so /usr/lib/liblibstatistics_collector.so
sudo ln -s /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_cpp.so /usr/lib/librcl_interfaces__rosidl_typesupport_cpp.so
sudo ln -s /opt/ros/humble/lib/librosgraph_msgs__rosidl_typesupport_cpp.so /usr/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
sudo ln -s /opt/ros/humble/lib/libstatistics_msgs__rosidl_typesupport_cpp.so /usr/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
sudo ln -s /opt/ros/humble/lib/librmw_implementation.so /usr/lib/librmw_implementation.so
sudo ln -s /opt/ros/humble/lib/libament_index_cpp.so /usr/lib/libament_index_cpp.so
sudo ln -s /opt/ros/humble/lib/librcl_logging_interface.so
sudo ln -s /opt/ros/humble/lib/librmw.so /usr/lib/librmw.so
sudo ln -s /opt/ros/humble/lib/librcpputils.so /usr/lib/librcpputils.so
sudo ln -s /opt/ros/humble/lib/librosidl_typesupport_cpp.so /usr/lib/librosidl_typesupport_cpp.so
sudo ln -s /opt/ros/humble/lib/librcl_logging_spdlog.so /usr/lib/librcl_logging_spdlog.so
sudo ln -s /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_c.so /usr/lib/librcl_interfaces__rosidl_typesupport_c.so
sudo ln -s /opt/ros/humble/lib/librcl_interfaces__rosidl_generator_c.so /usr/lib/librcl_interfaces__rosidl_generator_c.so
sudo ln -s /opt/ros/humble/lib/librosidl_runtime_c.so /usr/lib/librosidl_runtime_c.so
sudo ln -s /opt/ros/humble/lib/librosidl_typesupport_c.so /usr/lib/librosidl_typesupport_c.so
sudo ln -s /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_generator_c.so /usr/lib/libbuiltin_interfaces__rosidl_generator_c.so
sudo ln -s /opt/ros/humble/lib/librmw_cyclonedds_cpp.so /usr/lib/librmw_cyclonedds_cpp.so
sudo ln -s /opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so.0 /usr/lib/libddsc.so.0
sudo ln -s /opt/ros/humble/lib/librosidl_typesupport_introspection_cpp.so /usr/lib/librosidl_typesupport_introspection_cpp.so
sudo ln -s /opt/ros/humble/lib/librmw_dds_common.so /usr/lib/librmw_dds_common.so
sudo ln -s /opt/ros/humble/lib/librosidl_typesupport_introspection_c.so /usr/lib/librosidl_typesupport_introspection_c.so
sudo ln -s /opt/ros/humble/lib/librmw_dds_common__rosidl_typesupport_cpp.so /usr/lib/librmw_dds_common__rosidl_typesupport_cpp.so
sudo ln -s /opt/ros/humble/lib/x86_64-linux-gnu/libiceoryx_binding_c.so /usr/lib/libiceoryx_binding_c.so
sudo ln -s /opt/ros/humble/lib/x86_64-linux-gnu/libiceoryx_posh.so /usr/lib/libiceoryx_posh.so
sudo ln -s /opt/ros/humble/lib/x86_64-linux-gnu/libiceoryx_hoofs.so /usr/lib/libiceoryx_hoofs.so
sudo ln -s /opt/ros/humble/lib/x86_64-linux-gnu/libiceoryx_platform.so /usr/lib/libiceoryx_platform.so
sudo ln -s /opt/ros/humble/lib/librmw_dds_common__rosidl_typesupport_introspection_cpp.so /usr/lib/librmw_dds_common__rosidl_typesupport_introspection_cpp.so
sudo ln -s /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so /usr/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
sudo ln -s /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so /usr/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
sudo ln -s /opt/ros/humble/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so /usr/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
sudo ln -s /opt/ros/humble/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so /usr/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
sudo ln -s /opt/ros/humble/lib/libdiagnostic_msgs__rosidl_typesupport_introspection_cpp.so /usr/lib/libdiagnostic_msgs__rosidl_typesupport_introspection_cpp.so
sudo ln -s /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so /usr/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so