# Copyright 2024 Boluwatife Olabiran
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(autoware_raptor_dbw_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(RAPTOR_DBW_INTERFACE_LIB_SRC
  src/raptor_dbw_interface.cpp
)

set(RAPTOR_DBW_INTERFACE_LIB_HEADERS
  include/autoware_raptor_dbw_interface/autoware_raptor_dbw_interface.hpp
  include/autoware_raptor_dbw_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${RAPTOR_DBW_INTERFACE_LIB_SRC}
  ${RAPTOR_DBW_INTERFACE_LIB_HEADERS}
)

set(AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_SRC
  src/autoware_raptor_dbw_interface_node.cpp
)

set(AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HEADERS
  include/autoware_raptor_dbw_interface/autoware_raptor_dbw_interface_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_SRC}
  ${AUTOWARE_RAPTOR_DBW_INTERFACE_NODE_HEADERS}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast)

set(NODE_NAME ${PROJECT_NAME}_node_exe)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware_raptor_dbw_interface::RaptorDBWInterfaceNode"
  EXECUTABLE ${NODE_NAME}
)

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
