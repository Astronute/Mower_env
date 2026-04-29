cmake_minimum_required(VERSION 3.0.2)
project(global_planner)

set(PROJECT_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
set(THIRD_PARTY ${PROJECT_SOURCE_DIR}/../third_party)
set(PROTO_DIR ${PROJECT_SOURCE_DIR}/../msg/generated)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(nlohmann_json 3.12.0 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROTO_DIR})
include_directories(${THIRD_PARTY}/cppzmq-master)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${THIRD_PARTY}/osqp/include)
include_directories(${THIRD_PARTY}/osqp-eigen/include)
include_directories(${THIRD_PARTY}/yaml-cpp-master/include)
include_directories(${THIRD_PARTY}/geometry_utils/geometry_utils/inc)


link_directories(${THIRD_PARTY}/libzmq-4.3.5/build/lib)
link_directories(${THIRD_PARTY}/osqp/lib)
link_directories(${THIRD_PARTY}/osqp-eigen/lib)
link_directories(${THIRD_PARTY}/yaml-cpp-master/lib)
link_directories(${THIRD_PARTY}/geometry_utils/build/lib)

file(GLOB SRC_LIST_ 
  ${PROJECT_SOURCE_DIR}/src/*.cpp 
  ${PROJECT_SOURCE_DIR}/src/*.cc 
  ${PROTO_DIR}/pnc_msgs/*.cc
  ${PROTO_DIR}/geometry_msgs/*.cc
  ${PROTO_DIR}/nav_msgs/*.cc
  ${PROTO_DIR}/std_msgs/*.cc
)

add_executable(global_planner 
    ${SRC_LIST_} 
)

set_target_properties(global_planner PROPERTIES
    BUILD_RPATH "${THIRD_PARTY}/osqp-eigen/lib;${THIRD_PARTY}/osqp/lib;${THIRD_PARTY}/geometry_utils/build/lib"
    INSTALL_RPATH "${THIRD_PARTY}/osqp-eigen/lib;${THIRD_PARTY}/osqp/lib;${THIRD_PARTY}/geometry_utils/build/lib"
    # 强制使用 RPATH
    LINK_FLAGS "-Wl,--disable-new-dtags"
)

target_link_libraries(global_planner 
  ${PROTOBUF_LIBRARIES}
  ${OpenCV_LIBS} 
  CGAL::CGAL 
  libzmq.so
  atomic
  libOsqpEigen.so 
  libosqp.so 
  libgeometry_utils.so 
  libyaml-cpp.a 
  nlohmann_json::nlohmann_json
)

install(TARGETS
  global_planner
  DESTINATION ${PROJECT_INSTALL_PREFIX}/../../../install/${PROJECT_NAME}
)