cmake_minimum_required(VERSION 3.0.2)
project(motion_control)

set(PROJECT_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
set(THIRD_PARTY ${PROJECT_SOURCE_DIR}/../third_party)
set(PROTO_DIR ${PROJECT_SOURCE_DIR}/../msg/generated)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(glog REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROTO_DIR})
include_directories(${THIRD_PARTY}/cppzmq-master)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${THIRD_PARTY}/osqp/include)
include_directories(${THIRD_PARTY}/osqp-eigen/include)
include_directories(${THIRD_PARTY}/yaml-cpp-master/include)


link_directories(${THIRD_PARTY}/libzmq-4.3.5/build/lib)
link_directories(${THIRD_PARTY}/osqp/lib)
link_directories(${THIRD_PARTY}/osqp-eigen/lib)
link_directories(${THIRD_PARTY}/yaml-cpp-master/lib)

file(GLOB PROTO_LIST_
  ${PROTO_DIR}/pnc_msgs/*.cc
  ${PROTO_DIR}/geometry_msgs/*.cc
  ${PROTO_DIR}/nav_msgs/*.cc
  ${PROTO_DIR}/std_msgs/*.cc
)

file(GLOB SRC_LIST_ 
  ${PROJECT_SOURCE_DIR}/src/*.cpp 
  ${PROJECT_SOURCE_DIR}/src/*.cc 
)

add_executable(motion_control 
    ${SRC_LIST_} 
    ${PROTO_LIST_}
)

add_executable(request_control 
    ${PROJECT_SOURCE_DIR}/src/test_code/request_control.cpp
    ${PROJECT_SOURCE_DIR}/src/zmq_subscriber.cpp
    ${PROJECT_SOURCE_DIR}/src/common.cpp
    ${PROTO_LIST_}
)

add_executable(test_zmq 
    ${PROJECT_SOURCE_DIR}/src/test_code/test_zmq_main.cpp
    ${PROJECT_SOURCE_DIR}/src/zmq_subscriber.cpp
    ${PROJECT_SOURCE_DIR}/src/zmq_publisher.cpp
    ${PROJECT_SOURCE_DIR}/src/common.cpp
    ${PROTO_LIST_}
)

set_target_properties(motion_control PROPERTIES
    BUILD_RPATH "${THIRD_PARTY}/osqp-eigen/lib;${THIRD_PARTY}/osqp/lib"
    INSTALL_RPATH "${THIRD_PARTY}/osqp-eigen/lib;${THIRD_PARTY}/osqp/lib"
    # 设置 RPATH 优先于 LD_LIBRARY_PATH
    LINK_FLAGS "-Wl,--enable-new-dtags"
)

target_link_libraries(motion_control 
  ${PROTOBUF_LIBRARIES}
  libzmq.so
  atomic
  libOsqpEigen.so 
  libosqp.so 
  libyaml-cpp.a 
  glog::glog
)

target_link_libraries(request_control 
  ${PROTOBUF_LIBRARIES}
  libzmq.so
)

target_link_libraries(test_zmq 
  ${PROTOBUF_LIBRARIES}
  libzmq.so
)

install(TARGETS
  motion_control
  DESTINATION ${PROJECT_INSTALL_PREFIX}/../../../install/${PROJECT_NAME}
)

install(TARGETS
  request_control
  DESTINATION ${PROJECT_INSTALL_PREFIX}/../../../install/${PROJECT_NAME}
)

install(TARGETS
  test_zmq
  DESTINATION ${PROJECT_INSTALL_PREFIX}/../../../install/${PROJECT_NAME}
)