cmake_minimum_required(VERSION 3.0.2)
project(gpsdr_fusion)

set(PROJECT_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
set(THIRD_PARTY ${PROJECT_SOURCE_DIR}/../third_party)
set(PROTO_DIR ${PROJECT_SOURCE_DIR}/../msg/generated)

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROTO_DIR})
include_directories(${THIRD_PARTY}/cppzmq-master)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${THIRD_PARTY}/yaml-cpp-master/include)


link_directories(${THIRD_PARTY}/libzmq-4.3.5/build/lib)
link_directories(${THIRD_PARTY}/yaml-cpp-master/lib)

file(GLOB SRC_LIST_ 
  ${PROJECT_SOURCE_DIR}/src/*.cpp 
  ${PROJECT_SOURCE_DIR}/src/*.cc 
  ${PROTO_DIR}/pnc_msgs/*.cc
  ${PROTO_DIR}/geometry_msgs/*.cc
  ${PROTO_DIR}/nav_msgs/*.cc
  ${PROTO_DIR}/std_msgs/*.cc
)

add_executable(gpsdr_fusion 
    ${SRC_LIST_} 
)

target_link_libraries(gpsdr_fusion 
  ${PROTOBUF_LIBRARIES}
  libzmq.so
  atomic
  libyaml-cpp.a
)

install(TARGETS
  gpsdr_fusion
  DESTINATION ${PROJECT_INSTALL_PREFIX}/../../../install/${PROJECT_NAME}
)