# TOGT-Planner -> 크플 궤적 변환기 (오프라인 전용, colcon 빌드에 넣지 않는다)
#
#   cmake -S . -B build -DTOGT_DIR=/path/to/TOGT-Planner
#   cmake --build build -j
#   ./build/togt_plan <params_dir> <setups.yaml> <track.yaml> <out.csv>
#
# TOGT-Planner 는 별도로 clone 해 둔다(서브모듈로 넣지 않음 — 오프라인 도구 결정).
# upstream 은 cmake 3.25 를 요구하지만, 소스는 3.16 으로도 빌드된다(검증됨).
cmake_minimum_required(VERSION 3.16)
project(togt_plan CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT DEFINED TOGT_DIR)
  set(TOGT_DIR "$ENV{TOGT_DIR}")
endif()
if(TOGT_DIR STREQUAL "" OR NOT EXISTS "${TOGT_DIR}/include/drolib")
  message(FATAL_ERROR
    "TOGT_DIR 이 TOGT-Planner 소스를 가리켜야 합니다. "
    "-DTOGT_DIR=/path/to/TOGT-Planner 또는 환경변수 TOGT_DIR 로 지정하세요.")
endif()

find_package(Eigen3 3.3 REQUIRED)

# TOGT 는 rapidjson 을 minco_snap_trajectory.cpp 에서만 쓴다. 없으면 받아온다.
# rapidjson 자체 테스트/예제는 최신 컴파일러에서 -Werror 로 깨지므로 끈다.
set(RAPIDJSON_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(RAPIDJSON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RAPIDJSON_BUILD_DOC OFF CACHE BOOL "" FORCE)
include(FetchContent)
FetchContent_Declare(
  RapidJSON
  GIT_REPOSITORY "https://github.com/Tencent/rapidjson.git"
  GIT_TAG "master"
  GIT_SHALLOW TRUE
  SYSTEM FIND_PACKAGE_ARGS NAMES RapidJSON CONFIG 1.1.0)
FetchContent_MakeAvailable(RapidJSON)

file(GLOB_RECURSE TOGT_SRC "${TOGT_DIR}/src/*.cpp")
add_library(drolib STATIC ${TOGT_SRC})
target_include_directories(drolib PUBLIC
  "${TOGT_DIR}/include"
  "${rapidjson_SOURCE_DIR}/include")
target_link_libraries(drolib PUBLIC Eigen3::Eigen)
target_compile_features(drolib PUBLIC cxx_std_17)
target_compile_definitions(drolib PUBLIC EIGEN_STACK_ALLOCATION_LIMIT=1048576)

add_executable(togt_plan togt_plan.cpp)
target_link_libraries(togt_plan PRIVATE drolib)
