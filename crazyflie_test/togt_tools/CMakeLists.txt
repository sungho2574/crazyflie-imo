# TOGT-Planner -> 크플 궤적 변환기 (오프라인 전용, colcon 빌드에 넣지 않는다)
#
#   cmake -S . -B build && cmake --build build -j
#   ./build/togt_plan <params_dir> <setups.yaml> <track.yaml> <out.csv>
#
# TOGT-Planner 소스는 이 폴더의 **git 서브모듈** TOGT-Planner/ 에 있다(기본값).
#   git submodule update --init --recursive   # 처음 한 번
# 다른 경로를 쓰려면 -DTOGT_DIR=/path 또는 환경변수 TOGT_DIR.
# upstream 은 cmake 3.25 를 요구하지만, 소스는 3.16 으로도 빌드된다(검증됨).
cmake_minimum_required(VERSION 3.16)
project(togt_plan CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 우선순위: -DTOGT_DIR > 환경변수 TOGT_DIR > 서브모듈(TOGT-Planner/)
if(NOT DEFINED TOGT_DIR)
  set(TOGT_DIR "$ENV{TOGT_DIR}")
endif()
if(TOGT_DIR STREQUAL "")
  set(TOGT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/TOGT-Planner")
endif()
if(NOT EXISTS "${TOGT_DIR}/include/drolib")
  message(FATAL_ERROR
    "TOGT-Planner 소스를 못 찾았습니다(${TOGT_DIR}). 서브모듈을 받으세요:\n"
    "  git submodule update --init --recursive\n"
    "또는 -DTOGT_DIR=/path/to/TOGT-Planner 로 지정하세요.")
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
