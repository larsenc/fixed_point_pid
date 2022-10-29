cmake_minimum_required (VERSION 3.10)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)
include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

include(FetchContent)
FetchContent_Declare(
	matplotlib
	GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
)
FetchContent_MakeAvailable(matplotlib)
#FetchContent_GetProperties(matplotlib)
#if(NOT matplotlib_POPULATED)
#	FetchContent_Populate(matplotlib)
#endif()
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

include(FetchContent)
FetchContent_Declare(
	Eigen
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG 3.4.0
	GIT_SHALLOW TRUE
	GIT_PROGRESS TRUE
)
set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_LEAVE_TEST_IN_ALL_TARGET OFF)
FetchContent_MakeAvailable(Eigen)
MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}")
include_directories ( ${EIGEN3_INCLUDE_DIRS} )


add_executable(dc_motor_cascade_pid "dc_motor_cascade_pid.cpp")
target_compile_features(dc_motor_cascade_pid PUBLIC cxx_std_17)
target_link_libraries(dc_motor_cascade_pid PRIVATE ${PYTHON_LIBRARIES} Python3::NumPy fixed_point_pid Eigen3::Eigen)