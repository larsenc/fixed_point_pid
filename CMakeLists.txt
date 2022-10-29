cmake_minimum_required(VERSION 3.11)

project(fixed_point_pid
	VERSION 0.1
	LANGUAGES CXX
	DESCRIPTION "A header-only c++ library that implements a configurable fixed point P, PI, PD and PID controller"
	HOMEPAGE_URL "https://github.com/larsenc/fixed_point_pid"
)

option(FIXED_POINT_PID_BUILD_EXAMPLES "Build fixed point pid examples" OFF)

include(GNUInstallDirs)

set(FIXED_POINT_BUILD_TESTS OFF)
set(FIXED_POINT_BUILD_EXAMPLES OFF)
include(FetchContent)
FetchContent_Declare(
  fixed_point
  GIT_REPOSITORY	https://github.com/larsenc/fixed_point
)
FetchContent_MakeAvailable(fixed_point)

add_library(fixed_point_pid INTERFACE)

target_include_directories(fixed_point_pid
	INTERFACE	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
				$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(fixed_point_pid INTERFACE fixed_point)

if(FIXED_POINT_PID_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()

install(TARGETS fixed_point
	EXPORT ${PROJECT_NAME}_Targets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"fixed_point_pidConfigVersion.cmake"
	VERSION ${CURRENT_VERSION}
	COMPATIBILITY SameMajorVersion
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
	"${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
	INSTALL_DESTINATION
	${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(EXPORT ${PROJECT_NAME}_Targets
	FILE ${PROJECT_NAME}Targets.cmake
	NAMESPACE ${PROJECT_NAME}::
	DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(FILES	"${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
				"${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
	DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)