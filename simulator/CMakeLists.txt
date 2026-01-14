cmake_minimum_required(VERSION 3.16)
project(swarm_simulator LANGUAGES CXX)

# ns-3 (3.46+) headers use C++20 library features (e.g. remove_cvref_t).
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(swarm_sim
	src/main.cpp
	src/comm/RadioEnvironment.cpp
	src/core/Drone.cpp
	src/core/BaseStation.cpp
)

target_include_directories(swarm_sim PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Prefer ns-3 CMake package config (works with modern ns-3 installs).
find_package(ns3 CONFIG QUIET)

if(ns3_FOUND)
	message(STATUS "Found ns-3 via CMake package")
	target_link_libraries(swarm_sim PRIVATE
		ns3::core
		ns3::network
		ns3::internet
		ns3::wifi
		ns3::mobility
		ns3::propagation
	)
else()
	message(STATUS "ns-3 CMake package not found; trying pkg-config")
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(NS3DEV QUIET IMPORTED_TARGET ns3-dev)
	if(NS3DEV_FOUND)
		target_link_libraries(swarm_sim PRIVATE PkgConfig::NS3DEV)
	else()
		pkg_check_modules(NS3 REQUIRED IMPORTED_TARGET ns3)
		target_link_libraries(swarm_sim PRIVATE PkgConfig::NS3)
	endif()
endif()

