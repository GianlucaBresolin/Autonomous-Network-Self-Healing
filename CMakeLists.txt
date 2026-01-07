cmake_minimum_required(VERSION 3.10)
project(swarm_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Flooding module
add_library(flooding
    modules/flooding/flood.cpp
)

target_include_directories(flooding PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Fake radio (test utility)
add_library(fake_radio
    tests/fake_radio.cpp
)

target_include_directories(fake_radio PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Flooding app
add_executable(flooding_app
    apps/flooding/main.cpp
)

target_link_libraries(flooding_app
    flooding
    fake_radio
)

# Optional formatting helper
find_program(CLANG_FORMAT NAMES clang-format)

if(CLANG_FORMAT)
    file(GLOB_RECURSE FORMAT_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/apps/**/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/apps/**/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/**/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/**/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/interfaces/**/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/**/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/**/*.h
    )

    add_custom_target(format
        COMMAND ${CLANG_FORMAT} -i ${FORMAT_SOURCES}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Running clang-format on sources")
else()
    message(STATUS "clang-format not found: 'format' target disabled")
endif()
