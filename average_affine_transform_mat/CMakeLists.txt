cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME average_affine_transform_mat)
project( ${PROJECT_NAME} )

option(BUILD_EXAMPLE "Build example." ON)

include("compiler_options.cmake")

add_library(
    ${PROJECT_NAME} 
    INTERFACE
)

target_include_directories(
    ${PROJECT_NAME}
    INTERFACE
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if (BUILD_EXAMPLE)
    add_executable(${PROJECT_NAME}_example src/example.cpp)
    target_link_libraries(${PROJECT_NAME}_example PRIVATE ${PROJECT_NAME})
endif()
