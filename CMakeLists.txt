cmake_minimum_required(VERSION 3.15)
project(BI_OBJECTIVE)


set(CMAKE_CXX_STANDARD 11)

project(BI_OBJECTIVE LANGUAGES CXX)

set(RESOURCE_DIR "/home/luke/VRPInstance")
message(STATUS "Data dir: ${RESOURCE_DIR}")


include_directories(${RESOURCE_DIR})
#include_directories(/home/luke/VRPInstance)

message(STATUS "start running cmake...")
set(CMAKE_CXX_FLAGS -pthread)


find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
find_package(GSL)

message(STATUS "gsl_INCLUDE_DIRS: ${GSL_INCLUDE_DIR}")
message(STATUS "gsl_LIBRARIES: ${GSL_LIBRARIES}")
message(STATUS "gsl_VERSION: ${GSL_VERSION}")

message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIR}")
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
message(STATUS "Boost_VERSION: ${Boost_VERSION}")


#add_definitions(-D DEBUG)

# disable warnings
add_compile_options(-w)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${GSL_INCLUDE_DIR})

configure_file(./config.json ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

aux_source_directory(. SOURCE)

add_executable(BI_OBJECTIVE ${SOURCE})

target_link_libraries(BI_OBJECTIVE gsl gslcblas ${Boost_LIBRARIES})


# create symbolic link to resources
set (DESTINATION_DIR "${CMAKE_CURRENT_BINARY_DIR}/VRPInstance")

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink ${RESOURCE_DIR} ${DESTINATION_DIR}
        DEPENDS ${DESTINATION_DIR}
        COMMENT "symbolic link resources folder from ${RESOURCE_DIR} => ${DESTINATION_DIR}"
)