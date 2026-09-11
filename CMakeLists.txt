cmake_minimum_required(VERSION 3.16)
project(rm_auto_aim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 强制启用警告
add_definitions(-Wall -Werror)

if(NOT DEFINED AUTO_AIM_PREVIEW_IMAGE)
set(AUTO_AIM_PREVIEW_IMAGE 1)
endif()

if(AUTO_AIM_PREVIEW_IMAGE)
    add_compile_definitions(AUTO_AIM_PREVIEW_IMAGE=1)
else()
    add_compile_definitions(AUTO_AIM_PREVIEW_IMAGE=0)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

## 导出编译命令以便使用 clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Keep generated XRobot entry headers in sync for both CLI builds and CMake Tools.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(XROBOT_MAIN_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/User/xrobot_main.hpp)
set(XROBOT_CONSTEXPR_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/User/xrobot_constexpr.hpp)
add_custom_command(
    OUTPUT ${XROBOT_MAIN_HEADER} ${XROBOT_CONSTEXPR_HEADER}
    COMMAND ${Python3_EXECUTABLE} -m xrobot.GenerateMain --output ${XROBOT_MAIN_HEADER}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/User/xrobot.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/Modules/modules.yaml
    COMMENT "Generating XRobot Webots entry headers"
    VERBATIM
)
add_custom_target(xrobot_generate_webots_main
    DEPENDS ${XROBOT_MAIN_HEADER} ${XROBOT_CONSTEXPR_HEADER}
)

# libxr
set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
set(WEBOTS_HOME /usr/local/webots CACHE PATH "Webots installation")
set(LIBXR_SYSTEM webots CACHE STRING "LibXR system")
set(LIBXR_DRIVER webots CACHE STRING "LibXR driver")
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
set(LIBXR_LOG_OUTPUT_LEVEL 3)
set(LIBXR_PRINT_INTEGER_ENABLE_64BIT ON CACHE BOOL "" FORCE)
set(LIBXR_PRINT_FLOAT_ENABLE_DOUBLE ON CACHE BOOL "" FORCE)
set(LIBXR_PRINT_ENABLE_POINTER ON CACHE BOOL "" FORCE)
add_subdirectory(libxr)
set_property(TARGET xr PROPERTY CXX_STANDARD 20)
set_property(TARGET xr PROPERTY CXX_STANDARD_REQUIRED ON)

add_executable(${PROJECT_NAME} User/main.cpp)
add_dependencies(${PROJECT_NAME} xrobot_generate_webots_main)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC xr
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC $<TARGET_PROPERTY:xr,INTERFACE_INCLUDE_DIRECTORIES>
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/User
)

option(AUTO_AIM_BUILD_ACCEPTANCE "Build the Webots BSP evidence subscriber" OFF)
if(AUTO_AIM_BUILD_ACCEPTANCE)
    add_executable(rm_auto_aim_acceptance User/main.cpp)
    add_dependencies(rm_auto_aim_acceptance xrobot_generate_webots_main)
    target_compile_definitions(rm_auto_aim_acceptance PRIVATE XR_WEBOTS_ACCEPTANCE=1)
    target_include_directories(rm_auto_aim_acceptance PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/User)
    target_link_libraries(rm_auto_aim_acceptance PRIVATE xr)
endif()
