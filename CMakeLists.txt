cmake_minimum_required(VERSION 3.24)

set(RP_PROFILE "" CACHE STRING "Conan profile used for host")

if(NOT RP_PROFILE)
    message(FATAL_ERROR "Please set RP_PROFILE in the command line, e.g. -DRP_PROFILE=stm32/stm32-h7")
endif()

find_program(CONAN_EXECUTABLE NAMES conan REQUIRED)
set(CONAN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/conan")

set(CONAN_INSTALL_ARGS
    install "${CMAKE_CURRENT_SOURCE_DIR}"
    --output-folder "${CONAN_OUTPUT_DIR}"
    --profile:host "${RP_PROFILE}"
    --build=missing
)

execute_process(
    COMMAND "${CONAN_EXECUTABLE}" ${CONAN_INSTALL_ARGS}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE CONAN_RESULT
    OUTPUT_VARIABLE CONAN_OUTPUT
    ERROR_VARIABLE CONAN_ERROR
)

# 关键： 将conan生成的文件夹添加到CMAKE_PREFIX_PATH中，这样find_package才能找到conan安装的包
list(PREPEND CMAKE_PREFIX_PATH "${CONAN_OUTPUT_DIR}/build/RelWithDebInfo/generators")

if(NOT CONAN_RESULT EQUAL 0)
    message(FATAL_ERROR "conan install failed:\n${CONAN_ERROR}")
endif()

if(NOT BOARD)
    set(BOARD damiao_mc02)
    message("BOARD not defined, use default value: ${BOARD}")
else()
    message("Use BOARD: ${BOARD}")
endif()

# zephyr通过kconfig指定CMAKE_BUILD_TYPE，
# 但是conan在构建之处就需要指定，这里显示设置CMAKE_BUILD_TYPE为RelWithDebInfo
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
find_package(rp-midware REQUIRED)

target_link_libraries(app PRIVATE rp-midware::rp-midware)

# define cmake project
project(breeze-app-template)

target_sources(app PRIVATE
    ./src/main.cpp
)
