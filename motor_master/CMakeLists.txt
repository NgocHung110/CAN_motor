# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../components/motor_driver
                        ${CMAKE_CURRENT_LIST_DIR}/../components/encoder_driver
                        ${CMAKE_CURRENT_LIST_DIR}/../components/ssd1306
                        ${CMAKE_CURRENT_LIST_DIR}/../components/can_driver
                         )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(control_motor)
