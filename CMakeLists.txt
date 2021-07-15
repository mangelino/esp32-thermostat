# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS /Users/massi/Projects/esp32/esp-idf-lib/components components/coreJSON/source)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gpio)

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/cert.pem" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/priv.key" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/AmazonRootCA1.pem" TEXT)
