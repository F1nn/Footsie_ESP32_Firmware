cmake_minimum_required(VERSION 3.16.0)

set(IDF_TARGET "esp32s3")
set(EXTRA_COMPONENT_DIRS "src")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Footsie)
