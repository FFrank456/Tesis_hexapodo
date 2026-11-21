cmake_minimum_required(VERSION 3.16)
# El codigo de la prueba es comun a las dos placas y vive en ../comun
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../comun)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(can_maestra_s3)
