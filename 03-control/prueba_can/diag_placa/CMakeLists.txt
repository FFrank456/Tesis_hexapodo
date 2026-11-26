cmake_minimum_required(VERSION 3.16)
# Diagnostico CAN de una placa sola. El mismo codigo compila para la MOD_DRV (esp32)
# y para la maestra (esp32s3): se elige con el target.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(diag_placa)
