# WebAssembly build of the firmware UI for the webapp preview.
#
#   emcmake cmake -S sim -B sim/build -DCMAKE_BUILD_TYPE=Release
#   cmake --build sim/build -j
#
# Produces sim/build/dd_sim.js + dd_sim.wasm (ES module).
cmake_minimum_required(VERSION 3.16)
project(dd_sim C)

set(FW ${CMAKE_CURRENT_SOURCE_DIR}/..)

file(GLOB_RECURSE LVGL_SOURCES ${FW}/lvgl/src/*.c)
file(GLOB UI_SOURCES ${FW}/ui/*.c ${FW}/ui/themes/*.c ${FW}/ui/images/*.c)
file(GLOB PID_SOURCES ${FW}/lib/lib_pid/src/*.c)

add_executable(dd_sim
    ${LVGL_SOURCES}
    ${UI_SOURCES}
    ${PID_SOURCES}
    ${FW}/lib/lib_digitaldash_config/src/ke_config.c
    ${FW}/Core/Src/cJSON.c
    src/sim_main.c
    src/sim_stream.c
)

# Order matters: sim/inc shadows lv_conf.h and sim/inc/stub shadows
# lib_digital_dash.h and the STM32 CMSIS header.
target_include_directories(dd_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/stub
    ${FW}
    ${FW}/lvgl
    ${FW}/ui
    ${FW}/Core/Inc
    ${FW}/lib/lib_pid/src
    ${FW}/lib/lib_digitaldash_config/inc
    ${FW}/lib/lib_obdii/inc
    ${FW}/lib/lib_can_bus_sniffer/inc
)

target_compile_definitions(dd_sim PRIVATE LV_CONF_INCLUDE_SIMPLE)
target_compile_options(dd_sim PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/inc/sim_flash.h
    -Wno-unused-parameter
)

target_link_options(dd_sim PRIVATE
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createDigitalDashSim
    -sENVIRONMENT=web
    -sINITIAL_MEMORY=33554432
    -sALLOW_MEMORY_GROWTH=1
    -sEXPORTED_RUNTIME_METHODS=ccall,cwrap,HEAPU8,stringToNewUTF8
    -sEXPORTED_FUNCTIONS=_malloc,_free
)
