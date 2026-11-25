# A separate image uses the obstacle-extras controller and its original sweep.
set(OBSTACLE_EXTRAS_SRC ${PROJECT_SOURCE_DIR}/app_program/obstacle/extras)

add_library(obstacle_extras_runtime STATIC EXCLUDE_FROM_ALL
    ${OBSTACLE_EXTRAS_SRC}/comms.c
    ${OBSTACLE_EXTRAS_SRC}/drv_sonar.c
    ${OBSTACLE_EXTRAS_SRC}/extras_start.c
    ${OBSTACLE_EXTRAS_SRC}/flight_log.c
    ${OBSTACLE_EXTRAS_SRC}/planner.c
    ${OBSTACLE_EXTRAS_SRC}/scan.c
    ${OBSTACLE_EXTRAS_SRC}/scan_task.c
    ${OBSTACLE_EXTRAS_SRC}/service.c
    ${OBSTACLE_EXTRAS_SRC}/statechart.c)
target_link_libraries(obstacle_extras_runtime PUBLIC platform mtk3smp)
target_compile_options(obstacle_extras_runtime PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})

add_executable(obstacle_extras EXCLUDE_FROM_ALL
    app_program/bench/obstacle_extras_main.c
    app_program/ir/line/extras_line_start.c
    app_program/motion/obstacle_extras/navigation.c
    app_program/motion/obstacle_extras/setup.c
    app_program/motion/motion_start.c
    app_program/platform/platform.c
    app_program/platform/stack.c)
target_link_libraries(obstacle_extras PRIVATE
    obstacle_extras_runtime motion_service track platform mtk3smp)
add_executable(obstacle_extras_bench EXCLUDE_FROM_ALL
    app_program/bench/obstacle_extras_bench.c
    app_program/bench/obstacle_extras_bench_start.c
    app_program/platform/platform.c
    app_program/platform/stack.c)
target_link_libraries(obstacle_extras_bench PRIVATE
    obstacle_extras_runtime platform mtk3smp)

foreach(target IN ITEMS obstacle_extras obstacle_extras_bench)
    target_compile_options(${target} PRIVATE
        -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})
    target_link_options(${target} PRIVATE
        -nostartfiles
        -Wl,--gc-sections
        -Wl,--undefined=boot2
        -T "${MTK3SMP_PATH}/etc/linker/pico_rp2040/tkernel_map.ld")
    pico_add_extra_outputs(${target})
endforeach()
