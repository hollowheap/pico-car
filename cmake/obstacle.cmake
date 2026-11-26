# Original obstacle controller used by main and the scanner bench.
set(OBSTACLE_EXTRAS_SRC ${PROJECT_SOURCE_DIR}/app_program/obstacle/extras)

add_library(obstacle_extras_runtime STATIC
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

# Shared consumers see the extras scanner through the public obstacle API.
add_library(obstacle STATIC
    app_program/obstacle/geometry.c
    app_program/obstacle/scan_algo.c
    ${OBSTACLE_EXTRAS_SRC}/adapter.c)
target_link_libraries(obstacle PUBLIC obstacle_extras_runtime)
target_compile_options(obstacle PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})
target_link_libraries(comms PRIVATE obstacle)
