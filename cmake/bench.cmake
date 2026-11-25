# Measurement benches (build/car_bench.uf2, build/wdt_check.uf2). They are not
# part of main.uf2.
add_executable(car_bench app_program/bench/car_bench.c app_program/bench/trace.c)
target_include_directories(car_bench PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(car_bench PRIVATE ir motion_service platform supervisor mtk3smp)
if(PICO_CAR_GIT_HASH_RESULT EQUAL 0)
    set(CAR_BENCH_BUILD "${PICO_CAR_GIT_HASH}")
    if(PICO_CAR_BUILD_DIRTY)
        string(APPEND CAR_BENCH_BUILD "+dirty")
    endif()
    target_compile_definitions(car_bench PRIVATE PICO_CAR_BENCH_BUILD="${CAR_BENCH_BUILD}")
endif()
add_executable(wdt_check app_program/bench/wdt_check.c)
target_include_directories(wdt_check PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(wdt_check PRIVATE platform mtk3smp)
foreach(bench_target IN ITEMS car_bench wdt_check)
    target_compile_options(${bench_target} PRIVATE
        -ffreestanding -ffunction-sections -fdata-sections ${FIRMWARE_WARNINGS})
    target_link_options(${bench_target} PRIVATE
        -nostartfiles
        -Wl,--gc-sections
        -Wl,--undefined=boot2
        -T "${MTK3SMP_PATH}/etc/linker/pico_rp2040/tkernel_map.ld"
    )
    pico_add_extra_outputs(${bench_target})
endforeach()


# Guided IMU check with the motor pins held low.
add_executable(physics_check EXCLUDE_FROM_ALL
    app_program/bench/physics_check.c)
# The isolated check target uses the IMU task setup directly.
target_include_directories(physics_check PRIVATE ${IMU_SRC})
target_link_libraries(physics_check PRIVATE imu_task mtk3smp)
target_compile_options(physics_check PRIVATE
    -ffreestanding -ffunction-sections -fdata-sections
    -Werror -Wall -Wextra -Wparentheses -Wfloat-equal -Wswitch-default
    -Wimplicit-fallthrough -Wshadow -Wvla)
target_link_options(physics_check PRIVATE
    -nostartfiles
    -Wl,--gc-sections
    -Wl,--undefined=boot2
    -T "${MTK3SMP_PATH}/etc/linker/pico_rp2040/tkernel_map.ld")
pico_add_extra_outputs(physics_check)
