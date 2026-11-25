add_library(line_control STATIC
    app_program/ir/line/line_control/follow.c
    app_program/ir/line/line_control/junction.c
    app_program/ir/line/line_control/recovery.c)
target_include_directories(line_control PUBLIC app_program)
target_compile_options(line_control PRIVATE ${FIRMWARE_WARNINGS})

add_library(track STATIC
    app_program/ir/line/drv_ir_capture.c
    app_program/ir/barcode/barcode_task.c
    app_program/ir/barcode/barcode_width.c
    app_program/ir/line/line_filter.c
    app_program/ir/line/line_task.c
    app_program/motion/start_check.c
)
target_include_directories(track PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(track PUBLIC line_control platform barcode motion_service telemetry supervisor obstacle_motion mtk3smp)
target_compile_options(track PRIVATE ${FIRMWARE_WARNINGS})


add_library(barcode STATIC
    app_program/ir/barcode/action.c
    app_program/ir/barcode/parser.c
)
target_include_directories(barcode PUBLIC app_program)
target_compile_options(barcode PRIVATE ${FIRMWARE_WARNINGS})
