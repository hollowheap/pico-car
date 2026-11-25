add_library(ir_line_control STATIC
    app_program/ir/line/line_control/follow.c
    app_program/ir/line/line_control/junction.c
    app_program/ir/line/line_control/recovery.c)
target_include_directories(ir_line_control PUBLIC app_program)
target_compile_options(ir_line_control PRIVATE ${FIRMWARE_WARNINGS})

add_library(ir STATIC
    app_program/ir/line/drv_ir_capture.c
    app_program/ir/barcode/barcode_task.c
    app_program/ir/barcode/barcode_width.c
    app_program/ir/line/line_filter.c
    app_program/ir/line/line_task.c
    app_program/motion/start_check.c
)
target_include_directories(ir PRIVATE ${PROJECT_SOURCE_DIR}/app_program)
target_link_libraries(ir PUBLIC ir_line_control platform ir_barcode motion_service
    comms supervisor obstacle_motion_input mtk3smp)
target_compile_options(ir PRIVATE ${FIRMWARE_WARNINGS})


add_library(ir_barcode STATIC
    app_program/ir/barcode/action.c
    app_program/ir/barcode/parser.c
)
target_include_directories(ir_barcode PUBLIC app_program)
target_compile_options(ir_barcode PRIVATE ${FIRMWARE_WARNINGS})
