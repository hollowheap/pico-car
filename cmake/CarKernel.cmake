find_package(Git REQUIRED)

set(_car_kernel_submodule "${CMAKE_CURRENT_SOURCE_DIR}/lib/mtk3smp-rp2040")
set(_car_kernel_revision "5e305e29b8e6a7f8e6daba4893998e95b8c6700d")
set(_car_kernel_patch "${CMAKE_CURRENT_LIST_DIR}/mtk3smp-car.patch")


if (NOT EXISTS "${_car_kernel_submodule}/include/tk/tkernel.h")
    message(FATAL_ERROR "Run git submodule update --init --recursive before configuring.")
endif()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_car_kernel_patch}")
file(SHA256 "${_car_kernel_patch}" _car_kernel_patch_hash)
set(MTK3SMP_PATH "${CMAKE_CURRENT_BINARY_DIR}/car-kernel-${_car_kernel_patch_hash}")
set(_car_kernel_stamp "${MTK3SMP_PATH}/.car-kernel-revision")
set(_car_kernel_expected "${_car_kernel_revision} ${_car_kernel_patch_hash}")
set(_car_kernel_current "")
if (EXISTS "${_car_kernel_stamp}")
    file(READ "${_car_kernel_stamp}" _car_kernel_current)
endif()

if (NOT _car_kernel_current STREQUAL _car_kernel_expected)
    set(_car_kernel_archive "${CMAKE_CURRENT_BINARY_DIR}/car-kernel.tar")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${_car_kernel_submodule}" archive
                --format=tar "--output=${_car_kernel_archive}" "${_car_kernel_revision}"
        COMMAND_ERROR_IS_FATAL ANY
    )
    file(MAKE_DIRECTORY "${MTK3SMP_PATH}")
    file(ARCHIVE_EXTRACT INPUT "${_car_kernel_archive}" DESTINATION "${MTK3SMP_PATH}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E env "GIT_CEILING_DIRECTORIES=${CMAKE_CURRENT_BINARY_DIR}"
                "${GIT_EXECUTABLE}" -c core.autocrlf=false apply --no-index --ignore-whitespace --unidiff-zero
                --whitespace=nowarn "${_car_kernel_patch}"
        WORKING_DIRECTORY "${MTK3SMP_PATH}"
        COMMAND_ERROR_IS_FATAL ANY
    )
    file(WRITE "${_car_kernel_stamp}" "${_car_kernel_expected}")
    file(REMOVE "${_car_kernel_archive}")
endif()
