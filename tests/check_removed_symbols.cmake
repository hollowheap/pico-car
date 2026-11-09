# D95: fail if the removed STEER-word hug law (or anything only it used) is
# still named anywhere under src/ or include/. Run as:
#   cmake -DROOT=<repo root> -P check_removed_symbols.cmake
# It also fails if the one remaining hug law has no caller, so a rename of
# the survivor cannot hide the check.

if(NOT DEFINED ROOT)
    message(FATAL_ERROR "ROOT is not set")
endif()

set(REMOVED_SYMBOLS
    obstacle_plan_hug_steer
    obstacle_plan_steer_toward
    OBSTACLE_HUG_FAR_MM
    OBSTACLE_NAV_GUARD_HEAD_MAX_DECI)

file(GLOB_RECURSE SOURCES
    "${ROOT}/src/*.c" "${ROOT}/src/*.h" "${ROOT}/include/*.h")

list(LENGTH SOURCES SOURCE_COUNT)
if(SOURCE_COUNT EQUAL 0)
    message(FATAL_ERROR "no sources found under ${ROOT}/src")
endif()

set(FOUND "")
foreach(FILE IN LISTS SOURCES)
    file(READ "${FILE}" TEXT)
    foreach(SYMBOL IN LISTS REMOVED_SYMBOLS)
        if(TEXT MATCHES "(^|[^A-Za-z0-9_])${SYMBOL}([^A-Za-z0-9_]|$)")
            string(APPEND FOUND "  ${SYMBOL} in ${FILE}\n")
        endif()
    endforeach()
endforeach()

if(NOT FOUND STREQUAL "")
    message(FATAL_ERROR "removed hug-law symbols still present:\n${FOUND}")
endif()

# The survivor must be called outside its own definition and declaration.
set(CALLERS 0)
foreach(FILE IN LISTS SOURCES)
    file(READ "${FILE}" TEXT)
    string(REGEX MATCHALL "obstacle_plan_hug_ctrl *\\(" HITS "${TEXT}")
    list(LENGTH HITS HIT_COUNT)
    math(EXPR CALLERS "${CALLERS} + ${HIT_COUNT}")
endforeach()

# Declaration + definition + at least one call.
if(CALLERS LESS 3)
    message(FATAL_ERROR "obstacle_plan_hug_ctrl has no caller (${CALLERS} mentions)")
endif()
