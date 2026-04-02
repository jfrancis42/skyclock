# write_build_time.cmake
# Called by the ota_build_timestamp custom target at the start of every build.
# Writes two files:
#   HEADER_FILE  — build/build_timestamp.h  (#define CM_BUILD_TIME <epoch>LL)
#   STAMP_FILE   — build/ota_build_time.stamp  (plain integer, one line)
#
# If FIXED_TIME is set (passed via -DFIXED_TIME=<value>) that value is used
# instead of the current time.  This allows the release script to stamp all
# three platform builds with the same timestamp.
#
# Usage:
#   cmake -DHEADER_FILE=<path> -DSTAMP_FILE=<path> [-DFIXED_TIME=<epoch>] \
#         -P write_build_time.cmake

if(DEFINED FIXED_TIME AND NOT "${FIXED_TIME}" STREQUAL "")
    set(T "${FIXED_TIME}")
else()
    string(TIMESTAMP T "%s")   # seconds since Unix epoch; works on Linux (CMake ≥ 3.1)
endif()

file(WRITE "${HEADER_FILE}" "#pragma once\n#define CM_BUILD_TIME ${T}LL\n")
file(WRITE "${STAMP_FILE}"  "${T}\n")
