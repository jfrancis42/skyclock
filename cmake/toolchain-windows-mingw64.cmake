# CMake toolchain file for cross-compiling OTA to Windows 64-bit via mingw-w64
# Usage:
#   cmake -B build-windows \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-windows-mingw64.cmake \
#         -DQT_WINDOWS_DIR=$HOME/Qt-cross/windows/6.9.2/mingw_64 \
#         -DHAMLIB_WINDOWS_DIR=$HOME/Qt-cross/hamlib-windows \
#         [-DCMAKE_BUILD_TYPE=Release]

set(CMAKE_SYSTEM_NAME    Windows)
set(CMAKE_SYSTEM_VERSION 10)
set(CMAKE_SYSTEM_PROCESSOR AMD64)

# Cross-compilers installed via: sudo apt-get install mingw-w64
set(CMAKE_C_COMPILER   x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER  x86_64-w64-mingw32-windres)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Qt6 host tools (moc, rcc, uic) run on the build machine (Linux).
# QT_HOST_PATH must be the Qt installation prefix so Qt can find
# lib/qt6/libexec/moc etc. On Ubuntu/Debian this is /usr.
set(QT_HOST_PATH           /usr)
set(QT_HOST_PATH_CMAKE_DIR /usr/lib/x86_64-linux-gnu/cmake)

# These are set by build-windows.sh; you can also pass them on the cmake command line
if(DEFINED QT_WINDOWS_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${QT_WINDOWS_DIR}")
    list(APPEND CMAKE_FIND_ROOT_PATH "${QT_WINDOWS_DIR}")
endif()

if(DEFINED HAMLIB_WINDOWS_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${HAMLIB_WINDOWS_DIR}")
    list(APPEND CMAKE_FIND_ROOT_PATH "${HAMLIB_WINDOWS_DIR}")
endif()
