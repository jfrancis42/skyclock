# cmake/toolchain-aarch64-linux.cmake
# Cross-compilation toolchain for aarch64 Linux (Raspberry Pi 64-bit OS)
# Uses the Ubuntu multiarch arm64 packages.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Cross-compilers
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Sysroot — Ubuntu multiarch keeps arm64 libs/headers in-tree
set(CMAKE_SYSROOT "")
set(CMAKE_FIND_ROOT_PATH
    /usr/aarch64-linux-gnu
    /usr/lib/aarch64-linux-gnu
    /usr/include/aarch64-linux-gnu
)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Tell CMake where to find arm64 packages
set(CMAKE_PREFIX_PATH /usr/lib/aarch64-linux-gnu/cmake)

# Qt6: point to arm64 cmake config and native host tools
set(QT_HOST_PATH /usr/lib/x86_64-linux-gnu/cmake/Qt6)
set(Qt6_DIR /usr/lib/aarch64-linux-gnu/cmake/Qt6)

# pkg-config for arm64
set(PKG_CONFIG_EXECUTABLE /usr/bin/aarch64-linux-gnu-pkg-config)
