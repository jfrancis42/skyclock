cmake_minimum_required(VERSION 3.16)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" SC_VERSION_RAW)
string(STRIP "${SC_VERSION_RAW}" SC_VERSION)

project(skyclock VERSION 0.1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ── PortAudio ─────────────────────────────────────────────────────────────────
find_library(PORTAUDIO_LIB portaudio REQUIRED)
find_path(PORTAUDIO_INC portaudio.h)
if(NOT PORTAUDIO_INC)
    message(FATAL_ERROR "portaudio.h not found — install portaudio19-dev")
endif()

# ── Hamlib (optional) ─────────────────────────────────────────────────────────
find_library(HAMLIB_LIB hamlib)
find_path(HAMLIB_INC hamlib/rig.h)
if(HAMLIB_LIB AND HAMLIB_INC)
    set(HAVE_HAMLIB TRUE)
    message(STATUS "Hamlib found — enabling rig control")
else()
    message(STATUS "Hamlib not found — building without rig control")
endif()

# ── Platform libraries ────────────────────────────────────────────────────────
if(WIN32)
    set(SC_PLATFORM_LIBS winmm ole32)
    add_compile_definitions(_USE_MATH_DEFINES)
else()
    set(SC_PLATFORM_LIBS m pthread)
endif()

# ── skyclock executable ───────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/settings.cpp
    src/rigcontrol.cpp
    src/wwvdecoder.cpp
    src/clocksync.cpp
)

add_executable(skyclock ${SOURCES})

target_include_directories(skyclock PRIVATE src ${PORTAUDIO_INC})
target_link_libraries(skyclock PRIVATE ${PORTAUDIO_LIB} ${SC_PLATFORM_LIBS})
target_compile_definitions(skyclock PRIVATE SC_VERSION="${SC_VERSION}")

if(HAVE_HAMLIB)
    target_compile_definitions(skyclock PRIVATE HAVE_HAMLIB)
    target_link_libraries(skyclock PRIVATE ${HAMLIB_LIB})
    target_include_directories(skyclock PRIVATE ${HAMLIB_INC})
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS skyclock RUNTIME DESTINATION bin)

message(STATUS "skyclock ${SC_VERSION} — build type: ${CMAKE_BUILD_TYPE}")
