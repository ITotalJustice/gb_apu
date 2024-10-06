cmake_minimum_required(VERSION 3.13)

project(gb_apu
    VERSION 1.4.0
    LANGUAGES C
)

option(GB_APU_CXX
    "build with Blip_Buffer.cpp, gives access to bass and treble control"
    OFF
)

if (GB_APU_CXX)
    include(CheckLanguage)
    check_language(CXX)
    if (NOT CMAKE_CXX_COMPILER)
        message(WARNING "GB_APU_CXX set but ca CXX compiler could not be found")
        set(GB_APU_CXX OFF)
    endif()
endif()

# check to see if we have math.h
include(CheckIncludeFile)
CHECK_INCLUDE_FILE(math.h HAS_MATH_H)

if (HAS_MATH_H)
    message(STATUS "found math.h!")
else()
    if (GB_APU_CXX)
        message(WARNING "force disabling GB_APU_CXX as math.h not found!")
        set(GB_APU_CXX OFF)
    endif()
endif()

add_library(gb_apu gb_apu.c)
target_include_directories(gb_apu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(gb_apu PRIVATE
    GB_APU_CXX=$<BOOL:${GB_APU_CXX}>
    GB_APU_HAS_MATH_H=$<BOOL:${HAS_MATH_H}>
)

set_target_properties(gb_apu PROPERTIES C_STANDARD 99)

if (GB_APU_CXX)
    message(STATUS "gb_apu built with Blip_Buffer (CXX)")
    enable_language(CXX)
    set_target_properties(gb_apu PROPERTIES CXX_STANDARD 98)
    target_sources(gb_apu PRIVATE blargg/blip_wrap.cpp blargg/Blip_Buffer.cpp)
else()
    message(STATUS "gb_apu built with blip_buf (C)")
    target_sources(gb_apu PRIVATE blargg/blip_wrap.c blargg/blip_buf.c)
endif()

if (HAS_MATH_H)
    # only link with math.h if found as some platforms don't require linking against it
    find_library(MATH_LIB NAMES m)
    if (MATH_LIB)
        target_link_libraries(gb_apu PUBLIC ${MATH_LIB})
    endif()
endif()

# enable warnings
target_compile_options(gb_apu PRIVATE
    $<$<OR:$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>,$<C_COMPILER_ID:GNU>>:
        -Wall
        -Wextra
    >
    $<$<C_COMPILER_ID:MSVC>:
        /W4
    >
)
