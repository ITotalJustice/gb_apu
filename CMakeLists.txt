cmake_minimum_required(VERSION 3.13)


option(GB_APU_CXX "build with Blip_Buffer.cpp, gives access to bass and treble control" OFF)
option(GB_APU_ZOMBIE "build with zombie mode enabled, not yet complete" OFF)


project(gb_apu
    VERSION 1.2.0
    LANGUAGES C
)

add_library(gb_apu gb_apu.c)
target_include_directories(gb_apu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(gb_apu PRIVATE
    GB_APU_CXX=$<BOOL:${GB_APU_CXX}>
    GB_APU_ZOMBIE=$<BOOL:${GB_APU_ZOMBIE}>
)

if (GB_APU_CXX)
    enable_language(CXX)
    target_sources(gb_apu PRIVATE blip_wrap.cpp blargg/Blip_Buffer.cpp)
    set_target_properties(gb_apu PROPERTIES CXX_STANDARD 98)
else()
    target_sources(gb_apu PRIVATE blip_wrap.c blargg/blip_buf.c)
    set_target_properties(gb_apu PROPERTIES C_STANDARD 99)
endif()

target_compile_options(gb_apu PRIVATE
    $<$<OR:$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>,$<C_COMPILER_ID:GNU>>:
        -Wall
        -Wextra
    >
    $<$<C_COMPILER_ID:MSVC>:
        /W4
    >
)
