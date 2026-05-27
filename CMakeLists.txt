# liquidfun-on-box2d-3.x — standalone CMake build
#
# Produces a static library `liquidfun_box2d3` and an optional sample
# executable `dam_break` that exercises the full Ruby/render → particle →
# Box2D 3.x pipeline.
#
# Box2D 3.x is a required dependency; this project does NOT bundle it.
# You can install Box2D 3.x system-wide (e.g. `cmake --install .` from an
# erincatto/box2d build), or point us at it with `-DBOX2D_ROOT=/path/to/box2d`.

cmake_minimum_required(VERSION 3.16)
project(liquidfun_box2d3 CXX C)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# -- Box2D 3.x dependency ---------------------------------------------------
#
# Try find_package first (idiomatic). Fall back to a manual BOX2D_ROOT
# search for users who haven't installed Box2D system-wide. The fallback
# expects `${BOX2D_ROOT}/include/box2d/box2d.h` to exist.

find_package(box2d QUIET CONFIG)

if(NOT box2d_FOUND)
    if(NOT DEFINED BOX2D_ROOT)
        message(FATAL_ERROR
            "Box2D 3.x not found via find_package(box2d) and BOX2D_ROOT not set.\n"
            "Either install Box2D 3.x system-wide, or invoke cmake with\n"
            "  -DBOX2D_ROOT=/path/to/erincatto-box2d-checkout\n"
            "(must contain include/box2d/box2d.h)")
    endif()
    if(NOT EXISTS "${BOX2D_ROOT}/include/box2d/box2d.h")
        message(FATAL_ERROR
            "BOX2D_ROOT=${BOX2D_ROOT} does not contain include/box2d/box2d.h")
    endif()
    # Headers-only fallback: we don't need to link Box2D for the library
    # itself (it's implementation-detail of the user's adapter glue). The
    # sample binary, however, DOES need Box2D's compiled lib — see below.
    set(BOX2D_INCLUDE_DIR "${BOX2D_ROOT}/include")
    message(STATUS "Box2D 3.x: headers at ${BOX2D_INCLUDE_DIR} (no compiled lib located)")
else()
    message(STATUS "Box2D 3.x: found via find_package")
endif()

# -- Sources ----------------------------------------------------------------

file(GLOB LIQUIDFUN_PARTICLE_SRCS  CONFIGURE_DEPENDS src/Box2D/Particle/*.cpp)
file(GLOB LIQUIDFUN_COMMON_SRCS    CONFIGURE_DEPENDS src/Box2D/Common/*.cpp)

set(ADAPTER_SRCS
    src/liquidfun_side_adapter.cpp
    src/box2d3_side_adapter.cpp
)

# -- Static library ---------------------------------------------------------

add_library(liquidfun_box2d3 STATIC
    ${LIQUIDFUN_PARTICLE_SRCS}
    ${LIQUIDFUN_COMMON_SRCS}
    ${ADAPTER_SRCS}
)

target_include_directories(liquidfun_box2d3
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(box2d_FOUND)
    target_link_libraries(liquidfun_box2d3 PUBLIC box2d::box2d)
else()
    target_include_directories(liquidfun_box2d3 PUBLIC ${BOX2D_INCLUDE_DIR})
endif()

# LIQUIDFUN_SIMD_NEON is deliberately NOT defined. Upstream uses
# `#if defined(LIQUIDFUN_SIMD_NEON)` (defined-check, not truthy-check), so
# `-DLIQUIDFUN_SIMD_NEON=0` would still enable SIMD code paths that
# reference symbols only `b2ParticleAssembly.neon.s` provides. The
# non-SIMD path inside FindContacts() / UpdateProxies() is the C++ fallback
# and is what we use. Re-enabling NEON on ARM targets is a follow-up.
target_compile_definitions(liquidfun_box2d3 PRIVATE
    LIQUIDFUN_EXTERNAL_LANGUAGE_API=0
)

# LiquidFun's source carries a few cosmetic warnings (memset/memcpy on
# non-trivially-copyable b2ParticleColor, ignored nodiscard from
# std::remove_if). Silence them in the upstream sources so build output
# stays readable; KEEP warnings on for our adapter code.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    set_source_files_properties(${LIQUIDFUN_PARTICLE_SRCS} ${LIQUIDFUN_COMMON_SRCS}
        PROPERTIES COMPILE_FLAGS "-Wno-nontrivial-memcall -Wno-unused-result")
endif()

# -- Samples ----------------------------------------------------------------

option(LIQUIDFUN_BOX2D3_SAMPLES "Build sample applications" ON)
if(LIQUIDFUN_BOX2D3_SAMPLES)
    add_subdirectory(samples/dam_break)
endif()

# -- Install ----------------------------------------------------------------

install(TARGETS liquidfun_box2d3
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
