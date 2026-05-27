add_executable(dam_break main.cpp)

target_link_libraries(dam_break PRIVATE liquidfun_box2d3)

# The sample binary calls into Box2D 3.x directly (b2CreateWorld, b2World_Step,
# etc.), so it needs Box2D's compiled library on its link line. The library
# `liquidfun_box2d3` already links it transitively via find_package, but the
# headers-only BOX2D_ROOT fallback in the parent CMakeLists doesn't bring in
# a link-target — surface a clearer error if so.
if(NOT TARGET box2d::box2d)
    message(WARNING
        "dam_break: box2d::box2d target not available — sample will not link.\n"
        "Install Box2D 3.x system-wide or build the library without\n"
        "LIQUIDFUN_BOX2D3_SAMPLES if you only need the static lib.")
endif()
