# Two-TU split — see dam_break.h header for rationale.
#
# main.cpp:           sees ONLY Box2D 3.x headers (#include <box2d/box2d.h>)
# particle_world.cpp: sees ONLY LiquidFun headers (#include <Box2D/Common/b2Math.h> etc.)
#
# Both TUs must NEVER share LiquidFun + Box2D 3.x headers in the same
# preprocessor scope — those header sets define b2Vec2 / b2Mat22 / b2Rot
# / b2Transform with conflicting signatures and produce hundreds of
# redefinition errors. They communicate via the plain-C declarations
# in dam_break.h.
add_executable(dam_break main.cpp particle_world.cpp)

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
