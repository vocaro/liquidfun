// box2d3_handle_table.h
//
// Slot-table interface implemented by the consumer (the
// existing 1024-slot body/shape table). The Box2D-3.x-side adapter uses
// these accessors to translate lfa_*_handle integers into b2BodyId /
// b2ShapeId values for calling the Box2D 3.x C API.
//
// For local validation, we provide a no-op implementation in
// box2d3_handle_table_stub.cpp. Real consumer integrations wire this to their own body/shape tables.

#ifndef BOX2D3_HANDLE_TABLE_H
#define BOX2D3_HANDLE_TABLE_H

#include <stdint.h>
#include <box2d/box2d.h>  // for b2BodyId, b2ShapeId

#ifdef __cplusplus
extern "C" {
#endif

// Returns b2_nullBodyId for invalid handles.
b2BodyId b2x_handle_to_body(int32_t handle);

// Returns b2_nullShapeId for invalid handles.
b2ShapeId b2x_handle_to_shape(int32_t handle);

// World handle is the singleton b2WorldId. The adapter currently supports
// one world (matches our game's single-world architecture).
b2WorldId b2x_handle_to_world(int32_t handle);

#ifdef __cplusplus
}
#endif

#endif  // BOX2D3_HANDLE_TABLE_H
