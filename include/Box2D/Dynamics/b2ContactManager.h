// b2ContactManager.h — REPLACEMENT (minimal stub)
//
// Held as a value field by b2World. The particle solver only accesses
// m_contactFilter / m_contactListener pointers, both of which stay nullptr
// in our adapter (advanced LiquidFun features the port does not expose).

#ifndef B2_CONTACT_MANAGER_H
#define B2_CONTACT_MANAGER_H

#include <Box2D/Common/b2Settings.h>

class b2Contact;
class b2ContactFilter;
class b2ContactListener;
class b2BlockAllocator;
struct b2BroadPhase {};

class b2ContactManager {
public:
    b2Contact* m_contactList = nullptr;
    int32 m_contactCount = 0;
    b2ContactFilter* m_contactFilter = nullptr;
    b2ContactListener* m_contactListener = nullptr;
    b2BlockAllocator* m_allocator = nullptr;
    b2BroadPhase m_broadPhase;
};

#endif
