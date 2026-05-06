#pragma once
#include "Export.hpp"
#include "Vec2.hpp"
#include "Contact.hpp"

#include <optional>

namespace AxiomPhys {
    class Collider;
    class PhysicsWorld;

    class AXIOM_PHYS_API Physics2D
    {
    public:
        // Optional global context. When set, the single-argument query overloads
        // (OverlapsWith / ContainsPoint) operate against this world. The two-argument
        // overloads are pure pair tests and work without a context.
        static void SetContext(PhysicsWorld& world) noexcept;
        static void ClearContext() noexcept;
        static PhysicsWorld* GetContext() noexcept;

        // First contact between `collider` and any other collider in the active
        // world. Returns nullopt if no overlap or no context set.
        static std::optional<Contact> OverlapsWith(Collider& collider);

        // Pair test. Returns nullopt if the two colliders do not overlap.
        // Performs a precise narrowphase for circle/circle and circle/box; falls
        // back to AABB-vs-AABB for polygons.
        static std::optional<Contact> OverlapsWith(Collider& colliderA, Collider& colliderB);

        // Returns the first collider in the active world whose shape contains the
        // given world-space point, or nullptr if none.
        static const Collider* ContainsPoint(const Vec2& point);

        // True if the collider's shape contains the given world-space point.
        // Uses precise tests per shape (circle distance, polygon ray cast); falls
        // back to AABB for unknown shapes.
        static bool ContainsPoint(Collider& collider, const Vec2& point);

    private:
        static PhysicsWorld* s_context;
    };
}
