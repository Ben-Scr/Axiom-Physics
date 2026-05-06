#pragma once
#include "Export.hpp"
#include "BodyType.hpp"
#include "Vec2.hpp"

namespace AxiomPhys {
    class Collider;

    class AXIOM_PHYS_API Body
    {
    public:
        Body() noexcept;
        explicit Body(BodyType type) noexcept;
        ~Body() = default;
        void Destroy() noexcept;

        BodyType GetBodyType() const noexcept;
        void SetBodyType(BodyType type) noexcept;

        const Vec2& GetPosition() const noexcept;
        void SetPosition(const Vec2& p) noexcept;

        const Vec2& GetVelocity() const noexcept;
        void SetVelocity(const Vec2& v) noexcept;

        float GetMass() const noexcept;
        void SetMass(float mass) noexcept;

        // Bounciness in [0, 1]. 0 = fully inelastic (no bounce), 1 = perfectly elastic.
        float GetRestitution() const noexcept;
        void SetRestitution(float restitution) noexcept;

        // Coulomb friction coefficient in [0, 1]. The contact friction is the
        // geometric mean of the two participating bodies' coefficients.
        float GetFriction() const noexcept;
        void SetFriction(float friction) noexcept;

        bool IsBoundaryCheckEnabled() const noexcept;
        void SetBoundaryCheckEnabled(bool enabled) noexcept;

        bool IsGravityEnabled() const noexcept;
        void SetGravityEnabled(bool enabled) noexcept;

        Collider* GetCollider() noexcept;
        const Collider* GetCollider() const noexcept;

        // Raw setter — does not update the collider's back-pointer. Prefer
        // PhysicsWorld::AttachCollider, which keeps both sides in sync.
        void SetCollider(Collider* collider) noexcept;

    private:
        BodyType m_bodyType = BodyType::Dynamic;
        Vec2 m_position{ 0.0f, 0.0f };
        Vec2 m_velocity{ 0.0f, 0.0f };
        float m_mass = 1.0f;
        float m_restitution = 0.0f;
        float m_friction = 0.3f;
        bool m_boundaryCheckEnabled = true;
        bool m_gravityEnabled = true;
        Collider* m_collider = nullptr;
    };
}
