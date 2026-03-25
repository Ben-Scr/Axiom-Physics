#include "BoxCollider2D.hpp"
#include "Body2D.hpp"

#include <algorithm>
#include <cmath>

namespace BoltPhys {
    namespace {
        Vec2 SanitizeHalfExtents(const Vec2& halfExtents) noexcept
        {
            constexpr float kMinExtent = 1e-4f;
            return {
                std::max(std::fabs(halfExtents.x), kMinExtent),
                std::max(std::fabs(halfExtents.y), kMinExtent)
            };
        }
    }

    BoxCollider2D::BoxCollider2D() noexcept
        : Collider2D(ColliderType::Box), m_halfExtents({ 0.5f, 0.5f }) {

    }

    BoxCollider2D::BoxCollider2D(const Vec2& halfExtents)
        : Collider2D(ColliderType::Box),
        m_halfExtents(SanitizeHalfExtents(halfExtents))
    {}

    const Vec2& BoxCollider2D::GetHalfExtents() const noexcept
    {
        return m_halfExtents;
    }

    void BoxCollider2D::SetHalfExtents(const Vec2& halfExtents) noexcept
    {
        m_halfExtents = SanitizeHalfExtents(halfExtents);
    }

    AABB BoxCollider2D::ComputeAABB() const noexcept
    {
        const Vec2 center = GetBody() ? GetBody()->GetPosition() : Vec2{};
        return { center - m_halfExtents, center + m_halfExtents };
    }
}