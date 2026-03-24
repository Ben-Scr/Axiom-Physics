#include "Collider.hpp"

namespace BoltPhys {
    Collider::Collider(ColliderType type) noexcept
        : m_type(type)
    {
    }

    ColliderType Collider::GetType() const noexcept
    {
        return m_type;
    }

    Body* Collider::GetBody() noexcept
    {
        return m_body;
    }

    const Body* Collider::GetBody() const noexcept
    {
        return m_body;
    }

    void Collider::SetBody(Body* body) noexcept
    {
        m_body = body;
    }
}