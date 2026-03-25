#include "Collider2D.hpp"
#include "Body2D.hpp"

namespace BoltPhys {
    Collider2D::Collider2D(ColliderType type) noexcept
        : m_type(type)
    {
    }

    void Collider2D::Destroy() {
        if (m_body != nullptr) {
            m_body->AttachCollider(nullptr);
            m_body = nullptr;
        }
    }

    ColliderType Collider2D::GetType() const noexcept
    {
        return m_type;
    }

    Body2D* Collider2D::GetBody() noexcept
    {
        return m_body;
    }

    const Body2D* Collider2D::GetBody() const noexcept
    {
        return m_body;
    }

    void Collider2D::SetBody(Body2D* body) noexcept
    {
        m_body = body;
    }
}