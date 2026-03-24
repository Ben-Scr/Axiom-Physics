#include "PhysicsWorld.hpp"

#include <algorithm>

namespace BoltPhys {
    namespace {
        template <typename T>
        bool Contains(const std::vector<T*>& items, const T& item)
        {
            return std::find(items.begin(), items.end(), &item) != items.end();
        }

        float Clamp(float value, float minValue, float maxValue) noexcept
        {
            return std::max(minValue, std::min(value, maxValue));
        }

        Vec2 Clamp(const Vec2& value, const Vec2& minValue, const Vec2& maxValue) noexcept
        {
            return {
                Clamp(value.x, minValue.x, maxValue.x),
                Clamp(value.y, minValue.y, maxValue.y)
            };
        }
    }

    PhysicsWorld::PhysicsWorld() = default;

    PhysicsWorld::PhysicsWorld(const WorldSettings& settings)
        : m_settings(settings)
    {
    }

    void PhysicsWorld::SetSettings(const WorldSettings& settings) noexcept
    {
        m_settings = settings;
    }

    const WorldSettings& PhysicsWorld::GetSettings() const noexcept
    {
        return m_settings;
    }

    bool PhysicsWorld::RegisterBody(Body& body)
    {
        if (Contains(m_bodies, body)) {
            return false;
        }

        m_bodies.push_back(&body);
        return true;
    }

    Contact PhysicsWorld::BuildContact(Body& bodyA, Body& bodyB) const
    {
        const AABB aabbA = bodyA.GetCollider()->ComputeAABB();
        const AABB aabbB = bodyB.GetCollider()->ComputeAABB();

        const float overlapX = std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
        const float overlapY = std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);

        Contact contact{};
        contact.bodyA = &bodyA;
        contact.bodyB = &bodyB;
        contact.penetration = std::min(overlapX, overlapY);

        const Vec2 delta = bodyB.GetPosition() - bodyA.GetPosition();
        if (overlapX < overlapY) {
            contact.normal = { delta.x >= 0.0f ? 1.0f : -1.0f, 0.0f };
        }
        else {
            contact.normal = { 0.0f, delta.y >= 0.0f ? 1.0f : -1.0f };
        }

        return contact;
    }
}