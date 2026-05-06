#include "Physics2D.hpp"

#include "Body.hpp"
#include "BoxCollider.hpp"
#include "CircleCollider.hpp"
#include "Collider.hpp"
#include "PhysicsWorld.hpp"
#include "PolygonCollider.hpp"

#include <algorithm>
#include <cmath>

namespace AxiomPhys {
    PhysicsWorld* Physics2D::s_context = nullptr;

    namespace {
        Vec2 GetColliderWorldPosition(const Collider& collider) noexcept
        {
            const Body* body = collider.GetBody();
            return body != nullptr ? body->GetPosition() : Vec2{ 0.0f, 0.0f };
        }

        bool ContainsPointCircle(const CircleCollider& circle, const Vec2& point) noexcept
        {
            const Vec2 center = GetColliderWorldPosition(circle);
            const float radius = circle.GetRadius();
            return DistanceSq(point, center) <= radius * radius;
        }

        bool ContainsPointBox(const BoxCollider& box, const Vec2& point) noexcept
        {
            const Vec2 center = GetColliderWorldPosition(box);
            const Vec2 half = box.GetHalfExtents();
            const AABB aabb{ center - half, center + half };
            return aabb.Contains(point);
        }

        bool ContainsPointPolygon(const PolygonCollider& polygon, const Vec2& point) noexcept
        {
            const std::size_t vertexCount = polygon.GetVertexCount();
            const Vec2* vertices = polygon.GetVertices();
            if (vertices == nullptr || vertexCount < 3) {
                return false;
            }

            const Vec2 offset = GetColliderWorldPosition(polygon);
            bool inside = false;

            for (std::size_t i = 0, j = vertexCount - 1; i < vertexCount; j = i++) {
                const Vec2 vi = vertices[i] + offset;
                const Vec2 vj = vertices[j] + offset;

                const bool intersectsY = (vi.y > point.y) != (vj.y > point.y);
                if (!intersectsY) {
                    continue;
                }

                const float denominator = vj.y - vi.y;
                if (std::fabs(denominator) <= 1e-6f) {
                    continue;
                }

                const float xCross = ((vj.x - vi.x) * (point.y - vi.y) / denominator) + vi.x;
                if (point.x < xCross) {
                    inside = !inside;
                }
            }

            return inside;
        }

        bool ColliderContainsPoint(Collider& collider, const Vec2& point) noexcept
        {
            switch (collider.GetType()) {
            case ColliderType::Circle:
                return ContainsPointCircle(static_cast<CircleCollider&>(collider), point);
            case ColliderType::Box:
                return ContainsPointBox(static_cast<BoxCollider&>(collider), point);
            case ColliderType::Polygon:
                return ContainsPointPolygon(static_cast<PolygonCollider&>(collider), point);
            }
            return collider.ComputeAABB().Contains(point);
        }

        Contact MakeContact(Collider& a, Collider& b, const Vec2& normal, float penetration) noexcept
        {
            Contact contact{};
            contact.bodyA = a.GetBody();
            contact.bodyB = b.GetBody();
            contact.colliderA = &a;
            contact.colliderB = &b;
            contact.normal = normal;
            contact.penetration = penetration;
            return contact;
        }

        std::optional<Contact> NarrowphaseAABB(Collider& a, Collider& b) noexcept
        {
            const AABB aabbA = a.ComputeAABB();
            const AABB aabbB = b.ComputeAABB();
            if (!aabbA.Intersects(aabbB)) {
                return std::nullopt;
            }

            const float overlapX = std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
            const float overlapY = std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);
            if (overlapX <= 0.0f || overlapY <= 0.0f) {
                return std::nullopt;
            }

            const Vec2 delta = aabbB.GetCenter() - aabbA.GetCenter();
            Vec2 normal{ 0.0f, 0.0f };
            float penetration = 0.0f;
            if (overlapX < overlapY) {
                normal = { delta.x >= 0.0f ? 1.0f : -1.0f, 0.0f };
                penetration = overlapX;
            }
            else {
                normal = { 0.0f, delta.y >= 0.0f ? 1.0f : -1.0f };
                penetration = overlapY;
            }
            return MakeContact(a, b, normal, penetration);
        }

        std::optional<Contact> NarrowphaseCircleCircle(CircleCollider& a, CircleCollider& b) noexcept
        {
            const Vec2 centerA = GetColliderWorldPosition(a);
            const Vec2 centerB = GetColliderWorldPosition(b);
            const Vec2 delta = centerB - centerA;
            const float radiusSum = a.GetRadius() + b.GetRadius();
            const float distanceSq = LengthSq(delta);
            if (distanceSq >= radiusSum * radiusSum) {
                return std::nullopt;
            }

            const float distance = std::sqrt(distanceSq);
            Vec2 normal = (distance > 1e-6f) ? (delta / distance) : Vec2{ 1.0f, 0.0f };
            return MakeContact(a, b, normal, radiusSum - distance);
        }

        // Returns a contact whose normal points from `circle` toward `box`. The
        // resolver moves A by `-normal * penetration`, so for any case the math
        // here picks the normal whose negation is the direction the circle should
        // move to exit the overlap.
        std::optional<Contact> NarrowphaseCircleBox(CircleCollider& circle, BoxCollider& box) noexcept
        {
            const Vec2 circleCenter = GetColliderWorldPosition(circle);
            const Vec2 boxCenter = GetColliderWorldPosition(box);
            const Vec2 boxHalf = box.GetHalfExtents();
            const Vec2 boxMin = boxCenter - boxHalf;
            const Vec2 boxMax = boxCenter + boxHalf;

            const Vec2 closest{
                std::clamp(circleCenter.x, boxMin.x, boxMax.x),
                std::clamp(circleCenter.y, boxMin.y, boxMax.y)
            };

            const Vec2 delta = circleCenter - closest;
            const float distanceSq = LengthSq(delta);
            const float radius = circle.GetRadius();

            Vec2 normal{ 0.0f, 0.0f };
            float penetration = 0.0f;

            if (distanceSq > 1e-12f) {
                // Circle center is outside the box.
                const float distance = std::sqrt(distanceSq);
                if (distance >= radius) {
                    return std::nullopt;
                }
                normal = -(delta / distance);
                penetration = radius - distance;
            }
            else {
                // Circle center is inside the box; pop out the closest edge.
                const float distLeft   = circleCenter.x - boxMin.x;
                const float distRight  = boxMax.x - circleCenter.x;
                const float distBottom = circleCenter.y - boxMin.y;
                const float distTop    = boxMax.y - circleCenter.y;

                normal = {  1.0f,  0.0f }; penetration = distLeft;
                if (distRight  < penetration) { normal = { -1.0f,  0.0f }; penetration = distRight;  }
                if (distBottom < penetration) { normal = {  0.0f,  1.0f }; penetration = distBottom; }
                if (distTop    < penetration) { normal = {  0.0f, -1.0f }; penetration = distTop;    }
                penetration += radius;
            }

            return MakeContact(circle, box, normal, penetration);
        }

        std::optional<Contact> NarrowphaseBoxCircle(BoxCollider& box, CircleCollider& circle) noexcept
        {
            auto contact = NarrowphaseCircleBox(circle, box);
            if (!contact) {
                return std::nullopt;
            }
            // Returned contact has A=circle, B=box; we want A=box, B=circle.
            std::swap(contact->bodyA, contact->bodyB);
            std::swap(contact->colliderA, contact->colliderB);
            contact->normal = -contact->normal;
            return contact;
        }

        std::optional<Contact> Narrowphase(Collider& a, Collider& b) noexcept
        {
            const ColliderType ta = a.GetType();
            const ColliderType tb = b.GetType();

            if (ta == ColliderType::Circle && tb == ColliderType::Circle) {
                return NarrowphaseCircleCircle(static_cast<CircleCollider&>(a), static_cast<CircleCollider&>(b));
            }
            if (ta == ColliderType::Circle && tb == ColliderType::Box) {
                return NarrowphaseCircleBox(static_cast<CircleCollider&>(a), static_cast<BoxCollider&>(b));
            }
            if (ta == ColliderType::Box && tb == ColliderType::Circle) {
                return NarrowphaseBoxCircle(static_cast<BoxCollider&>(a), static_cast<CircleCollider&>(b));
            }
            return NarrowphaseAABB(a, b);
        }
    }

    void Physics2D::SetContext(PhysicsWorld& world) noexcept
    {
        s_context = &world;
    }

    void Physics2D::ClearContext() noexcept
    {
        s_context = nullptr;
    }

    PhysicsWorld* Physics2D::GetContext() noexcept
    {
        return s_context;
    }

    std::optional<Contact> Physics2D::OverlapsWith(Collider& collider)
    {
        if (s_context == nullptr) {
            return std::nullopt;
        }

        for (Collider* other : s_context->GetColliders()) {
            if (other == nullptr || other == &collider) {
                continue;
            }

            if (auto contact = OverlapsWith(collider, *other)) {
                return contact;
            }
        }

        return std::nullopt;
    }

    std::optional<Contact> Physics2D::OverlapsWith(Collider& colliderA, Collider& colliderB)
    {
        return Narrowphase(colliderA, colliderB);
    }

    const Collider* Physics2D::ContainsPoint(const Vec2& point)
    {
        if (s_context == nullptr) {
            return nullptr;
        }

        for (Collider* collider : s_context->GetColliders()) {
            if (collider == nullptr) {
                continue;
            }

            if (ContainsPoint(*collider, point)) {
                return collider;
            }
        }

        return nullptr;
    }

    bool Physics2D::ContainsPoint(Collider& collider, const Vec2& point)
    {
        return ColliderContainsPoint(collider, point);
    }
}
