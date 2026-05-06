#pragma once
#include "Export.hpp"
#include "Collider.hpp"

namespace AxiomPhys {
    class AXIOM_PHYS_API CircleCollider final : public Collider
    {
    public:
        explicit CircleCollider(float radius);

        float GetRadius() const noexcept;
        void SetRadius(float radius);

        AABB ComputeAABB() const noexcept override;

    private:
        float m_radius = 0.5f;
    };
}