#pragma once
#include "Vec2.hpp"

namespace AxiomPhys {
    class Body;

    struct Contact
    {
         Body* bodyA = nullptr;
         Body* bodyB = nullptr;
        Vec2 normal{};
        float penetration = 0.0f;
    };
}