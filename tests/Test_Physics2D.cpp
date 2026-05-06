#include "TestRunner.hpp"

#include "Body.hpp"
#include "BoxCollider.hpp"
#include "CircleCollider.hpp"
#include "PhysicsWorld.hpp"
#include "Physics2D.hpp"

using namespace AxiomPhys;

AXIOM_TEST_CASE(Physics2D_OverlapsWithReturnsValueLifetime)
{
    Body a; a.SetPosition({ 0.0f, 0.0f });
    Body b; b.SetPosition({ 0.6f, 0.0f });
    CircleCollider colA(0.5f);
    CircleCollider colB(0.5f);
    a.SetCollider(&colA); colA.SetBody(&a);
    b.SetCollider(&colB); colB.SetBody(&b);

    auto contact1 = Physics2D::OverlapsWith(colA, colB);
    auto contact2 = Physics2D::OverlapsWith(colA, colB);
    EXPECT_TRUE(contact1.has_value());
    EXPECT_TRUE(contact2.has_value());
    // Both copies are independent: contact1 is not invalidated by contact2.
    EXPECT_NEAR(contact1->penetration, 0.4f, 1e-5);
    EXPECT_NEAR(contact2->penetration, 0.4f, 1e-5);
}

AXIOM_TEST_CASE(Physics2D_ContextOverlapFindsAny)
{
    PhysicsWorld world;
    Physics2D::SetContext(world);

    Body a; a.SetPosition({ 0.0f, 0.0f });
    Body b; b.SetPosition({ 0.6f, 0.0f });
    CircleCollider colA(0.5f);
    CircleCollider colB(0.5f);

    world.RegisterBody(a); world.RegisterCollider(colA); world.AttachCollider(a, colA);
    world.RegisterBody(b); world.RegisterCollider(colB); world.AttachCollider(b, colB);

    auto contact = Physics2D::OverlapsWith(colA);
    EXPECT_TRUE(contact.has_value());

    Physics2D::ClearContext();
    EXPECT_FALSE(Physics2D::OverlapsWith(colA).has_value());
}

AXIOM_TEST_CASE(Physics2D_ContainsPointCircle)
{
    Body a; a.SetPosition({ 1.0f, 1.0f });
    CircleCollider col(0.5f);
    a.SetCollider(&col); col.SetBody(&a);

    EXPECT_TRUE(Physics2D::ContainsPoint(col, Vec2{ 1.0f, 1.0f }));
    EXPECT_TRUE(Physics2D::ContainsPoint(col, Vec2{ 1.4f, 1.0f }));
    EXPECT_FALSE(Physics2D::ContainsPoint(col, Vec2{ 1.6f, 1.0f }));
    // AABB-only logic would let (1.4, 1.4) through since it's inside the circle's AABB,
    // but distance sqrt(0.32) ~= 0.566 > 0.5 so it should be outside the actual circle.
    EXPECT_FALSE(Physics2D::ContainsPoint(col, Vec2{ 1.4f, 1.4f }));
}

AXIOM_TEST_CASE(Physics2D_ContainsPointBox)
{
    Body a; a.SetPosition({ 0.0f, 0.0f });
    BoxCollider col({ 1.0f, 1.0f });
    a.SetCollider(&col); col.SetBody(&a);

    EXPECT_TRUE(Physics2D::ContainsPoint(col, Vec2{ 0.5f, 0.5f }));
    EXPECT_FALSE(Physics2D::ContainsPoint(col, Vec2{ 1.5f, 0.5f }));
}
