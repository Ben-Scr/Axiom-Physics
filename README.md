# BoltPhys

BoltPhys is a minimalist 2D physics module for C++ with a clear separation between `Body`, colliders, and a registration-based `PhysicsWorld`.

## Build target
BoltPhys is configured as a **static library** in all Visual Studio configurations (`Debug/Release` and `Win32/x64`).

- Project output type: `StaticLibrary`
- Preprocessor define: `BT_STATIC`
- API macro behavior: `BOLT_PHYS_API` becomes empty for static builds

## Architecture
- **Body**: A single `Body` class with `BodyType` (`Static`, `Dynamic`, `Kinematic`).
- **Colliders**: `BoxCollider`, `CircleCollider`, and `PolygonCollider` derive from `Collider`.
- **World**: `PhysicsWorld` stores global settings, registration logic, simulation step, and contacts.
- **Utility Queries**: `Physics2D` provides overlap and point queries with and without a world context.

## Behavior
- Only bodies and colliders registered in `PhysicsWorld` are simulated.
- Gravity affects only bodies of type `BodyType::Dynamic` (if gravity is enabled for that body).
- World bounds can be enabled globally, while boundary checks can be toggled per body.
- `SetBodyType` applies sensible defaults:
  - `Static`: gravity off, boundary checks off, velocity reset to zero
  - `Kinematic`: gravity off, boundary checks on
  - `Dynamic`: gravity on, boundary checks on

## Example
```cpp
#include "PhysicsWorld.hpp"
#include "Body.hpp"
#include "BoxCollider.hpp"

int main()
{
    BoltPhys::PhysicsWorld world;

    BoltPhys::Body player;
    player.SetBodyType(BoltPhys::BodyType::Dynamic);
    player.SetPosition({ 0.0f, 2.0f });

    BoltPhys::BoxCollider playerCollider({ 0.5f, 1.0f });

    world.RegisterBody(player);
    world.RegisterCollider(playerCollider);
    world.AttachCollider(player, playerCollider);

    world.Step(1.0f / 60.0f);
    return 0;
}
```
