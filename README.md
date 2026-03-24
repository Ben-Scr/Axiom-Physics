# BoltPhys

BoltPhys is a minimalist, DLL-friendly 2D physics engine for C++ with a clear separation between bodies, colliders, and a registration-based `PhysicsWorld`.

## Architecture
- **Bodies**: `StaticBody`, `DynamicBody`, and `KinematicBody` derive from `Body`.
- **Colliders**: `BoxCollider`, `CircleCollider`, and `PolygonCollider` derive from `Collider`.
- **World**: `PhysicsWorld` stores global settings, registration logic, the simulation step, and the contact list.
- **Contacts**: Collisions are reported as simple `Contact` entries for overlapping AABBs.

## Behavior
- Only bodies and colliders registered in `PhysicsWorld` are simulated.
- Gravity affects only `DynamicBody` instances with gravity enabled.
- World bounds can be enabled globally, while boundary checks can be toggled on or off per body.
- `StaticBody` disables gravity and boundary checks by default.
- There is intentionally **no rotation** and **no bounce/restitution physics**.
- Overlaps are resolved with a simple penetration correction for dynamic bodies.

## Example
```cpp
BoltPhys::PhysicsWorld world;

BoltPhys::DynamicBody player;
BoltPhys::BoxCollider playerCollider({0.5f, 1.0f});

world.RegisterBody(player);
world.RegisterCollider(playerCollider);
world.AttachCollider(player, playerCollider);

world.Step(1.0f / 60.0f);
```

## How to integrate BoltPhys into your project
1. Add the BoltPhys source files and headers to your C++ project, or build the library as a DLL/static library and link it to your game or application.
2. Make sure your compiler can find the BoltPhys headers by adding the appropriate include directory to your build configuration.
3. Include the main BoltPhys headers wherever you want to create physics objects.
4. Create a `PhysicsWorld` instance during your initialization phase and configure its global settings if needed.
5. Create bodies and colliders, register them with the world, and attach colliders to the corresponding bodies.
6. Call `world.Step(deltaTime)` once per frame or tick to advance the simulation.
7. Read positions and contacts from the registered objects after each step to drive gameplay or rendering.

## Basic integration flow
```cpp
#include <BoltPhys/PhysicsWorld.hpp>
#include <BoltPhys/Bodies/DynamicBody.hpp>
#include <BoltPhys/Colliders/BoxCollider.hpp>

int main()
{
    BoltPhys::PhysicsWorld world;

    BoltPhys::DynamicBody player;
    BoltPhys::BoxCollider playerCollider({0.5f, 1.0f});

    world.RegisterBody(player);
    world.RegisterCollider(playerCollider);
    world.AttachCollider(player, playerCollider);

    const float deltaTime = 1.0f / 60.0f;
    world.Step(deltaTime);

    return 0;
}
```
