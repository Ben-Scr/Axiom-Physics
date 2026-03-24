#include "KinematicBody.hpp"

namespace BoltPhys {
    KinematicBody::KinematicBody()
        : Body(BodyType::Kinematic)
    {
        SetGravityEnabled(false);
    }
}