#include "StaticBody.hpp"

namespace BoltPhys {
    StaticBody::StaticBody()
        : Body(BodyType::Static)
    {
        SetBoundaryCheckEnabled(false);
        SetGravityEnabled(false);
    }
}
