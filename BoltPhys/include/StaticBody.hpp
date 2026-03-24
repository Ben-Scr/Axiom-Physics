#pragma once
#include "Export.hpp"
#include "Body.hpp"

namespace BoltPhys {
    class BOLT_PHYS_API StaticBody final : public Body
    {
    public:
        StaticBody();
    };
}