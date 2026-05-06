#include "TestRunner.hpp"

#include <cstdio>

int main()
{
    auto& registry = AxiomPhys::Tests::Registry();
    std::printf("Running %zu tests...\n", registry.size());

    for (const auto& test : registry) {
        AxiomPhys::Tests::CurrentTest() = test.name;
        const int failsBefore = AxiomPhys::Tests::FailCount();
        test.fn();
        const int failsAfter = AxiomPhys::Tests::FailCount();
        std::printf("  %s %s\n",
                    (failsAfter == failsBefore) ? "PASS" : "FAIL",
                    test.name);
    }

    const int failures = AxiomPhys::Tests::FailCount();
    const int assertions = AxiomPhys::Tests::AssertionCount();
    std::printf("\n%d / %d assertions passed across %zu tests. (%d failed)\n",
                assertions - failures, assertions, registry.size(), failures);

    return failures == 0 ? 0 : 1;
}
