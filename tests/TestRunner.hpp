#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace AxiomPhys::Tests {

    struct TestCase
    {
        const char* name;
        void (*fn)();
    };

    inline std::vector<TestCase>& Registry()
    {
        static std::vector<TestCase> registry;
        return registry;
    }

    inline int& FailCount()
    {
        static int failures = 0;
        return failures;
    }

    inline int& AssertionCount()
    {
        static int assertions = 0;
        return assertions;
    }

    inline std::string& CurrentTest()
    {
        static std::string name;
        return name;
    }

    inline void RecordFailure(const char* expr, const char* file, int line)
    {
        ++FailCount();
        std::fprintf(stderr, "  FAIL [%s] %s @ %s:%d\n",
                     CurrentTest().c_str(), expr, file, line);
    }

    struct AutoRegister
    {
        AutoRegister(const char* name, void (*fn)())
        {
            Registry().push_back({ name, fn });
        }
    };

}

#define AXIOM_TEST_CONCAT_IMPL(a, b) a##b
#define AXIOM_TEST_CONCAT(a, b) AXIOM_TEST_CONCAT_IMPL(a, b)

#define AXIOM_TEST_CASE(NAME)                                                      \
    static void AXIOM_TEST_CONCAT(Test_, NAME)();                                  \
    static const ::AxiomPhys::Tests::AutoRegister AXIOM_TEST_CONCAT(reg_, NAME) {  \
        #NAME, &AXIOM_TEST_CONCAT(Test_, NAME)                                     \
    };                                                                             \
    static void AXIOM_TEST_CONCAT(Test_, NAME)()

#define EXPECT_TRUE(cond) do {                                                     \
        ++::AxiomPhys::Tests::AssertionCount();                                    \
        if (!(cond)) {                                                             \
            ::AxiomPhys::Tests::RecordFailure(#cond, __FILE__, __LINE__);          \
        }                                                                          \
    } while (0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))

#define EXPECT_EQ(a, b) do {                                                       \
        ++::AxiomPhys::Tests::AssertionCount();                                    \
        if (!((a) == (b))) {                                                       \
            ::AxiomPhys::Tests::RecordFailure(#a " == " #b, __FILE__, __LINE__);   \
        }                                                                          \
    } while (0)

#define EXPECT_NEAR(a, b, eps) do {                                                \
        ++::AxiomPhys::Tests::AssertionCount();                                    \
        const double _da = static_cast<double>(a);                                 \
        const double _db = static_cast<double>(b);                                 \
        if (std::fabs(_da - _db) > (eps)) {                                        \
            ::AxiomPhys::Tests::RecordFailure(#a " ~= " #b, __FILE__, __LINE__);   \
        }                                                                          \
    } while (0)
