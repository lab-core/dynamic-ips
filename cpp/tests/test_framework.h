//
// Minimal, dependency-free unit-test framework.
//
// Define tests with TEST(name) { ... } and assert with CHECK / CHECK_EQ.
// Tests self-register; call testing::runAll() from main() to run them all.
// runAll() returns 0 if every test passed and 1 otherwise, so it plugs
// directly into CTest.
//
#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace testing {

/// @brief A single registered test case.
struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

/// @brief Number of failed checks in the test currently running.
inline int& currentFailures() {
    static int failures = 0;
    return failures;
}

/// @brief Self-registering helper created by the TEST macro.
struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

/// @brief Run every registered test and print a summary.
/// @return 0 if all tests passed, 1 otherwise.
inline int runAll() {
    int failedTests = 0;
    for (const auto& test : registry()) {
        currentFailures() = 0;
        try {
            test.fn();
        } catch (const std::exception& e) {
            std::cerr << "  unexpected exception: " << e.what() << "\n";
            ++currentFailures();
        }
        if (currentFailures() == 0) {
            std::cout << "[ PASS ] " << test.name << "\n";
        } else {
            std::cout << "[ FAIL ] " << test.name << "\n";
            ++failedTests;
        }
    }
    std::cout << "\n"
              << (registry().size() - failedTests) << "/" << registry().size()
              << " tests passed\n";
    return failedTests == 0 ? 0 : 1;
}

}  // namespace testing

#define TEST(name)                                                       \
    static void name();                                                  \
    static testing::Registrar registrar_##name(#name, name);             \
    static void name()

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::cerr << "  CHECK failed: " #cond " (" << __FILE__ << ":" \
                      << __LINE__ << ")\n";                              \
            ++testing::currentFailures();                                \
        }                                                                \
    } while (0)

#define CHECK_EQ(actual, expected)                                       \
    do {                                                                 \
        auto actual_v = (actual);                                        \
        auto expected_v = (expected);                                    \
        if (!(actual_v == expected_v)) {                                 \
            std::cerr << "  CHECK_EQ failed: " #actual " == " #expected   \
                      << " (got " << actual_v << ", expected "          \
                      << expected_v << ") (" << __FILE__ << ":"         \
                      << __LINE__ << ")\n";                              \
            ++testing::currentFailures();                                \
        }                                                                \
    } while (0)
