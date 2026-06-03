//
// Entry point for the unit-test executable. Test cases live in the other
// test_*.cpp files and self-register via the TEST macro.
//
#include "test_framework.h"

int main() {
    return testing::runAll();
}
