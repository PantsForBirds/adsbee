#include "hardware_unit_tests.hh"

UTEST_STATE();

// For some reason the ESP IDF isn't collecting the UTEST cases in other .cpp files, so we need to explicitly include
// them here for each file.
#include "test_spi_coprocessor.cpp"

bool RunHardwareUnitTests() {
    int argc = 0;
    char* argv[1] = {nullptr};
    // utest_main returns the number of failed tests.
    int ret = utest_main(argc, argv);
    if (ret == 0) {
        return true;
    }
    CONSOLE_ERROR("RunHardwareUnitTests", "%d hardware unit test(s) failed.", ret);
    return false;
}