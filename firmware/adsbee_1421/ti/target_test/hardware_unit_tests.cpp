#include "hardware_unit_tests.hh"

#include "adsbee.hh"

UTEST_STATE();

CPP_AT_CALLBACK(ATTestCallback) {
    if (op == '=') {
        CPP_AT_ERROR("AT+TEST command doesn't take any arguments.");
    }

    if (!adsbee.LR2021IsEnabled()) {
        // With AT+LR_ENABLE=0 an external host owns the LR2021 bus, which every test drives.
        CPP_AT_ERROR("LR2021 interface disabled (AT+LR_ENABLE=0); hardware unit tests unavailable.");
    }

    int argc = 0;
    const char* argv[1];
    // The tests reset the LR2021 and drop the receiver config; restore reception as it was afterwards.
    adsbee.BeginDirectLR2021Access();
    int ret = utest_main(argc, argv);
    if (!adsbee.EndDirectLR2021Access()) {
        CPP_AT_ERROR("Failed to restore the receiver config after the tests.");
    }
    // utest_main returns the number of failed tests.
    if (ret == 0) {
        CPP_AT_SUCCESS();
    } else {
        CPP_AT_ERROR("%d hardware unit test(s) failed.", ret);
    }
}