#include "hardware_unit_tests.hh"

#include "adsbee.hh"

UTEST_STATE();

static bool utest_main_called = false;

CPP_AT_CALLBACK(ATTestCallback) {
    if (op == '=') {
        CPP_AT_ERROR("AT+TEST command doesn't take any arguments.");
    }

    if (!adsbee.LR2021IsEnabled()) {
        // AT+LR_ENABLE=0 hands the LR2021 bus to an external host. Every test drives that bus, so running
        // them would contend with the host.
        CPP_AT_ERROR("LR2021 interface disabled (AT+LR_ENABLE=0); hardware unit tests unavailable.");
    }

    if (!utest_main_called) {
        int argc = 0;
        const char* argv[1];
        // The LR2021 tests reset and re-init the chip, which drops the receiver config. Keep the IRQ-paced
        // drain chain out of the way while they run, then restore reception as it was (including a
        // user-disabled receiver, which goes back into reset).
        adsbee.BeginDirectLR2021Access();
        int ret = utest_main(argc, argv);
        utest_main_called = true;
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

    CPP_AT_ERROR("Can't run utest_main multiple times because it'll break (janky af).");
}