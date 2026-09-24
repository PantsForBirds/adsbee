#include "hardware_unit_tests.hh"

UTEST_STATE();

static bool utest_main_called = false;

CPP_AT_CALLBACK(ATTestCallback) {
    if (op == '=') {
        CPP_AT_ERROR("ATTestCallback", "AT+TEST command doesn't take any arguments.");
    }

    if (!utest_main_called) {
        int argc = 0;
        const char* argv[1];
        int ret = utest_main(argc, argv);
        utest_main_called = true;
        if (ret >= 0) {
            CPP_AT_SUCCESS();
        } else {
            CPP_AT_ERROR("ATTestCallback", "utest_main returned code %d", ret);
        }
    }

    CPP_AT_ERROR("ATTestCallback", "Can't run utest_main multiple times because it'll break (janky af).");
}