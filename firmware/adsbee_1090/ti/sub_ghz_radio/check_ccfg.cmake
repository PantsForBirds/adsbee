# Fails the build if the image's CCFG lost the XOSC cap trim (ti_devices_config.c hand edit, dropped by a SysConfig
# regeneration). Usage: cmake -DBIN=<sub_ghz_radio.bin> -P check_ccfg.cmake
set(CCFG_OFFSET 0x57FA8)  # FLASH_CCFG in cc13x2_cc26x2_nortos.lds.
math(EXPR SIZE_AND_DIS_FLAGS_OFFSET "${CCFG_OFFSET} + 0x8")
math(EXPR MODE_CONF_OFFSET "${CCFG_OFFSET} + 0xC")

function(read_word offset out)
    file(READ ${BIN} hex OFFSET ${offset} LIMIT 4 HEX)
    string(SUBSTRING ${hex} 6 2 b3)
    string(SUBSTRING ${hex} 4 2 b2)
    string(SUBSTRING ${hex} 2 2 b1)
    string(SUBSTRING ${hex} 0 2 b0)
    math(EXPR word "0x${b3}${b2}${b1}${b0}")
    set(${out} ${word} PARENT_SCOPE)
endfunction()

read_word(${SIZE_AND_DIS_FLAGS_OFFSET} flags)
read_word(${MODE_CONF_OFFSET} mode_conf)
math(EXPR linear_disabled "(${flags} >> 4) & 1")
math(EXPR cap_mod "(${mode_conf} >> 17) & 1")
math(EXPR delta "(${mode_conf} >> 8) & 0xFF")
if(NOT linear_disabled EQUAL 0 OR NOT cap_mod EQUAL 0 OR NOT delta EQUAL 0xFD)
    message(FATAL_ERROR "CCFG XOSC cap trim missing (need DIS_LINEAR_CAPARRAY_DELTA_WORKAROUND 0, XOSC_CAP_MOD 0, "
                        "XOSC_CAPARRAY_DELTA 0xFD; see syscfg/ti_devices_config.c).")
endif()
