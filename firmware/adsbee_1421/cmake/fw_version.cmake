# Derives the ADSBee 1421 firmware version from its single source of truth: the
# kFirmwareVersion{Major,Minor,Patch,ReleaseCandidate} constants in ti/object_dictionary/
# object_dictionary.cpp. Shared by the ti (CC1314 application) and programmer (RP2040 jig) builds so
# the parse logic exists once in CMake. (The same regexes also live in programmer/scripts/
# hex_to_c.py, which bakes kFirmwareVersionStr into the programmer image, and in
# .github/workflows/firmware.yml -- keep the three in sync if the format ever changes.)
#
# Usage:
#   set(FW_VERSION_OBJECT_DICTIONARY <path to object_dictionary.cpp>)
#   include(<this file>)
# Sets:
#   FW_VERSION  -- "X.Y.Z" for releases, "X.Y.Z-rcN" for release candidates.
# Also appends the source file to CMAKE_CONFIGURE_DEPENDS so a version bump re-runs configure and
# any version-stamped artifact names stay in sync.
#
# Note the version is parsed from the *source* file. For the programmer this matches the
# kFirmwareVersionStr hex_to_c.py bakes (same file, same regexes), but a stale
# ti/build/Release/adsbee_1421.hex built from older source would carry an older internal version
# than the stamped filename claims -- the same staleness risk build.sh already warns about; CI
# avoids it by always passing a freshly built hex via ADSBEE_1421_HEX.

if(NOT DEFINED FW_VERSION_OBJECT_DICTIONARY)
    message(FATAL_ERROR "Set FW_VERSION_OBJECT_DICTIONARY before including fw_version.cmake")
endif()

file(READ "${FW_VERSION_OBJECT_DICTIONARY}" _od_contents)
foreach(_field Major Minor Patch ReleaseCandidate)
    string(REGEX MATCH "ObjectDictionary::kFirmwareVersion${_field}[ \t]*=[ \t]*([0-9]+)[uUlL]*[ \t]*;" _
                 "${_od_contents}")
    if(NOT CMAKE_MATCH_1 STREQUAL "")
        set(_fw_${_field} "${CMAKE_MATCH_1}")
    else()
        message(FATAL_ERROR "Could not parse kFirmwareVersion${_field} from ${FW_VERSION_OBJECT_DICTIONARY}")
    endif()
endforeach()
set(FW_VERSION "${_fw_Major}.${_fw_Minor}.${_fw_Patch}")
if(NOT _fw_ReleaseCandidate EQUAL 0)
    set(FW_VERSION "${FW_VERSION}-rc${_fw_ReleaseCandidate}")
endif()
message(STATUS "Firmware version: ${FW_VERSION}")
# Re-run CMake configure when the version source changes so filenames stay in sync.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${FW_VERSION_OBJECT_DICTIONARY}")
