#pragma once

#include "stdint.h"

class ObjectDictionary {
   public:
    static const uint8_t kFirmwareVersionMajor;
    static const uint8_t kFirmwareVersionMinor;
    static const uint8_t kFirmwareVersionPatch;
    static const uint8_t kFirmwareVersionReleaseCandidate;
    static const uint32_t kFirmwareVersion;
};

extern ObjectDictionary object_dictionary;