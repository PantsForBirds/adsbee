#pragma once

#include <new>  // For placement new.

#include "bsp.hh"            // For BSP::GNSSModuleType.
#include "gnss_receiver.hh"  // Base type used by the `gnss` reference.

// ============================================================================
//  ADD A NEW GNSS MODULE HERE — everything for a new vendor lives in this block:
//    1. #include its receiver header.
//    2. Add it as a member of GNSSReceiverStorage.
//    3. Add a `case` to MakeGNSSReceiver() mapping its BSP::GNSSModuleType -> class.
//  (Plus, in bsp.hh: add the enum value and its part-number case in the BSP switch.)
// ============================================================================

// 1. Concrete receiver headers.
#include "none_gnss_receiver.hh"
#include "generic_gnss_receiver.hh"
#include "ublox_max_m10.hh"

// 2. Backing storage for the selected receiver. Exactly one member is ever constructed; the union is
//    sized/aligned to the largest receiver. `inline` gives it a single definition shared across every
//    translation unit that includes this header (C++17+).
union GNSSReceiverStorage {
    NoneGNSSReceiver none;
    GenericGNSSReceiver generic;
    UbloxMAXM10 ublox;
    GNSSReceiverStorage() {}   // Members are constructed via placement new in MakeGNSSReceiver().
    ~GNSSReceiverStorage() {}  // Forever-lived global; never destroyed.
};
inline GNSSReceiverStorage gnss_storage;
inline BSP::GNSSModuleType gnss_type = BSP::kGNSSModuleNone;
inline bool gnss_enabled = false;

// 3. Map the board's GNSS module type to a concrete receiver: placement-new the selected type into
//    gnss_storage and return it. No heap; the returned reference lives for the program's lifetime.
inline GNSSReceiver* MakeGNSSReceiver(BSP::GNSSModuleType type) {
    gnss_type = type;
    switch (type) {
        case BSP::kGNSSModuleGeneric:
            return new (&gnss_storage.generic) GenericGNSSReceiver({});
        case BSP::kGNSSModuleUbloxMAXM10:
            return new (&gnss_storage.ublox) UbloxMAXM10({});
        case BSP::kGNSSModuleNone:
        default:
            return new (&gnss_storage.none) NoneGNSSReceiver({});
    }
}

inline const char* GNSSModuleTypeToStr(BSP::GNSSModuleType type) {
    switch (type) {
        case BSP::kGNSSModuleGeneric:
            return "GENERIC";
        case BSP::kGNSSModuleUbloxMAXM10:
            return "UBX_MIA";
        case BSP::kGNSSModuleNone:
        default:
            return "NONE";
    }
}

inline BSP::GNSSModuleType SettingsToGNSSModuleType(SettingsManager::GNSSReceiverType type) {
    switch (type) {
        case SettingsManager::kGNSSReceiverGeneric:
            return BSP::kGNSSModuleGeneric;
        case SettingsManager::kGNSSReceiverUBXMIA:
            return BSP::kGNSSModuleUbloxMAXM10;
        case SettingsManager::kGNSSReceiverNone:
        default:
            return BSP::kGNSSModuleNone;
    }
}

inline SettingsManager::GNSSReceiverType GNSSModuleTypeToSettings(BSP::GNSSModuleType type) {
    switch (type) {
        case BSP::kGNSSModuleGeneric:
            return SettingsManager::kGNSSReceiverGeneric;
        case BSP::kGNSSModuleUbloxMAXM10:
            return SettingsManager::kGNSSReceiverUBXMIA;
        case BSP::kGNSSModuleNone:
        default:
            return SettingsManager::kGNSSReceiverNone;
    }
}

inline void DestroyGNSSReceiver() {
    switch (gnss_type) {
        case BSP::kGNSSModuleGeneric:
            gnss_storage.generic.~GenericGNSSReceiver();
            break;
        case BSP::kGNSSModuleUbloxMAXM10:
            gnss_storage.ublox.~UbloxMAXM10();
            break;
        case BSP::kGNSSModuleNone:
        default:
            gnss_storage.none.~NoneGNSSReceiver();
            break;
    }
}
// ============================ end add-a-module block =========================

// The application-wide GNSS receiver (defined in main.cc, next to the other board globals).
extern GNSSReceiver* gnss;

inline bool ConfigureGNSSReceiver(bool enabled, BSP::GNSSModuleType type) {
    if (gnss != nullptr) {
        gnss->SetEnable(false);
        DestroyGNSSReceiver();
    }
    gnss = MakeGNSSReceiver(type);
    if (!enabled) {
        gnss->SetEnable(false);
        gnss_enabled = false;
        return true;
    }
    bool init_ok = gnss->Init();
    if (type == BSP::kGNSSModuleNone) {
        // NONE intentionally reports no active receiver, but it is still a valid configuration.
        gnss_enabled = false;
        return true;
    }
    gnss_enabled = init_ok;
    return init_ok;
}
