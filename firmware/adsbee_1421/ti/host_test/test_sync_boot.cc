// SYNC at boot: the shared LR2021 bus stays released until SYNC is low, and the boot pin table drives none of it.
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "sync_policy.hh"

using A = LR2021BusAction;

TEST(SyncBoot, SyncHighReleasesTheBusWhateverTheSettings) {
    for (int lr_enable = 0; lr_enable <= 1; lr_enable++) {
        for (int rx_enable = 0; rx_enable <= 1; rx_enable++) {
            EXPECT_EQ(LR2021BusActionFor(true, lr_enable, rx_enable), A::kRelease) << lr_enable << rx_enable;
        }
    }
}

TEST(SyncBoot, SyncLowFollowsLrEnableAndRxEnable) {
    EXPECT_EQ(LR2021BusActionFor(false, false, true), A::kRelease);  // AT+LR_ENABLE=0
    EXPECT_EQ(LR2021BusActionFor(false, false, false), A::kRelease);
    EXPECT_EQ(LR2021BusActionFor(false, true, false), A::kHoldReset);  // AT+RX_ENABLE=1090,0
    EXPECT_EQ(LR2021BusActionFor(false, true, true), A::kConfigure);
}

static SettingsManager::DeviceInfo WithPartCode(const char* code) {
    SettingsManager::DeviceInfo info;
    strncpy(info.part_code, code, SettingsManager::DeviceInfo::kPartCodeLen);
    return info;
}

TEST(SyncBoot, InternalPullDownOnlyBeforeRevD) {
    EXPECT_FALSE(BoardHasSyncPullDown(WithPartCode("010260002C-20260101-000001")));
    EXPECT_TRUE(BoardHasSyncPullDown(WithPartCode("010260002D-20260101-000001")));
    EXPECT_TRUE(BoardHasSyncPullDown(WithPartCode("010260002E")));
    // Blank, erased, or another part: keep the internal pull-down so a floating SYNC reads low.
    EXPECT_FALSE(BoardHasSyncPullDown(WithPartCode("")));
    EXPECT_FALSE(BoardHasSyncPullDown(WithPartCode("\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF")));
    EXPECT_FALSE(BoardHasSyncPullDown(WithPartCode("010260001D-20260101-000001")));
}

// Board_init() applies gpioPinConfigs[] before main() can read SYNC: every LR2021 bus pin must be an input there.
static std::string PinConfigLine(const std::string& source, const std::string& name) {
    std::istringstream lines(source);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.find("/* " + name + " */") != std::string::npos) {
            return line;
        }
    }
    return "";
}

TEST(SyncBoot, BootPinTableDrivesNoBusPin) {
    std::ifstream file(ADSBEE_1421_SYSCFG_DIR "/ti_drivers_config.c");
    ASSERT_TRUE(file.good());
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string source = buffer.str();
    const struct {
        const char* name;
        const char* pull;
    } pins[] = {
        {"LR_CS", "GPIO_CFG_PULL_UP_INTERNAL"},          {"LR_RESET", "GPIO_CFG_PULL_DOWN_INTERNAL"},
        {"COPRO_SPI_SCLK", "GPIO_CFG_PULL_DOWN_INTERNAL"}, {"COPRO_SPI_PICO", "GPIO_CFG_PULL_DOWN_INTERNAL"},
        {"COPRO_SPI_POCI", "GPIO_CFG_PULL_DOWN_INTERNAL"}, {"LR_BUSY", "GPIO_CFG_PULL_NONE_INTERNAL"},
        {"LR_IRQ", "GPIO_CFG_PULL_DOWN_INTERNAL"},
    };
    for (const auto& pin : pins) {
        std::string line = PinConfigLine(source, pin.name);
        ASSERT_FALSE(line.empty()) << pin.name;
        EXPECT_NE(line.find("GPIO_CFG_INPUT_INTERNAL"), std::string::npos) << line;
        EXPECT_EQ(line.find("OUTPUT"), std::string::npos) << line;
        EXPECT_NE(line.find(pin.pull), std::string::npos) << line;
    }
}
