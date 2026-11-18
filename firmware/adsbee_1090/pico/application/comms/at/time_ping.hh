#pragma once

#include <cstdint>
#include <cstdio>
#include <string_view>

struct TimePing {
    static constexpr size_t kReplySize = sizeof("TIME_PING=00000000000000000000,00000000000000000000,0000000000\r\n");

    std::string_view echo;
    bool epoch_valid = false;
    uint64_t epoch_ms = 0;
    uint32_t pps_count = 0;

    bool Format(char (&reply)[kReplySize]) const {
        if (echo.empty() || echo.size() > 20 || echo.find_first_not_of("0123456789") != echo.npos) return false;
        char epoch[21] = "";
        if (epoch_valid) snprintf(epoch, sizeof(epoch), "%llu", static_cast<unsigned long long>(epoch_ms));
        snprintf(reply, sizeof(reply), "TIME_PING=%.*s,%s,%lu\r\n", static_cast<int>(echo.size()), echo.data(),
                 epoch, static_cast<unsigned long>(pps_count));
        return true;
    }
};
