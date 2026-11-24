#pragma once

// t-1307 LAB ONLY (not for main): bench sources for the PPS capture path (AT+PPS_LAB).
//   PWM:    a hardware-timed PPS on the GNSS PPS pin (GPIO 2 = PWM slice 1 A, the MLAT jitter counter's slice). Once a
//           second core 0 switches the pin to PWM for exactly one counter wrap: the rising edge is at the wrap, so the
//           jitter counter value OnDemodBegin() reads first is the capture delay in clk_sys cycles.
//   PULSES: the PPS input is the comparator pulses pin (GPIO 19); a 1 Hz Waggler CW burst is the PPS (its own clock).
//           Edges are debounced (990 ms, and still high 3 us later) in RecordPPSEdge(). Each edge is printed; RAW output gives the frame's Mode S
//           stamp. A PPSClock fed with consecutive seconds gives the firmware fit's residuals.

#include "adsbee.hh"
#include "comms.hh"
#include <cmath>

#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pps_clock.hh"

class PPSLab {
   public:
    enum Mode : uint8_t { kOff = 0, kPWM, kPulses };
    static constexpr uint16_t kPWMHighCycles = 8192;  // 66 us high after the wrap.
    static constexpr uint16_t kBinCycles = 4;
    static constexpr uint16_t kNumBins = 256;  // Up to 1024 cycles (8 us); above: overflow.

    void SetMode(Mode mode) {
        uint16_t pps_pin = bsp.gnss_pps_pin;
        adsbee.SetPPSPin(UINT16_MAX);
        adsbee.pps_lab_debounce_us = 0;
        gpio_init(pps_pin);  // SIO input.
        mode_ = mode;
        Reset();
        if (mode == kPWM) {
            gpio_set_dir(pps_pin, GPIO_OUT);
            gpio_put(pps_pin, 0);
            pwm_set_chan_level(pwm_gpio_to_slice_num(pps_pin), pwm_gpio_to_channel(pps_pin), kPWMHighCycles);
            adsbee.SetPPSPin(pps_pin);
        } else if (mode == kPulses) {
            adsbee.pps_lab_debounce_us = 990'000;  // The next 1 Hz burst is due 10 ms later.
            adsbee.SetPPSPin(bsp.r1090_pulses_pin);
        }
        next_pulse_us_ = time_us_32() + 1'000'000;
        PPSClock::Edge latest, previous;
        seen_edges_ = adsbee.GetPPSEdges(latest, previous);
    }

    void Reset() {
        n_ = 0;
        missed_ = 0;
        min_ = UINT16_MAX;
        max_ = 0;
        sum_ = 0;
        sum_sq_ = 0;
        overflow_ = 0;
        for (auto& b : bins_) b = 0;
        clock_.Restart();
        first_edge_ = true;
    }

    // Core 0 main loop.
    void Update() {
        if (mode_ == kOff) return;
        uint32_t now_us = time_us_32();
        if (mode_ == kPWM && static_cast<int32_t>(now_us - next_pulse_us_) >= 0) {
            next_pulse_us_ += 1'000'000;
            EmitPWMEdge();
        }
        PPSClock::Edge latest, previous;
        uint16_t jitter_counts = 0;
        uint32_t num_edges = adsbee.GetPPSEdges(latest, previous, &jitter_counts);
        if (num_edges == seen_edges_) return;
        missed_ += num_edges - seen_edges_ - 1;
        seen_edges_ = num_edges;
        if (mode_ == kPWM) {
            AddLatency(jitter_counts);
        } else {
            // Consecutive Waggler frames are a second apart (on its clock): number them by the MLAT counter.
            if (first_edge_) {
                first_mlat_ = latest.mlat_48mhz_counts;
                first_edge_ = false;
            }
            uint64_t second = (latest.mlat_48mhz_counts - first_mlat_ + 24'000'000) / 48'000'000;
            PPSClock::Result result = clock_.AddPoint(latest.mlat_48mhz_counts, 1'800'000'000ULL + second);
            CONSOLE_PRINTF("PPSLAB_EDGE=%lu,%llu,%llu,%d,%ld,%lld\r\n", static_cast<unsigned long>(num_edges),
                           static_cast<unsigned long long>(latest.mlat_48mhz_counts),
                           static_cast<unsigned long long>(second), static_cast<int>(result),
                           static_cast<long>(clock_.GetLastResidualNs()),
                           static_cast<long long>(clock_.GetRateErrorPPB()));
        }
    }

    void Print() {
        double mean = n_ ? static_cast<double>(sum_) / n_ : 0;
        double rms = n_ ? sqrt(static_cast<double>(sum_sq_) / n_ - mean * mean) : 0;
        // Percentiles from the 4-cycle bins.
        uint32_t p50 = 0, p99 = 0, p999 = 0, acc = 0;
        for (uint16_t i = 0; i < kNumBins; i++) {
            acc += bins_[i];
            if (!p50 && acc * 2 >= n_) p50 = (i + 1) * kBinCycles;
            if (!p99 && acc * 100 >= n_ * 99) p99 = (i + 1) * kBinCycles;
            if (!p999 && acc * 1000 >= n_ * 999) p999 = (i + 1) * kBinCycles;
        }
        CONSOLE_PRINTF(
            "PPS_LAB=mode=%d,n=%lu,missed=%lu,min=%u,max=%u,mean=%.2f,rms=%.2f,p50<=%lu,p99<=%lu,p999<=%lu,over8us=%lu "
            "(clk_sys cycles, 8 ns)\r\n",
            mode_, static_cast<unsigned long>(n_), static_cast<unsigned long>(missed_), min_ == UINT16_MAX ? 0 : min_,
            max_, mean, rms, static_cast<unsigned long>(p50), static_cast<unsigned long>(p99),
            static_cast<unsigned long>(p999), static_cast<unsigned long>(overflow_));
        CONSOLE_PRINTF("PPS_LAB_HIST=");
        for (uint16_t i = 0; i < kNumBins; i++) {
            if (bins_[i]) CONSOLE_PRINTF("%u:%lu ", i * kBinCycles, static_cast<unsigned long>(bins_[i]));
        }
        CONSOLE_PRINTF("\r\n");
    }

    Mode mode() const { return mode_; }

   private:
    // One rising edge, at a wrap of the jitter counter (clk_sys exact).
    void __not_in_flash_func(EmitPWMEdge)() {
        uint16_t pin = bsp.gnss_pps_pin;
        volatile uint32_t* ctr = &pwm_hw->slice[pwm_gpio_to_slice_num(pin)].ctr;
        uint32_t interrupts = save_and_disable_interrupts();
        uint32_t c;
        do {
            c = *ctr;
        } while (c < kPWMHighCycles + 512 || c > 0xFFFF - 4096);  // Low phase, well before the wrap.
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint32_t previous = *ctr;
        while ((c = *ctr) >= previous) previous = c;  // The wrap: the edge.
        while (*ctr < kPWMHighCycles + 64) {
        }  // Low again.
        gpio_set_function(pin, GPIO_FUNC_SIO);  // Back to the SIO output, low.
        restore_interrupts(interrupts);
    }

    void AddLatency(uint16_t cycles) {
        n_++;
        if (cycles < min_) min_ = cycles;
        if (cycles > max_) max_ = cycles;
        sum_ += cycles;
        sum_sq_ += static_cast<uint64_t>(cycles) * cycles;
        if (cycles / kBinCycles < kNumBins) {
            bins_[cycles / kBinCycles]++;
        } else {
            overflow_++;
        }
    }

    Mode mode_ = kOff;
    uint32_t next_pulse_us_ = 0;
    uint32_t seen_edges_ = 0;
    uint32_t n_ = 0, missed_ = 0, overflow_ = 0;
    uint16_t min_ = UINT16_MAX, max_ = 0;
    uint64_t sum_ = 0, sum_sq_ = 0;
    uint32_t bins_[kNumBins] = {};
    PPSClock clock_;
    bool first_edge_ = true;
    uint64_t first_mlat_ = 0;
};

inline PPSLab pps_lab;
