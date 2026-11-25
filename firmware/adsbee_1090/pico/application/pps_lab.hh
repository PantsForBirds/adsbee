#pragma once

// t-1307 LAB ONLY (not for main): bench sources for the PPS capture path (AT+PPS_LAB).
//   PWM:    a hardware-timed PPS on the GNSS PPS pin (GPIO 2 = PWM slice 1 A, the MLAT jitter counter's slice). Once a
//           second core 0 arms one high pulse of the PWM output: the rising edge is at a counter wrap, so the
//           jitter counter value OnDemodBegin() reads first is the capture delay in clk_sys cycles.
//   PULSES: the PPS input is the comparator pulses pin (GPIO 19); a 1 Hz Waggler CW burst is the PPS (its own clock).
//           Edges are debounced (990 ms, and still high 0.8 us later) in RecordPPSEdge(). Each edge is printed; RAW output gives the frame's Mode S
//           stamp. A PPSClock fed with consecutive seconds gives the firmware fit's residuals.
//   ASYNC:  (t-1336) the PWM edge again (exact: at a wrap of the jitter counter), but core 0 neither polls the bus nor
//           holds interrupts off near the edge. The pin waits as a low SIO output; once a second (plus a random
//           0-1 ms) core 0's main loop reads the counter once and, if it is 66-460 us before a wrap, switches the pin
//           to PWM (low until the wrap, then high for 66 us). A hardware alarm switches it back to SIO ~250 us after
//           the wrap. Per edge, core 0 does one of (at random): nothing; flash-heavy work (random XIP reads from
//           arming until 100 us past the edge); warm (reads the GPIO IRQ handler's flash lines through the XIP cache
//           at arming); flush (flushes the XIP cache at arming). Each has its own histogram.

#include "adsbee.hh"
#include "comms.hh"
#include <cmath>

#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/timer.h"
#include "hardware/structs/busctrl.h"
#include "hardware/sync.h"
#include "pps_clock.hh"
#include "t1339_diag.hh"  // t-1339 LAB.
#include "hardware/structs/iobank0.h"
#include "hardware/structs/pio.h"
#include "hardware/structs/dma.h"

// t-1339 LAB: core 0's view of the GPIO/PIO/DMA state plus core 1's sampled state.
struct T1339Snap {
    uint32_t t_us, pps_edges, arms, load_ring;
    uint32_t intr[4], p1_inte[4], p1_ints[4], p0_inte[4], p0_ints[4], io2_ctrl, io2_status;
    uint32_t pio0_irq, pio0_inte0, pio0_intr, pio_pc[8], dma_ints1, dma_inte1, dma_busy;
    T1339Diag d;
    void Take(uint32_t edges, uint32_t a, uint32_t lr) {
        t_us = time_us_32();
        pps_edges = edges;
        arms = a;
        load_ring = lr;
        for (int i = 0; i < 4; i++) {
            intr[i] = io_bank0_hw->intr[i];
            p1_inte[i] = io_bank0_hw->proc1_irq_ctrl.inte[i];
            p1_ints[i] = io_bank0_hw->proc1_irq_ctrl.ints[i];
            p0_inte[i] = io_bank0_hw->proc0_irq_ctrl.inte[i];
            p0_ints[i] = io_bank0_hw->proc0_irq_ctrl.ints[i];
        }
        io2_ctrl = io_bank0_hw->io[2].ctrl;
        io2_status = io_bank0_hw->io[2].status;
        pio0_irq = pio0_hw->irq;
        pio0_inte0 = pio0_hw->inte0;
        pio0_intr = pio0_hw->intr;
        for (int i = 0; i < 4; i++) {
            pio_pc[i] = pio0_hw->sm[i].addr;
            pio_pc[4 + i] = pio1_hw->sm[i].addr;
        }
        dma_ints1 = dma_hw->ints1;
        dma_inte1 = dma_hw->inte1;
        dma_busy = 0;
        for (int c = 0; c < 12; c++)
            if (dma_hw->ch[c].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) dma_busy |= 1u << c;
        const volatile uint32_t* src = reinterpret_cast<const volatile uint32_t*>(&t1339_diag);
        volatile uint32_t* dst = reinterpret_cast<volatile uint32_t*>(&d);
        for (uint32_t i = 0; i < sizeof(T1339Diag) / 4; i++) dst[i] = src[i];
    }
    void Print(const char* tag) const {
        CONSOLE_PRINTF("T1339_%s=t_us=%lu,edges=%lu,arms=%lu,loads=%08lx,loop=%lu,primask=%lu,ticks=%lu,pc=%08lx,lr=%08lx,xpsr=%08lx,icsr=%08lx,iser=%08lx,ispr=%08lx,gpio_in=%lu,gpio_out=%lu,hf=%lu/%lu/%08lx/%08lx/%08lx\r\n",
                       tag, (unsigned long)t_us, (unsigned long)pps_edges, (unsigned long)arms, (unsigned long)load_ring,
                       (unsigned long)d.loop, (unsigned long)d.loop_primask, (unsigned long)d.ticks, (unsigned long)d.pc,
                       (unsigned long)d.lr, (unsigned long)d.xpsr, (unsigned long)d.icsr, (unsigned long)d.iser,
                       (unsigned long)d.ispr, (unsigned long)d.gpio_isr_enter, (unsigned long)d.gpio_isr_exit,
                       (unsigned long)d.hf_count, (unsigned long)d.hf_core, (unsigned long)d.hf_pc, (unsigned long)d.hf_lr,
                       (unsigned long)d.hf_xpsr);
        CONSOLE_PRINTF("T1339_%s_IO=intr=%08lx %08lx %08lx %08lx,p1_inte=%08lx %08lx %08lx %08lx,p1_ints=%08lx %08lx %08lx %08lx,p0_inte=%08lx %08lx %08lx %08lx,p0_ints=%08lx %08lx %08lx %08lx,io2=%08lx/%08lx\r\n",
                       tag, (unsigned long)intr[0], (unsigned long)intr[1], (unsigned long)intr[2], (unsigned long)intr[3],
                       (unsigned long)p1_inte[0], (unsigned long)p1_inte[1], (unsigned long)p1_inte[2], (unsigned long)p1_inte[3],
                       (unsigned long)p1_ints[0], (unsigned long)p1_ints[1], (unsigned long)p1_ints[2], (unsigned long)p1_ints[3],
                       (unsigned long)p0_inte[0], (unsigned long)p0_inte[1], (unsigned long)p0_inte[2], (unsigned long)p0_inte[3],
                       (unsigned long)p0_ints[0], (unsigned long)p0_ints[1], (unsigned long)p0_ints[2], (unsigned long)p0_ints[3],
                       (unsigned long)io2_ctrl, (unsigned long)io2_status);
        CONSOLE_PRINTF("T1339_%s_PIO=pio0_irq=%02lx,inte0=%03lx,intr=%03lx,pc=%lu %lu %lu %lu|%lu %lu %lu %lu,dma_ints1=%03lx,inte1=%03lx,busy=%03lx\r\n",
                       tag, (unsigned long)pio0_irq, (unsigned long)pio0_inte0, (unsigned long)pio0_intr,
                       (unsigned long)pio_pc[0], (unsigned long)pio_pc[1], (unsigned long)pio_pc[2], (unsigned long)pio_pc[3],
                       (unsigned long)pio_pc[4], (unsigned long)pio_pc[5], (unsigned long)pio_pc[6], (unsigned long)pio_pc[7],
                       (unsigned long)dma_ints1, (unsigned long)dma_inte1, (unsigned long)dma_busy);
        CONSOLE_PRINTF("T1339_%s_PCS=", tag);
        for (int i = 0; i < 16; i++) CONSOLE_PRINTF("%08lx ", (unsigned long)d.pc_ring[i]);
        CONSOLE_PRINTF(" IPSR:");
        for (int i = 0; i < 48; i++)
            if (d.ipsr_hist[i]) CONSOLE_PRINTF("%d:%lu ", i, (unsigned long)d.ipsr_hist[i]);
        CONSOLE_PRINTF("\r\n");
    }
};

class PPSLab {
   public:
    enum Mode : uint8_t { kOff = 0, kPWM, kPulses, kPWMBusPriority, kAsync };
    enum Load : uint8_t { kLoadNone = 0, kLoadFlash, kLoadWarm, kLoadFlush, kNumLoads };
    static constexpr uint16_t kPWMHighCycles = 8192;  // 66 us high after the wrap.
    static constexpr uint16_t kBinCycles = 4;
    static constexpr uint16_t kNumBins = 256;  // Up to 1024 cycles (8 us); above: overflow.

    void SetLoadOnly(uint8_t l) { load_only_ = static_cast<Load>(l); }
    void SetMode(Mode mode) {
        uint16_t pps_pin = bsp.gnss_pps_pin;
        adsbee.SetPPSPin(UINT16_MAX);
        adsbee.pps_lab_debounce_us = 0;
        gpio_init(pps_pin);  // SIO input.
        mode_ = mode;
        Reset();
        // kPWMBusPriority: as kPWM, with core 1 given priority on the bus fabric (does it remove the delay under load?).
        busctrl_hw->priority = mode == kPWMBusPriority ? BUSCTRL_BUS_PRIORITY_PROC1_BITS : 0;
        if (mode == kPWM || mode == kPWMBusPriority) {
            pwm_set_chan_level(pwm_gpio_to_slice_num(pps_pin), pwm_gpio_to_channel(pps_pin), 0);  // Low.
            gpio_set_function(pps_pin, GPIO_FUNC_PWM);
            adsbee.SetPPSPin(pps_pin);
        } else if (mode == kAsync) {
            gpio_put(pps_pin, 0);
            gpio_set_dir(pps_pin, GPIO_OUT);  // A low SIO output between edges.
            pwm_set_chan_level(pwm_gpio_to_slice_num(pps_pin), pwm_gpio_to_channel(pps_pin), kPWMHighCycles);
            if (alarm_num_ < 0) {
                alarm_num_ = hardware_alarm_claim_unused(true);
                hardware_alarm_set_callback(alarm_num_, OnDisarmAlarm);  // Its IRQ runs on core 0 (this core).
            }
            rand_state_ ^= time_us_32() | 1;
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
        for (auto& l : load_bins_)
            for (auto& b : l) b = 0;
        for (auto& n : load_n_) n = 0;
        for (auto& n : load_over_) n = 0;
        not_armed_ = 0;
        extra_edges_ = 0;
        min_lead_ = UINT16_MAX;
        max_lead_ = 0;
        clock_.Restart();
        first_edge_ = true;
    }

    // Core 0 main loop.
    void Update() {
        if (mode_ == kOff) return;
        uint32_t now_us = time_us_32();
        if ((mode_ == kPWM || mode_ == kPWMBusPriority) && static_cast<int32_t>(now_us - next_pulse_us_) >= 0) {
            next_pulse_us_ += 1'000'000;
            EmitPWMEdge();
        }
        if (mode_ == kAsync) UpdateAsync(now_us);
        PPSClock::Edge latest, previous;
        uint16_t jitter_counts = 0;
        uint32_t num_edges = adsbee.GetPPSEdges(latest, previous, &jitter_counts);
        if (mode_ == kAsync && num_edges == seen_edges_ && !t1339_latched_ && seen_edges_ > 0 &&
            now_us - t1339_last_edge_us_ > 3'000'000) {
            t1339_snap_[0].Take(num_edges, t1339_arms_, t1339_load_ring_);  // t-1339: edges stopped.
            busy_wait_us(10'000);
            t1339_snap_[1].Take(num_edges, t1339_arms_, t1339_load_ring_);
            t1339_latched_ = true;
        }
        if (num_edges == seen_edges_) return;
        t1339_last_edge_us_ = now_us;
        missed_ += num_edges - seen_edges_ - 1;
        seen_edges_ = num_edges;
        if (mode_ == kAsync) {
            if (!edge_pending_) {
                extra_edges_++;  // Only one edge per arming is expected.
                return;
            }
            edge_pending_ = false;
            AddLatency(jitter_counts);
            uint16_t b = jitter_counts / kBinCycles;
            load_n_[load_]++;
            if (b < kNumBins) {
                load_bins_[load_][b]++;
            } else {
                load_over_[load_]++;
            }
        } else if (mode_ != kPulses) {
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
        {
            const T1339Diag& d = t1339_prev;
            CONSOLE_PRINTF("T1339_PREV=wd_reason=%lx,chip_reset=%08lx,scratch=%08lx %08lx %08lx %08lx,loop=%lu,ticks=%lu,pc=%08lx,lr=%08lx,xpsr=%08lx,icsr=%08lx,ispr=%08lx,gpio_in=%lu,gpio_out=%lu,hf=%lu/%lu/pc %08lx/lr %08lx/xpsr %08lx/sp %08lx r0-3 %08lx %08lx %08lx %08lx r12 %08lx\r\n",
                           (unsigned long)t1339_prev_wd_reason, (unsigned long)t1339_prev_chip_reset,
                           (unsigned long)t1339_prev_scratch[0], (unsigned long)t1339_prev_scratch[1],
                           (unsigned long)t1339_prev_scratch[2], (unsigned long)t1339_prev_scratch[3],
                           (unsigned long)d.loop, (unsigned long)d.ticks, (unsigned long)d.pc, (unsigned long)d.lr,
                           (unsigned long)d.xpsr, (unsigned long)d.icsr, (unsigned long)d.ispr,
                           (unsigned long)d.gpio_isr_enter, (unsigned long)d.gpio_isr_exit, (unsigned long)d.hf_count,
                           (unsigned long)d.hf_core, (unsigned long)d.hf_pc, (unsigned long)d.hf_lr,
                           (unsigned long)d.hf_xpsr, (unsigned long)d.hf_sp, (unsigned long)d.hf_r0, (unsigned long)d.hf_r1,
                           (unsigned long)d.hf_r2, (unsigned long)d.hf_r3, (unsigned long)d.hf_r12);
            CONSOLE_PRINTF("T1339_PREV_PCS=");
            for (int i = 0; i < 16; i++) CONSOLE_PRINTF("%08lx ", (unsigned long)d.pc_ring[(d.ticks + i) % 16]);
            CONSOLE_PRINTF(" IPSR:");
            for (int i = 0; i < 48; i++)
                if (d.ipsr_hist[i]) CONSOLE_PRINTF("%d:%lu ", i, (unsigned long)d.ipsr_hist[i]);
            CONSOLE_PRINTF("\r\n");
        }
        if (mode_ != kAsync) return;
        {
            T1339Snap now;
            now.Take(seen_edges_, t1339_arms_, t1339_load_ring_);
            now.Print("NOW");
            if (t1339_latched_) {
                t1339_snap_[0].Print("HANG0");
                t1339_snap_[1].Print("HANG1");
            }
        }
        static const char* kLoadNames[kNumLoads] = {"none", "flash", "warm", "flush"};
        CONSOLE_PRINTF("PPS_LAB_ASYNC=not_armed=%lu,extra_edges=%lu,lead_cycles=%u-%u\r\n",
                       static_cast<unsigned long>(not_armed_), static_cast<unsigned long>(extra_edges_),
                       min_lead_ == UINT16_MAX ? 0 : min_lead_, max_lead_);
        for (uint16_t l = 0; l < kNumLoads; l++) {
            CONSOLE_PRINTF("PPS_LAB_HIST_%s=n=%lu,over=%lu ", kLoadNames[l], static_cast<unsigned long>(load_n_[l]),
                           static_cast<unsigned long>(load_over_[l]));
            for (uint16_t i = 0; i < kNumBins; i++) {
                if (load_bins_[l][i]) CONSOLE_PRINTF("%u:%lu ", i * kBinCycles, static_cast<unsigned long>(load_bins_[l][i]));
            }
            CONSOLE_PRINTF("\r\n");
        }
    }

    Mode mode() const { return mode_; }

   private:
    // One rising edge, at a wrap of the jitter counter (clk_sys exact). The compare level is double-buffered (latched
    // at a wrap): set it well before a wrap for one high pulse after it, clear it after that wrap. Core 0 makes no bus
    // access around the edge (it would compete with the interrupt it times).
    void __not_in_flash_func(EmitPWMEdge)() {
        uint16_t pin = bsp.gnss_pps_pin;
        uint slice = pwm_gpio_to_slice_num(pin), channel = pwm_gpio_to_channel(pin);
        volatile uint32_t* ctr = &pwm_hw->slice[slice].ctr;
        uint32_t interrupts = save_and_disable_interrupts();
        uint32_t c;
        do {
            c = *ctr;
        } while (c > 0xFFFF - 8192);  // At least 66 us before the wrap.
        pwm_set_chan_level(slice, channel, kPWMHighCycles);
        restore_interrupts(interrupts);
        busy_wait_at_least_cycles(0x10000 - c + 4096);  // Past the wrap (the edge), without touching the bus.
        pwm_set_chan_level(slice, channel, 0);          // Low from the next wrap on.
    }

    uint32_t Random() {  // xorshift32
        rand_state_ ^= rand_state_ << 13;
        rand_state_ ^= rand_state_ >> 17;
        rand_state_ ^= rand_state_ << 5;
        return rand_state_;
    }

    static void OnDisarmAlarm(uint alarm_num) {
        (void)alarm_num;
        gpio_set_function(bsp.gnss_pps_pin, GPIO_FUNC_SIO);  // Back to the low SIO output (the PWM is low by now).
    }

    // Core 0 main loop, interrupts on. One counter read per call while due; nothing else touches the bus near the edge
    // except the chosen load.
    void UpdateAsync(uint32_t now_us) {
        if (static_cast<int32_t>(now_us - next_pulse_us_) < 0) return;
        if (edge_pending_ && now_us - armed_us_ < 1'000'000) return;  // The edge of the last arming isn't in yet.
        if (edge_pending_) {
            not_armed_++;  // Armed, but no edge came: count it and go on.
            edge_pending_ = false;
        }
        uint16_t pin = bsp.gnss_pps_pin;
        uint slice = pwm_gpio_to_slice_num(pin);
        uint32_t c = pwm_hw->slice[slice].ctr;
        if (c < kPWMHighCycles + 1024 || c > 0xFFFF - kPWMHighCycles) return;  // Not 66-460 us before a wrap: later.
        load_ = load_only_ < kNumLoads ? load_only_ : static_cast<Load>(Random() % kNumLoads);  // t-1339: ASYNCn.
        uint32_t lead = 0x10000 - c;  // Cycles from now to the edge (at 125 MHz).
        if (load_ == kLoadWarm) {
            // The handler's flash lines (vector of IO_IRQ_BANK0) through the cached XIP alias.
            const volatile uint32_t* h =
                reinterpret_cast<const volatile uint32_t*>(reinterpret_cast<uintptr_t>(irq_get_vtable_handler(IO_IRQ_BANK0)) & ~3u);
            if (reinterpret_cast<uintptr_t>(h) >= XIP_BASE && reinterpret_cast<uintptr_t>(h) < SRAM_BASE) {
                uint32_t sum = 0;
                for (uint16_t i = 0; i < 256; i += 2) sum += h[i];  // 1 KB, every 8-byte line.
                warm_sink_ = sum;
            }
        } else if (load_ == kLoadFlush) {
            xip_ctrl_hw->flush = 1;
            (void)xip_ctrl_hw->flush;  // Returns once the flush is done.
        }
        c = pwm_hw->slice[slice].ctr;
        if (c < kPWMHighCycles + 1024 || c > 0xFFFF - 2048) return;  // The load took too long: try again.
        lead = 0x10000 - c;
        gpio_set_function(pin, GPIO_FUNC_PWM);  // Low until the wrap (c > level), then the edge.
        uint32_t edge_us = time_us_32() + lead / 125;
        hardware_alarm_set_target(alarm_num_, from_us_since_boot(time_us_64() + lead / 125 + 250));
        armed_us_ = now_us;
        edge_pending_ = true;
        t1339_arms_++;
        t1339_load_ring_ = (t1339_load_ring_ << 4) | load_;
        if (lead < min_lead_) min_lead_ = lead;
        if (lead > max_lead_) max_lead_ = lead;
        next_pulse_us_ = now_us + 1'000'000 + Random() % 1000;
        if (load_ == kLoadFlash) {
            // Random XIP reads (nearly all cache misses) until 100 us past the edge, interrupts on.
            uint32_t sum = 0;
            while (static_cast<int32_t>(time_us_32() - edge_us) < 100) {
                for (uint16_t i = 0; i < 16; i++) {
                    sum += *reinterpret_cast<const volatile uint32_t*>(XIP_BASE + (Random() & 0x3FFFFC));
                }
            }
            warm_sink_ = sum;
        }
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

    T1339Snap t1339_snap_[2] = {};
    bool t1339_latched_ = false;
    uint32_t t1339_last_edge_us_ = 0, t1339_arms_ = 0, t1339_load_ring_ = 0;
    Load load_only_ = kNumLoads;  // t-1339: AT+PPS_LAB=ASYNC<n> uses only load n.
    Mode mode_ = kOff;
    Load load_ = kLoadNone;
    int alarm_num_ = -1;
    uint32_t rand_state_ = 0x1336BEE5;
    bool edge_pending_ = false;
    uint32_t armed_us_ = 0;
    uint32_t not_armed_ = 0, extra_edges_ = 0;
    uint16_t min_lead_ = UINT16_MAX, max_lead_ = 0;
    volatile uint32_t warm_sink_ = 0;
    uint32_t load_bins_[kNumLoads][kNumBins] = {};
    uint32_t load_n_[kNumLoads] = {}, load_over_[kNumLoads] = {};
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
