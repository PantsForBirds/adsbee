# ADC candidates for digitizing the AD8313 video output (ADSBee next revision)

Desk survey, 2026-10-26. The target is the AD8313 video output (about 0.5-2.0 V, about 7 MHz bandwidth, DC-coupled) digitized at 8-20 Msps by RP2040/RP2350 PIO. Mode S chips are 0.5 us long, so 10 Msps gives 5 samples per chip and 20 Msps gives 10.

**Sources and conventions**
- **"JLC/LCSC"**: live JLCPCB parts API (`jlcpcb.com/api/.../selectSmtComponentList`, the same catalog as LCSC), queried 2026-10-26. The price is the highest listed break at or below 1000 pcs. Many listings stop at a 100+ break; those are marked (100+). Stock is JLC/LCSC stock on that date.
- **"ADI list"**: the 1ku list price on the analog.com product page, as returned by web search on 2026-10-26.
- **Not verified**: DigiKey and Mouser blocked automated access (HTTP 403 or timeout). Every price here is from LCSC/JLC or the manufacturer.
- **Lifecycle**: from the TI/ADI product page where it could be fetched. "EOL@LCSC" means JLC/LCSC flags the listing "no longer manufactured", which may refer to that ordering code only.
- **est.**: an estimate or unverified figure.
- **MCU pins**: data bits + 1 clock output (PIO side-set). OTR, OE and PDWN can be tied off. PIO `in pins, N` needs N consecutive GPIOs.

## 1. Parallel CMOS ADCs (the realistic class)

| Part | Maker | Bits | Max fs (min fs) | Supply / 3.3 V I/O OK? | Power near 10 Msps | Package | MCU pins | Input / driver | Price @1k (source, 2026-10-26) | Status | Note |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ADC08100CIMTCX | TI | 8 | 100 Msps (min 20 Msps, per LCSC attr.) | 2.7-3.6 V, separate VDR output supply / **yes** | 1.3 mW/Msps → ~26 mW at its 20 Msps minimum | TSSOP-24 | 9 | Single-ended, VRT/VRB ladder (1.6 Vpp nominal, adjustable) → DC-coupled 0.4-2.0 V possible; buffer recommended | **$3.95** (LCSC C701581, 1308 in stock); TI.com ~$7.86 (est., from web snippet) | Active (TI) | Best stocked 8-bit. Must clock at ≥20 Msps; decimate in PIO/DMA. |
| ADC08060CIMTX | TI | 8 | 60 Msps (min 20) | 2.7-3.6 V / yes | ~26 mW at 20 Msps | TSSOP-24 | 9 | As ADC08100 | $7.37 (100+) (LCSC C701580, 488) | Active | Pin-compatible with ADC08100, costs more. No reason to choose it. |
| ADC1173CIMTC | TI | 8 | 15 Msps | 2.7-3.6 V / yes | 33 mW | TSSOP-24 | 9 | Single-ended, 2 Vpp, ladder ref | $0.93 (LCSC C1542655, **0 stock**); CIMTCX $9.74 (100+) (C701593, 20) | Active (TI) | Very good fit at 12-15 Msps. Price/stock is the problem (the $0.93 listing has no stock). |
| TLC5510INSR / IPW | TI | 8 | 20 Msps | **5 V only**, 5 V CMOS outputs / **no** for RP2040 (RP2350 FT pins: verify) | ~128 mW | SOP-24 / TSSOP-24 | 9 | Internal 2 V ladder from VDDA, DC single-ended | $1.67 (100+) (C21746, 0); IPW $8.06 (C1542825, 20) | Active (TI) | Classic and cheap, but needs 5 V and level shifting. |
| TLC5540 | TI | 8 | 40 Msps | 5 V (3.3 V DVDD not confirmed) / est. no | 85 mW | SOP-24 / TSSOP-24 | 9 | Internal 2 V ladder | $4.09 (100+) INS (C2877998, 0); IPWR $11.31 (100+) (10) | Active (TI); CNS EOL@LCSC | 5 V part. No advantage over ADC08100. |
| ADS830E | TI | 8 | 60 Msps | 5 V core, VDRV 3-5 V outputs (per datasheet; est.) | 170 mW (page also says 215 mW) | SSOP-20 | 9 | 1 or 2 Vpp, single-ended 1.5-3.5 V | $8.41 (100+) (LCSC C125184, 0) | Active (TI) | Too hot and too costly. |
| ADS930E | TI | 8 | 30 Msps | 3 or 5 V / yes at 3 V | 66 mW at 3 V | SSOP-28 | 9 | Single-ended 1-2 Vpp | $4.80 (E/1K, C2670424, 0); $15.0 (C1542965, 1) | Active (TI) | OK spec, poor LCSC stock. |
| ADS931E | TI | 8 | 30 Msps | 2.7-5.5 V / yes | est. ~60 mW | SSOP-28 | 9 | Single-ended | $3.53 (C5555087, 0) | **EOL@LCSC**; TI product page returns 404 → likely obsolete | Avoid. |
| AD9280ARSZ | ADI | 8 | 32 Msps | 2.7-5.5 V, DRVDD 3 V / yes | 95 mW at 32 Msps (est. ~50-60 mW at 10 Msps) | SSOP-28 | 9 | Single-ended or diff., 1/2/4 Vpp, clamp, programmable ref | ADI list $5.15; LCSC $5.55 (C653442, 0) / RL $15.50 (100+) (46) | Production (ADI) | Reference design for the Chinese clones below. |
| AD9057BRSZ-40/60 | ADI | 8 | 40/60/80 Msps | 3 or 5 V, 3.3 V outputs / yes | est. ~100 mW | SSOP-20 | 9 | Single-ended, internal ref | ADI list $4.78; LCSC $3.02 (-60, C653177, 0) | Production (ADI) | Fine part, nothing in stock at LCSC. |
| MAX1192ETI+ | ADI/Maxim | 8 (dual) | 22 Msps | 2.7-3.6 V / yes | est. ~30 mW | TQFN-28 5×5 | 9 (multiplexed A/B bus) | Differential, needs driver | $15.09 (LCSC C5668169, 0) | ETI EOL@LCSC; ETI+ not verified | Dual multiplexed output; overkill. |
| ADC3908S025IRSMR | TI | 8 | 25 Msps (family to 125) | **1.8 V AVDD**, CMOS I/O 1.8 V or 3.3 V option / yes (or run RP2350 IOVDD at 1.8 V) | 39 mW | VQFN-32 4×4 | 9 | Buffered, single-ended or diff., 1.9 Vpp | $7.70 (100+) (LCSC C43740631, 12) | Active, TI's suggested successor | Modern TI part; needs a 1.8 V rail. |
| AD9200ARSZ | ADI | 10 | 20 Msps | 2.7-5.5 V / yes | 80 mW at 20 Msps (est. ~50 mW at 10) | SSOP-28 | 11 | Single-ended, clamp, programmable ref | ADI list $5.99; LCSC $10.19 (100+) (C653234, 10) | Production (ADI) | The 10-bit footprint the clones copy. |
| AD9203ARUZ | ADI | 10 | 40 Msps | 2.7-3.6 V / yes | 74 mW at 40 Msps, power scales with rate (est. ~25 mW at 10) | TSSOP-28 | 11 | 1-2 Vpp, single-ended or diff. | ADI list $8.98; LCSC $13.05 (100+) (C653239, 0) | Production (ADI) | Good spec, too expensive. |
| ADC10065CIMT | TI | 10 | 65 Msps (min 20) | 3 V, adjustable output supply / yes | 68 mW at 65 (est. ~35 mW at 20) | TSSOP-28 | 11 | Differential preferred (1/1.5/2 Vpp) → driver | $11.13 (100+) (C127883, 45) | Active | Too expensive. |
| ADC10080CIMT | TI | 10 | 80 Msps (min 20) | 3 V, adjustable output supply / yes | 79 mW at 80 | TSSOP-28 | 11 | Differential preferred | $13.58 (100+) (C528641, 0) | Active | Too expensive. |
| **MS9280** | Hangzhou Ruimeng (China) | **10** | 35 Msps | 2.7-5.5 V / yes (est.) | est. ~60-90 mW (AD9200-class) | SSOP-28 | 11 | Single-ended, clamp, programmable ref, IF sampling to 135 MHz | **$0.80** (LCSC C1509021, **842 stock**) | Active@LCSC | 10-bit despite the "9280" name. Its feature list matches AD9200, so the AD9200 footprint is likely but not confirmed. Datasheet could not be fetched. |
| HT9280ARSZ | HTCSEMI (China) | 8 | 50 Msps | 2.7-3.6 V / yes | est. AD9280-class | SSOP-28 | 9 | Single- or differential, internal/external ref | $3.39 (LCSC C5336214, 21) | Active@LCSC | Probable AD9280 clone (not verified). Low stock. |
| HT9201ARSZ | HTCSEMI (China) | 10 (dual) | 20 Msps | 2.7-5.5 V / yes | 215 mW at 3 V (both channels) | SSOP-28 | 11 | Input buffers on chip | $4.71 (C3019546, 168) | Active@LCSC | AD9201-style dual I/Q. Too much power. |
| 3PA1030 | 3PEAK (China) | 10 | 50 Msps | 2.7-5.5 V / yes | 84 mW at 3 V | TSSOP-28 | 11 | Single-ended, clamp, OTR | $2.26 (LCSC C248725, 0) | **EOL@LCSC** | Datasheet claims pin compatibility with THS1030/AD9200/AD876. Would be ideal, but discontinued. |
| MS5510 | Ruimeng (China) | 8 | 20 Msps | 5 V / no | est. ~90 mW | SOP-24 | 9 | TLC5510-style ladder | $0.48 (C1509020, 0) | **EOL@LCSC** | TLC5510 clone. Gone. |
| RS1506S2 | Runic (China) | 8 (dual) | 125 Msps | 2.7-3.6 V, 3.3/2.5 V outputs / yes | 204 mW at 125 Msps (one channel can sleep) | LQFP-48 | 9 (one channel) | 1 Vpp differential | $1.42 (LCSC C50202004, 0) | Listed, 0 stock | Datasheet says AD9288 pin-compatible. Large package and too fast. |

Not on LCSC/JLC under any searched name: SGMicro, Chipways, or other clones of AD9220 or TLC5510 at 8-20 Msps. The searches covered MS92xx, CBM92xx, RS/TP/SGM9280 and the brand names. The other Chinese "fast ADCs" stocked (Zynalog ZGAD, CBM, Acelamicro, CIMO) are 1.8 V, 65-250 Msps parts with LVDS or serial outputs at $5-100+, so they don't fit here.

## 2. Fast serial SAR ADCs: throughput vs need

PIO limit: a serial SCLK costs at least 2 SM cycles per bit, so SCLK ≤ 75 MHz at 150 MHz sysclk (≤ 66 MHz on RP2040 at 133 MHz). At 16-18 SCLK per conversion, that gives **≤ ~4 Msps per ADC**. The requirement is 8-20 Msps, and 10 bits × 10 Msps alone is ≥ 100 Mbit/s plus framing.

| Part | Maker | Bits | Max fs | SCLK at max fs | Supply | Package | Price @1k (JLC/LCSC, 2026-10-26) | Verdict |
|---|---|---|---|---|---|---|---|---|
| LTC2315-12 | ADI | 12 | 5 Msps | 87.5 MHz | 2.7-5.25 V | TSOT-23-8 | $3.66 C-grade (C680000, 0); I-grade $15.86 (100+) | Fastest here, but still below 8 Msps, and PIO can't reach 87.5 MHz. ≈4 Msps max via PIO. |
| LTC2311-12 | ADI | 12 | 5 Msps | 105 MHz (diff. input) | 3.3/5 V | MSOP-16 | $4.00 (C679952, 0) | Same limit as LTC2315. |
| AD7276 | ADI | 12 | 3 Msps | 48 MHz | 2.35-3.6 V | TSOT-6 / MSOP-8 | $5.17 (100+) (C650752, 3047) | Too slow (×3-7). |
| ADS7883 | TI | 12 | 3 Msps | 48 MHz | 2.7-5.5 V | SOT-23-6 | $3.68 (C2669912, 28) | Too slow. |
| MAX11105 | ADI/Maxim | 12 | 2 Msps | ~32 MHz (est.) | 2.2-3.6 V | SOT-23-6 | $7.44 (100+) (C490284, 0) | Too slow. |
| AD7476A | ADI | 12 | 1 Msps | 20 MHz | 2.35-5.25 V | SOT-23-6 | $3.22-10.22 (various) | Too slow (×8-20). |
| ADS7041 | TI | 10 | 1 Msps | ~14-16 MHz | 1.65-3.6 V | X2QFN-8 / VSSOP-8 | $1.01 (C2669869, 568) | Too slow. |
| ADS7042 | TI | 12 | 1 Msps | 16 MHz | 1.65-3.6 V | X2QFN-8 / VSSOP-8 | $1.33 (C701642, 1002) | Too slow. |

Conclusion: no single serial SAR reaches 8 Msps. Interleaving 2-4 × LTC2315 with staggered CS would work on paper but costs $8-60, adds offset/gain mismatch spurs, and uses as many pins as a parallel part. **Rejected.**

## 3. On-chip ADCs (confirmed too slow)

| MCU | ADC | Rate | ENOB | Inputs | Source |
|---|---|---|---|---|---|
| RP2040 | 12-bit SAR | **500 ksps** (48 MHz ADC clock) | ~8.7 | 4 + temp | Pico SDK hardware_adc docs; RP2040 datasheet |
| RP2350 | 12-bit SAR | **500 ksps** | ~9.2 | 4 (QFN-60) / 8 (QFN-80) + temp | Pico SDK docs; RP2350 product brief |

Both are 16-40× below the 8-20 Msps needed (1 sample per 4 Mode S chips).

## 4. Cheap alternative: a 2-bit "flash" from extra comparators

Three comparators on a resistor ladder (or PWM-set thresholds) give 4 amplitude levels on 3 consecutive GPIOs, sampled by the existing PIO with `in pins, 3`. With the existing MCP6566 the cost is 2-3 extra parts.

| Part | Type | Delay | Output | Price @1k (JLC/LCSC, 2026-10-26) | Note |
|---|---|---|---|---|---|
| TLV3502AIDR (TI) | dual | 4.5 ns | push-pull | $1.29 (C131165, 2590) | Fast; 3.2 mA per channel |
| TLV3501AIDBVR (TI) | single | 4.5 ns | push-pull | $1.24 (C193413, 5047) | Tokmas clone $0.36 (C54582190) |
| LMV7219M5X/NOPB (TI) | single | 7 ns | push-pull | **$0.36** (C283912, 12091) | HGSEMI clone $0.22 @500 (C20613263) |
| MCP6567T-E/SN (Microchip) | dual | 56 ns | open-drain | $0.63 (C623754, 980) | Same family as the present MCP6566 ($0.62, C623507). HGSEMI clone $0.35. 56 ns delay is fine for 0.5 us chips. |
| TP1962 (3PEAK) | dual | ~19 ns (LCSC attr.) | push-pull | $0.36 (C124561, 482) | Low-cost Chinese option |

A 2-bit flash adds about $0.7-1.3 and 2-3 GPIOs. It gives coarse amplitude for preamble/level checks and multi-threshold slicing, but no real waveform.

## 5. RP2040 / RP2350 facts relevant to PIO capture

| | RP2040 | RP2350A | RP2350B |
|---|---|---|---|
| PIO blocks / state machines | 2 / 8 | **3 / 12** | **3 / 12** |
| Max sysclk (datasheet) | 133 MHz (200 MHz qualified later in SDK 2.1.1; not checked here) | **150 MHz** | **150 MHz** |
| Package / GPIO | QFN-56, 30 GPIO | **QFN-60, 30 GPIO** | **QFN-80, 48 GPIO** |
| IOVDD | 1.8-3.3 V | 1.8-3.3 V | 1.8-3.3 V |
| Price (Raspberry Pi, 2025-03-17: single / 500 reel / big reel) | $1.00 / $0.80 / $0.70 | $1.10 / $0.90 / $0.80 | $1.20 / $1.00 / $0.90 |
| JLC/LCSC @1k (2026-10-26) | $0.73 (C2040, 71k) | $1.10 (C42411118, 5760) | $1.09 (C42415655, 3546) |

Sources: RP2350 product brief (pip-assets.raspberrypi.com, RP-008374-DS-1) and raspberrypi.com/news "RP2350 now available to buy" (2025-03-17).

PIO feasibility for parallel capture:
- **Clock and sample**: a 2-instruction loop (`in pins, N side 0 [d]` / `nop side 1 [d]`) generates the ADC clock and samples the bus.
- **Clock rates**: at 150 MHz, integer divisions give 18.75 Msps (8 cycles), 15 Msps (10) or 10 Msps (15). A 12 Msps clock needs 12.5 cycles; use the fractional divider (adds jitter) or retune sysclk.
- **Data rate**: about 20 MB/s at 20 Msps × 8 bit packed 4 samples per word, or 40 MB/s for 10-bit in 16-bit slots. Both are easy for DMA.
- **Real bottleneck**: CPU demodulation of every sample, not capture.
- **Pin budget**: 9 to 11 *consecutive* GPIOs is the main cost. That argues for the RP2350B (48 GPIO) if the 1090U's other peripherals already use most of the RP2040/RP2350A's 30 GPIOs.

## 6. Recommendation

1. **Best 8-bit:** TI **ADC08100CIMTCX**: active, 3 V with native 3.3 V I/O, TSSOP-24, **9 MCU pins** (8 data + clock), **$3.95 @1k at LCSC with 1308 in stock**. It must run at ≥20 Msps (e.g. 18.75-25 Msps from PIO, verify the minimum), and its ladder reference can be set to about 0.4-2.0 V to DC-couple the AD8313 output.
2. **Best 10-bit:** Ruimeng **MS9280** (10-bit, 35 Msps, 2.7-5.5 V, SSOP-28): **11 MCU pins**, **$0.80 @1k, 842 in stock at LCSC**. It is the only 10-bit part inside a $1-4 adder. Pinout, power and quality are unverified, so prototype it against the AD9200 footprint. The western fallback is AD9200ARSZ (ADI list $5.99) or AD9203 ($8.98).
3. Under 15 Msps, TI ADC1173 (8-bit, 33 mW) fits perfectly but has no LCSC stock at the $0.93 price. Avoid 5 V parts (TLC5510/5540, ADS830) and serial SARs (≤4-5 Msps).
4. Budget about $0.3 for a unity-gain buffer between the AD8313 (about 400 uA drive) and any pipeline or ladder ADC input (est.). A parallel ADC realistically wants the RP2350B (48 GPIO).
5. Lowest cost: add 2 comparators (LMV7219 at $0.36 each, or a TLV3502 dual at $1.29) next to the MCP6566 to get a 2-bit flash on 3 GPIOs for under about $1.3.
