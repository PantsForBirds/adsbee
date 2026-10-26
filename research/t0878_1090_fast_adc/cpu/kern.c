// Amplitude demod kernels for the CPU budget estimate (t-0878). 8-bit ADC samples at 10 Msps, 5 samples per chip.
#include <stdint.h>
#define SPC 5
// Running chip sums: C[t] = sum of x[t..t+SPC-1]; built incrementally (1 add, 1 sub per sample).
void chip_sums(const uint8_t* x, uint16_t* C, int n) {
    uint32_t s = x[0] + x[1] + x[2] + x[3] + x[4];
    for (int t = 0; t < n; t++) { C[t] = (uint16_t)s; s += x[t + SPC] - x[t]; }
}
// Preamble scan on chip sums: pulses at chips 0,2,7,9, quiet chips 4,5,11..14. Evaluated every `step` samples.
// Early exit on the first pulse chip vs the noise level estimate; returns number of candidates written.
int preamble_scan(const uint16_t* C, int n, int step, int noise5, int thr5, int* cand, int max_cand) {
    int nc = 0;
    for (int t = 0; t < n; t += step) {
        int p0 = C[t];
        if (p0 < noise5 + thr5) continue;                 // most samples stop here
        int p2 = C[t + 2 * SPC]; if (p2 < noise5 + thr5) continue;
        int p7 = C[t + 7 * SPC], p9 = C[t + 9 * SPC];
        int q = C[t + 4 * SPC] + C[t + 5 * SPC] + C[t + 11 * SPC] + C[t + 12 * SPC] + C[t + 13 * SPC] + C[t + 14 * SPC];
        int pmin = p0 < p2 ? p0 : p2; pmin = pmin < p7 ? pmin : p7; pmin = pmin < p9 ? pmin : p9;
        if (6 * pmin - q > 6 * thr5) { if (nc < max_cand) cand[nc++] = t; t += SPC; }
    }
    return nc;
}
// Bit decisions for nbits from data start o: margin = chip1 - chip2.
void demod_bits(const uint16_t* C, int o, int nbits, int16_t* m, uint8_t* bytes) {
    for (int i = 0; i < nbits; i += 8) {
        uint32_t b = 0;
        for (int k = 0; k < 8; k++) {
            int a = o + 2 * SPC * (i + k);
            int d = (int)C[a] - (int)C[a + SPC];
            m[i + k] = (int16_t)d; b = (b << 1) | (d > 0);
        }
        bytes[i >> 3] = (uint8_t)b;
    }
}
// Timing search: confidence of the first 12 bits at offsets lo..hi.
int timing(const uint16_t* C, int lo, int hi) {
    int best = lo, bc = -1;
    for (int o = lo; o <= hi; o++) {
        int c = 0;
        for (int i = 0; i < 12; i++) { int a = o + 2 * SPC * i; int d = (int)C[a] - (int)C[a + SPC]; c += d < 0 ? -d : d; }
        if (c > bc) { bc = c; best = o; }
    }
    return best;
}
