"""Friis cascade for the 1090U RX chain (t-0877). Values at 1090 MHz; sources in the report table.
Run: python cascade.py -> prints table and writes ../cascade.json"""
import json, math

KT = -174.0


def db2lin(x):
    return 10 ** (x / 10)


# name, gain dB (negative = loss), NF dB (passive: NF = loss), BW note, source
STAGES = [
    ("U.FL J1 + 50 ohm trace", -0.10, None, "-", "est. Hirose U.FL ~0.1 dB at 1 GHz + ~5 mm trace"),
    ("ESD7351 (0.6 pF shunt)", -0.05, None, "-", "mismatch of 0.6 pF at 1090 MHz: |S21|^2 = -0.05 dB"),
    ("Bias tee L1 100 nH + C12 100 pF", -0.05, None, "-", "|Z_L1| > 600 ohm shunt, C12 Xc = 1.5 ohm"),
    ("U2 BLB01 @3.3 V", 18.6, 0.55, "0.5-1.5 GHz", "BLB01 rev 6.5, Vd=3 V table, interp. 900/1500 MHz"),
    ("X1 TFS1090F", -1.58, None, "27.5 MHz (-3 dB)", "TFS1090F vector plot (t-0061 results.json)"),
    ("U3 BLB01 @3.3 V", 18.6, 0.55, "0.5-1.5 GHz", "as U2"),
    ("X2 TFS1090F", -1.58, None, "cascade 23.7 MHz", "as X1"),
    ("TJ1 MM8930-2620 + traces", -0.20, None, "-", "est."),
    ("C11/C18/L6 match", 8.0, None, "narrow", "voltage gain into ~300 ohm input, est. (AD8313 Table 5: 9 dB theor. at 900 MHz)"),
]
# AD8313 input noise: 0.6 nV/rtHz (datasheet p.11) = -171.4 dBm/Hz re 50 ohm, over ~3.5 GHz (-76 dBm).
AD8313_NOISE_DBM_HZ = -171.4
ENBW_HZ = 21.9e6     # SAW cascade ENBW (sim.enbw('tfs2'))


def run():
    rows = []
    f_tot = 1.0
    g_tot = 1.0
    for name, g, nf, bw, src in STAGES:
        if name.startswith("C11"):
            break
        f = db2lin(nf if nf is not None else -g)
        f_tot += (f - 1) / g_tot
        g_tot *= db2lin(g)
        rows.append(dict(stage=name, gain=g, nf=nf if nf is not None else -g, bw=bw, src=src,
                         cum_gain=10 * math.log10(g_tot), cum_nf=10 * math.log10(f_tot)))
    nf_fe = 10 * math.log10(f_tot)
    g_fe = 10 * math.log10(g_tot)
    match = STAGES[-1][1]
    # detector noise referred to the antenna, as an equivalent density over its own ~3.5 GHz
    det_ant = AD8313_NOISE_DBM_HZ - match - g_fe
    # in-band equivalent: the detector sees its noise over 3.5 GHz; as total power
    det_tot_ant = det_ant + 10 * math.log10(3.5e9)
    fe_noise_ant = KT + nf_fe + 10 * math.log10(ENBW_HZ)
    out = dict(rows=rows, nf_fe=nf_fe, gain_fe=g_fe, match_gain=match, det_noise_ant_dbm=det_tot_ant,
               fe_noise_ant_dbm=fe_noise_ant,
               det_penalty_db=10 * math.log10(1 + db2lin(det_tot_ant - fe_noise_ant)),
               intercept_chip_dbm=-144.6 + g_fe + match)
    return out


if __name__ == "__main__":
    o = run()
    for r in o["rows"]:
        print("%-34s %6.2f %5.2f | cum G %6.2f NF %5.2f" % (r["stage"], r["gain"], r["nf"], r["cum_gain"], r["cum_nf"]))
    for k, v in o.items():
        if k != "rows":
            print(k, round(v, 2))
    json.dump(o, open("../cascade.json", "w"), indent=1)
