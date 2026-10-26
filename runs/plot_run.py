#!/usr/bin/env python3
"""
plot_run.py - plot speed and acceleration from a M.I.S.L. run log (RUNnnn.CSV).

Usage:
    python plot_run.py RUN001.CSV
    python plot_run.py RUN001.CSV --window 0.5 --save run1.png
    python plot_run.py RUN001.CSV --inches-per-spoke 4.48     # after a roll-out calibration
    python plot_run.py RUN001.CSV --cut 0 30                   # also remove 0-30 s by hand
    python plot_run.py RUN001.CSV --no-clean                   # show everything, unfiltered

Needs numpy and matplotlib:   pip install numpy matplotlib

How it works
  * Every CSV row is one spoke, so spoke number x inches-per-spoke is the distance rolled.
  * Rows flagged used=0 (the low-speed double-trigger filter) are dropped by default so they
    don't count as distance. Use --keep-all to keep them.
  * Automatic clean-up removes two kinds of non-wheel events (and says exactly what it removed;
    removed time ranges are shaded gray on the plot):
      - chatter: a long run of fast, irregular spoke intervals. A real wheel's intervals change
        by about 1% from spoke to spoke; chatter (e.g. a spoke edge sitting right at the sensor's
        threshold) varies by 20-30%.
      - report-locked events: spokes arriving exactly every 250 ms (the firmware's display refresh
        interval) over and over, which no real wheel does.
  * Speed and acceleration come from a sliding quadratic fit of distance vs time
    (like a Savitzky-Golay filter, but for unevenly spaced samples). One fit gives both:
    d(t) ~ c0 + c1*t + c2*t^2, so speed = c1 and acceleration = 2*c2. This is much
    cleaner than differencing neighboring spokes, whose timestamps are quantized to 0.2 ms.
  * Gaps longer than --gap seconds (buggy stopped, or a removed section) split the run, and
    fits never cross them.
"""
import argparse
import csv
import math
import sys

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

IN_PER_SPOKE_DEFAULT = math.pi * 7.2 / 5   # 7.2 in wheel, 5 spokes -> 4.52 in
IPS_PER_MPH = 17.6                         # inches per second in 1 mph
G_IPS2 = 386.089                           # inches per second^2 in 1 g


def load(path, keep_all):
    """Return (times in s, number of rows dropped as used=0, number of malformed rows)."""
    times, used = [], []
    bad = 0
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or "time_ms" not in reader.fieldnames:
            sys.exit("This doesn't look like a M.I.S.L. log (no 'time_ms' column).")
        has_used = "used" in reader.fieldnames
        for row in reader:
            try:
                t = float(row["time_ms"]) / 1000.0
                if has_used:
                    flag = row["used"]
                    if flag not in ("0", "1"):      # truncated last line after a power loss, etc.
                        raise ValueError
                    used.append(int(flag))
                else:
                    used.append(1)
                times.append(t)
            except (KeyError, TypeError, ValueError):
                bad += 1
    t = np.array(times)
    used = np.array(used)
    dropped = 0 if keep_all else int(np.sum(used == 0))
    if not keep_all:
        t = t[used == 1]
    return t, dropped, bad


def _big_clusters(mask, t, min_events, pad_s, join=25):
    """Keep only clusters of flagged events with at least min_events members.
    Each kept cluster also takes in events within pad_s seconds of its edges."""
    out = np.zeros_like(mask)
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return out
    for g in np.split(idx, np.flatnonzero(np.diff(idx) > join) + 1):
        if len(g) >= min_events:
            out[(t >= t[g[0]] - pad_s) & (t <= t[g[-1]] + pad_s)] = True
    return out


def find_chatter(t, window=6, cv_limit=0.08, fast_ms=40.0, min_events=20, pad_s=0.1):
    """Flag long stretches of fast, irregular spoke intervals (not a real wheel)."""
    n = len(t)
    mask = np.zeros(n, dtype=bool)
    if n < window + 2:
        return mask
    dt = np.diff(t) * 1000.0
    w = sliding_window_view(dt, window)
    mean = w.mean(axis=1)
    cv = w.std(axis=1) / mean
    for j in np.flatnonzero((cv > cv_limit) & (mean < fast_ms)):
        mask[j: j + window + 1] = True
    return _big_clusters(mask, t, min_events, pad_s)


def find_report_locked(t, report_ms=250.0, tol_ms=3.0, min_run=4, max_sd_ms=1.5):
    """Flag runs of spokes arriving every report_ms (display refresh) almost exactly."""
    n = len(t)
    mask = np.zeros(n, dtype=bool)
    if n < min_run + 1:
        return mask
    dt = np.diff(t) * 1000.0
    near = np.abs(dt - report_ms) <= tol_ms
    i = 0
    while i < len(near):
        if not near[i]:
            i += 1
            continue
        j = i
        while j < len(near) and near[j]:
            j += 1
        run = dt[i:j]
        if len(run) >= min_run and run.std() <= max_sd_ms and abs(run.mean() - report_ms) <= 1.5:
            mask[i: j + 1] = True
        i = j
    return mask


def spans(t, mask, join=2.0):
    """Contiguous time spans (start, end, count) covered by flagged events."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    out = []
    for g in np.split(idx, np.flatnonzero(np.diff(t[idx]) > join) + 1):
        out.append((t[g[0]], t[g[-1]], len(g)))
    return out


def local_fit(t, x, window, gap, min_pts=7):
    """Sliding quadratic fit of position x(t). Returns speed (in/s) and accel (in/s^2) arrays."""
    n = len(t)
    v = np.full(n, np.nan)
    a = np.full(n, np.nan)
    starts = np.flatnonzero(np.r_[True, np.diff(t) > gap])    # first index of each segment
    ends = np.r_[starts[1:], n]                                # one past the last index
    seg = np.searchsorted(starts, np.arange(n), side="right") - 1
    half = window / 2.0
    for i in range(n):
        s_lo, s_hi = starts[seg[i]], ends[seg[i]]
        if s_hi - s_lo < min_pts:
            continue                                           # segment too short to fit
        lo = max(s_lo, np.searchsorted(t, t[i] - half, side="left"))
        hi = min(s_hi, np.searchsorted(t, t[i] + half, side="right"))
        if hi - lo < min_pts:                                  # slow: widen to min_pts points
            lo = max(s_lo, min(i - min_pts // 2, s_hi - min_pts))
            hi = lo + min_pts
        tr = t[lo:hi] - t[i]
        xr = x[lo:hi] - x[i]
        c2, c1, _ = np.polyfit(tr, xr, 2)
        v[i] = c1
        if i - lo >= 3 and hi - 1 - i >= 3:      # acceleration needs points on both sides
            a[i] = 2.0 * c2
    return v, a


def main():
    p = argparse.ArgumentParser(description="Plot speed and acceleration from a M.I.S.L. CSV log.")
    p.add_argument("csv_file", help="run log, e.g. RUN001.CSV")
    p.add_argument("--inches-per-spoke", type=float, default=IN_PER_SPOKE_DEFAULT,
                   help="rim travel per spoke (default %.3f = pi x 7.2 in / 5)" % IN_PER_SPOKE_DEFAULT)
    p.add_argument("--window", type=float, default=0.25,
                   help="smoothing window in seconds (default 0.25). Shorter (0.15-0.2) follows sharp "
                        "pushes but is noisier; longer (0.5-1.0) gives smoother curves but flattens peaks")
    p.add_argument("--gap", type=float, default=1.0,
                   help="a pause longer than this many seconds splits the run (default 1.0)")
    p.add_argument("--keep-all", action="store_true", help="keep rows flagged used=0")
    p.add_argument("--no-clean", action="store_true", help="don't auto-remove chatter / report-locked events")
    p.add_argument("--cut", nargs=2, type=float, action="append", metavar=("START", "END"),
                   help="also remove all spokes between START and END seconds (repeatable)")
    p.add_argument("--report-ms", type=float, default=250.0,
                   help="firmware display-refresh interval for the report-locked check (default 250)")
    p.add_argument("--save", metavar="FILE", help="also save the figure (e.g. run1.png)")
    p.add_argument("--no-show", action="store_true", help="don't open a window (use with --save)")
    args = p.parse_args()

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t_all, dropped, bad = load(args.csv_file, args.keep_all)
    if len(t_all) < 8:
        sys.exit("Only %d usable spoke rows - not enough to plot." % len(t_all))

    # ---- remove events that aren't the wheel ----
    removed = np.zeros(len(t_all), dtype=bool)
    notes = []
    if not args.no_clean:
        m = find_chatter(t_all)
        if m.any():
            notes.append(("chatter (fast, irregular intervals)", m))
        m = find_report_locked(t_all, args.report_ms)
        if m.any():
            notes.append(("events locked to the %.0f ms display refresh" % args.report_ms, m))
    for a_, b_ in (args.cut or []):
        m = (t_all >= a_) & (t_all <= b_)
        if m.any():
            notes.append(("manual cut %.1f-%.1f s" % (a_, b_), m))
    for _, m in notes:
        removed |= m
    t = t_all[~removed]
    if len(t) < 8:
        sys.exit("Fewer than 8 spokes left after clean-up - try --no-clean.")

    x = np.arange(len(t)) * args.inches_per_spoke          # distance rolled, inches
    v_ips, a_ips2 = local_fit(t, x, args.window, args.gap)
    v_mph = v_ips / IPS_PER_MPH
    a_g = a_ips2 / G_IPS2

    # raw per-spoke speed for reference (noisy, quantized to 0.2 ms)
    dt = np.diff(t)
    raw_mph = args.inches_per_spoke / dt / IPS_PER_MPH
    raw_mph[dt > args.gap] = np.nan
    t_mid = (t[1:] + t[:-1]) / 2.0

    ok = ~np.isnan(v_mph)
    if not ok.any():
        sys.exit("Spokes are too far apart (or the run too short) to fit speed. Try --gap with a larger value.")

    # break the plotted lines across pauses instead of drawing through them
    brk = np.flatnonzero(np.diff(t) > args.gap) + 1
    tb = np.insert(t, brk, np.nan)
    vb = np.insert(v_mph, brk, np.nan)
    ab = np.insert(a_g, brk, np.nan)

    # ---- console summary ----
    top_i = int(np.nanargmax(v_mph))
    print("File:            %s" % args.csv_file)
    print("Spokes used:     %d  (dropped used=0: %d, malformed lines skipped: %d)" % (len(t), dropped, bad))
    for label, m in notes:
        sp = spans(t_all, m)
        where = ", ".join("%.1f-%.1f s" % (s, e) for s, e, _ in sp[:4]) + (" ..." if len(sp) > 4 else "")
        print("Removed:         %d events - %s  [%s]" % (int(m.sum()), label, where))
    print("Duration:        %.1f s   Distance: %.1f ft" % (t[-1] - t[0], x[-1] / 12.0))
    print("Top speed:       %.1f mph at t = %.2f s" % (v_mph[top_i], t[top_i]))
    if np.isnan(a_g).all():
        print("Peak accel:      n/a (too few spokes per window)")
    else:
        print("Peak accel:      %+.2f g    Peak decel: %+.2f g" % (np.nanmax(a_g), np.nanmin(a_g)))
    print("Inches/spoke:    %.3f   Smoothing window: %.2f s" % (args.inches_per_spoke, args.window))

    # ---- plot ----
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    first = True
    for label, m in notes:
        for s, e, _ in spans(t_all, m):
            for ax in (ax1, ax2):
                ax.axvspan(s, e, color="gray", alpha=0.18, lw=0,
                           label="removed (not the wheel)" if (first and ax is ax1) else None)
            first = False
    ax1.plot(t_mid, raw_mph, ".", ms=2, alpha=0.35, color="tab:gray", label="per-spoke (noisy)")
    ax1.plot(tb, vb, lw=1.8, color="tab:blue", label="smoothed (%.2f s fit)" % args.window)
    ax1.annotate("top %.1f mph" % v_mph[top_i], (t[top_i], v_mph[top_i]),
                 xytext=(10, 12), textcoords="offset points", arrowprops=dict(arrowstyle="->"))
    ax1.set_ylabel("Speed (mph)")
    ax1.set_ylim(0, v_mph[top_i] * 1.2)
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right")
    ax1.set_title(args.csv_file)

    ax2.axhline(0, color="k", lw=0.8)
    ax2.plot(tb, ab, lw=1.5, color="tab:red")
    ax2.fill_between(t, 0, np.nan_to_num(a_g), where=~np.isnan(a_g), alpha=0.15, color="tab:red")
    ax2.set_ylabel("Acceleration (g)")
    ax2.set_xlabel("Time since recording started (s)")
    ax2.grid(alpha=0.3)

    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=150)
        print("Saved:           %s" % args.save)
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()