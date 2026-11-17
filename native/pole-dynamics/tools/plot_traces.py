#!/usr/bin/env python3
"""Plot Pole Dynamics traces (the JSON written by the tests or pdtrace) as SVG.

    python3 tools/plot_traces.py build/traces/*.json -o docs/traces

Two panels per trace: pole tilt (degrees) and the maximum wire tension per
span (kN), against time, with the events marked. No dependencies beyond the
standard library.
"""
import argparse
import json
import os
import sys

COLORS = ["#2a6fdb", "#d1495b", "#2e9e5b", "#e08a1e", "#7a4fc9", "#11939a", "#8a8f98", "#c4458f"]
W, PANEL_H, ML, MR, MT, GAP = 900, 230, 64, 150, 46, 56


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def nice_max(v):
    if v <= 0:
        return 1.0
    for e in range(-2, 6):
        for m in (1, 2, 2.5, 5):
            c = m * 10 ** e
            if c >= v:
                return c
    return v


def panel(out, y0, title, unit, t, series, labels, events, tmax):
    pw = W - ML - MR
    vmax = nice_max(max((max(s) for s in series if s), default=1.0) * 1.05)
    sx = lambda x: ML + pw * x / tmax
    sy = lambda v: y0 + PANEL_H - PANEL_H * v / vmax
    out.append(f'<text x="{ML}" y="{y0 - 10}" class="h">{esc(title)}</text>')
    for k in range(6):
        v = vmax * k / 5
        y = sy(v)
        out.append(f'<line x1="{ML}" x2="{ML + pw}" y1="{y:.1f}" y2="{y:.1f}" class="grid"/>')
        out.append(f'<text x="{ML - 8}" y="{y + 4:.1f}" class="tick" text-anchor="end">{v:g}</text>')
    step = 1 if tmax <= 10 else 2
    x = 0.0
    while x <= tmax + 1e-9:
        out.append(f'<text x="{sx(x):.1f}" y="{y0 + PANEL_H + 16}" class="tick" text-anchor="middle">{x:g}</text>')
        x += step
    out.append(f'<text x="{ML + pw / 2}" y="{y0 + PANEL_H + 34}" class="tick" text-anchor="middle">time (s)</text>')
    out.append(f'<text x="16" y="{y0 + PANEL_H / 2}" class="tick" transform="rotate(-90 16 {y0 + PANEL_H / 2})" text-anchor="middle">{esc(unit)}</text>')
    for et in events:
        out.append(f'<line x1="{sx(et):.1f}" x2="{sx(et):.1f}" y1="{y0}" y2="{y0 + PANEL_H}" class="ev"/>')
    for i, s in enumerate(series):
        if not s:
            continue
        c = COLORS[i % len(COLORS)]
        pts = " ".join(f"{sx(tt):.1f},{sy(v):.1f}" for tt, v in zip(t, s))
        out.append(f'<polyline points="{pts}" fill="none" stroke="{c}" stroke-width="1.8"/>')
        ly = y0 + 8 + i * 17
        out.append(f'<line x1="{W - MR + 14}" x2="{W - MR + 32}" y1="{ly}" y2="{ly}" stroke="{c}" stroke-width="3"/>')
        out.append(f'<text x="{W - MR + 38}" y="{ly + 4}" class="tick">{esc(labels[i])}</text>')


def plot(path, outdir):
    d = json.load(open(path))
    t = d["t"]
    tmax = max(t[-1], 1e-6)
    events = []
    for e in d.get("events", []):
        parts = e.split()
        if len(parts) >= 2 and parts[1] in ("POLE_SNAPPED", "WIRE_SNAPPED", "CLAMP_FAILED", "TRANSFORMER_DETACHED", "POLE_FELL"):
            events.append(float(parts[0]))
    H = MT + 2 * PANEL_H + GAP + 60 + 44
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="system-ui,sans-serif">',
           "<style>.h{font-size:14px;font-weight:600;fill:#1d2330}.tick{font-size:11px;fill:#5b6270}"
           ".grid{stroke:#e3e6eb;stroke-width:1}.ev{stroke:#d1495b;stroke-width:1;stroke-dasharray:3 3;opacity:.45}</style>",
           f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
           f'<text x="{ML}" y="24" style="font-size:17px;font-weight:700;fill:#1d2330">{esc(d.get("title", os.path.basename(path)))}</text>']
    panel(out, MT + 20, "Pole tilt", "degrees", t, d["tilt"], [f"pole {i}" for i in range(len(d["tilt"]))], events, tmax)
    panel(out, MT + 20 + PANEL_H + GAP + 20, "Max wire tension per span", "kN", t, d["tension"],
          [f"span {i}" for i in range(len(d["tension"]))], events, tmax)
    out.append(f'<text x="{ML}" y="{H - 8}" class="tick">dashed: snaps, broken wires, clamp failures, transformer detach</text>')
    out.append("</svg>")
    name = os.path.splitext(os.path.basename(path))[0] + ".svg"
    with open(os.path.join(outdir, name), "w") as f:
        f.write("\n".join(out))
    return name


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("traces", nargs="+")
    ap.add_argument("-o", "--outdir", default=".")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    for p in a.traces:
        print(plot(p, a.outdir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
