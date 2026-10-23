#!/usr/bin/env python3
"""Render docs/nn_diagram.svg from source/weights.h (re-run after exporting new weights).

    python tools/make_nn_diagram.py
"""
import os, re

ROOT = os.path.join(os.path.dirname(__file__), "..")
src = open(os.path.join(ROOT, "source", "weights.h")).read()

def grab(name):
    m = re.search(r"static const float " + name + r"\[[^=]*\]\s*=\s*\{(.*?)\};", src, re.S)
    nums = re.findall(r"-?\d+(?:\.\d+)?(?:e[-+]?\d+)?(?=f)", m.group(1))
    return [float(n) for n in nums]

IN, H, OUT = 5, 8, 2
W1 = grab("W1"); B1 = grab("B1"); W2 = grab("W2"); B2 = grab("B2")
w1 = [[W1[j * IN + i] for i in range(IN)] for j in range(H)]
w2 = [[W2[k * H + j] for j in range(H)] for k in range(OUT)]

W, Hh = 920, 600
xi, xh, xo = 190, 470, 750
yi = [150 + i * 62 for i in range(IN)]
yh = [112 + j * 41 for j in range(H)]
yo = [215, 285]
m1 = max(abs(v) for row in w1 for v in row) or 1
m2 = max(abs(v) for row in w2 for v in row) or 1

BLUE, RED, NAVY, TEXT = "#2fa3e6", "#e0524a", "#145e92", "#56677a"
out = []
a = out.append
a(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {Hh}" width="{W}" height="{Hh}" font-family="Verdana, Arial, sans-serif">')
a('<defs><linearGradient id="bg" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#f7fbff"/><stop offset="1" stop-color="#cde4f5"/></linearGradient>'
  '<linearGradient id="hd" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#62cdf8"/><stop offset="1" stop-color="#1c8cd4"/></linearGradient>'
  '<marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0 0L10 5L0 10z" fill="#8aa3b8"/></marker></defs>')
a(f'<rect width="{W}" height="{Hh}" rx="18" fill="url(#bg)"/>')
a(f'<rect width="{W}" height="54" rx="18" fill="url(#hd)"/><rect y="36" width="{W}" height="18" fill="url(#hd)"/>')
a(f'<text x="28" y="35" font-size="22" font-weight="bold" fill="#fff">WiiBox neural controller: MLP 5 → 8 → 2 (tanh)</text>')

# column headers
for x, t in ((xi, "INPUTS"), (xh, "HIDDEN (tanh)"), (xo, "OUTPUTS (tanh)")):
    a(f'<text x="{x}" y="88" font-size="14" font-weight="bold" fill="{NAVY}" text-anchor="middle">{t}</text>')

# edges: input -> hidden, hidden -> output (colour = sign, width/opacity = magnitude)
def edge(x0, y0, x1, y1, w, mx):
    if abs(w) < 0.05: return
    n = min(abs(w) / mx, 1.0)
    a(f'<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y1}" stroke="{BLUE if w > 0 else RED}" '
      f'stroke-width="{0.6 + 2.6 * n:.2f}" stroke-opacity="{0.18 + 0.72 * n:.2f}" stroke-linecap="round"/>')
for j in range(H):
    for i in range(IN): edge(xi, yi[i], xh, yh[j], w1[j][i], m1)
for k in range(OUT):
    for j in range(H): edge(xh, yh[j], xo, yo[k], w2[k][j], m2)

# nodes
labels = ["R45  (-45°)", "R22  (-22.5°)", "FWD  (0°)", "L22  (+22.5°)", "L45  (+45°)"]
for i in range(IN):
    a(f'<circle cx="{xi}" cy="{yi[i]}" r="16" fill="#e4f3fd" stroke="{BLUE}" stroke-width="3"/>')
    a(f'<text x="{xi}" y="{yi[i]+5}" font-size="13" font-weight="bold" fill="{NAVY}" text-anchor="middle">s{i}</text>')
    a(f'<text x="{xi-28}" y="{yi[i]+5}" font-size="13" fill="{TEXT}" text-anchor="end">{labels[i]}</text>')
for j in range(H):
    a(f'<circle cx="{xh}" cy="{yh[j]}" r="14" fill="#ffffff" stroke="{NAVY}" stroke-width="3"/>')
    a(f'<text x="{xh}" y="{yh[j]+4}" font-size="12" font-weight="bold" fill="{NAVY}" text-anchor="middle">h{j}</text>')
for k, (n, c) in enumerate((("LEFT wheel", "#46c467"), ("RIGHT wheel", "#ff9f1f"))):
    a(f'<circle cx="{xo}" cy="{yo[k]}" r="18" fill="{c}" stroke="#fff" stroke-width="3"/>')
    a(f'<text x="{xo}" y="{yo[k]+5}" font-size="14" font-weight="bold" fill="#fff" text-anchor="middle">{"L" if k == 0 else "R"}</text>')
    a(f'<text x="{xo+30}" y="{yo[k]+5}" font-size="13" fill="{TEXT}">{n}</text>')
a(f'<text x="{xo+30}" y="{yo[0]+22}" font-size="11" fill="{TEXT}">command in [-1, 1]</text>')

# input note + legend
a(f'<text x="{xi}" y="{yi[-1]+46}" font-size="12" fill="{TEXT}" text-anchor="middle">s = ray distance / 4 m  (1 = clear, 0 = touching)</text>')
lx, ly = 650, 130
a(f'<rect x="{lx}" y="{ly+170}" width="236" height="62" rx="10" fill="#fff" stroke="#d0e2f0"/>')
a(f'<line x1="{lx+14}" y1="{ly+190}" x2="{lx+54}" y2="{ly+190}" stroke="{BLUE}" stroke-width="3.5"/><text x="{lx+62}" y="{ly+194}" font-size="12" fill="{TEXT}">positive weight</text>')
a(f'<line x1="{lx+14}" y1="{ly+212}" x2="{lx+54}" y2="{ly+212}" stroke="{RED}" stroke-width="3.5"/><text x="{lx+62}" y="{ly+216}" font-size="12" fill="{TEXT}">negative weight (thicker = stronger)</text>')

# pipeline strip
py = 468
a(f'<text x="28" y="{py-12}" font-size="14" font-weight="bold" fill="{NAVY}">Control pipeline (every frame, 60 Hz)</text>')
steps = ["5 raycasts\n(Box2D)", "normalise\ns = d / 4 m", "MLP 5-8-2\n(nn.h)", "blend with\npath planner", "command\nshaper", "wheel servos\n(Box2D forces)"]
bw, gap = 128, 22
x = 28
for n, t in enumerate(steps):
    a(f'<rect x="{x}" y="{py}" width="{bw}" height="64" rx="12" fill="#fff" stroke="{BLUE}" stroke-width="2.5"/>')
    l1, l2 = t.split("\n")
    a(f'<text x="{x+bw/2}" y="{py+28}" font-size="12.5" font-weight="bold" fill="{NAVY}" text-anchor="middle">{l1}</text>')
    a(f'<text x="{x+bw/2}" y="{py+46}" font-size="11.5" fill="{TEXT}" text-anchor="middle">{l2}</text>')
    if n < len(steps) - 1:
        a(f'<line x1="{x+bw+2}" y1="{py+32}" x2="{x+bw+gap-3}" y2="{py+32}" stroke="#8aa3b8" stroke-width="2.5" marker-end="url(#ar)"/>')
    x += bw + gap

a(f'<text x="{W/2}" y="{Hh-14}" font-size="11" fill="{TEXT}" text-anchor="middle">Edge colours show the weights currently in source/weights.h. Regenerate with tools/make_nn_diagram.py.</text>')
a('</svg>')

path = os.path.join(ROOT, "docs", "nn_diagram.svg")
open(path, "w").write("\n".join(out))
print("wrote", os.path.normpath(path))
