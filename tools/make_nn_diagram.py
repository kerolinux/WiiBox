#!/usr/bin/env python3
"""Render docs/nn_diagram.svg from source/weights.h (re-run after exporting new weights).

    python tools/make_nn_diagram.py
"""
import os, re

ROOT = os.path.join(os.path.dirname(__file__), "..")
src = open(os.path.join(ROOT, "source", "weights.h")).read()


def grab(name):
    m = re.search(r"static const float " + name + r"\[[^=]*\]\s*=\s*\{(.*?)\};", src, re.S)
    return [float(n) for n in re.findall(r"-?\d+(?:\.\d+)?(?:e[-+]?\d+)?(?=f)", m.group(1))]


IN, H, OUT = 5, 8, 2
W1, W2 = grab("W1"), grab("W2")
w1 = [[W1[j * IN + i] for i in range(IN)] for j in range(H)]
w2 = [[W2[k * H + j] for j in range(H)] for k in range(OUT)]
m1 = max(abs(v) for r in w1 for v in r) or 1
m2 = max(abs(v) for r in w2 for v in r) or 1

W, HT = 900, 640
xi, xh, xo = 200, 450, 700
yi = [150 + i * 64 for i in range(IN)]
yh = [118 + j * 42 for j in range(H)]
yo = [225, 295]

POS, NEG = "#1f5f99", "#b5482f"      # muted blue / brick red
INK, MUTE, RULE = "#222222", "#666666", "#cccccc"
FONT = "Helvetica, Arial, sans-serif"

o = []
a = o.append
a(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {HT}" width="{W}" height="{HT}" font-family="{FONT}">')
a('<defs><marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">'
  f'<path d="M0 0L10 5L0 10z" fill="{MUTE}"/></marker></defs>')
a(f'<rect width="{W}" height="{HT}" fill="#ffffff"/>')

# title
a(f'<text x="32" y="38" font-size="20" font-weight="bold" fill="{INK}">Neural network controller: MLP 5-8-2</text>')
a(f'<line x1="32" y1="50" x2="{W-32}" y2="50" stroke="{RULE}" stroke-width="1"/>')

# layer headers
for x, t, s in ((xi, "Input layer", "5 neurons"), (xh, "Hidden layer", "8 neurons, tanh"), (xo, "Output layer", "2 neurons, tanh")):
    a(f'<text x="{x}" y="84" font-size="14" font-weight="bold" fill="{INK}" text-anchor="middle">{t}</text>')
    a(f'<text x="{x}" y="101" font-size="12" fill="{MUTE}" text-anchor="middle">{s}</text>')


def edge(x0, y0, x1, y1, w, mx):
    if abs(w) < 0.05:
        return
    n = min(abs(w) / mx, 1.0)
    a(f'<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y1}" stroke="{POS if w > 0 else NEG}" '
      f'stroke-width="{0.5 + 2.0 * n:.2f}" stroke-opacity="{0.25 + 0.65 * n:.2f}"/>')


for j in range(H):
    for i in range(IN):
        edge(xi, yi[i], xh, yh[j], w1[j][i], m1)
for k in range(OUT):
    for j in range(H):
        edge(xh, yh[j], xo, yo[k], w2[k][j], m2)


def node(x, y, r, label, size):
    a(f'<circle cx="{x}" cy="{y}" r="{r}" fill="#ffffff" stroke="{INK}" stroke-width="1.6"/>')
    a(f'<text x="{x}" y="{y + size * 0.35:.1f}" font-size="{size}" fill="{INK}" text-anchor="middle" font-style="italic">{label}</text>')


in_lbl = ["distance at -45°", "distance at -22.5°", "distance at 0°", "distance at +22.5°", "distance at +45°"]
for i in range(IN):
    node(xi, yi[i], 17, f"x{i+1}", 13)
    a(f'<text x="{xi-30}" y="{yi[i]+4}" font-size="12" fill="{MUTE}" text-anchor="end">{in_lbl[i]}</text>')
for j in range(H):
    node(xh, yh[j], 15, f"h{j+1}", 12)
for k, t in enumerate(("left wheel command", "right wheel command")):
    node(xo, yo[k], 19, f"y{k+1}", 14)
    a(f'<text x="{xo+32}" y="{yo[k]+4}" font-size="12" fill="{MUTE}">{t}</text>')

a(f'<text x="{xi}" y="{yi[-1]+50}" font-size="11.5" fill="{MUTE}" text-anchor="middle">x = ray distance / 4 m, in [0, 1]</text>')
a(f'<text x="{xo}" y="{yo[1]+50}" font-size="11.5" fill="{MUTE}" text-anchor="middle">y in [-1, 1]</text>')

# legend
lx, ly = 590, 400
a(f'<line x1="{lx}" y1="{ly}" x2="{lx+34}" y2="{ly}" stroke="{POS}" stroke-width="2.2"/><text x="{lx+42}" y="{ly+4}" font-size="11.5" fill="{MUTE}">positive weight</text>')
a(f'<line x1="{lx+150}" y1="{ly}" x2="{lx+184}" y2="{ly}" stroke="{NEG}" stroke-width="2.2"/><text x="{lx+192}" y="{ly+4}" font-size="11.5" fill="{MUTE}">negative weight</text>')
a(f'<text x="{lx}" y="{ly+20}" font-size="11.5" fill="{MUTE}">Line weight is proportional to |w|.</text>')

# equations
a(f'<text x="32" y="{HT-150}" font-size="13" fill="{INK}" font-style="italic">h = tanh(W₁x + b₁),    y = tanh(W₂h + b₂)</text>')
a(f'<text x="32" y="{HT-132}" font-size="11.5" fill="{MUTE}">W₁ is 8×5, W₂ is 2×8; constant arrays in source/weights.h, evaluated by nn.h.</text>')

# pipeline
py = HT - 100
a(f'<line x1="32" y1="{py-24}" x2="{W-32}" y2="{py-24}" stroke="{RULE}" stroke-width="1"/>')
a(f'<text x="32" y="{py-6}" font-size="13" font-weight="bold" fill="{INK}">Control pipeline (every frame, 60 Hz)</text>')
steps = [("Raycast sensors", "5 rays, 4 m range"), ("Normalise", "x = d / 4 m"), ("MLP 5-8-2", "nn.h"),
         ("Path planner blend", "when a goal is set"), ("Command shaper", "damping, smoothing"), ("Wheel actuators", "Box2D forces")]
bw, gap = 124, 25
x = 32
for n, (t1, t2) in enumerate(steps):
    a(f'<rect x="{x}" y="{py+6}" width="{bw}" height="56" fill="#f6f6f6" stroke="#555555" stroke-width="1.2"/>')
    a(f'<text x="{x+bw/2}" y="{py+28}" font-size="12" font-weight="bold" fill="{INK}" text-anchor="middle">{t1}</text>')
    a(f'<text x="{x+bw/2}" y="{py+46}" font-size="11" fill="{MUTE}" text-anchor="middle">{t2}</text>')
    if n < len(steps) - 1:
        a(f'<line x1="{x+bw+2}" y1="{py+34}" x2="{x+bw+gap-3}" y2="{py+34}" stroke="{MUTE}" stroke-width="1.5" marker-end="url(#ar)"/>')
    x += bw + gap

a('</svg>')
path = os.path.join(ROOT, "docs", "nn_diagram.svg")
open(path, "w").write("\n".join(o))
print("wrote", os.path.normpath(path))
