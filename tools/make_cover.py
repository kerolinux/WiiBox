#!/usr/bin/env python3
"""Generate the Homebrew Channel icon (apps/WiiBox/icon.png, 128x48) and a GitHub banner
(docs/cover.png, 1280x640).   pip install pillow   &&   python tools/make_cover.py
"""
import os, math
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.join(os.path.dirname(__file__), "..")
FONTS = ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
         "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
         "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf"]

def font(size):
    for p in FONTS:
        if os.path.exists(p):
            return ImageFont.truetype(p, int(size))
    return ImageFont.load_default()

BLUE, BLUE_DK, NAVY = (47, 163, 230), (22, 114, 176), (20, 94, 146)
WALL, WALL_HI = (59, 120, 189), (125, 180, 232)
TXT, WHITE = (86, 103, 122), (255, 255, 255)
GREEN, YELLOW, ORANGE, RED = (47, 191, 95), (242, 194, 0), (255, 138, 31), (240, 58, 58)

def gradient(w, h, top=(247, 251, 255), bot=(190, 221, 243)):
    img = Image.new("RGB", (w, h))
    px = img.load()
    for y in range(h):
        t = y / max(h - 1, 1)
        c = tuple(int(top[i] + (bot[i] - top[i]) * t) for i in range(3))
        for x in range(w):
            px[x, y] = c
    return img

def rover(d, cx, cy, L, W):
    """top-down rover facing east; L = length, W = width (pixels)."""
    wl, ww = L * 0.38, W * 0.16
    for side in (-1, 1):
        y0 = cy + side * W / 2
        d.rounded_rectangle([cx - wl / 2, min(y0, y0 + side * ww), cx + wl / 2, max(y0, y0 + side * ww)],
                            radius=ww / 3, fill=(60, 71, 82))
    d.rounded_rectangle([cx - L / 2, cy - W / 2, cx + L / 2, cy + W / 2], radius=W * 0.22, fill=BLUE_DK)
    m = W * 0.07
    d.rounded_rectangle([cx - L / 2 + m, cy - W / 2 + m, cx + L / 2 - m, cy + W / 2 - m], radius=W * 0.18, fill=BLUE)
    d.rectangle([cx + L / 2 - L * 0.17, cy - W / 2 + W * 0.18, cx + L / 2 - L * 0.07, cy + W / 2 - W * 0.18], fill=WHITE)
    for side in (-1, 1):
        ex, ey, r = cx + L * 0.12, cy + side * W * 0.2, W * 0.15
        d.ellipse([ex - r, ey - r, ex + r, ey + r], fill=WHITE)
        pr = r * 0.5
        d.ellipse([ex + r * 0.35 - pr, ey - pr, ex + r * 0.35 + pr, ey + pr], fill=(35, 48, 60))

def rays(d, x, y, lengths, colors, width):
    for ang, ln, col in zip((-45, -22.5, 0, 22.5, 45), lengths, colors):
        a = math.radians(ang)
        d.line([x, y, x + ln * math.cos(a), y - ln * math.sin(a)], fill=col, width=width)

# ---------------------------------------------------------------- icon 128x48
def make_icon():
    S = 6; W, H = 128 * S, 48 * S
    img = gradient(W, H).convert("RGBA")
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([3 * S, 3 * S, W - 3 * S, H - 3 * S], radius=9 * S, outline=BLUE, width=int(1.6 * S))
    rover(d, 22 * S, 24 * S, 28 * S, 19 * S)
    rays(d, 22 * S + 14 * S, 24 * S, [9 * S, 11 * S, 14 * S, 11 * S, 8 * S],
         [GREEN, GREEN, GREEN, YELLOW, ORANGE], int(1.4 * S))
    f = font(14 * S)
    d.text((58 * S + S, 17 * S + S), "WiiBox", font=f, fill=NAVY)       # shadow
    d.text((58 * S, 17 * S), "WiiBox", font=f, fill=BLUE)
    d.text((59 * S, 36 * S), "ROBOT LAB", font=font(6.0 * S), fill=TXT)
    out = img.resize((128, 48), Image.LANCZOS)
    path = os.path.join(ROOT, "apps", "WiiBox", "icon.png")
    out.save(path)
    print("wrote", os.path.normpath(path))

# ---------------------------------------------------------------- banner 1280x640
def make_cover():
    S = 2; W, H = 1280 * S, 640 * S
    img = gradient(W, H, (250, 252, 255), (181, 216, 241)).convert("RGBA")
    d = ImageDraw.Draw(img, "RGBA")
    for i in range(-H, W, 44 * S):                                    # diagonal pinstripes
        d.line([i, 0, i + H, H], fill=(255, 255, 255, 60), width=2 * S)
    for cx, cy, r in ((200, 560, 220), (1150, 70, 160)):              # soft bubbles
        d.ellipse([(cx - r) * S, (cy - r) * S, (cx + r) * S, (cy + r) * S], fill=(255, 255, 255, 55))

    # arena card (right)
    ax, ay, aw, ah, bt = 770, 120, 450, 400, 22
    d.rounded_rectangle([(ax - bt + 8) * S, (ay - bt + 10) * S, (ax + aw + bt + 8) * S, (ay + ah + bt + 10) * S],
                        radius=18 * S, fill=(150, 180, 205, 120))
    d.rounded_rectangle([(ax - bt) * S, (ay - bt) * S, (ax + aw + bt) * S, (ay + ah + bt) * S], radius=18 * S, fill=WALL)
    tile = 50
    for gy in range(ah // tile):
        for gx in range(aw // tile):
            c = (232, 242, 250) if (gx + gy) % 2 else (249, 252, 255)
            d.rectangle([(ax + gx * tile) * S, (ay + gy * tile) * S, (ax + (gx + 1) * tile) * S, (ay + (gy + 1) * tile) * S], fill=c)
    d.rectangle([(ax + 200) * S, (ay + 100) * S, (ax + 230) * S, (ay + 300) * S], fill=WALL)      # interior wall
    d.rectangle([(ax + 200) * S, (ay + 100) * S, (ax + 230) * S, (ay + 104) * S], fill=WALL_HI)

    # crate + ball
    d.rectangle([(ax + 300 + 5) * S, (ay + 255 + 6) * S, (ax + 350 + 5) * S, (ay + 305 + 6) * S], fill=(180, 200, 218))
    d.rectangle([(ax + 300) * S, (ay + 255) * S, (ax + 350) * S, (ay + 305) * S], fill=(217, 122, 0))
    d.rectangle([(ax + 306) * S, (ay + 261) * S, (ax + 344) * S, (ay + 299) * S], fill=(255, 176, 50))
    bx, by, br = ax + 330, ay + 130, 24
    d.ellipse([(bx - br + 5) * S, (by - br + 6) * S, (bx + br + 5) * S, (by + br + 6) * S], fill=(180, 200, 218))
    d.ellipse([(bx - br) * S, (by - br) * S, (bx + br) * S, (by + br) * S], fill=(184, 48, 42))
    d.ellipse([(bx - br + 3) * S, (by - br + 3) * S, (bx + br - 3) * S, (by + br - 3) * S], fill=(242, 88, 78))
    d.ellipse([(bx - 14) * S, (by - 16) * S, (bx - 4) * S, (by - 7) * S], fill=(255, 255, 255, 170))

    # goal bullseye + flag
    gx, gy = ax + 385, ay + 90
    for r, col in ((34, GREEN), (25, WHITE), (16, GREEN), (7, WHITE)):
        d.ellipse([(gx - r) * S, (gy - r) * S, (gx + r) * S, (gy + r) * S], fill=col)
    d.line([gx * S, gy * S, gx * S, (gy - 58) * S], fill=TXT, width=3 * S)
    for ry in range(3):
        for rx in range(4):
            d.rectangle([(gx + 3 + rx * 9) * S, (gy - 58 + ry * 9) * S, (gx + 3 + (rx + 1) * 9) * S, (gy - 58 + (ry + 1) * 9) * S],
                        fill=WHITE if (rx + ry) % 2 else (35, 48, 60))

    # rover + sensor rays
    rx0, ry0 = ax + 110, ay + 330
    rays(d, (rx0 + 40) * S, ry0 * S, [150 * S, 190 * S, 230 * S, 150 * S, 90 * S],
         [GREEN, GREEN, GREEN, YELLOW, ORANGE], 4 * S)
    rover(d, rx0 * S, ry0 * S, 80 * S, 54 * S)

    # title block (left)
    def text(x, y, s, size, fill, shadow=None):
        f = font(size * S)
        if shadow:
            d.text(((x + 4) * S, (y + 4) * S), s, font=f, fill=shadow)
        d.text((x * S, y * S), s, font=f, fill=fill)
    text(64, 150, "WiiBox", 150, WHITE, NAVY)
    text(70, 332, "Robotics & Physics Sandbox", 36, NAVY)
    text(70, 382, "Homebrew for the Nintendo Wii", 28, TXT)
    px = 70
    for label, col in (("Box2D", BLUE), ("5-8-2 Neural Net", GREEN), ("Mazes + Goals", ORANGE)):
        f = font(24 * S)
        tw = d.textlength(label, font=f) / S
        d.rounded_rectangle([px * S, 450 * S, (px + tw + 36) * S, 496 * S], radius=23 * S, fill=col)
        d.rounded_rectangle([(px + 16) * S, 453 * S, (px + tw + 20) * S, 459 * S], radius=3 * S, fill=(255, 255, 255, 90))
        d.text(((px + 18) * S, 461 * S), label, font=f, fill=WHITE)
        px += tw + 36 + 14
    out = img.convert("RGB").resize((1280, 640), Image.LANCZOS)
    path = os.path.join(ROOT, "docs", "cover.png")
    out.save(path)
    print("wrote", os.path.normpath(path))

if __name__ == "__main__":
    make_icon()
    make_cover()
