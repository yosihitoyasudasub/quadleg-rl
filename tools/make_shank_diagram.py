# -*- coding: utf-8 -*-
"""docs/img/hopper-shank.svg を生成する。

シャンク（下腿 l2 = 130 mm）の両端の関節構造を示す。
膝 K ではシャンクが二股でクランクを挟み、足先 F ではシャンクが挟まれる側に回る。
寸法は docs/hopper-mechanical-spec.md に一致させる。
"""
import os

W, H = 1140, 1000
out = []
A = out.append

# ---- 色 ----
CK_F, CK_S = '#dbe6f3', '#4a6fa5'      # クランク（青）
SH_F, SH_S = '#d8efdc', '#3f8f57'      # シャンク（緑）
FB_F, FB_S = '#efe3f7', '#8e5bd6'      # 足先ブロック（紫）
PN_F, PN_S = '#c8ccd2', '#5b6470'      # ピン（灰）
BU_F, BU_S = '#ffe0b2', '#c9762b'      # ブッシュ（橙）
WS_F, WS_S = '#e8eaed', '#9aa0a6'      # ワッシャ


def rect(x, y, w, h, f, s, sw=1.4):
    A('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" stroke="%s" '
      'stroke-width="%.1f"/>' % (x, y, w, h, f, s, sw))


def outline(x, y, w, h, s, sw=1.6):
    A('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="none" stroke="%s" '
      'stroke-width="%.1f"/>' % (x, y, w, h, s, sw))


def txt(x, y, s, size=12, fill='#222', anchor='start', weight='normal'):
    A('<text x="%.1f" y="%.1f" font-size="%s" fill="%s" text-anchor="%s" font-weight="%s">%s</text>'
      % (x, y, size, fill, anchor, weight, s))


def line(x1, y1, x2, y2, s, sw=1.2):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="%.1f"/>'
      % (x1, y1, x2, y2, s, sw))


def dim(x1, y1, x2, y2, label, off=0, size=11):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#5b6470" stroke-width="1" '
      'marker-start="url(#d2)" marker-end="url(#d1)"/>' % (x1, y1, x2, y2))
    txt((x1 + x2) / 2, (y1 + y2) / 2 + off, label, size, '#5b6470', 'middle')


A('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
  'font-family="Segoe UI, Meiryo, sans-serif" font-size="12">' % (W, H, W, H))
A('<defs>'
  '<marker id="d1" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
  '<path d="M0,0 L8,4 L0,8 z" fill="#5b6470"/></marker>'
  '<marker id="d2" markerWidth="8" markerHeight="8" refX="1" refY="4" orient="auto">'
  '<path d="M8,0 L0,4 L8,8 z" fill="#5b6470"/></marker></defs>')
rect(0, 0, W, H, '#ffffff', 'none', 0)

txt(22, 34, 'シャンク（下腿 l2 = 130 mm）の両端の関節構造', 19, '#111', 'start', 'bold')
txt(22, 56, '膝 K ではシャンクが「挟む側（二股）」、足先 F では「挟まれる側（タング）」。'
            '1 本の中で役割が入れ替わる', 12.5, '#5b6470')

# ==================================================================
# A. シャンク単体（2 面図）
# ==================================================================
txt(30, 96, 'A. シャンク単体（2 面図）— 左端が膝 K、右端が足先 F', 14.5, '#111', 'start', 'bold')

S = 3.0
x0 = 165.0                      # 膝ピン中心
x1 = x0 + 130 * S               # 足先ピン中心
yv = 158.0                      # 正面図の中心線
yt = 270.0                      # 上面図の中心線

# --- 正面図（ピン軸方向から見る。幅 10 が見える）---
txt(30, yv - 2, '正面図', 12.5, '#5b6470', 'start', 'bold')
txt(30, yv + 16, '（ピン軸方向から）', 11, '#9aa0a6')
rect(x0, yv - 5 * S, x1 - x0, 10 * S, SH_F, SH_S, 1.6)
A('<path d="M %.1f %.1f A %.1f %.1f 0 0 0 %.1f %.1f Z" fill="%s" stroke="%s" stroke-width="1.6"/>'
  % (x0, yv - 5 * S, 5 * S, 5 * S, x0, yv + 5 * S, SH_F, SH_S))
A('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="%s" stroke="%s" stroke-width="1.6"/>'
  % (x1, yv, 7.5 * S, SH_F, SH_S))
rect(x1 - 34, yv - 5 * S, 34, 10 * S, SH_F, 'none', 0)
for sg in (-1, 1):
    line(x1 - 34, yv + sg * 5 * S, x1 - 8, yv + sg * 5 * S, SH_S, 1.6)
for cx_, r_ in ((x0, 2 * S), (x1, 3 * S)):
    A('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="#ffffff" stroke="%s" stroke-width="1.4"/>'
      % (cx_, yv, r_, SH_S))
txt(x0, yv + 4, 'K', 12, SH_S, 'middle', 'bold')
txt(x1, yv + 4, 'F', 12, SH_S, 'middle', 'bold')
txt(x0, yv - 5 * S - 14, '穴 φ4 H7', 11.5, '#c1121f', 'middle')
txt(x1, yv - 7.5 * S - 12, '穴 φ6 H7', 11.5, '#c1121f', 'middle')
dim(x0 + 140, yv + 5 * S + 24, x1, yv + 5 * S + 24, '130（ピン中心間）', -5)
txt(x0 + 48, yv + 4, '幅 10', 11, '#5b6470')
txt(x1 + 32, yv + 4, 'ボス幅 15', 11, '#5b6470')

# --- 上面図（板の厚み方向が見える）---
txt(30, yt - 2, '上面図', 12.5, '#5b6470', 'start', 'bold')
txt(30, yt + 16, '（脚面の真上から）', 11, '#9aa0a6')
rect(x0 + 20 * S, yt - 1.25 * S, (x1 - 14 * S) - (x0 + 20 * S), 2.5 * S, SH_F, SH_S, 1.6)
for zz in (-4.5, 4.5):
    rect(x0 - 5 * S, yt + (zz - 1.5) * S, 17 * S, 3 * S, SH_F, SH_S, 1.6)
    A('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="#ffffff" stroke="%s" stroke-width="1.2"/>'
      % (x0, yt + zz * S, 2 * S, SH_S))
A('<path d="M %.1f %.1f L %.1f %.1f L %.1f %.1f L %.1f %.1f Z" fill="%s" stroke="%s" '
  'stroke-width="1.6"/>' % (x0 + 12 * S, yt - 6 * S, x0 + 20 * S, yt - 1.25 * S,
                            x0 + 20 * S, yt + 1.25 * S, x0 + 12 * S, yt + 6 * S, SH_F, SH_S))
rect(x1 - 14 * S, yt - 2 * S, 21.5 * S, 4 * S, SH_F, SH_S, 1.6)
A('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="#ffffff" stroke="%s" stroke-width="1.2"/>'
  % (x1, yt, 3 * S, SH_S))
txt(x0 + 6 * S, yt - 14 * S, '膝 K 側：二股（フォーク）', 12, SH_S, 'middle', 'bold')
txt(x0 + 6 * S, yt - 14 * S + 16, '爪厚 3 × 2、スリット 6.0', 11, SH_S, 'middle')
txt(x1 - 3 * S, yt + 8 * S + 6, '足先 F 側：タング 厚さ 4', 12, SH_S, 'middle', 'bold')
txt(x0 + 34 * S, yt + 5 * S, '主スパン 厚さ 2.5', 11, '#5b6470', 'middle')

# ==================================================================
# 断面図の共通ヘルパ
# ==================================================================
SS = 9.0   # px/mm


def bolt(zx, vy, z_head, z_end):
    """段付ボルト 軸 φ4 / ねじ M3"""
    rect(zx(z_head - 2), vy(3.5), 2 * SS, 7 * SS, PN_F, PN_S, 1.3)           # 頭
    rect(zx(z_head), vy(2), (z_end - z_head) * SS, 4 * SS, PN_F, PN_S, 1.3)  # 段部 φ4
    rect(zx(z_end), vy(1.5), 4 * SS, 3 * SS, PN_F, PN_S, 1.1)                # ねじ M3
    rect(zx(z_end + 0.3), vy(3), 2.4 * SS, 6 * SS, PN_F, PN_S, 1.3)          # ナット


def bush(zx, vy, z_a, z_b):
    """すべり軸受 内径 φ4 / 外径 φ6、フランジ φ8 × 0.5（左から挿入）"""
    for v_top in (3, -2):
        rect(zx(z_a), vy(v_top), (z_b - z_a) * SS, 1 * SS, BU_F, BU_S, 1.1)
    rect(zx(z_a - 0.5), vy(4), 0.5 * SS, 8 * SS, BU_F, BU_S, 1.1)


def washer(zx, vy, z_a):
    rect(zx(z_a), vy(4), 0.5 * SS, 8 * SS, WS_F, WS_S, 1.1)


# ==================================================================
# B. 膝 K の断面
# ==================================================================
BX, BY = 320.0, 585.0
zx = lambda z: BX + z * SS
vy = lambda v: BY - v * SS
txt(30, 348, 'B. 膝 K の断面（ピン軸を含む断面。上がサーボ側、下が足先側）',
    14.5, '#111', 'start', 'bold')
txt(30, 369, 'シャンクの二股がクランクを挟む。回るのはクランク側 → ブッシュはクランクの穴に入る',
    12, '#5b6470')

# クランク（平板 t5、上へ伸びる）
rect(zx(-2.5), vy(18), 5 * SS, 25 * SS, CK_F, CK_S, 1.6)
rect(zx(-2.5), vy(3), 5 * SS, 6 * SS, '#ffffff', 'none', 0)     # 穴（白抜き）
line(zx(-2.5), vy(18), zx(-2.5), vy(-7), CK_S, 1.6)
line(zx(2.5), vy(18), zx(2.5), vy(-7), CK_S, 1.6)
line(zx(-2.5), vy(-7), zx(2.5), vy(-7), CK_S, 1.6)
# シャンクの二股（爪 t3、下へ伸びる）
for za in (-6.0, 3.0):
    rect(zx(za), vy(6), 3 * SS, 22 * SS, SH_F, SH_S, 1.6)
    rect(zx(za), vy(2), 3 * SS, 4 * SS, '#ffffff', 'none', 0)
    outline(zx(za), vy(6), 3 * SS, 22 * SS, SH_S)
bush(zx, vy, -2.5, 2.5)
washer(zx, vy, 2.5)
bolt(zx, vy, -6, 6)

txt(BX, 402, 'クランク（平板 t5）', 12.5, CK_S, 'middle', 'bold')
txt(BX, 419, '↑ サーボホーンへ', 11, CK_S, 'middle')
txt(zx(6) + 14, vy(-9), 'シャンク（二股）', 12.5, SH_S, 'start', 'bold')
txt(zx(6) + 14, vy(-9) + 17, '↓ 足先へ', 11, SH_S, 'start')
line(zx(0), vy(3.4), zx(0) + 60, vy(3.4) - 50, '#c9762b', 1.1)
txt(zx(0) + 64, vy(3.4) - 52, 'すべり軸受 φ4 / φ6 フランジ付', 11, '#c9762b')
txt(zx(0) + 64, vy(3.4) - 38, '（クランクの穴 φ6 H7 に圧入）', 11, '#c9762b')
line(zx(-7), vy(0), zx(-7) - 38, vy(0) - 56, '#5b6470', 1.1)
txt(zx(-7) - 42, vy(0) - 58, '段付ボルト', 11.5, '#5b6470', 'end')
txt(zx(-7) - 42, vy(0) - 44, '軸 φ4 × 12 / ねじ M3', 11, '#5b6470', 'end')
dim(zx(-6), vy(-21), zx(6), vy(-21), '3 ＋ 6.0 ＋ 3 = 12', -7)

# ==================================================================
# C. 足先 F の断面
# ==================================================================
CX, CY = 840.0, 585.0
zx = lambda z: CX + z * SS
vy = lambda v: CY - v * SS
txt(620, 348, 'C. 足先 F の断面（同じ向き。上が膝側、下が接地側）', 14.5, '#111', 'start', 'bold')
txt(620, 369, '足先ブロックが二股。シャンク 2 本は挟まれる側 → ブッシュはシャンクの穴に入る',
    12, '#5b6470')

# 足先ブロック（コの字）
for za in (-7.25, 4.75):
    rect(zx(za), vy(6), 2.5 * SS, 14 * SS, FB_F, FB_S, 1.6)
    rect(zx(za), vy(2), 2.5 * SS, 4 * SS, '#ffffff', 'none', 0)
    outline(zx(za), vy(6), 2.5 * SS, 14 * SS, FB_S)
rect(zx(-7.25), vy(-8), 14.5 * SS, 4 * SS, FB_F, FB_S, 1.6)
rect(zx(-1.5), vy(-12), 3 * SS, 3 * SS, FB_F, FB_S, 1.6)      # バネ軸
# シャンク 2 本のタング（t4、上へ伸びる）
for za in (-4.25, 0.25):
    rect(zx(za), vy(18), 4 * SS, 24 * SS, SH_F, SH_S, 1.6)
    rect(zx(za), vy(3), 4 * SS, 6 * SS, '#ffffff', 'none', 0)
    outline(zx(za), vy(18), 4 * SS, 24 * SS, SH_S)
    bush(zx, vy, za, za + 4)
washer(zx, vy, 4.25)
bolt(zx, vy, -7.25, 7.25)

txt(CX, 402, 'シャンク ① ②（タング t4）', 12.5, SH_S, 'middle', 'bold')
txt(CX, 419, '↑ 膝へ', 11, SH_S, 'middle')
txt(zx(-7.25) - 14, vy(-2), '足先ブロック（二股）', 12.5, FB_S, 'end', 'bold')
txt(CX, vy(-15) + 18, '↓ バネ・足パッドへ', 11, FB_S, 'middle')
line(zx(-2.25), vy(3.4), zx(-2.25) - 26, vy(3.4) - 56, '#c9762b', 1.1)
txt(zx(-2.25) - 30, vy(3.4) - 58, 'すべり軸受 φ4 / φ6', 11, '#c9762b', 'end')
txt(zx(-2.25) - 30, vy(3.4) - 44, '（シャンクの穴 φ6 H7 に圧入）', 11, '#c9762b', 'end')
dim(zx(-7.25), vy(-21), zx(7.25), vy(-21),
    '2.5 ＋ 4 ＋ 4 ＋ 2.5 ＋ ワッシャ = 14.5', -7)
txt(CX, vy(-21) + 20, '段付ボルト 軸 φ4 × 15 / ねじ M3', 11, '#5b6470', 'middle')

# ==================================================================
# まとめ
# ==================================================================
YT = 862
rect(30, YT - 26, W - 60, 118, '#f7f9fb', '#c9d3de', 1.2)
txt(50, YT, '要点', 13, '#111', 'start', 'bold')
lines = [
    '・ピンはどちらも同じ段付ボルト φ4（ねじ M3）。変わるのは軸長（膝 12 / 足先 15）と'
    '「どちらの部品に穴 φ6 を開けるか」だけ。',
    '・すべり軸受は必ず「回る側」に入れる。膝ではクランク、足先ではシャンク。'
    'ピンは固定側の爪に締め付けて、回らないようにする。',
    '・シャンクの穴は膝側 φ4 H7・足先側 φ6 H7 で径が違う。'
    'φ6 は幅 10 mm では縁の肉が足りないので、足先端だけ幅 15 mm のボスにする。',
    '・左右 2 本のシャンクは同一形状。足先では Z 方向に 4 mm ずれて並ぶ'
    '（2 本の脚面がもともと別の層にあるので矛盾しない）。',
]
for i, s in enumerate(lines):
    txt(50, YT + 23 + i * 20, s, 12, '#333')

A('</svg>')

svg = '\n'.join(out)
dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img')
os.makedirs(dst, exist_ok=True)
p = os.path.join(dst, 'hopper-shank.svg')
with open(p, 'w', encoding='utf-8') as fh:
    fh.write(svg)
print('wrote', os.path.normpath(p), len(svg), 'bytes')
