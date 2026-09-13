# -*- coding: utf-8 -*-
"""docs/img/hopper-bisector.svg を生成する。

足先ブロックの向きを「下腿 2 本の二等分線」に決める菱形リンク（docs/hopper-log.md §8.12、案 1）。
F–A1–C–A2 の 4 辺が等しい菱形なので、対角線 F–C が角 A1–F–A2 を二等分する。
足先ブロックは F でピン留めし、腕のスロットに C のピンを通す。
姿勢は hopper/kinematics.py の運動学から計算する。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from hopper.kinematics import LegGeom, ik, fk

G = LegGeom(0.040, 0.090, 0.130)
AL = 30.0          # 菱形の辺 a [mm]（F→A1、A1→C）
TUBE = 110.0       # ガイド管の長さ [mm]
LS0 = 100.0        # ばね自由長 [mm]


def pose(x, y):
    """先端 (x, y) [mm]。F を原点にした相対座標で返す（+X 前、+Y 上）。"""
    t1, t2 = ik(G, x / 1000.0, y / 1000.0)
    f = fk(G, t1, t2)
    F = (f['F'][0] * 1000, f['F'][1] * 1000)
    K1 = (f['A'][0] * 1000 - F[0], f['A'][1] * 1000 - F[1])
    K2 = (f['B'][0] * 1000 - F[0], f['B'][1] * 1000 - F[1])
    u1 = (K1[0] / 130.0, K1[1] / 130.0)
    u2 = (K2[0] / 130.0, K2[1] / 130.0)
    A1 = (u1[0] * AL, u1[1] * AL)
    A2 = (u2[0] * AL, u2[1] * AL)
    bx, by = u1[0] + u2[0], u1[1] + u2[1]
    bl = math.hypot(bx, by)
    b = (bx / bl, by / bl)                                    # 二等分線（F → C 向き、上向き）
    theta = math.acos(max(-1, min(1, u1[0] * u2[0] + u1[1] * u2[1])))   # 内角
    C = (b[0] * 2 * AL * math.cos(theta / 2), b[1] * 2 * AL * math.cos(theta / 2))
    o = (-F[0], -F[1])                                        # F → O
    ol = math.hypot(*o)
    o = (o[0] / ol, o[1] / ol)
    err = math.degrees(math.atan2(b[0] * o[1] - b[1] * o[0], b[0] * o[0] + b[1] * o[1]))
    return dict(F=F, K1=K1, K2=K2, A1=A1, A2=A2, C=C, b=b, o=o, theta=math.degrees(theta),
                err=err, ang=math.degrees(math.atan2(b[0], b[1])))   # ang: 軸の傾き（鉛直から）


W, H = 1280, 1000
out = []
A = out.append

SH = '#3f8f57'; SH_F = '#d8efdc'
LK = '#c9762b'; LK_F = '#ffe0b2'
FB = '#8e5bd6'; FB_F = '#efe3f7'
PN = '#5b6470'; PN_F = '#c8ccd2'
SP = '#e07b1f'
OF = '#c1121f'


def txt(x, y, s, size=12, fill='#222', anchor='start', weight='normal'):
    A('<text x="%.1f" y="%.1f" font-size="%s" fill="%s" text-anchor="%s" font-weight="%s">%s</text>'
      % (x, y, size, fill, anchor, weight, s))


def line(x1, y1, x2, y2, s, sw=1.5, dash=''):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="%.1f" '
      'stroke-linecap="round" %s/>' % (x1, y1, x2, y2, s, sw,
                                       ('stroke-dasharray="%s"' % dash) if dash else ''))


def rect(x, y, w, h, f, s, sw=1.5, rx=0):
    A('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="%.1f" fill="%s" stroke="%s" '
      'stroke-width="%.1f"/>' % (x, y, w, h, rx, f, s, sw))


def circ(x, y, r, f, s, sw=1.5):
    A('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="%s" stroke="%s" stroke-width="%.1f"/>'
      % (x, y, r, f, s, sw))


def coil(x1, y1, x2, y2, n, amp, s, sw=2.0):
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    px, py = -uy, ux
    pts = []
    N = n * 24
    for i in range(N + 1):
        t = i / N
        a = math.sin(2 * math.pi * n * t) * amp
        pts.append('%.1f,%.1f' % (x1 + ux * L * t + px * a, y1 + uy * L * t + py * a))
    A('<polyline points="%s" fill="none" stroke="%s" stroke-width="%.1f"/>' % (' '.join(pts), s, sw))


def dimv(x, y1, y2, label, side=1):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#5b6470" stroke-width="1" '
      'marker-start="url(#d2)" marker-end="url(#d1)"/>' % (x, y1, x, y2))
    txt(x + 6 * side, (y1 + y2) / 2 + 4, label, 11, '#5b6470', 'start' if side > 0 else 'end')


A('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
  'font-family="Segoe UI, Meiryo, sans-serif" font-size="12">' % (W, H, W, H))
A('<defs>'
  '<marker id="d1" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
  '<path d="M0,0 L8,4 L0,8 z" fill="#5b6470"/></marker>'
  '<marker id="d2" markerWidth="8" markerHeight="8" refX="1" refY="4" orient="auto">'
  '<path d="M8,0 L0,4 L8,8 z" fill="#5b6470"/></marker></defs>')
rect(0, 0, W, H, '#ffffff', 'none', 0)
txt(22, 34, '足先ブロックの向きを決める菱形リンク（二等分線方式）', 19, '#111', 'start', 'bold')
txt(22, 56, 'F–A1–C–A2 は 4 辺が等しい菱形。対角線 F–C は角 A1–F–A2 を必ず二等分する。'
            '足先ブロックは F でピン留めし、腕のスロットに C のピンを通すだけ', 12.5, '#5b6470')


def draw_mech(X0, Y0, S, p, lw=1.0, shank_len=95, labels=True):
    """F を (X0, Y0) に置いて機構を描く。S は px/mm。"""
    gx = lambda q: X0 + q[0] * S
    gy = lambda q: Y0 - q[1] * S
    ang = p['ang']
    # 軸（F→C の延長。上は股へ、下はガイド管）
    A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (X0, Y0, ang))
    # ガイド管（F の下）
    rect(-8.6 * S, 2 * S, 17.2 * S, TUBE * S, '#fff4e5', LK, 1.3 * lw, 2)
    rect(-7.6 * S, 2 * S, 15.2 * S, TUBE * S, '#ffffff', 'none', 0)
    # ばね・ピストン・パッド
    A('</g>')
    tip = lambda d: (p['b'][0] * -d, p['b'][1] * -d)   # 軸に沿って F から下へ d
    coil(gx(tip(6)), gy(tip(6)), gx(tip(LS0 - 4)), gy(tip(LS0 - 4)), 9, 6.2 * S, SP, 2.0 * lw)
    A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (gx(tip(LS0)), gy(tip(LS0)), ang))
    rect(-7.4 * S, -8 * S, 14.8 * S, 14 * S, '#e8eaed', PN, 1.4 * lw, 2)
    rect(-13 * S, 6 * S, 26 * S, 6 * S, '#4b4b4b', '#333', 1.2, 2)
    A('</g>')
    # 足先ブロックの腕（F から上へ。スロット付き）
    A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (X0, Y0, ang))
    rect(-4 * S, -58 * S, 8 * S, 66 * S, FB_F, FB, 1.6 * lw, 3)
    rect(-1.5 * S, -56 * S, 3 * S, 18 * S, '#ffffff', FB, 1.2 * lw, 1.5)        # スロット 38〜56
    A('</g>')
    # シャンク（K 側は途中まで）
    for K, Apt in ((p['K1'], p['A1']), (p['K2'], p['A2'])):
        u = (K[0] / 130.0, K[1] / 130.0)
        Ke = (u[0] * shank_len, u[1] * shank_len)
        line(gx((0, 0)), gy((0, 0)), gx(Ke), gy(Ke), SH, 9 * lw)
    # 小リンク
    for Apt in (p['A1'], p['A2']):
        line(gx(Apt), gy(Apt), gx(p['C']), gy(p['C']), LK, 5.5 * lw)
    # ピン
    for q, c in ((p['A1'], LK), (p['A2'], LK), (p['C'], LK)):
        circ(gx(q), gy(q), 3.6 * lw, '#ffffff', c, 1.6)
    circ(gx((0, 0)), gy((0, 0)), 4.5 * lw, '#ffffff', FB, 2.0)
    if labels:
        txt(gx((0, 0)) + 10, gy((0, 0)) + 5, 'F', 13, FB, 'start', 'bold')
        txt(gx(p['A1']) - 10, gy(p['A1']) + 4, 'A1', 12, LK, 'end', 'bold')
        txt(gx(p['A2']) + 10, gy(p['A2']) + 4, 'A2', 12, LK, 'start', 'bold')
        txt(gx(p['C']) + 10, gy(p['C']) - 6, 'C', 13, LK, 'start', 'bold')
    return gx, gy


# ==================================================================
# A. 対称姿勢（その場ホップ）
# ==================================================================
p = pose(0.0, -145.0)
txt(40, 96, 'A. 対称姿勢（先端 (0, −145)。内角 θ = %.1f°）' % p['theta'], 14.5, '#111', 'start', 'bold')
AX, AY, SA = 300.0, 380.0, 2.2
gx, gy = draw_mech(AX, AY, SA, p, shank_len=100)
# O→F の線（鉛直）
line(AX, AY - 150 * SA + 60, AX, AY + 130 * SA - 40, OF, 1.2, '7 4')
txt(AX + 8, AY - 150 * SA + 78, '股 O へ ↑', 11, OF, 'start', 'bold')
txt(AX + 8, AY - 150 * SA + 93, '（O→F の線）', 10.5, OF, 'start')
# 寸法
dimv(AX + 118, gy(p['C']), gy((0, 0)), '2a·cos(θ/2) = %.0f mm' % (p['C'][1]), 1)
dimv(AX - 110, gy(p['A1']), gy((0, 0)), 'a = 30', -1)
# ラベル
k1e = (p['K1'][0] / 130 * 100, p['K1'][1] / 130 * 100)
k2e = (p['K2'][0] / 130 * 100, p['K2'][1] / 130 * 100)
txt(gx(k1e) - 6, gy(k1e) - 10, 'シャンク①', 11.5, SH, 'end', 'bold')
txt(gx(k2e) + 6, gy(k2e) - 10, 'シャンク②', 11.5, SH, 'start', 'bold')
txt(gx(p['A1']) - 62, gy(p['A1']) - 26, '小リンク（長さ a）', 11.5, LK, 'end', 'bold')
line(gx(p['A1']) - 58, gy(p['A1']) - 30, (gx(p['A1']) + gx(p['C'])) / 2 - 6,
     (gy(p['A1']) + gy(p['C'])) / 2, LK, 1.0)
txt(gx(p['C']) + 24, gy(p['C']) - 34, '足先ブロックの腕', 11.5, FB, 'start', 'bold')
txt(gx(p['C']) + 24, gy(p['C']) - 20, 'スロット（幅 3 × 長さ 18）', 10.5, FB, 'start')
line(gx(p['C']) + 20, gy(p['C']) - 30, gx(p['C']) + 4 * SA + 2, gy(p['C']) - 14 * SA, FB, 1.0)
txt(gx((0, 0)) + 42, gy((0, 0)) + 40, 'ガイド管（腕と一体）', 11.5, LK, 'start', 'bold')
txt(gx((0, 0)) + 42, gy((0, 0)) + 54, '内径 15.2、長さ 110', 10.5, LK, 'start')
line(gx((0, 0)) + 38, gy((0, 0)) + 36, gx((0, 0)) + 8.6 * SA + 2, gy((0, 0)) + 22 * SA, LK, 1.0)
txt(gx((0, 0)) + 42, gy((0, 0)) + 92, 'ばね（サミニ 11-1437）', 11, SP, 'start')
txt(gx((0, 0)) + 42, gy((0, 0)) + 110 * SA + 10, '足（ピストン）＋パッド', 11, '#333', 'start')
# 説明
txt(40, 760, 'F–A1 = F–A2 = A1–C = A2–C = a の菱形。菱形の対角線は対角を二等分するので、', 11.5, '#333')
txt(40, 778, 'F→C は常に角 A1–F–A2 の二等分線。ブロックの軸は C と F で決まる。', 11.5, '#333')
txt(40, 800, '荷重の流れ: 接地力 → 足 → ばね → 管 → ブロック → ピン F → シャンク 2 本。', 11.5, '#333')
txt(40, 818, '軸は F を通るので、接地力は F まわりにモーメントを作らない。', 11.5, '#333')
txt(40, 836, '小リンクが受けるのは足パッドの摩擦（横力）× 100 mm のモーメントだけ（横力 30 N で約 140 N）。', 11.5, '#333')

# ==================================================================
# B. 前後に振ったとき
# ==================================================================
txt(640, 96, 'B. 脚を振ったとき — C はスロットを滑り、軸は二等分線を向き続ける', 14.5, '#111', 'start', 'bold')
SB = 1.25
for BX, tipx, tipy, lab in ((760.0, 40.0, -139.0, '前に 40 mm'),
                            (1010.0, -40.0, -139.0, '後ろに 40 mm')):
    BY = 330.0
    q = pose(tipx, tipy)
    bgx, bgy = draw_mech(BX, BY, SB, q, lw=0.75, shank_len=90)
    txt(BX, 128, lab, 12.5, '#111', 'middle', 'bold')
    # O→F の線と二等分線を延長して角度差を示す
    o = q['o']
    line(BX, BY, BX + o[0] * 120 * SB, BY - o[1] * 120 * SB, OF, 1.2, '7 4')
    b = q['b']
    line(BX, BY, BX + b[0] * 120 * SB, BY - b[1] * 120 * SB, FB, 1.2, '3 3')
    txt(BX, BY + 178, '内角 θ = %.1f°、C は F から %.0f mm'
        % (q['theta'], math.hypot(*q['C'])), 10.5, '#333', 'middle')
    txt(BX, BY + 196, '二等分線と O→F の差 %.1f°' % abs(q['err']), 11.5, OF, 'middle', 'bold')
txt(640, 548, '赤破線 = O→F（股→足先）、紫点線 = 二等分線。差は最大 2.1°、対称姿勢で 0、', 11, '#333')
txt(640, 564, '振り方向で符号が反転する。固定の向き誤差なら 3° が限界（5° で転倒。hopper-log §8.12）だが、', 11, '#333')
txt(640, 580, 'この誤差は蹴り出し（対称姿勢）で消えるので効かない。2D の全条件でロッド方式と同一性能。', 11, '#333')

# ==================================================================
# C. Z 方向の層（ピン A2 と ピン C を通る展開断面）
# ==================================================================
txt(640, 612, 'C. Z 方向の層（ピン A2 とピン C を通る展開断面。上が膝側、v = 0 がピン A2 の高さ）',
    14.5, '#111', 'start', 'bold')
CX, CY, SZ, SV = 790.0, 815.0, 14.0, 4.6        # z → 横 [px/mm]、v（軸方向）→ 縦 [px/mm]
zx = lambda z: CX + z * SZ
vy = lambda v: CY - v * SV
# 層: (名前, z0, z1, 塗り, 線, v_top, v_bot, ラベル高さ)
layers = [
    ('リンク①',   -6.5, -4.5, LK_F, LK,  25,  -4, 30),
    ('シャンク①', -4.25, -0.25, SH_F, SH,  8, -26, 12),
    ('シャンク②',  0.25,  4.25, SH_F, SH,  8, -26, 12),
    ('腕',         4.75,  7.25, FB_F, FB, 34, -12, 38),
    ('リンク②',    7.5,   9.5, LK_F, LK,  25,  -4, 30),
]
for name, z0, z1, fc, sc, top, bot, lv in layers:
    rect(zx(z0), vy(top), (z1 - z0) * SZ, (top - bot) * SV, fc, sc, 1.4)
    txt(zx((z0 + z1) / 2), vy(lv), name, 10.5, sc, 'middle', 'bold')
rect(zx(4.75), vy(32), 2.5 * SZ, 16 * SV, '#ffffff', FB, 1.0)                 # スロット（v 16〜32）
rect(zx(-8.5), vy(22.5), 19 * SZ, 3 * SV, PN_F, PN, 1.3)                       # ピン C（v = 21）
for z in (-8.7, 10.3):
    rect(zx(z) - 0.25 * SZ, vy(24.5), 0.5 * SZ, 7 * SV, PN_F, PN, 1.0)         # E リング
rect(zx(-0.5), vy(1.5), 11 * SZ, 3 * SV, PN_F, PN, 1.3)                         # ピン A2（v = 0）
rect(zx(4.25), vy(1.5), 3.25 * SZ, 3 * SV, '#e8eaed', '#9aa0a6', 1.0)           # スペーサ
txt(zx(12.5), vy(23), 'ピン C φ3 × 22', 11, PN, 'start', 'bold')
txt(zx(12.5), vy(19), 'リンク①・腕（スロット）・リンク② を貫く', 10, PN, 'start')
txt(zx(12.5), vy(15.5), '両端 E リング', 10, PN, 'start')
txt(zx(12.5), vy(1), 'ピン A2 φ3 × 14', 11, PN, 'start', 'bold')
txt(zx(12.5), vy(-3), 'シャンク②・スペーサ 3.25・リンク②', 10, PN, 'start')
txt(zx(12.5), vy(-7), 'A1 は鏡像: シャンク①・リンク①', 10, PN, 'start')
txt(zx(12.5), vy(-10.5), '（腕を避ける必要がないのでスペーサ無し）', 10, PN, 'start')
txt(zx(-8.5), vy(-31), 'z = −6.5', 10, '#5b6470', 'start')
txt(zx(9.5), vy(-31), '+9.5 mm', 10, '#5b6470', 'end')
txt(640, vy(-38), 'シャンク①②は §5-1 の足先と同じ層。小リンクは各シャンクの外面に、腕はブロックの片方の耳を上へ延ばす。',
    10.5, '#333', 'start')

A('</svg>')
svg = '\n'.join(out)
dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img')
os.makedirs(dst, exist_ok=True)
pth = os.path.join(dst, 'hopper-bisector.svg')
with open(pth, 'w', encoding='utf-8') as fh:
    fh.write(svg)
print('wrote', os.path.normpath(pth), len(svg), 'bytes')
for x, y in ((0, -145), (40, -139), (-40, -139), (0, -195)):
    q = pose(x, y)
    print('  tip (%4d,%5d): θ %.1f°  C %.1f mm  bisector−OF %.2f°' % (x, y, q['theta'], math.hypot(*q['C']), q['err']))
