# -*- coding: utf-8 -*-
"""docs/img/hopper-spring.svg を生成する。

直列バネの支持方式：ガイドロッドはキャリア（＝足先ブロック）に固定し、股軸 O では
「首振りするリニアブッシュ」で受ける。屈伸による股→足の長さ変化はロッドが股ブッシュを
滑って吸収し、同時にロッドの線は必ず O を通るので、バネの軸は常に O→F 方向を向く。

MJCF（hopper/model.py）との対応:
  legbar の股ヒンジ        → 股の首振りブッシュブロック
  carrier の slide  `cx`   → ロッドが股ブッシュを滑る
  foot の slide     `fz`   → 足がロッド下部を滑る（＝バネ本体）
姿勢は hopper/kinematics.py の運動学から計算する。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from hopper.kinematics import LegGeom, ik, fk

G = LegGeom(0.040, 0.090, 0.130)
LS0 = 80.0          # バネ自然長＝ストローク [mm]（キャリア → 足）
PAD = 12.0          # 足＋パッドの半径ぶん [mm]
ROD_UP = 212.0      # キャリアからロッド上端まで [mm]（最大 |OF| 199 ＋ ブッシュ半長）
ROD_DN = 100.0      # キャリアからロッド下端まで [mm]


def pose(x, y):
    t1, t2 = ik(G, x / 1000.0, y / 1000.0)
    f = fk(G, t1, t2)
    F = (f['F'][0] * 1000, f['F'][1] * 1000)
    n = math.hypot(*F)
    u = (F[0] / n, F[1] / n)                       # 股 O → 足先 F の単位ベクトル
    q = lambda d: (F[0] + u[0] * d, F[1] + u[1] * d)
    return dict(P1=(-20.0, 0.0), P2=(20.0, 0.0),
                K1=(f['A'][0] * 1000, f['A'][1] * 1000),
                K2=(f['B'][0] * 1000, f['B'][1] * 1000),
                F=F, u=u, n=n, at=q,
                foot=q(LS0), rtop=q(-ROD_UP), rbot=q(ROD_DN))


W, H = 1280, 1030
out = []
A = out.append

CK = '#4a6fa5'      # クランク
SH = '#3f8f57'      # シャンク
FB = '#8e5bd6'      # キャリア（足先ブロック）
RD = '#5b6470'      # ロッド
BU = '#c9762b'      # ブッシュ
SP = '#e07b1f'      # バネ
RF = '#c1121f'      # 強調


def txt(x, y, s, size=12, fill='#222', anchor='start', weight='normal'):
    A('<text x="%.1f" y="%.1f" font-size="%s" fill="%s" text-anchor="%s" font-weight="%s">%s</text>'
      % (x, y, size, fill, anchor, weight, s))


def line(x1, y1, x2, y2, s, sw=1.5, dash=''):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="%.1f" %s/>'
      % (x1, y1, x2, y2, s, sw, ('stroke-dasharray="%s"' % dash) if dash else ''))


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
    A('<polyline points="%s" fill="none" stroke="%s" stroke-width="%.1f" stroke-linejoin="round"/>'
      % (' '.join(pts), s, sw))


def ground(xc, y, w, s='#9aa0a6'):
    line(xc - w / 2, y, xc + w / 2, y, s, 2.0)
    x = xc - w / 2
    while x < xc + w / 2 - 6:
        line(x, y, x + 7, y + 9, s, 1.0)
        x += 9


def dimline(x1, y1, x2, y2):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#5b6470" stroke-width="1" '
      'marker-start="url(#d2)" marker-end="url(#d1)"/>' % (x1, y1, x2, y2))


A('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
  'font-family="Segoe UI, Meiryo, sans-serif" font-size="12">' % (W, H, W, H))
A('<defs>'
  '<marker id="d1" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
  '<path d="M0,0 L8,4 L0,8 z" fill="#5b6470"/></marker>'
  '<marker id="d2" markerWidth="8" markerHeight="8" refX="1" refY="4" orient="auto">'
  '<path d="M8,0 L0,4 L8,8 z" fill="#5b6470"/></marker></defs>')
rect(0, 0, W, H, '#ffffff', 'none', 0)

txt(22, 34, '直列バネの支持方式 — ロッドはキャリアに固定し、股では滑らせる', 19, '#111', 'start', 'bold')
txt(22, 56, '屈伸による股→足の長さ変化はロッドが股のブッシュを滑って吸収する。'
            'それでもロッドの線は必ず O を通るので、バネの軸は常に O→F 方向を向く',
    12.5, '#5b6470')


def draw_leg(X0, Y0, S, p, lw=1.0):
    """脚一式を描く。X0,Y0 は股 O の画面位置、S は px/mm"""
    gx = lambda q: X0 + q[0] * S
    gy = lambda q: Y0 - q[1] * S
    ang = math.degrees(math.atan2(p['u'][0], -p['u'][1]))
    line(gx(p['rtop']), gy(p['rtop']), gx(p['rbot']), gy(p['rbot']), RD, 4.5 * lw)
    line(gx(p['P1']), gy(p['P1']), gx(p['K1']), gy(p['K1']), CK, 5.5 * lw)
    line(gx(p['P2']), gy(p['P2']), gx(p['K2']), gy(p['K2']), CK, 5.5 * lw)
    line(gx(p['K1']), gy(p['K1']), gx(p['F']), gy(p['F']), SH, 4.0 * lw)
    line(gx(p['K2']), gy(p['K2']), gx(p['F']), gy(p['F']), SH, 4.0 * lw)
    coil(gx(p['at'](14)), gy(p['at'](14)), gx(p['at'](LS0 - 10)), gy(p['at'](LS0 - 10)),
         8, 8 * lw, SP, 2.2 * lw)
    A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (gx(p['F']), gy(p['F']), ang))
    rect(-17 * lw, -13 * lw, 34 * lw, 26 * lw, '#efe3f7', FB, 1.8, 3)
    A('</g>')
    circ(gx(p['F']), gy(p['F']), 4.2 * lw, '#ffffff', FB, 1.6)
    A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (gx(p['foot']), gy(p['foot']), ang))
    rect(-11 * lw, -9 * lw, 22 * lw, 15 * lw, '#e8eaed', RD, 1.6, 2)
    rect(-13 * lw, 6 * lw, 26 * lw, 7 * lw, '#4b4b4b', '#333', 1.2, 2)
    A('</g>')
    A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (X0, Y0, ang))
    rect(-14 * lw, -13 * lw, 28 * lw, 26 * lw, '#ffe0b2', BU, 1.8, 3)
    A('</g>')
    for q, c in ((p['P1'], CK), (p['P2'], CK), (p['K1'], SH), (p['K2'], SH)):
        circ(gx(q), gy(q), 4.0 * lw, '#ffffff', c, 1.6)
    circ(X0, Y0, 4.5 * lw, '#ffffff', RD, 2.0)
    return gx, gy


# ==================================================================
# A. 脚全体
# ==================================================================
txt(40, 98, 'A. 脚全体（先端を前に 30 mm 振った姿勢）', 14.5, '#111', 'start', 'bold')

AX, AY, SA = 180.0, 250.0, 1.18
p = pose(30.0, -150.0)
ground(AX + p['foot'][0] * SA, AY - (p['foot'][1] - PAD) * SA, 240)
ax, ay = draw_leg(AX, AY, SA, p)
line(ax(p['at'](LS0 + PAD)), ay(p['at'](LS0 + PAD)),
     ax(p['at'](-ROD_UP - 18)), ay(p['at'](-ROD_UP - 18)), RF, 1.3, '7 4')
mx, my = (p['P2'][0] + p['K2'][0]) / 2, (p['P2'][1] + p['K2'][1]) / 2
txt(ax((mx, my)) + 10, ay((mx, my)) - 12, 'クランク l1', 11.5, CK, 'start', 'bold')
txt(ax(p['K1']) + 10, ay(p['F']) - 26, 'シャンク l2', 11.5, SH, 'end', 'bold')
txt(AX - 26, AY - 36, '股軸 O', 11.5, RD, 'end', 'bold')
txt(AX - 26, AY - 20, '首振りリニアブッシュ', 11, BU, 'end', 'bold')
txt(AX - 26, AY - 6, '（股軸まわりに回転自由。', 10.5, BU, 'end')
txt(AX - 26, AY + 8, 'ロッドはこの中を滑る）', 10.5, BU, 'end')
txt(ax(p['rtop']) + 14, ay(p['rtop']) + 4, 'ロッド上端', 11, RD, 'start', 'bold')
txt(ax(p['rtop']) + 14, ay(p['rtop']) + 18, '（屈むと胴体側へ出る）', 10.5, RD, 'start')
txt(ax(p['F']) + 26, ay(p['F']) - 6, 'キャリア（＝足先ブロック）', 11.5, FB, 'start', 'bold')
txt(ax(p['F']) + 26, ay(p['F']) + 9, 'ロッドに固定。滑らない', 10.5, FB, 'start')
txt(ax(p['at'](44)) + 30, ay(p['at'](44)) + 4, 'バネ', 11.5, SP, 'start', 'bold')
line(ax(p['at'](44)) + 26, ay(p['at'](44)), ax(p['at'](44)) + 12, ay(p['at'](44)), SP, 1.1)
txt(ax(p['at'](LS0)) + 24, ay(p['at'](LS0)) + 4, '足（ロッドを滑る）', 11, '#333', 'start')
txt(40, ay(p['at'](LS0 + PAD)) + 30, '接地力の作用線は常に股 O を通る', 11.5, RF, 'start', 'bold')

# ==================================================================
# B. 屈伸で何が動くか
# ==================================================================
txt(420, 98, 'B. 屈伸で何が動くか', 14.5, '#111', 'start', 'bold')
txt(420, 118, 'ロッドが股ブッシュを滑るので、股→足の長さはちゃんと変わる', 11.5, '#5b6470')

SB = 0.78
for BXc, tipy, lab, sd in ((490.0, -125.0, '屈（|OF| = 125）', -1),
                           (680.0, -195.0, '伸（|OF| = 195）', 1)):
    BYc = 250.0
    q = pose(0.0, tipy)
    ground(BXc, BYc - (q['foot'][1] - PAD) * SB, 120)
    bx, by = draw_leg(BXc, BYc, SB, q, lw=0.8)
    txt(BXc, 150, lab, 12, '#111', 'middle', 'bold')
    circ(bx(q['rtop']), by(q['rtop']), 4.2, RF, RF, 1.0)
    txt(bx(q['rtop']) + 9, by(q['rtop']) + 4, '＋%d' % round(ROD_UP - q['n']), 10.5,
        RF, 'start', 'bold')
    xo = BXc + sd * 78
    an = 'start' if sd > 0 else 'end'
    line(BXc, BYc, xo + sd * 6, BYc, '#c9d3de', 1.0)
    line(bx(q['foot']), by(q['foot']), xo + sd * 6, by(q['foot']), '#c9d3de', 1.0)
    dimline(xo, BYc, xo, by(q['foot']))
    txt(xo + sd * 9, (BYc + by(q['foot'])) / 2 - 3, '股→足', 10.5, '#5b6470', an)
    txt(xo + sd * 9, (BYc + by(q['foot'])) / 2 + 12, '%d mm' % round(q['n'] + LS0), 11.5,
        '#5b6470', an, 'bold')

txt(420, 534, '赤丸＝ロッド上端。股ブッシュより上へ出る量が 87 → 17 mm と変わり、',
    11.5, '#333')
txt(420, 552, 'その差ぶんだけ股→足が 205 → 275 mm に伸びる。ロッドは股で固定されていない。',
    11.5, '#333')
txt(420, 576, 'バネはこの屈伸では縮まない。接地して初めて縮む。', 11.5, RF, 'start', 'bold')

# ==================================================================
# C. キャリアまわりの断面
# ==================================================================
txt(930, 98, 'C. キャリアまわり（ロッド軸に沿った断面）', 14.5, '#111', 'start', 'bold')

CXp, CY0, SC = 1030.0, 158.0, 2.45
cy = lambda v: CY0 + v * SC        # v = キャリア F からロッドに沿って下向き [mm]
RW = 5.0 * SC / 2                  # ロッド半径 φ5

rect(CXp - RW, cy(-18), 2 * RW, 118 * SC, '#dfe3e8', RD, 1.6)
rect(CXp - 17 * SC, cy(-9), 34 * SC, 18 * SC, '#efe3f7', FB, 1.8, 3)
for sg in (-1, 1):
    line(CXp + sg * RW, cy(-9), CXp + sg * RW, cy(9), FB, 1.4)
    circ(CXp + sg * 12 * SC, cy(0), 3.2 * SC / 2, '#ffffff', SH, 1.6)
    line(CXp + sg * 25 * SC, cy(-6), CXp + sg * 17 * SC, cy(-2), SH, 3.5)
coil(CXp, cy(10), CXp, cy(72), 8, 11 * SC / 2, SP, 2.4)
rect(CXp - 11 * SC, cy(72), 22 * SC, 16 * SC, '#e8eaed', RD, 1.8, 2)
for x_ in (CXp - RW - 1.5 * SC, CXp + RW):
    rect(x_, cy(73), 1.5 * SC, 14 * SC, '#ffe0b2', BU, 1.2)
rect(CXp - 13 * SC, cy(88), 26 * SC, 7 * SC, '#4b4b4b', '#333', 1.3, 2)
rect(CXp - 7 * SC, cy(97), 14 * SC, 3 * SC, '#f2f4f6', '#9aa0a6', 1.3)
ground(CXp, cy(95), 185)


def lead(v, side, s, sub=None, col='#333', tx=108):
    x = CXp + (tx if side > 0 else -tx)
    txt(x, cy(v) + 4, s, 11.5, col, 'start' if side > 0 else 'end', 'bold')
    if sub:
        txt(x, cy(v) + 19, sub, 10.5, col, 'start' if side > 0 else 'end')


lead(-15, -1, 'ロッド φ5', '↑ 股のブッシュへ', RD)
lead(0, 1, 'シャンクのピン F', 'キャリアに 2 本が集まる', SH)
lead(6, -1, 'キャリア', 'ロッドに固定（滑らない）', FB)
lead(40, 1, '圧縮コイルばね', 'ロッドに巻く（座屈防止）', SP)
lead(78, 1, '足', 'ロッドを滑る（＝ストローク）', BU)
lead(97, 1, '伸び切りストッパ', 'E リング', '#5b6470')
line(CXp + 104, cy(0), CXp + 17 * SC, cy(0), SH, 1.0)
line(CXp - 104, cy(6), CXp - 17 * SC, cy(4), FB, 1.0)
line(CXp - 104, cy(-15), CXp - RW, cy(-15), RD, 1.0)
line(CXp + 104, cy(40), CXp + 16, cy(40), SP, 1.0)
line(CXp + 104, cy(78), CXp + RW + 1.5 * SC, cy(78), BU, 1.0)
line(CXp + 104, cy(97), CXp + 7 * SC, cy(98), '#5b6470', 1.0)

# ==================================================================
# まとめ
# ==================================================================
YT = 645
rect(30, YT - 26, W - 60, 358, '#f7f9fb', '#c9d3de', 1.2)
txt(50, YT, 'なぜこの方式なのか', 13.5, '#111', 'start', 'bold')
body = [
    ('1. バネの軸は「股 O → 足先 F」を向いている必要がある。',
     ['バネを脚に対して固定の向きで付けると、脚を前後に振ったときに接地力の作用線が股からずれ、'
      'ピッチモーメントが出て蹴り出すたびに機首が上がる',
      '（初回 Colab で発生。hopper/model.py の legbar のコメントに経緯がある）。']),
    ('2. 足先ブロックは単独では向きが決まらない。',
     ['シャンク 2 本は同じ 1 本のピン F に集まる（別ピンにすると自由度が 1 個増えて機構が成立しない）ので、'
      'ブロックは F まわりの自由な振り子になる。']),
    ('3. ロッドの線が股 O を通れば、向きは自動的に決まる。',
     ['ロッドをキャリアに固定し、股側を首振りするリニアブッシュで受ける。'
      'ブッシュがロッドの線を必ず O に通すので、角度を決める機構が要らない。']),
    ('4. 長さ変化はロッドが股ブッシュを滑って吸収する。',
     ['股→足は 205〜275 mm（|OF| 125〜195 ＋ バネ 80）で変わる。'
      'ロッドを股で「固定」してしまうと股→足が一定になり、脚が伸縮できなくなる。',
      'ロッドは軸力を受けない。力は 足 → バネ → キャリア → 5 節リンク → 股 と流れ、'
      'ロッドは向きを決める案内にすぎない。']),
    ('5. ガイドは必須。',
     ['圧縮コイルばねは「自由長 / コイル平均径」が 2.6（両端ピン）〜5.2（両端平座固定）を超えると座屈する。'
      '候補は 130 / 14 = 9.3 なので、ガイドなしでは確実に座屈する。']),
    ('MJCF との対応（hopper/model.py）',
     ['legbar の股ヒンジ → 股の首振りブッシュブロック。carrier の slide `cx` → ロッドが股ブッシュを滑る。'
      'foot の slide `fz` → 足がロッド下部を滑る（＝バネ本体）。']),
]
y = YT + 24
for head, subs in body:
    txt(50, y, head, 12, '#111', 'start', 'bold')
    y += 17
    for sub in subs:
        txt(50, y, sub, 11.5, '#444')
        y += 16
    y += 9

A('</svg>')

svg = '\n'.join(out)
dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img')
os.makedirs(dst, exist_ok=True)
pth = os.path.join(dst, 'hopper-spring.svg')
with open(pth, 'w', encoding='utf-8') as fh:
    fh.write(svg)
print('wrote', os.path.normpath(pth), len(svg), 'bytes')
