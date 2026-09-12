# -*- coding: utf-8 -*-
"""docs/img/hopper-spring.svg を生成する。

直列バネの支持方式：ガイドロッドを股軸 O でピン支持し、足先 F のキャリア（＝足先ブロック）を
そのロッドに通す。ロッドは O を支点として F を通るので、必ず O→F 方向を向く。
姿勢は hopper/kinematics.py の運動学から計算する。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from hopper.kinematics import LegGeom, ik, fk

G = LegGeom(0.040, 0.090, 0.130)
LS0 = 80.0          # バネ自然長＝ストローク [mm]
PAD = 12.0          # 足＋パッドの半径ぶん [mm]


def pose(x, y):
    """先端 (x, y) [mm] のときの P1,P2,K1,K2,F,足 を返す"""
    t1, t2 = ik(G, x / 1000.0, y / 1000.0)
    f = fk(G, t1, t2)
    K1 = (f['A'][0] * 1000, f['A'][1] * 1000)
    K2 = (f['B'][0] * 1000, f['B'][1] * 1000)
    F = (f['F'][0] * 1000, f['F'][1] * 1000)
    n = math.hypot(*F)
    u = (F[0] / n, F[1] / n)                       # 股 O から足先 F への単位ベクトル
    foot = (F[0] + u[0] * LS0, F[1] + u[1] * LS0)
    return dict(P1=(-20.0, 0.0), P2=(20.0, 0.0), K1=K1, K2=K2, F=F, foot=foot, u=u)


W, H = 1180, 900
out = []
A = out.append

CK = '#4a6fa5'      # クランク
SH = '#3f8f57'      # シャンク
FB = '#8e5bd6'      # キャリア（足先ブロック）
RD = '#5b6470'      # ロッド
SP = '#e07b1f'      # バネ
RF = '#c1121f'      # 接地力


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
    """(x1,y1)-(x2,y2) を軸とするコイルばねを正弦で描く"""
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    px, py = -uy, ux                       # 軸に垂直
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


A('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
  'font-family="Segoe UI, Meiryo, sans-serif" font-size="12">' % (W, H, W, H))
A('<defs>'
  '<marker id="rf" markerWidth="10" markerHeight="10" refX="9" refY="5" orient="auto">'
  '<path d="M0,1 L10,5 L0,9 z" fill="%s"/></marker>'
  '<marker id="d1" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
  '<path d="M0,0 L8,4 L0,8 z" fill="#5b6470"/></marker>'
  '<marker id="d2" markerWidth="8" markerHeight="8" refX="1" refY="4" orient="auto">'
  '<path d="M8,0 L0,4 L8,8 z" fill="#5b6470"/></marker></defs>' % RF)
rect(0, 0, W, H, '#ffffff', 'none', 0)

txt(22, 34, '直列バネの支持方式 — ガイドロッドを股軸で支える', 19, '#111', 'start', 'bold')
txt(22, 56, 'ロッドは股軸 O でピン支持し、足先 F のキャリアをそのロッドに通す。'
            'ロッドは O を支点として F を通るので、必ず O→F 方向を向く', 12.5, '#5b6470')

# ==================================================================
# A. 脚全体（前振り姿勢）
# ==================================================================
txt(40, 98, 'A. 脚全体（先端を前に 30 mm 振った姿勢）', 14.5, '#111', 'start', 'bold')

AX, AY, SA = 195.0, 168.0, 1.45
ax = lambda x: AX + x * SA
ay = lambda y: AY - y * SA
p = pose(30.0, -150.0)

ground(ax(p['foot'][0]), ay(p['foot'][1] - PAD), 250)
# 鉛直の基準線
line(ax(0), ay(0), ax(0), ay(-250), '#d5dae0', 1.2, '5 5')
# 接地力の作用線（ロッドより先に描く）
u = p['u']
line(ax(p['foot'][0] - u[0] * PAD), ay(p['foot'][1] - u[1] * PAD),
     ax(-u[0] * 34), ay(-u[1] * 34), RF, 1.3, '7 4')
A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="1.3" '
  'marker-end="url(#rf)"/>' % (ax(u[0] * 6), ay(u[1] * 6), ax(-u[0] * 34), ay(-u[1] * 34), RF))
# ロッド（O → 足）
line(ax(0), ay(0), ax(p['foot'][0]), ay(p['foot'][1]), RD, 5.0)
# リンク
for K, c in ((p['K1'], CK), (p['K2'], CK)):
    pass
line(ax(p['P1'][0]), ay(p['P1'][1]), ax(p['K1'][0]), ay(p['K1'][1]), CK, 6.0)
line(ax(p['P2'][0]), ay(p['P2'][1]), ax(p['K2'][0]), ay(p['K2'][1]), CK, 6.0)
line(ax(p['K1'][0]), ay(p['K1'][1]), ax(p['F'][0]), ay(p['F'][1]), SH, 4.5)
line(ax(p['K2'][0]), ay(p['K2'][1]), ax(p['F'][0]), ay(p['F'][1]), SH, 4.5)
# バネ（F → 足）
coil(ax(p['F'][0]), ay(p['F'][1]), ax(p['foot'][0]), ay(p['foot'][1]), 9, 9, SP, 2.2)
# 節点
for q, lab, col in ((p['P1'], 'P1', CK), (p['P2'], 'P2', CK),
                    (p['K1'], 'K1', SH), (p['K2'], 'K2', SH)):
    circ(ax(q[0]), ay(q[1]), 4.5, '#ffffff', col, 1.8)
circ(ax(0), ay(0), 6, '#ffffff', RD, 2.2)
txt(ax(0) - 12, ay(0) - 10, 'O', 13, RD, 'end', 'bold')
txt(ax(0) - 12, ay(0) + 6, '股軸', 10.5, RD, 'end')
# キャリア
cxp, cyp = ax(p['F'][0]), ay(p['F'][1])
ang = math.degrees(math.atan2(p['u'][0], -p['u'][1]))
A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (cxp, cyp, ang))
rect(-19, -11, 38, 22, '#efe3f7', FB, 1.8, 3)
A('</g>')
circ(cxp, cyp, 4.5, '#ffffff', FB, 1.8)
# 足
circ(ax(p['foot'][0]), ay(p['foot'][1]), 8, '#e8eaed', RD, 1.8)
A('<g transform="translate(%.1f,%.1f) rotate(%.1f)">' % (ax(p['foot'][0]), ay(p['foot'][1]), ang))
rect(-13, 6, 26, 8, '#4b4b4b', '#333', 1.2, 2)
A('</g>')
# ラベル
txt(ax(-48), ay(10), 'クランク l1', 11.5, CK, 'middle', 'bold')
txt(ax(-72), ay(-112), 'シャンク l2', 11.5, SH, 'middle', 'bold')
txt(cxp + 26, cyp - 4, 'キャリア', 11.5, FB, 'start', 'bold')
txt(cxp + 26, cyp + 11, '（＝足先ブロック）', 10.5, FB, 'start')
txt(ax(30) + 40, ay(-200), 'バネ', 11.5, SP, 'start', 'bold')
line(ax(30) + 36, ay(-200) - 4, ax(22), ay(-197), SP, 1.1)
txt(ax(-6) - 14, ay(-70), 'ガイドロッド', 11.5, RD, 'end', 'bold')
line(ax(-6) - 10, ay(-72), ax(6), ay(-68), RD, 1.1)
txt(ax(0) - 6, ay(-232), '鉛直', 10.5, '#9aa0a6', 'end')
txt(ax(-102), ay(-236), '接地力の作用線は', 11, RF, 'start', 'bold')
txt(ax(-102), ay(-236) + 15, '常に股 O を通る', 11, RF, 'start', 'bold')

# ==================================================================
# B. 3 姿勢の重ね描き
# ==================================================================
txt(430, 98, 'B. ロッドの向きは自動で決まる', 14.5, '#111', 'start', 'bold')
txt(430, 118, '支点 O と通過点 F が決まれば、ロッドの角度は一意', 11.5, '#5b6470')

BX, BY, SB = 590.0, 188.0, 1.12
bx = lambda x: BX + x * SB
by = lambda y: BY - y * SB
poses = [(-35.0, -150.0, '#b9c6d6'), (0.0, -150.0, '#7f8a97'), (35.0, -150.0, '#b9c6d6')]
ground(bx(0), by(-150 - LS0 - PAD), 300)
for x, y, col in poses:
    q = pose(x, y)
    solid = (col == '#7f8a97')
    lw = 3.4 if solid else 2.2
    line(bx(q['P1'][0]), by(q['P1'][1]), bx(q['K1'][0]), by(q['K1'][1]),
         CK if solid else '#b9c6d6', lw)
    line(bx(q['P2'][0]), by(q['P2'][1]), bx(q['K2'][0]), by(q['K2'][1]),
         CK if solid else '#b9c6d6', lw)
    line(bx(q['K1'][0]), by(q['K1'][1]), bx(q['F'][0]), by(q['F'][1]),
         SH if solid else '#c2d8c9', lw)
    line(bx(q['K2'][0]), by(q['K2'][1]), bx(q['F'][0]), by(q['F'][1]),
         SH if solid else '#c2d8c9', lw)
    line(bx(0), by(0), bx(q['foot'][0]), by(q['foot'][1]), RD if solid else col, 4.0 if solid else 3.0)
    circ(bx(q['F'][0]), by(q['F'][1]), 4.0, '#ffffff', FB if solid else col, 1.6)
    circ(bx(q['foot'][0]), by(q['foot'][1]), 6.0, '#e8eaed', RD if solid else col, 1.5)
circ(bx(0), by(0), 6, '#ffffff', RD, 2.2)
txt(bx(0) - 12, by(0) - 10, 'O', 13, RD, 'end', 'bold')
txt(bx(-95), by(-242) + 36, '足先 F が前後に動いても、ロッドは O を支点に振れるだけ。',
    11, '#5b6470', 'start')
txt(bx(-95), by(-242) + 53, '角度を決める機構は要らない', 11, '#5b6470', 'start')

# ==================================================================
# C. 足先まわりの拡大
# ==================================================================
txt(790, 98, 'C. 足先まわり（ロッド軸に沿った断面）', 14.5, '#111', 'start', 'bold')

CXp, CY0, SC = 900.0, 182.0, 2.6
cy = lambda v: CY0 + v * SC        # v = F からロッドに沿って下向きの距離 [mm]
RODW = 5.0 * SC / 2

# ロッド
rect(CXp - RODW, cy(-22), 2 * RODW, (118) * SC, '#dfe3e8', RD, 1.6)
# キャリア（足先ブロック）
rect(CXp - 17 * SC, cy(-9), 34 * SC, 18 * SC, '#efe3f7', FB, 1.8, 3)
# リニアブッシュ
for x_ in (CXp - RODW - 1.5 * SC, CXp + RODW):
    rect(x_, cy(-8), 1.5 * SC, 16 * SC, '#ffe0b2', '#c9762b', 1.2)
# シャンクのピン F
circ(CXp - 12 * SC, cy(0), 3.2 * SC / 2, '#ffffff', SH, 1.6)
circ(CXp + 12 * SC, cy(0), 3.2 * SC / 2, '#ffffff', SH, 1.6)
line(CXp - 25 * SC, cy(-6), CXp - 17 * SC, cy(-2), SH, 3.5)
line(CXp + 25 * SC, cy(-6), CXp + 17 * SC, cy(-2), SH, 3.5)
# バネ
coil(CXp, cy(9), CXp, cy(74), 8, 11 * SC / 2, SP, 2.4)
# 下側ばね座
rect(CXp - 9 * SC, cy(74), 18 * SC, 4 * SC, '#dfe3e8', RD, 1.6)
# ストッパ
rect(CXp - 7 * SC, cy(30), 14 * SC, 3 * SC, '#f2f4f6', '#9aa0a6', 1.3)
# 足＋パッド
rect(CXp - 11 * SC, cy(78), 22 * SC, 8 * SC, '#e8eaed', RD, 1.8, 2)
rect(CXp - 13 * SC, cy(86), 26 * SC, 6 * SC, '#4b4b4b', '#333', 1.3, 2)
ground(CXp, cy(92), 200)

def lead(v, dx, s, sub=None, col='#333'):
    x0 = CXp + (17 * SC if dx > 0 else -17 * SC)
    txt(CXp + dx, cy(v) + 4, s, 11.5, col, 'start' if dx > 0 else 'end', 'bold')
    if sub:
        txt(CXp + dx, cy(v) + 19, sub, 10.5, col, 'start' if dx > 0 else 'end')

lead(0, 118, 'シャンクのピン F', 'キャリアに 2 本が集まる', SH)
lead(-16, -122, 'キャリア', '（＝足先ブロック）', FB)
lead(8, -122, 'リニアブッシュ', 'ロッド上を 125〜199 mm 滑る', '#c9762b')
lead(40, 118, '圧縮コイルばね', 'ロッドに巻く（座屈防止）', SP)
lead(30, -118, '底付きストッパ', '密着させない', '#5b6470')
lead(76, 118, '下側ばね座', 'セットカラー／E リング', RD)
lead(88, -118, '足＋ゴムパッド', None, '#333')
line(CXp + 114, cy(0), CXp + 17 * SC, cy(0), SH, 1.0)
line(CXp - 118, cy(-16), CXp - 17 * SC, cy(-7), FB, 1.0)
line(CXp - 118, cy(8), CXp - RODW - 1.5 * SC, cy(6), '#c9762b', 1.0)
line(CXp + 114, cy(40), CXp + 15, cy(40), SP, 1.0)
line(CXp - 114, cy(30), CXp - 7 * SC, cy(31), '#5b6470', 1.0)
line(CXp + 114, cy(76), CXp + 9 * SC, cy(76), RD, 1.0)
line(CXp - 114, cy(88), CXp - 13 * SC, cy(88), '#333', 1.0)

# ==================================================================
# まとめ
# ==================================================================
YT = 630
rect(30, YT - 26, W - 60, 248, '#f7f9fb', '#c9d3de', 1.2)
txt(50, YT, 'なぜこの方式でなければならないか', 13.5, '#111', 'start', 'bold')
body = [
    ('1. バネの軸は「股 O → 足先 F」を向いている必要がある。',
     ['バネを脚に対して固定の向きで付けると、脚を前後に振ったときに接地力の作用線が股からずれ、'
      'ピッチモーメントが出て蹴り出すたびに機首が上がる',
      '（初回 Colab で発生。hopper/model.py の legbar のコメントに経緯がある）。']),
    ('2. 足先ブロックは単独では向きが決まらない。',
     ['シャンク 2 本は同じ 1 本のピン F に集まる（別ピンにすると自由度が 1 個増えて機構が成立しない）ので、',
      'ブロックは F まわりの自由な振り子になる。ここにバネを直付けしても角度が定まらない。']),
    ('3. ロッドを股で支持すれば、向きは自動的に決まる。',
     ['支点 O と通過点 F が決まればロッドの角度は一意。角度を決める機構が要らない。',
      'Raibert の元祖ホッパー（伸縮脚＋股関節）と同じ構成で、5 節リンクは「脚軸に沿ってキャリアを'
      '押し引きするアクチュエータ」として働く。']),
    ('4. ガイドは必須。',
     ['圧縮コイルばねは「自由長 / コイル平均径」が 2.6（両端ピン）〜5.2（両端平座固定）を超えると座屈する。'
      '候補は 130 / 14 = 9.3 なので、ガイドなしでは確実に座屈する。']),
    ('参考：2 本のシャンクの二等分線は O→F 方向と最大 2.1°（常用域 1° 以内）しかずれない。',
     ['ただし二等分線を機械的に作るには差動歯車が要るので、ロッド方式のほうが単純。']),
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
