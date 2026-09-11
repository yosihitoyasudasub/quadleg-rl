# -*- coding: utf-8 -*-
"""docs/img/hopper-leg-structure.svg を生成する。

寸法は hopper/kinematics.py（MuJoCo モデル）と docs/hopper-mechanical-spec.md に一致させる。
姿勢は飛行・引込み（リンク先端長 145 mm）。
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from hopper.kinematics import LegGeom, ik, fk

# ---- 実寸（mm） ----
g = LegGeom(0.040, 0.090, 0.130)
t1, t2 = ik(g, 0.0, -0.145)
f = fk(g, t1, t2)
P1, P2 = (-20.0, 0.0), (20.0, 0.0)
K1 = (f['A'][0] * 1000, f['A'][1] * 1000)
K2 = (f['B'][0] * 1000, f['B'][1] * 1000)
F = (f['F'][0] * 1000, f['F'][1] * 1000)
FT = (0.0, F[1] - 80.0)          # 足の接地点（バネ自然長 80 mm）

SC, OX, OY = 1.25, 300.0, 200.0
X = lambda x: OX + x * SC
Y = lambda y: OY - y * SC

out = []
A = out.append
A('<svg xmlns="http://www.w3.org/2000/svg" width="1060" height="690" viewBox="0 0 1060 690" '
  'font-family="Segoe UI, Meiryo, sans-serif" font-size="13">')
A('<defs>'
  '<marker id="dim" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
  '<path d="M0,0 L8,4 L0,8 z" fill="#5b6470"/></marker>'
  '<marker id="dim2" markerWidth="8" markerHeight="8" refX="1" refY="4" orient="auto">'
  '<path d="M8,0 L0,4 L8,8 z" fill="#5b6470"/></marker></defs>')
A('<rect width="1060" height="690" fill="#ffffff"/>')
A('<text x="22" y="30" font-size="18" font-weight="bold">'
  '一本脚ホッパー 脚の構造（5 節リンク ＋ 直列バネ ＋ ロール軸）</text>')
A('<text x="22" y="50" font-size="12" fill="#5b6470">'
  '姿勢は飛行・引込み（リンク先端長 145 mm）。寸法は MuJoCo モデルと docs/hopper-mechanical-spec.md に一致</text>')

# ======== A: 側面図 ========
A('<text x="40" y="72" font-size="14" font-weight="bold">'
  'A. 側面図（脚面 = XY。+X 前方・+Y 上。脚長サーボの軸は紙面に垂直）</text>')
A('<rect x="%.0f" y="%.0f" width="%.0f" height="%.0f" fill="#eef2f7" stroke="#4a6fa5" '
  'stroke-width="1.5" stroke-dasharray="6 4"/>' % (X(-150), Y(92), 300 * SC, 56 * SC))
A('<text x="%.0f" y="%.0f" font-size="11.5" fill="#4a6fa5">胴体（重心を股軸の高さに置く。実際は股まわりを囲む）</text>'
  % (X(-144), Y(66)))
A('<line x1="%.0f" y1="%.0f" x2="%.0f" y2="%.0f" stroke="#8e5bd6" stroke-width="2" '
  'stroke-dasharray="10 3 2 3"/>' % (X(-176), Y(0), X(150), Y(0)))
A('<text x="%.0f" y="%.0f" font-size="11.5" fill="#8e5bd6">ロール軸 ③（X 方向）</text>'
  % (X(-174), Y(0) + 15))
for c, lbl in ((P1, '1'), (P2, '2')):
    A('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="3" fill="#ffe9d2" stroke="#c9762b" '
      'stroke-width="1.8"/>' % (X(c[0] - 16.75), Y(13.75), 33.5 * SC, 58.5 * SC))
    A('<text x="%.0f" y="%.0f" font-size="11" fill="#8a4b10" text-anchor="middle">%s</text>'
      % (X(c[0]), Y(-30), lbl))
    A('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="none" stroke="#c9762b" stroke-width="1.2"/>'
      % (X(c[0]), Y(c[1]), 13 * SC))
for p, k in ((P1, K1), (P2, K2)):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#4a6fa5" stroke-width="9" '
      'stroke-linecap="round"/>' % (X(p[0]), Y(p[1]), X(k[0]), Y(k[1])))
for k in (K1, K2):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#2f9e8f" stroke-width="6" '
      'stroke-linecap="round"/>' % (X(k[0]), Y(k[1]), X(F[0]), Y(F[1])))
n = 11
x0, y0, x1, y1 = X(F[0]), Y(F[1]), X(FT[0]), Y(FT[1])
pts = ['%.0f,%.0f' % (x0, y0)]
for i in range(1, n):
    pts.append('%.0f,%.0f' % (x0 + (-1) ** i * 11, y0 + (y1 - y0) * i / n))
pts.append('%.0f,%.0f' % (x1, y1))
A('<polyline points="%s" fill="none" stroke="#c9762b" stroke-width="2.4"/>' % ' '.join(pts))
A('<rect x="%.0f" y="%.0f" width="%.0f" height="%.0f" rx="2" fill="#5b6470" stroke="#333"/>'
  % (X(-25), Y(-225), 50 * SC, 6 * SC))
A('<line x1="%.0f" y1="%.0f" x2="%.0f" y2="%.0f" stroke="#999" stroke-width="2"/>'
  % (X(-62), Y(-231), X(62), Y(-231)))
for gx in range(-58, 62, 12):
    A('<line x1="%.0f" y1="%.0f" x2="%.0f" y2="%.0f" stroke="#c8c8c8"/>'
      % (X(gx), Y(-231), X(gx - 7), Y(-238)))
for c, lbl, dx, dy in ((P1, 'P1', -32, -6), (P2, 'P2', 13, -6), (K1, 'K1', -32, 5),
                       (K2, 'K2', 13, 5), (F, 'F', 15, 5)):
    A('<circle cx="%.1f" cy="%.1f" r="4.5" fill="#fff" stroke="#222" stroke-width="1.6"/>'
      % (X(c[0]), Y(c[1])))
    A('<text x="%.0f" y="%.0f" font-size="11.5" font-weight="600">%s</text>'
      % (X(c[0]) + dx, Y(c[1]) + dy, lbl))


def dim(ax, ay, bx, by, label, off=0.0, side=1):
    dx, dy = bx - ax, by - ay
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    nx, ny = -uy * off * side, ux * off * side
    A('<line x1="%.0f" y1="%.0f" x2="%.0f" y2="%.0f" stroke="#5b6470" stroke-width="1" '
      'marker-start="url(#dim2)" marker-end="url(#dim)"/>' % (ax + nx, ay + ny, bx + nx, by + ny))
    A('<text x="%.0f" y="%.0f" font-size="11" fill="#5b6470" text-anchor="middle">%s</text>'
      % ((ax + bx) / 2 + nx, (ay + by) / 2 + ny - 5, label))


dim(X(P1[0]), Y(24), X(P2[0]), Y(24), 'd0 = 40')
dim(X(P1[0]), Y(P1[1]), X(K1[0]), Y(K1[1]), 'l1 = 90', 20, 1)
dim(X(K1[0]), Y(K1[1]), X(F[0]), Y(F[1]), 'l2 = 130', 27, 1)
A('<line x1="%.0f" y1="%.0f" x2="%.0f" y2="%.0f" stroke="#5b6470" stroke-width="1" '
  'marker-start="url(#dim2)" marker-end="url(#dim)"/>' % (X(122), Y(0), X(122), Y(-145)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#5b6470">先端長 145</text>' % (X(126), Y(-72)))
A('<line x1="%.0f" y1="%.0f" x2="%.0f" y2="%.0f" stroke="#5b6470" stroke-width="1" '
  'marker-start="url(#dim2)" marker-end="url(#dim)"/>' % (X(122), Y(-145), X(122), Y(-225)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#5b6470">バネ 80</text>' % (X(126), Y(-185)))
A('<line x1="%.0f" y1="%.0f" x2="%.0f" y2="%.0f" stroke="#5b6470" stroke-width="1" '
  'marker-start="url(#dim2)" marker-end="url(#dim)"/>' % (X(-132), Y(0), X(-132), Y(-225)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#5b6470">L0 = 240</text>' % (X(-186), Y(-106)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#5b6470">（股→接地）</text>' % (X(-192), Y(-120)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#4a6fa5">θ1 = −143.5°</text>' % (X(-62), Y(-52)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#4a6fa5">θ2 = −36.5°</text>' % (X(16), Y(-52)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#2f9e8f">内角 90.5°</text>' % (X(19), Y(-133)))
A('<text x="%.0f" y="%.0f" font-size="11" fill="#333">ゴムパッド φ50</text>' % (X(30), Y(-222)))
for i, t in enumerate([
        '①② XM540-W270（脚長、10.6 N·m / 30 rpm）　③ XH540-W150（ロール、7.1 N·m / 70 rpm）',
        'P1 / P2: サーボ出力軸（駆動）　K1 / K2: 膝（受動）　F: 足先（下腿 2 本が合流 ＝ 閉ループ）',
        '下腿は膝と足先の 2 点だけで支持される「二力部材」で、軸力しか掛からない',
        '（曲げの設計が要らない）。膝は受動関節なので、脚にサーボが載らない。',
        '2 個のサーボが荷重を分担し、脚長と前後の振りを同時に作る。']):
    A('<text x="40" y="%d" font-size="11.5" fill="#5b6470">%s</text>' % (528 + i * 19, t))

# ======== B: Z 方向の層構成 ========
BX, BY = 560, 80
A('<text x="%d" y="%d" font-size="14" font-weight="bold">'
  'B. Z 方向の層構成（脚長サーボの軸に沿った断面）</text>' % (BX, BY))
z0 = BX + 24
zx = lambda z: z0 + (z + 44) * 4.15
leg = []
for a, b, fill, st, lbl in ((-44, 0, '#ffe9d2', '#c9762b', 'サーボ本体 44'),
                            (0, 2.6, '#f6d5b0', '#c9762b', 'ホーン'),
                            (2.6, 8.6, '#dbe6f3', '#4a6fa5', 'クランク 6'),
                            (8.6, 12.6, '#e8e2f6', '#8e5bd6', '軸受'),
                            (12.6, 16.6, '#e3e7ec', '#5b6470', '外側板')):
    A('<rect x="%.0f" y="%d" width="%.0f" height="52" fill="%s" stroke="%s" stroke-width="1.6"/>'
      % (zx(a), BY + 26, zx(b) - zx(a), fill, st))
    cx = (zx(a) + zx(b)) / 2
    A('<line x1="%.0f" y1="%d" x2="%.0f" y2="%d" stroke="#aaa" stroke-width="0.8"/>'
      % (cx, BY + 78, cx, BY + 86 + 15 * len(leg)))
    leg.append((cx, lbl, st))
for i, (cx, lbl, st) in enumerate(leg):
    A('<text x="%.0f" y="%d" font-size="10.5" fill="%s">%s</text>' % (cx + 4, BY + 90 + 15 * i, st, lbl))
A('<line x1="%.0f" y1="%d" x2="%.0f" y2="%d" stroke="#c0392b" stroke-width="1.6" '
  'stroke-dasharray="8 3 2 3"/>' % (zx(-48), BY + 52, zx(21), BY + 52))
A('<text x="%.0f" y="%d" font-size="11" fill="#c0392b">サーボ軸（Z）</text>' % (zx(21) + 5, BY + 56))
BY = BY + 68
A('<text x="%d" y="%d" font-size="11.5" font-weight="600" fill="#c0392b">'
  'クランクを片持ちにしない</text>' % (BX, BY + 118))
for i, t in enumerate([
        '設計荷重ではクランク先端に 107 N。ホーン側だけで受けるとサーボ出力軸に',
        'ラジアル荷重が集中する。外側にもう 1 枚板を置き、軸受で受けて両持ちにする',
        '（コの字フレーム）。脚長サーボ 2 個は軸が平行なので外側板は共用できる。']):
    A('<text x="%d" y="%d" font-size="11.5" fill="#5b6470">%s</text>' % (BX, BY + 138 + i * 17, t))

# ======== C: 関節の断面 ========
CX, CY = 560, 348
A('<text x="%d" y="%d" font-size="14" font-weight="bold">'
  'C. 膝 K・足先 F の関節（二面せん断 = クレビス）</text>' % (CX, CY))
cx0, cy0 = CX + 70, CY + 34
A('<rect x="%d" y="%d" width="150" height="16" fill="#dbe6f3" stroke="#4a6fa5" stroke-width="1.6"/>'
  % (cx0, cy0))
A('<rect x="%d" y="%d" width="150" height="16" fill="#dbe6f3" stroke="#4a6fa5" stroke-width="1.6"/>'
  % (cx0, cy0 + 58))
A('<rect x="%d" y="%d" width="26" height="74" fill="#dbe6f3" stroke="#4a6fa5" stroke-width="1.6"/>'
  % (cx0, cy0))
A('<rect x="%d" y="%d" width="150" height="26" fill="#d6efe9" stroke="#2f9e8f" stroke-width="1.6"/>'
  % (cx0 + 96, cy0 + 24))
for yy in (cy0 + 24, cy0 + 44):
    A('<rect x="%d" y="%d" width="26" height="6" fill="#f3d9d9" stroke="#c0392b" stroke-width="1.2"/>'
      % (cx0 + 118, yy))
A('<rect x="%d" y="%d" width="18" height="14" rx="2" fill="#9aa4b0" stroke="#444" stroke-width="1.4"/>'
  % (cx0 + 122, cy0 - 14))
A('<rect x="%d" y="%d" width="14" height="74" fill="#b9c2cc" stroke="#444" stroke-width="1.4"/>'
  % (cx0 + 124, cy0))
A('<rect x="%d" y="%d" width="10" height="13" fill="#7f8894" stroke="#444" stroke-width="1.2"/>'
  % (cx0 + 126, cy0 + 74))
A('<text x="%d" y="%d" font-size="11.5" fill="#333">段付ボルト（φ4 軸 / M3 ねじ）</text>'
  % (cx0 + 150, cy0 - 4))
A('<text x="%d" y="%d" font-size="11.5" fill="#333">ねじ部で締結</text>' % (cx0 + 150, cy0 + 92))
A('<text x="%d" y="%d" font-size="11.5" fill="#4a6fa5">クランク</text>' % (cx0 - 62, cy0 + 14))
A('<text x="%d" y="%d" font-size="11.5" fill="#4a6fa5">（二股）</text>' % (cx0 - 62, cy0 + 30))
A('<text x="%d" y="%d" font-size="11.5" fill="#2f9e8f">下腿</text>' % (cx0 + 252, cy0 + 42))
A('<text x="%d" y="%d" font-size="11.5" fill="#c0392b">軸受</text>' % (cx0 + 150, cy0 + 46))
for i, t in enumerate([
        '強度は φ3 でも十分（二面せん断 7.6 MPa、面圧 8.9 MPa）。寸法を決めるのは',
        '軸受の内径とガタ。樹脂すべり軸受なら PV 0.07（許容 0.3〜1）で余裕があり、',
        '1 個 0.15 g と軽い。ガタが問題になったら 684ZZ（φ4×φ9×4、1.5 g）に置換。']):
    A('<text x="%d" y="%d" font-size="11.5" fill="#5b6470">%s</text>' % (CX, CY + 140 + i * 17, t))
A('<text x="%d" y="%d" font-size="11.5" font-weight="600" fill="#c0392b">片持ちは不可</text>'
  % (CX, CY + 200))
A('<text x="%d" y="%d" font-size="11.5" fill="#5b6470">'
  ': 軸に曲げが入るうえ、リンク面が Z 方向にずれて</text>' % (CX + 88, CY + 200))
A('<text x="%d" y="%d" font-size="11.5" fill="#5b6470">'
  '接地力の作用線が脚面から外れる（3D で転倒条件になった問題）。</text>' % (CX, CY + 217))
A('<rect x="%d" y="%d" width="452" height="82" fill="#fdf6e8" stroke="#c98a12" stroke-width="1.2" rx="4"/>'
  % (CX, CY + 238))
A('<text x="%d" y="%d" font-size="11.5" font-weight="600" fill="#8a5d00">'
  '質量制約: 下腿＋足で 50 g（150 g で転倒）</text>' % (CX + 12, CY + 259))
A('<text x="%d" y="%d" font-size="11.5" fill="#5b6470">'
  '実部品で積むと 54 g。重いのはバネ 14 g と段付ボルト 3 本で 8.7 g。</text>' % (CX + 12, CY + 277))
A('<text x="%d" y="%d" font-size="11.5" fill="#5b6470">'
  'バネを硬くする（k 900 → 2000）と 5.6 g に減り 48 g に収まる。</text>' % (CX + 12, CY + 294))
A('<text x="%d" y="%d" font-size="11" fill="#8a5d00">'
  '※ 50 g と 150 g の間は未検証。実質量が出たらシミュレーションで確認できる</text>' % (CX + 12, CY + 311))
A('</svg>')

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img',
                    'hopper-leg-structure.svg')
with open(path, 'w', encoding='utf-8') as fp:
    fp.write('\n'.join(out))
print('written:', os.path.normpath(path))
print('K1 (%.1f, %.1f)  K2 (%.1f, %.1f)  F (%.1f, %.1f)' % (K1 + K2 + F))
