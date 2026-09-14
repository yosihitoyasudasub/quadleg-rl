# -*- coding: utf-8 -*-
"""docs/img/hopper-bisector-parts.svg を生成する。

菱形リンク（docs/hopper-mechanical-spec.md §7-2）の部品図。
A. 小リンク  B. シャンクの追加穴  C. 足先ブロックの腕  D. ピンまわりの組立断面
機構全体の動きは img/hopper-bisector.png（tools/make_bisector_diagram.py）を見ること。
"""
import math
import os

W, H = 1280, 1020
out = []
A = out.append

SH, SH_F = '#3f8f57', '#d8efdc'      # シャンク
LK, LK_F = '#c9762b', '#ffe0b2'      # 小リンク・ブッシュ
FB, FB_F = '#8e5bd6', '#efe3f7'      # 足先ブロック・腕
PN, PN_F = '#5b6470', '#c8ccd2'      # ピン
WS_F = '#e8eaed'
DIM = '#5b6470'
RED = '#c1121f'


def txt(x, y, s, size=12, fill='#222', anchor='start', weight='normal'):
    A('<text x="%.1f" y="%.1f" font-size="%s" fill="%s" text-anchor="%s" font-weight="%s">%s</text>'
      % (x, y, size, fill, anchor, weight, s))


def line(x1, y1, x2, y2, s, sw=1.4, dash=''):
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="%.1f" %s/>'
      % (x1, y1, x2, y2, s, sw, ('stroke-dasharray="%s"' % dash) if dash else ''))


def rect(x, y, w, h, f, s, sw=1.4, rx=0):
    A('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="%.1f" fill="%s" stroke="%s" '
      'stroke-width="%.1f"/>' % (x, y, w, h, rx, f, s, sw))


def circ(x, y, r, f, s, sw=1.4):
    A('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="%s" stroke="%s" stroke-width="%.1f"/>'
      % (x, y, r, f, s, sw))


def cl(x1, y1, x2, y2):                       # 中心線
    line(x1, y1, x2, y2, '#b9414b', 0.9, '9 3 2 3')


def dimh(x1, x2, y, label, ext=None, size=10.5):
    """水平寸法。ext は寸法補助線を引く元の y"""
    if ext is not None:
        for x in (x1, x2):
            line(x, ext, x, y + (4 if y > ext else -4), DIM, 0.7)
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="0.9" '
      'marker-start="url(#a2)" marker-end="url(#a1)"/>' % (x1, y, x2, y, DIM))
    txt((x1 + x2) / 2, y - 4, label, size, DIM, 'middle')


def dimv(y1, y2, x, label, ext=None, side=1, size=10.5):
    if ext is not None:
        for y in (y1, y2):
            line(ext, y, x + (4 if x > ext else -4), y, DIM, 0.7)
    A('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="0.9" '
      'marker-start="url(#a2)" marker-end="url(#a1)"/>' % (x, y1, x, y2, DIM))
    txt(x + 5 * side, (y1 + y2) / 2 + 4, label, size, DIM, 'start' if side > 0 else 'end')


def lead(x0, y0, x1, y1, s, col=DIM, size=10.5, anchor='start'):
    line(x0, y0, x1, y1, col, 0.9)
    txt(x1 + (4 if anchor == 'start' else -4), y1 + 4, s, size, col, anchor)


A('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
  'font-family="Segoe UI, Meiryo, sans-serif" font-size="12">' % (W, H, W, H))
A('<defs>'
  '<marker id="a1" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">'
  '<path d="M0,0.5 L8,4 L0,7.5 z" fill="%s"/></marker>'
  '<marker id="a2" markerWidth="8" markerHeight="8" refX="1" refY="4" orient="auto">'
  '<path d="M8,0.5 L0,4 L8,7.5 z" fill="%s"/></marker></defs>' % (DIM, DIM))
rect(0, 0, W, H, '#ffffff', 'none', 0)
txt(22, 34, '菱形リンクの部品図（二等分線方式・§7-2）', 19, '#111', 'start', 'bold')
txt(22, 56, '機構としての動きは img/hopper-bisector.png を見ること。ここは作るための寸法。'
            '単位 mm、板厚方向を z とする', 12.5, DIM)

# ==================================================================
# A. 小リンク
# ==================================================================
txt(40, 96, 'A. 小リンク（2 枚。左右で同じ形）', 14.5, '#111', 'start', 'bold')
S = 5.6
ax0, ay0 = 120.0, 175.0                      # A1 側の穴中心
ax1 = ax0 + 30 * S
# 正面図（ステーディアム）
A('<path d="M %.1f %.1f L %.1f %.1f A %.1f %.1f 0 0 1 %.1f %.1f L %.1f %.1f '
  'A %.1f %.1f 0 0 1 %.1f %.1f Z" fill="%s" stroke="%s" stroke-width="1.6"/>'
  % (ax0, ay0 - 4 * S, ax1, ay0 - 4 * S, 4 * S, 4 * S, ax1, ay0 + 4 * S,
     ax0, ay0 + 4 * S, 4 * S, 4 * S, ax0, ay0 - 4 * S, LK_F, LK))
for x in (ax0, ax1):
    circ(x, ay0, 1.5 * S, '#ffffff', LK, 1.4)
    cl(x, ay0 - 6 * S, x, ay0 + 6 * S)
cl(ax0 - 6 * S, ay0, ax1 + 6 * S, ay0)
dimh(ax0, ax1, ay0 - 5.2 * S, '30 ±0.05', ay0 - 4 * S)
dimv(ay0 - 4 * S, ay0 + 4 * S, ax1 + 5.6 * S, '8', ax1 + 4 * S, 1)
lead(ax0 - 1.06 * S, ay0 - 1.06 * S, ax0 - 5.5 * S, ay0 - 3.4 * S, 'φ3 H7 × 2', LK, 10.5, 'end')
lead(ax1 + 4 * S, ay0 - 2.8 * S, ax1 + 2.6 * S, ay0 - 6.2 * S, 'R4', LK, 10.5, 'start')
# 側面図
by = ay0 + 8.4 * S
rect(ax0 - 4 * S, by, 38 * S, 2 * S, LK_F, LK, 1.6)
for x in (ax0, ax1):
    cl(x, by - 1.5 * S, x, by + 3.5 * S)
dimv(by, by + 2 * S, ax1 + 5.6 * S, 't2', ax0 + 34 * S, 1)
txt(ax0 - 4 * S, by + 4.4 * S, '材質 A2017。二力部材（力は軸方向のみ）', 11, '#333')
txt(ax0 - 4 * S, by + 4.4 * S + 16, '軸力 140 N → 引張 14 MPa、穴の面圧 23 MPa、ピンせん断 10 MPa', 11, '#333')
txt(ax0 - 4 * S, by + 4.4 * S + 32, '質量 1.54 g / 枚', 11, '#333')
txt(ax0 - 4 * S, by + 4.4 * S + 52, '※ 穴間 30 mm の精度が向きの精度を決める。', 11, RED, 'start', 'bold')
txt(ax0 - 4 * S, by + 4.4 * S + 68, '　 4 辺が等しくないと二等分線からずれる（3° で破綻）', 11, RED)

# ==================================================================
# B. シャンクの追加穴
# ==================================================================
txt(500, 96, 'B. シャンクに穴を 1 つ足す（既存部品の変更）', 14.5, '#111', 'start', 'bold')
S2 = 3.4
bx0, by0 = 580.0, 185.0                      # F の穴中心
bx1 = bx0 + 30 * S2                          # A の穴中心
mx = lambda v: bx0 + v * S2
my = lambda v: by0 - v * S2
# 足先側のボス（幅 15、左端 R7.5）→ テーパ → 主スパン（幅 10）
A('<path d="M %.1f %.1f L %.1f %.1f L %.1f %.1f L %.1f %.1f L %.1f %.1f L %.1f %.1f '
  'L %.1f %.1f L %.1f %.1f A %.1f %.1f 0 0 1 %.1f %.1f Z" '
  'fill="%s" stroke="%s" stroke-width="1.6"/>'
  % (mx(0), my(7.5), mx(9), my(7.5), mx(17), my(5), mx(40), my(5),
     mx(40), my(-5), mx(17), my(-5), mx(9), my(-7.5), mx(0), my(-7.5),
     7.5 * S2, 7.5 * S2, mx(0), my(7.5), SH_F, SH))
line(mx(9), my(-7.5), mx(9), my(-5), SH, 1.0, '4 3')
line(mx(9), my(7.5), mx(9), my(5), SH, 1.0, '4 3')
line(mx(40), my(5), mx(40), my(-5), SH, 1.0, '5 4')
circ(bx0, by0, 3 * S2, '#ffffff', SH, 1.4)
circ(bx1, by0, 1.5 * S2, '#ffffff', RED, 1.8)
cl(mx(-11), by0, mx(43), by0)
for x in (bx0, bx1):
    cl(x, my(11), x, my(-11))
dimh(bx0, bx1, my(9.5), '30 ±0.05', my(7.5))
lead(bx0 - 2.1 * S2, my(-2.1), mx(-4), my(-10), 'F（既存 φ6 H7）', SH, 10.5, 'end')
lead(bx1, my(1.5), mx(26), my(12), 'A1 / A2（追加 φ3 H7）', RED, 10.5, 'start')
txt(mx(43), by0 + 4, '→ 膝 K へ', 10.5, SH, 'start')
txt(500, my(-17), '穴はシャンクの中心線（K–F を結ぶ線）上。左右とも同じ位置。', 11, '#333')
txt(500, my(-17) + 17, '主スパン 幅 10 に対して縁肉 3.5 mm。§5-2 の断面のまま開けられる。', 11, '#333')
txt(500, my(-17) + 34, '足先側のボス（幅 15）には掛からない。', 11, '#333')

# ==================================================================
# C. 足先ブロックの腕
# ==================================================================
txt(860, 96, 'C. 足先ブロックの腕（2 枚。両方の耳を延長する）', 14.5, '#111', 'start', 'bold')
S3 = 4.6
cx0, cy0 = 960.0, 480.0                      # F の穴中心（下端）
# 腕（F から上へ 60、幅 8。下端はブロック本体につながる）
A('<path d="M %.1f %.1f L %.1f %.1f A %.1f %.1f 0 0 1 %.1f %.1f L %.1f %.1f Z" '
  'fill="%s" stroke="%s" stroke-width="1.6"/>'
  % (cx0 - 4 * S3, cy0, cx0 - 4 * S3, cy0 - 56 * S3, 4 * S3, 4 * S3,
     cx0 + 4 * S3, cy0 - 56 * S3, cx0 + 4 * S3, cy0, FB_F, FB))
rect(cx0 - 9 * S3, cy0, 18 * S3, 10 * S3, FB_F, FB, 1.6, 2)
txt(cx0, cy0 + 6 * S3, 'ブロック本体へ', 10.5, FB, 'middle')
# スロット（幅 3、丸端の中心を F から 40 と 55）
A('<path d="M %.1f %.1f L %.1f %.1f A %.1f %.1f 0 0 1 %.1f %.1f L %.1f %.1f '
  'A %.1f %.1f 0 0 1 %.1f %.1f Z" fill="#ffffff" stroke="%s" stroke-width="1.5"/>'
  % (cx0 - 1.5 * S3, cy0 - 40 * S3, cx0 - 1.5 * S3, cy0 - 55 * S3,
     1.5 * S3, 1.5 * S3, cx0 + 1.5 * S3, cy0 - 55 * S3,
     cx0 + 1.5 * S3, cy0 - 40 * S3, 1.5 * S3, 1.5 * S3,
     cx0 - 1.5 * S3, cy0 - 40 * S3, FB))
circ(cx0, cy0, 3 * S3, '#ffffff', FB, 1.4)
cl(cx0, cy0 + 6 * S3, cx0, cy0 - 62 * S3)
for v in (40, 55):
    cl(cx0 - 6 * S3, cy0 - v * S3, cx0 + 6 * S3, cy0 - v * S3)
cl(cx0 - 11 * S3, cy0, cx0 + 11 * S3, cy0)
dimv(cy0 - 40 * S3, cy0, cx0 - 6.5 * S3, '40', cx0 - 1.5 * S3, -1)
dimv(cy0 - 55 * S3, cy0, cx0 - 11.5 * S3, '55', cx0 - 1.5 * S3, -1)
dimv(cy0 - 60 * S3, cy0, cx0 + 8.5 * S3, '60', cx0 + 4 * S3, 1)
dimh(cx0 - 4 * S3, cx0 + 4 * S3, cy0 - 64 * S3, '8', cy0 - 56 * S3)
lead(cx0 + 1.5 * S3, cy0 - 52 * S3, cx0 + 9 * S3, cy0 - 56 * S3, 'スロット 幅 3', FB)
lead(cx0 + 1.5 * S3, cy0 - 44 * S3, cx0 + 9 * S3, cy0 - 40 * S3, 'C のピン φ3 が滑る', FB)
txt(cx0 + 13 * S3, cy0 - 40 * S3 + 20, '実移動 42.2 〜 53.1', 10.5, RED, 'start', 'bold')
txt(cx0 + 13 * S3, cy0 - 40 * S3 + 36, '（両端に 2 mm の余裕）', 10.5, RED, 'start')
lead(cx0 - 2.2 * S3, cy0, cx0 - 12 * S3, cy0 + 3.2 * S3, 'F（φ6 H7）', FB, 10.5, 'end')
txt(860, 560, '厚さ 2.5。ブロックの二股の耳を両方とも上へ延ばす（z = ±4.75〜±7.25）。', 11, '#333')
txt(860, 576, '材質はブロックと同じ。質量 1.3 g × 2 枚。', 11, '#333')
txt(860, 594, '1 枚だけだとピン C の偶力を受けられない（D を見ること）', 11, RED, 'start', 'bold')

# ==================================================================
# D. ピンまわりの組立断面
# ==================================================================
txt(40, 614, 'D. ピンまわりの組立断面（z 方向。上が膝側）', 14.5, '#111', 'start', 'bold')
SZ, SV = 11.0, 3.4
L_C = [('リンク①', -9.5, -7.5, LK_F, LK, 12, -12),
       ('腕①', -7.25, -4.75, FB_F, FB, 14, -10),
       ('シャンク①', -4.25, -0.25, SH_F, SH, 6, -18),
       ('シャンク②', 0.25, 4.25, SH_F, SH, 6, -18),
       ('腕②', 4.75, 7.25, FB_F, FB, 14, -10),
       ('リンク②', 7.5, 9.5, LK_F, LK, 12, -12)]
L_A = [('シャンク②', 0.25, 4.25, SH_F, SH, 6, -18),
       ('リンク②', 7.5, 9.5, LK_F, LK, 12, -12)]


def zsec(X, Y, title, sub, layers, through, spacer=None, slots=()):
    zx = lambda z: X + z * SZ
    vy = lambda v: Y - v * SV
    txt(X, 646, title, 12.5, '#111', 'middle', 'bold')
    txt(X, 662, sub, 10.5, DIM, 'middle')
    for name, z0, z1, fc, sc, top, bot in layers:
        rect(zx(z0), vy(top), (z1 - z0) * SZ, (top - bot) * SV, fc, sc, 1.3)
        txt(zx((z0 + z1) / 2), vy(bot) + 13, name, 9.5, sc, 'middle')
    for z0 in slots:
        rect(zx(z0), vy(10), 2.5 * SZ, 16 * SV, '#ffffff', FB, 1.0)
    z0, z1 = through
    rect(zx(z0), vy(1.5), (z1 - z0) * SZ, 3 * SV, PN_F, PN, 1.3)
    if spacer is not None:
        rect(zx(spacer[0]), vy(1.5), (spacer[1] - spacer[0]) * SZ, 3 * SV, WS_F, '#9aa0a6', 1.1)
    return zx, vy


zx, vy = zsec(300.0, 725.0, 'ピン C（左右対称に受ける）',
              'リンク① ＋ 腕① ＋ 〔空〕＋ 腕② ＋ リンク②',
              L_C, (-10.7, 10.7), slots=(-7.25, 4.75))
for z in (-10.9, 10.5):
    rect(zx(z) - 0.25 * SZ, vy(3.5), 0.5 * SZ, 7 * SV, PN_F, PN, 1.0)
lead(zx(-10.9), vy(5), zx(-10.9) - 10, vy(5) - 40, 'E リング × 2', PN, 10.5, 'end')
txt(300.0, 820, 'φ3 × 22。リンク①(−8.5) を 腕①(−6) が、リンク②(+8.5) を 腕②(+6) が受ける。',
    10.5, '#333', 'middle')
txt(300.0, 836, 'スパン 2.5 mm、曲げ 93 MPa（SF 2.2）。スロットの中を滑るので締め付けない。',
    10.5, '#333', 'middle')

zx, vy = zsec(880.0, 725.0, 'ピン A2（A1 は鏡像）', 'シャンク② ＋ スペーサ ＋ リンク②',
              L_A, (-0.5, 10.5), spacer=(4.25, 7.5))
lead(zx(5.9), vy(3), zx(5.9) + 22, vy(3) - 40, 'スペーサ 3.25', '#9aa0a6')
txt(880.0, 820, 'φ3 × 12。腕はこの位置（F から 30 mm、シャンクの上）には無いので、', 10.5, '#333', 'middle')
txt(880.0, 836, 'スペーサはリンクを層の位置に保つためだけ。A1 側も同じ。', 10.5, '#333', 'middle')

# ==================================================================
# まとめ
# ==================================================================
rect(30, 856, W - 60, 148, '#f7f9fb', '#c9d3de', 1.2)
txt(50, 880, '組む順番と確認', 13.5, '#111', 'start', 'bold')
for i, s in enumerate([
    '1. シャンク①② に φ3 の穴（F から 30）を開ける。2. 小リンクを 2 枚作る（穴間 30）。'
    '3. 足先ブロックに腕とスロットを付ける。4. ジョイントを組む。',
    '   ジョイント: A1・A2・C は回転、腕と C は「ピン-スロット」（スライダにすると拘束が 1 つ多く、'
    '過拘束になる）。F は既存の回転に足先ブロックを足す。',
    '   検算: 菱形は対辺が平行なので、リンク①（A1→C）は常にシャンク②（F→K2）と平行。'
    '全姿勢で差 0.000°。Fusion の計測で確かめられる。',
    '   質量: 小リンク 1.54×2 ＋ 腕 1.3×2 ＋ ピン C 1.23 ＋ ピン A 0.67×2 ＋ スペーサ 0.18×2 ＋ E リング 0.1 ＝ '
    '8.7 g が足の質量に加わる。',
]):
    txt(50, 902 + i * 22, s, 11.5, '#333')
txt(50, 994, '※ 精度が効くのは「4 辺が等しいこと」だけ。穴間 30 mm を左右 4 箇所で揃える。'
             '絶対位置の精度は要らない。', 11.5, RED, 'start', 'bold')

A('</svg>')
svg = '\n'.join(out)
dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img')
os.makedirs(dst, exist_ok=True)
pth = os.path.join(dst, 'hopper-bisector-parts.svg')
with open(pth, 'w', encoding='utf-8') as fh:
    fh.write(svg)
print('wrote', os.path.normpath(pth), len(svg), 'bytes')
