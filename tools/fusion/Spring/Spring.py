# -*- coding: utf-8 -*-
"""圧縮コイルばねを生成する Fusion 360 スクリプト。

既定値は採用品 **サミニ 11-1437**（docs/hopper-mechanical-spec.md §7-4）。
メーカーの 3D モデルが無いので、カタログ値から作る。

  線径 φ1.6 / 外径 φ14 / 自由長 100 mm / ばね定数 1.56 N/mm / 全タワミ 63 mm / 端面研削あり

**圧縮状態を指定できる**のがポイント。DEFLECTION を変えれば、その縮み量での形状が出る。
§7-3 の CAD 確認項目「バネ機構 ↔ 下腿（ストローク全域で）」はこれで見る。

  DEFLECTION = 0     自由長（組付け前）
  DEFLECTION = 9.4   静止立位
  DEFLECTION = 47.5  落下 10 cm（常用の最大）
  DEFLECTION = 58    底付きストッパ位置

形状のモデル化:
  死巻（上下 1 巻ずつ）はピッチ 0 の平らな巻き、有効部は等ピッチ。
  中心線の上昇 = p·Na、ソリッド高さ = p·Na + d となるので p = (自由長 − d) / Na とする。
  これで自由長がカタログ値どおりになり、端面も平ら（研削端の座面）になる。

使い方:
  1. このフォルダを %APPDATA%\\Autodesk\\Autodesk Fusion 360\\API\\Scripts\\Spring\\ に置く
     （AddIns\\ に置いてもスクリプトとして実行できる）
  2. Fusion で ユーティリティ → アドイン → スクリプト → Spring → 実行
  ※ 寸法を変えるときは下の「パラメータ」だけ直す
  ※ パーツ デザイン ドキュメントでもアセンブリでも動く
     （前者は 1 コンポーネントしか持てないのでルートに直接作る）
"""

import math
import traceback

import adsk.core
import adsk.fusion

# ---------------- パラメータ（mm）----------------
WIRE_D = 1.6          # 線径
OUTER_D = 14.0        # 外径
FREE_L = 100.0        # 自由長
SPRING_K = 1.56       # ばね定数 [N/mm]。総巻数を逆算するのに使う
TOTAL_COILS = None    # 総巻数を直接指定するならここに入れる（None なら SPRING_K から逆算）
DEFLECTION = 0.0      # 縮み量。0 なら自由長
G_MODULUS = 78500.0   # 横弾性係数 [N/mm²]（ピアノ線 SWP-A）
RIGHT_HAND = True     # 巻き方向
PTS_PER_TURN = 24     # スプラインの 1 巻あたり点数。粗くすると軽いが多角形になる
NAME = 'Spring 11-1437'

MM = 0.1              # Fusion API の内部単位は cm


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('デザインドキュメントを開いてから実行してください。')
            return

        d = WIRE_D
        Dm = OUTER_D - d                       # コイル平均径
        if TOTAL_COILS is not None:
            Nt = float(TOTAL_COILS)
            Na = Nt - 2.0
        else:
            Na = G_MODULUS * d ** 4 / (8.0 * Dm ** 3 * SPRING_K)   # 有効巻数
            Nt = Na + 2.0                                          # ＋死巻 2 巻
        solid_l = Nt * d
        height = FREE_L - DEFLECTION
        if height < solid_l:
            ui.messageBox('縮めすぎです。密着長 %.1f mm に対して高さ %.1f mm。'
                          % (solid_l, height))
            return
        pitch = (height - d) / Na              # 有効部のピッチ（死巻はピッチ 0）

        # ---- 中心線の点列 ----
        r = Dm / 2.0
        sign = 1.0 if RIGHT_HAND else -1.0
        z0 = d / 2.0
        n = max(8, int(round(PTS_PER_TURN)))
        pts = adsk.core.ObjectCollection.create()
        steps = int(round(Nt * n))
        for i in range(steps + 1):
            turn = Nt * i / steps              # 0 〜 Nt [巻]
            if turn <= 1.0:                    # 下の死巻：平ら
                z = z0
            elif turn <= 1.0 + Na:             # 有効部：等ピッチ
                z = z0 + pitch * (turn - 1.0)
            else:                              # 上の死巻：平ら
                z = z0 + pitch * Na
            a = sign * 2.0 * math.pi * turn
            pts.add(adsk.core.Point3D.create(r * math.cos(a) * MM,
                                             r * math.sin(a) * MM,
                                             z * MM))

        # ---- 作る場所 ----
        # 「パーツ デザイン ドキュメント」は 1 コンポーネントしか持てないので、
        # 新規コンポーネントの作成に失敗したらルートに直接作る。
        root = design.rootComponent
        try:
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = NAME
        except Exception:
            comp = root

        sk_path = comp.sketches.add(comp.xYConstructionPlane)
        sk_path.name = 'coil path'
        spline = sk_path.sketchCurves.sketchFittedSplines.add(pts)

        # ---- 掃引の断面（経路始点で経路に垂直な面に描く）----
        planes = comp.constructionPlanes
        pin = planes.createInput()
        pin.setByDistanceOnPath(spline, adsk.core.ValueInput.createByReal(0.0))
        plane = planes.add(pin)
        plane.name = 'wire section'

        sk_sec = comp.sketches.add(plane)
        sk_sec.name = 'wire'
        sk_sec.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), d / 2.0 * MM)

        # ---- 掃引 ----
        path = adsk.fusion.Path.create(spline, adsk.fusion.ChainedCurveOptions.noChainedCurves)
        sw_in = comp.features.sweepFeatures.createInput(
            sk_sec.profiles.item(0), path,
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        sweep = comp.features.sweepFeatures.add(sw_in)
        body = sweep.bodies.item(0)
        body.name = NAME

        # 材質（質量を §9 に返すために入れておく）
        try:
            lib = app.materialLibraries.itemByName('Fusion Material Library')
            mat = lib.materials.itemByName('Steel')
            if mat:
                body.material = mat
        except Exception:
            pass

        sk_path.isLightBulbOn = False
        sk_sec.isLightBulbOn = False
        plane.isLightBulbOn = False

        # ---- 結果 ----
        wire_len = Nt * math.sqrt((math.pi * Dm) ** 2 + pitch ** 2)
        vol = wire_len * math.pi / 4.0 * d * d
        load = SPRING_K * DEFLECTION
        where = 'ルートに直接' if comp is root else '新規コンポーネントに'
        ui.messageBox(
            '%s を%s生成しました。\n\n'
            '線径 φ%.1f / 外径 φ%.1f / 内径 φ%.1f / 平均径 φ%.2f\n'
            '有効巻数 %.2f / 総巻数 %.2f / 密着長 %.1f mm\n'
            '自由長 %.1f mm → 縮み %.1f mm で高さ %.1f mm\n'
            'そのときのピッチ %.3f mm（コイル隙間 %.3f mm）/ 荷重 %.1f N\n'
            '線材長さ %.0f mm / 体積 %.0f mm³ / 質量（鋼 7.85）%.2f g'
            % (NAME, where, d, OUTER_D, Dm - d, Dm, Na, Nt, solid_l,
               FREE_L, DEFLECTION, height, pitch, pitch - d, load,
               wire_len, vol, vol * 7.85e-3))

    except Exception:
        if ui:
            ui.messageBox('失敗しました:\n{}'.format(traceback.format_exc()))
