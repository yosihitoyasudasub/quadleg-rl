# -*- coding: utf-8 -*-
"""
QuadLegLinkage.py — Fusion 360 アドイン
四脚ロボット脚（股関節2サーボ＋四節リンク）の骨格リンク・サーボ本体・回転ジョイントを生成する。
デザイン作業スペースの「ソリッド」タブ →「作成」パネルに「Quad Leg Linkage」ボタンを追加し、
押すたびにダイアログが開く（Web アプリと同じ項目を数値入力）。前回の入力値は自動で記憶される。

インストール:
  1. フォルダ  QuadLegLinkage/  を作り、この QuadLegLinkage.py と QuadLegLinkage.manifest を入れる
     （Windows: %APPDATA%\\Autodesk\\Autodesk Fusion 360\\API\\AddIns\\QuadLegLinkage\\ に置くと一覧に出る）
  2. Fusion 360 → ユーティリティ → スクリプトとアドイン →「アドイン」タブ →（一覧になければ「＋」でフォルダを指定）
     → 選択して「実行」。「起動時に実行」にチェックすると常駐する
  3. ソリッドタブ → 作成 → Quad Leg Linkage → ダイアログで値を調整 →「生成」

スクリプトとして実行しても動く（その場合はボタンを押して閉じるまで常駐し、停止で消える）。

座標系: Web アプリと同じ。股関節 O が原点、+X 前方、+Y 上、リンクは XY 平面上、Z 方向に板厚。
Fusion API の内部単位は cm / rad（ダイアログの値も内部では cm / rad で返る）。

サーボ: DYNAMIXEL XM540-W270 を既定値とする。
  本体 33.5(W) × 58.5(H) × 44(D) mm、165 g、ストール 10.6 N·m @12 V（e-Manual より）
  ホーン HN13-N101: Ø26、8-M2.5 P.C.D 22、中央ボス Ø10、中心固定 M3（ROBOTIS 図面 2019/03/18 より）
"""
import math
import os
import json
import traceback

# ---------------- 既定値（mm / deg） ----------------
DEFAULTS = dict(
    L1=100.0, L2=100.0, Lc=30.0, Lr=100.0, Le=30.0,
    bx=0.0, by=0.0, delta=0.0, cfg=1, t1=-110.0, t2=130.0,
    sA1=90.0, sA2=90.0,          # サーボ本体の向き（ホーン軸まわり、長手方向が伸びる向き）
    link_w=14.0, link_t=4.0, gap=1.0, pin_d=3.0,
)

# サーボプリセット: (W 幅, H 長さ, D 奥行(ケースのみ), off ホーン軸〜本体端, ホーン外径, PCD, 穴数, 穴径, 中心穴径, ホーン突出)
SERVOS = [
    # XM540: ROBOTIS 図面 XM540/XH540/XD540 (HINGE) 2019/03/18 より
    #   本体 33.5×58.5×44、ホーン軸は本体上端から 13.75、ホーン HN13-N101 Ø26、8-M2.5 P.C.D 22、
    #   中央ボス Ø10（リンク側は Ø10.2 逃がし）、中心固定 WB M3x08、ケース固定 4-M2.5 (27×52)、ホーン突出 2.6
    ('XM540-W270 (DYNAMIXEL)', 33.5, 58.5, 44.0, 13.75, 26.0, 22.0, 8, 2.7, 10.2, 2.6),
    ('XL430-W250 (DYNAMIXEL)', 28.5, 46.5, 34.0, 19.0, 22.0, 16.0, 8, 2.1, 6.0, 3.0),   # 未検証（図面要確認）
    # X330: ROBOTIS 図面 X330 2020/05/28 より。本体 20×34×23、ホーン突出 3（全長 26）、ホーン軸は上端から 9.5、
    #   ホーン Ø16、4-Ø1.6 P.C.D 12（M2 タッピング → リンク側 Ø2.2）、ケース固定穴 16×30
    ('XL330-M288 (DYNAMIXEL)', 20.0, 34.0, 23.0, 9.5, 16.0, 12.0, 4, 2.2, 4.0, 3.0),
    # XL-320: ROBOTIS 図面 2019/05/23 より。本体 24×36×24、ホーン突出 3（全長 27）、ホーン軸は上端から 9、ホーン Ø17.8、
    #   4-Ø4 リベット穴 P.C.D 12（＋4-Ø1.6 タッピング穴 P.C.D 12、45° 位相）。ここではリベット穴 4 個を採用
    ('XL-320 (DYNAMIXEL)',     24.0, 36.0, 24.0, 9.0, 17.8, 12.0, 4, 4.1, 6.0, 3.0),
    ('汎用 RC サーボ (SG90 級)', 12.2, 22.8, 26.0, 6.0, 7.0, 0.0, 0, 0.0, 3.0, 3.0),
    ('カスタム',               33.5, 58.5, 44.0, 13.75, 26.0, 22.0, 8, 2.7, 10.2, 2.6),
]
SERVO_KEYS = ('sW', 'sH', 'sD', 'sOff', 'hornD', 'pcd', 'nHole', 'holeD', 'centerD', 'hornT')

CMD_ID = 'QuadLegLinkageCmd'
PANEL_ID = 'SolidCreatePanel'          # ソリッドタブ → 作成パネル
LAST_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'qll_last.json')
_handlers = []
_app = None
_ui = None


def load_last():
    try:
        with open(LAST_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return {}


def save_last(p):
    try:
        with open(LAST_FILE, 'w', encoding='utf-8') as f:
            json.dump(p, f, ensure_ascii=False, indent=1)
    except Exception:
        pass


# ============================ 運動学 ============================
def fk(p):
    """Web アプリと同一の順運動学。戻り値は mm 単位の点辞書。"""
    a1, a2 = math.radians(p['t1']), math.radians(p['t2'])
    O = (0.0, 0.0)
    B = (p['bx'], p['by'])
    K = (p['L1'] * math.cos(a1), p['L1'] * math.sin(a1))
    C = (B[0] + p['Lc'] * math.cos(a2), B[1] + p['Lc'] * math.sin(a2))
    dx, dy = C[0] - K[0], C[1] - K[1]
    d = math.hypot(dx, dy)
    Le, Lr = p['Le'], p['Lr']
    if d < 1e-9 or d > Le + Lr or d < abs(Le - Lr):
        raise ValueError('リンクが閉じません (C-K 距離 %.1f mm, 許容 %.1f〜%.1f)' % (d, abs(Le - Lr), Le + Lr))
    a = (Le * Le - Lr * Lr + d * d) / (2 * d)
    h = math.sqrt(max(0.0, Le * Le - a * a))
    px, py = K[0] + a * dx / d, K[1] + a * dy / d
    D = (px - p['cfg'] * h * dy / d, py + p['cfg'] * h * dx / d)
    ux, uy = -(D[0] - K[0]) / Le, -(D[1] - K[1]) / Le
    dl = math.radians(p['delta'])
    rx, ry = ux * math.cos(dl) - uy * math.sin(dl), ux * math.sin(dl) + uy * math.cos(dl)
    F = (K[0] + p['L2'] * rx, K[1] + p['L2'] * ry)
    return dict(O=O, B=B, K=K, C=C, D=D, F=F)


def servo_rect(c, ang, W, H, off):
    """ホーン軸 c、長手向き ang のサーボ本体フットプリント 4 隅 (mm)。"""
    a = math.radians(ang)
    ca, sa = math.cos(a), math.sin(a)
    w, top, bot = W / 2.0, off, -(H - off)
    return [(c[0] + y * ca - x * sa, c[1] + y * sa + x * ca)
            for (x, y) in ((-w, top), (w, top), (w, bot), (-w, bot))]


# ============================ エントリ ============================
def run(context):
    global _app, _ui
    import adsk.core
    _app = adsk.core.Application.get()
    _ui = _app.userInterface
    try:
        _cleanup_ui()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, 'Quad Leg Linkage', '四脚ロボット脚リンク機構を生成\n股関節2サーボ＋四節リンクの骨格・サーボ・ジョイントを作成します')
        on_created = CreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        # リボン: デザイン作業スペース → ソリッドタブ → 作成パネル にボタン追加
        panel = _find_panel()
        if panel:
            ctrl = panel.controls.addCommand(cmd_def)
            ctrl.isPromoted = True          # パネル上に常時表示
            ctrl.isPromotedByDefault = True
        # 起動時（アドインの自動実行）でなければ、すぐにダイアログも開く
        if not (context and context.get('IsApplicationStartup')):
            cmd_def.execute()
        import adsk
        adsk.autoTerminate(False)   # スクリプト実行時もボタンを残す
    except Exception:
        if _ui:
            _ui.messageBox('起動失敗:\n' + traceback.format_exc())


def stop(context):
    try:
        _cleanup_ui()
    except Exception:
        pass


def _find_panel():
    ws = _ui.workspaces.itemById('FusionSolidEnvironment')
    if ws:
        for tab_id in ('SolidTab', 'ToolsTab'):
            tab = ws.toolbarTabs.itemById(tab_id)
            if tab:
                panel = tab.toolbarPanels.itemById(PANEL_ID)
                if panel:
                    return panel
    return _ui.allToolbarPanels.itemById(PANEL_ID)


def _cleanup_ui():
    panel = _find_panel()
    if panel:
        ctrl = panel.controls.itemById(CMD_ID)
        if ctrl:
            ctrl.deleteMe()
    cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
    if cmd_def:
        cmd_def.deleteMe()


# ============================ ダイアログ ============================
def _mm(v):
    import adsk.core
    return adsk.core.ValueInput.createByReal(v * 0.1)


def _deg(v):
    import adsk.core
    return adsk.core.ValueInput.createByReal(math.radians(v))


def build_inputs(inputs, d):
    """d: DEFAULTS に前回値 (qll_last.json) を重ねた辞書。"""
    import adsk.core
    g = inputs.addGroupCommandInput('gPos', '動力の位置').children
    g.addValueInput('bx', 'サーボ② X 位置 Bx', 'mm', _mm(d['bx']))
    g.addValueInput('by', 'サーボ② Y 位置 By', 'mm', _mm(d['by']))
    g.addValueInput('sA1', 'サーボ① 本体向き', 'deg', _deg(d['sA1']))
    g.addValueInput('sA2', 'サーボ② 本体向き', 'deg', _deg(d['sA2']))

    g = inputs.addGroupCommandInput('gLink', 'リンク長さ').children
    g.addValueInput('L1', '大腿 L1', 'mm', _mm(d['L1']))
    g.addValueInput('L2', '下腿 L2', 'mm', _mm(d['L2']))
    g.addValueInput('Lc', 'クランク Lc', 'mm', _mm(d['Lc']))
    g.addValueInput('Lr', '連結ロッド Lr', 'mm', _mm(d['Lr']))
    g.addValueInput('Le', '下腿レバー Le', 'mm', _mm(d['Le']))
    g.addValueInput('delta', '下腿の曲げ角 δ', 'deg', _deg(d['delta']))
    dd = g.addDropDownCommandInput('cfg', 'リンク組立の向き', adsk.core.DropDownStyles.TextListDropDownStyle)
    dd.listItems.add('A', d['cfg'] == 1)
    dd.listItems.add('B（反転）', d['cfg'] == -1)

    g = inputs.addGroupCommandInput('gPose', '姿勢（サーボ角）').children
    g.addValueInput('t1', 'サーボ① 大腿角 θ1', 'deg', _deg(d['t1']))
    g.addValueInput('t2', 'サーボ② クランク角 θ2', 'deg', _deg(d['t2']))

    g = inputs.addGroupCommandInput('gServo', 'サーボ').children
    dd = g.addDropDownCommandInput('servo', '機種', adsk.core.DropDownStyles.TextListDropDownStyle)
    idx = int(d.get('servoIdx', 0))
    if not 0 <= idx < len(SERVOS):
        idx = 0
    for i, s in enumerate(SERVOS):
        dd.listItems.add(s[0], i == idx)
    sv = dict(zip(SERVO_KEYS, SERVOS[idx][1:]))
    sv.update({k: d[k] for k in SERVO_KEYS if k in d})   # 前回値があれば優先
    g.addValueInput('sW', '本体幅 W（ホーン面）', 'mm', _mm(sv['sW']))
    g.addValueInput('sH', '本体長さ H', 'mm', _mm(sv['sH']))
    g.addValueInput('sD', '本体奥行 D（ケースのみ）', 'mm', _mm(sv['sD']))
    g.addValueInput('sOff', 'ホーン軸〜本体端', 'mm', _mm(sv['sOff']))
    g.addValueInput('hornD', 'ホーン外径', 'mm', _mm(sv['hornD']))
    g.addValueInput('hornT', 'ホーン突出量', 'mm', _mm(sv['hornT']))
    g.addValueInput('pcd', 'ホーン穴 PCD', 'mm', _mm(sv['pcd']))
    g.addIntegerSpinnerCommandInput('nHole', 'ホーン穴数', 0, 16, 1, int(sv['nHole']))
    g.addValueInput('holeD', 'ホーン穴径', 'mm', _mm(sv['holeD']))
    g.addValueInput('centerD', '中心穴径（リンク側）', 'mm', _mm(sv['centerD']))
    g.addBoolValueInput('newDoc', '新規デザインに生成', True, '', bool(d.get('newDoc', True)))

    g = inputs.addGroupCommandInput('gShape', 'リンク形状').children
    g.addValueInput('link_w', 'リンク幅', 'mm', _mm(d['link_w']))
    g.addValueInput('link_t', '板厚', 'mm', _mm(d['link_t']))
    g.addValueInput('gap', '層間クリアランス', 'mm', _mm(d['gap']))
    g.addValueInput('pin_d', 'ピン穴径', 'mm', _mm(d['pin_d']))

    inputs.addTextBoxCommandInput('status', '状態', '', 3, True)


def read_inputs(inputs):
    p = {}
    for k in ('bx', 'by', 'L1', 'L2', 'Lc', 'Lr', 'Le', 'sW', 'sH', 'sD', 'sOff',
              'hornD', 'hornT', 'pcd', 'holeD', 'centerD', 'link_w', 'link_t', 'gap', 'pin_d'):
        p[k] = inputs.itemById(k).value * 10.0          # cm → mm
    for k in ('sA1', 'sA2', 'delta', 't1', 't2'):
        p[k] = math.degrees(inputs.itemById(k).value)   # rad → deg
    p['cfg'] = 1 if inputs.itemById('cfg').selectedItem.index == 0 else -1
    p['nHole'] = inputs.itemById('nHole').value
    p['newDoc'] = inputs.itemById('newDoc').value
    p['servoIdx'] = inputs.itemById('servo').selectedItem.index
    p['servoName'] = inputs.itemById('servo').selectedItem.name
    return p


def update_status(inputs):
    box = inputs.itemById('status')
    try:
        p = read_inputs(inputs)
        pts = fk(p)
        clash = ''
        if math.hypot(p['bx'], p['by']) > 1e-6:
            A = servo_rect(pts['O'], p['sA1'], p['sW'], p['sH'], p['sOff'])
            B = servo_rect(pts['B'], p['sA2'], p['sW'], p['sH'], p['sOff'])
            if polys_overlap(A, B):
                clash = '\n注意: サーボ本体が同一面で干渉します'
        box.text = 'OK  足先 = (%.1f, %.1f) mm, 膝 K = (%.1f, %.1f)%s' % (
            pts['F'][0], pts['F'][1], pts['K'][0], pts['K'][1], clash)
        return True
    except Exception as e:
        box.text = 'NG  ' + str(e)
        return False


def polys_overlap(A, B):
    for poly in (A, B):
        n = len(poly)
        for i in range(n):
            p, q = poly[i], poly[(i + 1) % n]
            nx, ny = -(q[1] - p[1]), q[0] - p[0]
            a = [v[0] * nx + v[1] * ny for v in A]
            b = [v[0] * nx + v[1] * ny for v in B]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def _make_handlers():
    import adsk.core

    class Created(adsk.core.CommandCreatedEventHandler):
        def notify(self, args):
            try:
                cmd = args.command
                cmd.isRepeatable = False
                cmd.okButtonText = '生成'
                cmd.setDialogInitialSize(380, 760)
                d = dict(DEFAULTS)
                d.update(load_last())
                build_inputs(cmd.commandInputs, d)
                update_status(cmd.commandInputs)
                for ev, H in ((cmd.execute, Execute), (cmd.inputChanged, Changed),
                              (cmd.validateInputs, Validate), (cmd.destroy, Destroy)):
                    h = H()
                    ev.add(h)
                    _handlers.append(h)
            except Exception:
                _ui.messageBox('ダイアログ作成失敗:\n' + traceback.format_exc())

    class Changed(adsk.core.InputChangedEventHandler):
        def notify(self, args):
            try:
                inputs = args.inputs
                if args.input.id == 'servo':
                    s = SERVOS[args.input.selectedItem.index]
                    for key, v in zip(SERVO_KEYS, s[1:]):
                        item = inputs.itemById(key)
                        if key == 'nHole':
                            item.value = int(v)
                        else:
                            item.value = v * 0.1
                if args.input.id != 'status':
                    update_status(inputs)
            except Exception:
                _ui.messageBox('入力更新失敗:\n' + traceback.format_exc())

    class Validate(adsk.core.ValidateInputsEventHandler):
        def notify(self, args):
            try:
                read_inputs(args.inputs)
                fk(read_inputs(args.inputs))
                args.areInputsValid = True
            except Exception:
                args.areInputsValid = False

    class Execute(adsk.core.CommandEventHandler):
        def notify(self, args):
            try:
                p = read_inputs(args.command.commandInputs)
                save_last(p)
                build_model(p)
            except Exception:
                _ui.messageBox('生成失敗:\n' + traceback.format_exc())

    class Destroy(adsk.core.CommandEventHandler):
        def notify(self, args):
            pass   # ダイアログを閉じてもアドイン（リボンのボタン）は残す

    return Created, Changed, Validate, Execute, Destroy


class CreatedHandler(object):
    """run() から参照するための薄いラッパ（adsk import 後にクラス生成）。"""
    def __new__(cls):
        Created, _, _, _, _ = _make_handlers()
        return Created()


# ============================ モデル生成 ============================
def build_model(p):
    import adsk.core, adsk.fusion
    pts = fk(p)
    design = adsk.fusion.Design.cast(_app.activeProduct)
    if p.get('newDoc', True) or design is None:
        _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(_app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    mm = 0.1
    T, s = p['link_t'] * mm, (p['link_t'] + p['gap']) * mm
    hT, D = p['hornT'] * mm, p['sD'] * mm
    coax = math.hypot(p['bx'], p['by']) < 1e-6
    horn = dict(hornD=p['hornD'], pcd=p['pcd'], n=p['nHole'], holeD=p['holeD'], centerD=p['centerD'])
    ctx = dict(p=p, mm=mm, T=T, horn=horn)

    # 層配置（Z、サーボ①のケース前面を 0 とする。ホーンは実突出量 hT だけ出て、その面にリンクを直接固定）
    #   サーボ① 本体 -D〜0 / ホーン① 0〜hT / 大腿 lay(0) / 下腿 lay(1) / ロッド lay(2)
    #   同軸  : クランク lay(3)、その上にホーン②、サーボ② 本体（背中合わせで脚を挟む）
    #   非同軸: クランク lay(0)（サーボ②も -D〜0 側、ホーン② 0〜hT）
    #   胴体  : サーボのケースを保持するブラケット（両サーボの上端をまたぐ橋）。ホーン面には何も挟まない
    def lay(i):
        return hT + i * s

    z_crank = lay(3) if coax else lay(0)
    thigh_occ = make_link(root, ctx, 'Thigh L1', [pts['O'], pts['K']], z=lay(0), horn_at=[pts['O']])
    shank_occ = make_link(root, ctx, 'Shank L2+Le', [pts['D'], pts['K'], pts['F']], z=lay(1))
    rod_occ = make_link(root, ctx, 'Rod Lr', [pts['C'], pts['D']], z=lay(2))
    crank_occ = make_link(root, ctx, 'Crank Lc', [pts['B'], pts['C']], z=z_crank, horn_at=[pts['B']])

    sv1 = make_servo(root, ctx, 'Servo 1 ' + p['servoName'], pts['O'], p['sA1'],
                     z0=-D, horn_z0=0.0, horn_z1=hT)
    if coax:
        z_h2 = lay(3) + T                       # クランク上面
        sv2 = make_servo(root, ctx, 'Servo 2 ' + p['servoName'], pts['B'], p['sA2'],
                         z0=z_h2 + hT, horn_z0=z_h2, horn_z1=z_h2 + hT)
        z_body = (-D, z_h2 + hT + D)
    else:
        sv2 = make_servo(root, ctx, 'Servo 2 ' + p['servoName'], pts['B'], p['sA2'],
                         z0=-D, horn_z0=0.0, horn_z1=hT)
        z_body = (-D, 0.0)
    body_occ = make_body(root, ctx, pts, z_body)
    for occ in (body_occ, sv1, sv2):
        occ.isGrounded = True

    revolute(root, sv1, thigh_occ, pts['O'], mm, 'J_O hip servo 1')
    revolute(root, sv2, crank_occ, pts['B'], mm, 'J_B hip servo 2')
    revolute(root, thigh_occ, shank_occ, pts['K'], mm, 'J_K knee')
    revolute(root, crank_occ, rod_occ, pts['C'], mm, 'J_C')
    revolute(root, rod_occ, shank_occ, pts['D'], mm, 'J_D')

    up = design.userParameters
    for k, unit in (('L1', 'mm'), ('L2', 'mm'), ('Lc', 'mm'), ('Lr', 'mm'), ('Le', 'mm'), ('bx', 'mm'), ('by', 'mm'),
                    ('t1', 'deg'), ('t2', 'deg'), ('delta', 'deg')):
        expr = '%g %s' % (p[k], unit)
        prm = up.itemByName('qll_' + k)
        if prm:
            prm.expression = expr
        else:
            up.add('qll_' + k, adsk.core.ValueInput.createByString(expr), unit, 'Quad Leg Linkage Lab')

    _app.activeViewport.fit()
    _ui.messageBox('生成完了\n足先 = (%.1f, %.1f) mm\nホーン突出 %.1f mm、リンク層ピッチ %.1f mm\n'
                   'J_O / J_B をドラッグして動作確認できます。' % (pts['F'][0], pts['F'][1], p['hornT'], p['link_t'] + p['gap']))


# ---------- スケッチ部品 ----------
def _P(mm, x, y):
    import adsk.core
    return adsk.core.Point3D.create(x * mm, y * mm, 0)


def _slot(sk, A, B, w, mm):
    """点 A→B を結ぶ幅 w の長穴形状（外形）。"""
    dx, dy = B[0] - A[0], B[1] - A[1]
    L = math.hypot(dx, dy)
    ux, uy = dx / L, dy / L
    nx, ny = -uy, ux
    r = w / 2.0
    a1 = (A[0] + nx * r, A[1] + ny * r); b1 = (B[0] + nx * r, B[1] + ny * r)
    a2 = (A[0] - nx * r, A[1] - ny * r); b2 = (B[0] - nx * r, B[1] - ny * r)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    lines.addByTwoPoints(_P(mm, *a1), _P(mm, *b1))
    lines.addByTwoPoints(_P(mm, *a2), _P(mm, *b2))
    arcs.addByCenterStartSweep(_P(mm, *B), _P(mm, *b1), -math.pi)
    arcs.addByCenterStartSweep(_P(mm, *A), _P(mm, *a2), -math.pi)


def _circle(sk, c, d, mm):
    sk.sketchCurves.sketchCircles.addByCenterRadius(_P(mm, *c), d / 2.0 * mm)


def _rect(sk, corners, mm):
    lines = sk.sketchCurves.sketchLines
    n = len(corners)
    for i in range(n):
        lines.addByTwoPoints(_P(mm, *corners[i]), _P(mm, *corners[(i + 1) % n]))


def _extrude_all(comp, sk, z, thickness, op):
    import adsk.core, adsk.fusion
    coll = adsk.core.ObjectCollection.create()
    for prof in sk.profiles:
        coll.add(prof)
    feats = comp.features.extrudeFeatures
    inp = feats.createInput(coll, op)
    inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(thickness))
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(z))
    return feats.add(inp)


def _new_occ(root, name):
    import adsk.core
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def _horn_holes(sk, c, horn, mm):
    """ホーン固定穴パターン（中心穴＋PCD 上の n 穴）。"""
    _circle(sk, c, horn['centerD'], mm)
    if horn['n'] > 0 and horn['pcd'] > 0:
        r = horn['pcd'] / 2.0
        for i in range(horn['n']):
            a = 2 * math.pi * i / horn['n']
            _circle(sk, (c[0] + r * math.cos(a), c[1] + r * math.sin(a)), horn['holeD'], mm)


def make_link(root, ctx, name, chain, z, horn_at=()):
    """点列 chain を長穴で結んだ板リンク。各点にピン穴、horn_at の点にはホーン穴パターン＋円形ボス。"""
    import adsk.fusion
    p, mm, T, horn = ctx['p'], ctx['mm'], ctx['T'], ctx['horn']
    occ = _new_occ(root, name)
    comp = occ.component
    # 外形
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = name + ' outline'
    for a, b in zip(chain[:-1], chain[1:]):
        _slot(sk, a, b, p['link_w'], mm)
    for c in horn_at:
        _circle(sk, c, max(horn['hornD'], horn['pcd'] + horn['holeD'] + 4.0), mm)
    _extrude_all(comp, sk, z, T, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    # 穴
    sk2 = comp.sketches.add(comp.xYConstructionPlane); sk2.name = name + ' holes'
    for pt in chain:
        if not any(math.hypot(pt[0] - h[0], pt[1] - h[1]) < 1e-6 for h in horn_at):
            _circle(sk2, pt, p['pin_d'], mm)
    for c in horn_at:
        _horn_holes(sk2, c, horn, mm)
    _extrude_all(comp, sk2, z, T, adsk.fusion.FeatureOperations.CutFeatureOperation)
    return occ


def make_body(root, ctx, pts, z_range):
    """胴体ブラケット: 2 台のサーボ本体の上端をまたぐ橋（XY で帯、Z は両サーボの背面間）。
    ホーン面とリンクの間には何も挟まない。実機ではサーボのケース穴で保持する部分の置き換え。"""
    import adsk.fusion
    p, mm, T = ctx['p'], ctx['mm'], ctx['T']
    occ = _new_occ(root, 'Body (hip bracket)')
    comp = occ.component
    rects = servo_rect(pts['O'], p['sA1'], p['sW'], p['sH'], p['sOff']) + \
        servo_rect(pts['B'], p['sA2'], p['sW'], p['sH'], p['sOff'])
    xs, ys = [q[0] for q in rects], [q[1] for q in rects]
    x0, x1 = min(xs) - 2.0, max(xs) + 2.0
    y0 = max(ys) + 0.5                       # サーボ上端の少し上
    y1 = y0 + p['link_t']
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Body bracket'
    _rect(sk, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], mm)
    z_lo, z_hi = min(z_range), max(z_range)
    _extrude_all(comp, sk, z_lo, z_hi - z_lo, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    return occ


def make_servo(root, ctx, name, c, ang, z0, horn_z0, horn_z1):
    """サーボ本体ボックス（z0 から奥行 D）とホーン円柱（horn_z0→horn_z1）。"""
    import adsk.fusion
    p, mm, horn = ctx['p'], ctx['mm'], ctx['horn']
    occ = _new_occ(root, name)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Servo body'
    _rect(sk, servo_rect(c, ang, p['sW'], p['sH'], p['sOff']), mm)
    _extrude_all(comp, sk, z0, p['sD'] * mm, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sk2 = comp.sketches.add(comp.xYConstructionPlane); sk2.name = 'Horn'
    _circle(sk2, c, horn['hornD'], mm)
    zlo, zhi = min(horn_z0, horn_z1), max(horn_z0, horn_z1)
    _extrude_all(comp, sk2, zlo, zhi - zlo, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    return occ


# ---------- ジョイント ----------
def _circular_edge_at(occ, Pt, mm):
    import adsk.core
    best, bestd = None, 1e9
    for body in occ.bRepBodies:
        for e in body.edges:
            g = e.geometry
            if isinstance(g, adsk.core.Circle3D):
                d = math.hypot(g.center.x - Pt[0] * mm, g.center.y - Pt[1] * mm)
                if d < bestd:
                    best, bestd = e, d
    if best is None or bestd > 0.01:
        raise RuntimeError('ジョイント用の円エッジが見つかりません: %s' % (Pt,))
    return best


def revolute(root, occ1, occ2, Pt, mm, name):
    import adsk.fusion
    edge = _circular_edge_at(occ1, Pt, mm)
    geo = adsk.fusion.JointGeometry.createByCurve(edge, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
    joints = root.asBuiltJoints
    inp = joints.createInput(occ1, occ2, geo)
    inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    j = joints.add(inp)
    j.name = name
    return j


# Fusion 外で実行した場合は座標だけ表示（動作確認用）
if __name__ == '__main__':
    for k, v in fk(DEFAULTS).items():
        print('%s = (%.2f, %.2f) mm' % (k, v[0], v[1]))
