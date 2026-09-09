# -*- coding: utf-8 -*-
"""
HopperLeg.py — Fusion 360 アドイン
一本脚ホッパーの脚（5 節リンク ＋ 直列バネ ＋ ロール軸）の骨格・サーボ本体・ジョイントを生成する。
デザイン作業スペースの「ソリッド」タブ →「作成」パネルに「Hopper Leg」ボタンを追加する。

要件は docs/hopper-mechanical-spec.md を正とする（寸法・荷重・可動域・必須制約）。
既定値は MuJoCo で成立を確認した構成（hopper/model.py の HopperParams）と同じ:
  d0 40 / l1 90 / l2 130 mm、L0 240、引込み 15、バネ 80 mm、XM540-W270 ×2 ＋ XH540-W150 ×1

インストール:
  1. フォルダ HopperLeg/ を作り、この HopperLeg.py と HopperLeg.manifest を入れる
     （Windows: %APPDATA%\\Autodesk\\Autodesk Fusion 360\\API\\AddIns\\HopperLeg\\ ）
  2. Fusion 360 → ユーティリティ → スクリプトとアドイン →「アドイン」タブ → 選択して「実行」
  3. ソリッドタブ → 作成 → Hopper Leg → ダイアログで値を調整 →「生成」

座標系（四脚版と同じ CAD 規約）:
  股軸の中点 O が原点。**+X 前方、+Y 上、脚面は XY 平面、+Z は板厚（左右）方向。**
  MuJoCo 側は Z 上なので、CAD の +Y が MuJoCo の +Z に対応する。
  脚長サーボ ①② の出力軸は Z 方向、ロールサーボ ③ の出力軸は X 方向（脚面を貫通する）。

**このアドインが作らないもの**: サーボ同士をつなぐブラケット類。
ロールサーボの軸を脚面に通しつつ本体を前後へ逃がす設計（要件 §4-2）は CAD の主要な検討点そのものなので、
アドインは「動かせない要素の正しい位置と軸」だけを置き、ブラケットは利用者が設計する。
"""
import math
import os
import json
import traceback

# ---------------- 既定値（mm / deg。MuJoCo の HopperParams と同じ） ----------------
DEFAULTS = dict(
    # 脚（5 節リンク）
    d0=40.0, l1=90.0, l2=130.0,
    # 姿勢: リンク先端長 Ln（股→足先ピン）と足先の前後オフセット xf
    #   飛行・引込み = 145、蹴り出し最大 = 195（L0 240 − 引込み 15 − バネ 80 ＋ 蹴り出し 0〜50）
    Ln=145.0, xf=0.0,
    # 直列バネ・足
    ls0=80.0, comp=0.0, springD=16.0, padR=25.0, padT=6.0,
    # ロール軸（③）
    rollOff=60.0, rollAng=180.0,
    # 胴体エンベロープ（慣性の箱。重心＝股軸高さの確認用）
    bl=300.0, bh=180.0, bw=160.0, showBody=True,
    # 形状
    link_w=18.0, link_t=4.0, gap=1.0, pin_d=4.0,
    newDoc=True, servoIdx=0, servoIdx3=1,
)

# サーボ: (名前, W 幅, H 長さ, D 奥行, off ホーン軸〜本体端, ホーン外径, PCD, 穴数, 穴径, 中心穴径, ホーン突出)
# XM540 / XH540 は外形共通（ROBOTIS 図面 XM540/XH540/XD540 (HINGE) 2019/03/18）。
# ホーン HN13-N101: Ø26、8-M2.5 P.C.D 22、中央ボス Ø10（リンク側 Ø10.2 逃がし）、ホーン突出 2.6
SERVOS = [
    ('XM540-W270 (脚長 ①②)', 33.5, 58.5, 44.0, 13.75, 26.0, 22.0, 8, 2.7, 10.2, 2.6),
    ('XH540-W150 (ロール ③)', 33.5, 58.5, 44.0, 13.75, 26.0, 22.0, 8, 2.7, 10.2, 2.6),
    ('XL430-W250',            28.5, 46.5, 34.0, 19.00, 22.0, 16.0, 8, 2.1,  6.0, 3.0),
    ('カスタム',              33.5, 58.5, 44.0, 13.75, 26.0, 22.0, 8, 2.7, 10.2, 2.6),
]
SERVO_KEYS = ('sW', 'sH', 'sD', 'sOff', 'hornD', 'pcd', 'nHole', 'holeD', 'centerD', 'hornT')

CMD_ID = 'HopperLegCmd'
PANEL_ID = 'SolidCreatePanel'
LAST_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hopper_last.json')
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


# ============================ 運動学（hopper/kinematics.py と同一） ============================
def ik(p, fx, fy):
    """足先ピン (fx, fy) → クランク角 (t1, t2) [rad]。角度は +X から +Y へ反時計回り。
    到達不能なら ValueError。膝は外側（後側は後ろ、前側は前）に張り出す分岐を選ぶ。"""
    out = []
    for px, sgn in ((-p['d0'] / 2.0, -1.0), (p['d0'] / 2.0, 1.0)):
        dx, dy = fx - px, fy
        d = math.hypot(dx, dy)
        if d > p['l1'] + p['l2'] or d < abs(p['l1'] - p['l2']) or d < 1e-9:
            raise ValueError('足先に届きません（軸からの距離 %.1f mm、許容 %.1f〜%.1f）'
                             % (d, abs(p['l1'] - p['l2']), p['l1'] + p['l2']))
        a = (p['l1'] ** 2 - p['l2'] ** 2 + d * d) / (2 * d)
        h2 = p['l1'] ** 2 - a * a
        if h2 < 0:
            raise ValueError('リンクが閉じません')
        h = math.sqrt(h2)
        mx, my = px + a * dx / d, a * dy / d
        out.append(math.atan2(my + sgn * h * dx / d, mx - sgn * h * dy / d - px))
    return out[0], out[1]


def geometry(p):
    """姿勢から各点（mm）を返す。O 股軸中点、P1/P2 サーボ軸、K1/K2 膝、F 足先ピン、S 足接地点。"""
    fy = -math.sqrt(max(0.0, p['Ln'] ** 2 - p['xf'] ** 2))
    t1, t2 = ik(p, p['xf'], fy)
    P1 = (-p['d0'] / 2.0, 0.0)
    P2 = (p['d0'] / 2.0, 0.0)
    K1 = (P1[0] + p['l1'] * math.cos(t1), P1[1] + p['l1'] * math.sin(t1))
    K2 = (P2[0] + p['l1'] * math.cos(t2), P2[1] + p['l1'] * math.sin(t2))
    F = (p['xf'], fy)
    # 直列バネは股 O → 足先 F の延長線上（MuJoCo の「脚バー」と同じ。要件 §4-5）
    n = math.hypot(F[0], F[1]) or 1.0
    u = (F[0] / n, F[1] / n)
    span = max(0.0, p['ls0'] - p['comp'])
    S = (F[0] + u[0] * span, F[1] + u[1] * span)
    # 内角（下腿どうしが足先でなす角）と伝達比 r = dL/dα
    v1 = (F[0] - K1[0], F[1] - K1[1])
    v2 = (F[0] - K2[0], F[1] - K2[1])
    inner = math.degrees(math.acos(max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (p['l2'] ** 2)))))
    return dict(O=(0.0, 0.0), P1=P1, P2=P2, K1=K1, K2=K2, F=F, S=S, u=u,
                t1=math.degrees(t1), t2=math.degrees(t2), inner=inner, r=ratio(p, t1, t2))


def ratio(p, t1, t2, e=1e-5):
    """脚長方向の伝達比 r = dL/dα [mm/rad]（対称モード）。0 に近いと特異点。"""
    def L(a):
        q = p['d0'] / 2.0 - p['l1'] * math.cos(a)
        if abs(q) > p['l2']:
            return None
        return math.sqrt(p['l2'] ** 2 - q * q) - p['l1'] * math.sin(a)
    am = (t1 + (-math.pi - t2)) / 2.0
    lo, hi = L(am - e), L(am + e)
    return abs((hi - lo) / (2 * e)) if (lo is not None and hi is not None) else float('nan')


def servo_rect(c, ang, W, H, off):
    """ホーン軸 c、長手向き ang のサーボ本体フットプリント 4 隅。
    ローカル y が本体の長手方向（ホーン側 +off、反対側 −(H−off)）。"""
    a = math.radians(ang)
    ca, sa = math.cos(a), math.sin(a)
    w, top, bot = W / 2.0, off, -(H - off)
    return [(c[0] + y * ca - x * sa, c[1] + y * sa + x * ca)
            for (x, y) in ((-w, top), (w, top), (w, bot), (-w, bot))]


def polys_overlap(A, B):
    for poly in (A, B):
        n = len(poly)
        for i in range(n):
            q, r = poly[i], poly[(i + 1) % n]
            nx, ny = -(r[1] - q[1]), r[0] - q[0]
            a = [v[0] * nx + v[1] * ny for v in A]
            b = [v[0] * nx + v[1] * ny for v in B]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


# ============================ エントリ ============================
def run(context):
    global _app, _ui
    import adsk.core
    _app = adsk.core.Application.get()
    _ui = _app.userInterface
    try:
        _cleanup_ui()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, 'Hopper Leg',
            '一本脚ホッパーの脚を生成\n5 節リンク＋直列バネ＋ロール軸の骨格・サーボ・ジョイントを作成します')
        on_created = CreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)
        panel = _find_panel()
        if panel:
            ctrl = panel.controls.addCommand(cmd_def)
            ctrl.isPromoted = True
            ctrl.isPromotedByDefault = True
        if not (context and context.get('IsApplicationStartup')):
            cmd_def.execute()
        import adsk
        adsk.autoTerminate(False)
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
    import adsk.core
    g = inputs.addGroupCommandInput('gLink', '脚（5 節リンク）').children
    g.addValueInput('d0', 'サーボ軸間 d0', 'mm', _mm(d['d0']))
    g.addValueInput('l1', 'クランク l1', 'mm', _mm(d['l1']))
    g.addValueInput('l2', '下腿 l2', 'mm', _mm(d['l2']))

    g = inputs.addGroupCommandInput('gPose', '姿勢').children
    g.addValueInput('Ln', 'リンク先端長 Ln（引込 145 / 伸展 195）', 'mm', _mm(d['Ln']))
    g.addValueInput('xf', '足先の前後オフセット xf', 'mm', _mm(d['xf']))

    g = inputs.addGroupCommandInput('gSpring', '直列バネ・足').children
    g.addValueInput('ls0', 'バネ自然長＝ストローク', 'mm', _mm(d['ls0']))
    g.addValueInput('comp', 'バネ圧縮量（0〜自然長）', 'mm', _mm(d['comp']))
    g.addValueInput('springD', 'バネ機構の外形幅', 'mm', _mm(d['springD']))
    g.addValueInput('padR', '足パッド半径', 'mm', _mm(d['padR']))
    g.addValueInput('padT', '足パッド厚', 'mm', _mm(d['padT']))

    g = inputs.addGroupCommandInput('gRoll', 'ロール軸 ③').children
    g.addValueInput('rollOff', 'ロールサーボ ホーン面の後方逃がし', 'mm', _mm(d['rollOff']))
    g.addValueInput('rollAng', 'ロールサーボ 本体の向き', 'deg', _deg(d['rollAng']))
    dd = g.addDropDownCommandInput('servo3', '機種', adsk.core.DropDownStyles.TextListDropDownStyle)
    i3 = int(d.get('servoIdx3', 1))
    for i, s in enumerate(SERVOS):
        dd.listItems.add(s[0], i == i3)

    g = inputs.addGroupCommandInput('gServo', '脚長サーボ ①②').children
    dd = g.addDropDownCommandInput('servo', '機種', adsk.core.DropDownStyles.TextListDropDownStyle)
    idx = int(d.get('servoIdx', 0))
    if not 0 <= idx < len(SERVOS):
        idx = 0
    for i, s in enumerate(SERVOS):
        dd.listItems.add(s[0], i == idx)
    sv = dict(zip(SERVO_KEYS, SERVOS[idx][1:]))
    sv.update({k: d[k] for k in SERVO_KEYS if k in d})
    g.addValueInput('sW', '本体幅 W', 'mm', _mm(sv['sW']))
    g.addValueInput('sH', '本体長さ H', 'mm', _mm(sv['sH']))
    g.addValueInput('sD', '本体奥行 D', 'mm', _mm(sv['sD']))
    g.addValueInput('sOff', 'ホーン軸〜本体端', 'mm', _mm(sv['sOff']))
    g.addValueInput('hornD', 'ホーン外径', 'mm', _mm(sv['hornD']))
    g.addValueInput('hornT', 'ホーン突出量', 'mm', _mm(sv['hornT']))
    g.addValueInput('pcd', 'ホーン穴 PCD', 'mm', _mm(sv['pcd']))
    g.addIntegerSpinnerCommandInput('nHole', 'ホーン穴数', 0, 16, 1, int(sv['nHole']))
    g.addValueInput('holeD', 'ホーン穴径', 'mm', _mm(sv['holeD']))
    g.addValueInput('centerD', '中心穴径（リンク側）', 'mm', _mm(sv['centerD']))

    g = inputs.addGroupCommandInput('gBody', '胴体エンベロープ').children
    g.addBoolValueInput('showBody', '慣性の箱を生成する', True, '', bool(d.get('showBody', True)))
    g.addValueInput('bl', '前後 X', 'mm', _mm(d['bl']))
    g.addValueInput('bh', '上下 Y', 'mm', _mm(d['bh']))
    g.addValueInput('bw', '左右 Z', 'mm', _mm(d['bw']))

    g = inputs.addGroupCommandInput('gShape', 'リンク形状').children
    g.addValueInput('link_w', 'リンク幅', 'mm', _mm(d['link_w']))
    g.addValueInput('link_t', '板厚', 'mm', _mm(d['link_t']))
    g.addValueInput('gap', '層間クリアランス', 'mm', _mm(d['gap']))
    g.addValueInput('pin_d', 'ピン穴径', 'mm', _mm(d['pin_d']))
    g.addBoolValueInput('newDoc', '新規デザインに生成', True, '', bool(d.get('newDoc', True)))

    inputs.addTextBoxCommandInput('status', '状態', '', 5, True)


def read_inputs(inputs):
    p = {}
    for k in ('d0', 'l1', 'l2', 'Ln', 'xf', 'ls0', 'comp', 'springD', 'padR', 'padT', 'rollOff',
              'sW', 'sH', 'sD', 'sOff', 'hornD', 'hornT', 'pcd', 'holeD', 'centerD',
              'bl', 'bh', 'bw', 'link_w', 'link_t', 'gap', 'pin_d'):
        p[k] = inputs.itemById(k).value * 10.0          # cm → mm
    p['rollAng'] = math.degrees(inputs.itemById('rollAng').value)
    p['nHole'] = inputs.itemById('nHole').value
    p['newDoc'] = inputs.itemById('newDoc').value
    p['showBody'] = inputs.itemById('showBody').value
    p['servoIdx'] = inputs.itemById('servo').selectedItem.index
    p['servoName'] = inputs.itemById('servo').selectedItem.name
    p['servoIdx3'] = inputs.itemById('servo3').selectedItem.index
    p['servoName3'] = inputs.itemById('servo3').selectedItem.name
    return p


def update_status(inputs):
    box = inputs.itemById('status')
    try:
        p = read_inputs(inputs)
        g = geometry(p)
        warn = []
        if g['inner'] < 20.0:
            warn.append('内角 %.0f° は特異点に近い（20° 未満は使わない）' % g['inner'])
        A = servo_rect(g['P1'], 90.0, p['sW'], p['sH'], p['sOff'])
        B = servo_rect(g['P2'], 90.0, p['sW'], p['sH'], p['sOff'])
        if polys_overlap(A, B):
            warn.append('脚長サーボ ①② の本体が干渉（軸間 %.0f mm < 本体幅 %.1f mm）' % (p['d0'], p['sW']))
        if p['comp'] > p['ls0']:
            warn.append('バネ圧縮量が自然長を超えている（底付き）')
        box.text = ('OK  クランク① %.1f°  クランク② %.1f°\n内角 %.1f°  伝達比 %.0f mm/rad\n'
                    '足先ピン (%.1f, %.1f)  接地点 (%.1f, %.1f) mm%s'
                    % (g['t1'], g['t2'], g['inner'], g['r'], g['F'][0], g['F'][1], g['S'][0], g['S'][1],
                       ('\n注意: ' + ' / '.join(warn)) if warn else ''))
        return True
    except Exception as e:
        box.text = 'NG  ' + str(e)
        return False


def _make_handlers():
    import adsk.core

    class Created(adsk.core.CommandCreatedEventHandler):
        def notify(self, args):
            try:
                cmd = args.command
                cmd.isRepeatable = False
                cmd.okButtonText = '生成'
                cmd.setDialogInitialSize(400, 820)
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
                geometry(read_inputs(args.inputs))
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
            pass

    return Created, Changed, Validate, Execute, Destroy


class CreatedHandler(object):
    def __new__(cls):
        Created, _, _, _, _ = _make_handlers()
        return Created()


# ============================ モデル生成 ============================
def build_model(p):
    import adsk.core, adsk.fusion
    g = geometry(p)
    design = adsk.fusion.Design.cast(_app.activeProduct)
    if p.get('newDoc', True) or design is None:
        _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(_app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    mm = 0.1
    T, s = p['link_t'] * mm, (p['link_t'] + p['gap']) * mm
    hT, D = p['hornT'] * mm, p['sD'] * mm
    horn = dict(hornD=p['hornD'], pcd=p['pcd'], n=p['nHole'], holeD=p['holeD'], centerD=p['centerD'])
    ctx = dict(p=p, mm=mm, T=T, horn=horn)

    # Z 層（サーボ①② のケース前面を 0 とする）
    #   サーボ本体 -D〜0 / ホーン 0〜hT / クランク①② lay(0) / 下腿① lay(1) / 下腿② lay(2) / バネ・足 lay(3)
    def lay(i):
        return hT + i * s

    z_crank, z_sh1, z_sh2, z_spring = lay(0), lay(1), lay(2), lay(3)
    z_mid = (lay(0) + lay(2) + T) / 2.0          # 脚面の中心。ロール軸はここを通す（要件 §4-2）

    crank1 = make_link(root, ctx, 'Crank 1 (l1)', [g['P1'], g['K1']], z=z_crank, horn_at=[g['P1']])
    crank2 = make_link(root, ctx, 'Crank 2 (l1)', [g['P2'], g['K2']], z=z_crank, horn_at=[g['P2']])
    shank1 = make_link(root, ctx, 'Shank 1 (l2)', [g['K1'], g['F']], z=z_sh1)
    shank2 = make_link(root, ctx, 'Shank 2 (l2)', [g['K2'], g['F']], z=z_sh2)
    spring = make_spring(root, ctx, g, z=z_spring)

    sv1 = make_servo_xy(root, ctx, 'Servo 1 ' + p['servoName'], g['P1'], 90.0, z0=-D, horn_z=(0.0, hT))
    sv2 = make_servo_xy(root, ctx, 'Servo 2 ' + p['servoName'], g['P2'], 90.0, z0=-D, horn_z=(0.0, hT))
    sv3 = make_servo_yz(root, ctx, 'Servo 3 ' + p['servoName3'], z_mid)
    axis = make_roll_axis(root, ctx, z_mid)
    body = make_body_envelope(root, ctx, z_mid) if p.get('showBody', True) else None

    for occ in [sv1, sv2, sv3, axis] + ([body] if body else []):
        occ.isGrounded = True

    revolute(root, sv1, crank1, g['P1'], mm, 'J_c1 (脚長サーボ ①)')
    revolute(root, sv2, crank2, g['P2'], mm, 'J_c2 (脚長サーボ ②)')
    revolute(root, crank1, shank1, g['K1'], mm, 'J_k1 (膝・受動)')
    revolute(root, crank2, shank2, g['K2'], mm, 'J_k2 (膝・受動)')
    revolute(root, shank1, shank2, g['F'], mm, 'J_tip (足先・閉ループ)')
    revolute(root, shank1, spring, g['F'], mm, 'J_spring (バネ取付・仮)')

    up = design.userParameters
    for k, unit in (('d0', 'mm'), ('l1', 'mm'), ('l2', 'mm'), ('Ln', 'mm'), ('xf', 'mm'),
                    ('ls0', 'mm'), ('comp', 'mm'), ('rollOff', 'mm')):
        expr = '%g %s' % (p[k], unit)
        prm = up.itemByName('hop_' + k)
        if prm:
            prm.expression = expr
        else:
            up.add('hop_' + k, adsk.core.ValueInput.createByString(expr), unit, 'Hopper Leg')

    _app.activeViewport.fit()
    _ui.messageBox(
        '生成完了\n'
        'クランク① %.1f°  クランク② %.1f°  内角 %.1f°  伝達比 %.0f mm/rad\n'
        '足先ピン (%.1f, %.1f)  接地点 (%.1f, %.1f) mm\n'
        'ロール軸 = X 軸（Z = %.1f mm、脚面の中心を貫通）\n\n'
        'J_c1 / J_c2 をドラッグすると 5 節リンクが動きます（可動域と干渉の確認）。\n'
        'ブラケット類（サーボ ①② を保持する脚フレーム、ロールホーンとの接続）は生成していません。\n'
        '要件は docs/hopper-mechanical-spec.md を参照。'
        % (g['t1'], g['t2'], g['inner'], g['r'], g['F'][0], g['F'][1], g['S'][0], g['S'][1], z_mid / mm))


# ---------- スケッチ部品 ----------
def _P(mm, x, y):
    import adsk.core
    return adsk.core.Point3D.create(x * mm, y * mm, 0)


def _slot(sk, A, B, w, mm):
    dx, dy = B[0] - A[0], B[1] - A[1]
    L = math.hypot(dx, dy)
    if L < 1e-9:
        return
    ux, uy = dx / L, dy / L
    nx, ny = -uy, ux
    r = w / 2.0
    a1 = (A[0] + nx * r, A[1] + ny * r); b1 = (B[0] + nx * r, B[1] + ny * r)
    a2 = (A[0] - nx * r, A[1] - ny * r)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    lines.addByTwoPoints(_P(mm, *a1), _P(mm, *b1))
    lines.addByTwoPoints(_P(mm, *a2), _P(mm, *(B[0] - nx * r, B[1] - ny * r)))
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
    _circle(sk, c, horn['centerD'], mm)
    if horn['n'] > 0 and horn['pcd'] > 0:
        r = horn['pcd'] / 2.0
        for i in range(horn['n']):
            a = 2 * math.pi * i / horn['n']
            _circle(sk, (c[0] + r * math.cos(a), c[1] + r * math.sin(a)), horn['holeD'], mm)


def make_link(root, ctx, name, chain, z, horn_at=()):
    """点列を長穴で結んだ板リンク。各点にピン穴、horn_at にはホーン穴パターン。"""
    import adsk.fusion
    p, mm, T, horn = ctx['p'], ctx['mm'], ctx['T'], ctx['horn']
    occ = _new_occ(root, name)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = name + ' outline'
    for a, b in zip(chain[:-1], chain[1:]):
        _slot(sk, a, b, p['link_w'], mm)
    for c in horn_at:
        _circle(sk, c, max(horn['hornD'], horn['pcd'] + horn['holeD'] + 4.0), mm)
    _extrude_all(comp, sk, z, T, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sk2 = comp.sketches.add(comp.xYConstructionPlane); sk2.name = name + ' holes'
    for pt in chain:
        if not any(math.hypot(pt[0] - h[0], pt[1] - h[1]) < 1e-6 for h in horn_at):
            _circle(sk2, pt, p['pin_d'], mm)
    for c in horn_at:
        _horn_holes(sk2, c, horn, mm)
    _extrude_all(comp, sk2, z, T, adsk.fusion.FeatureOperations.CutFeatureOperation)
    return occ


def make_spring(root, ctx, g, z):
    """直列バネ＋足パッド。股 O → 足先 F の延長線上に伸びる（要件 §4-5）。
    バネは外形の帯として表現する（実体はリニアガイドか平行リンク。CAD で決める）。"""
    import adsk.fusion
    p, mm, T = ctx['p'], ctx['mm'], ctx['T']
    occ = _new_occ(root, 'Spring + Foot (直列バネ・足パッド)')
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Spring'
    _slot(sk, g['F'], g['S'], p['springD'], mm)
    # 足パッド: 接地点 S に幅 2R・厚 padT の板（円板の外接矩形。Z 方向にも 2R 広げる）
    u = g['u']
    n = (-u[1], u[0])
    R, t = p['padR'], p['padT']
    c = (g['S'][0] + u[0] * t / 2.0, g['S'][1] + u[1] * t / 2.0)
    corners = [(c[0] + n[0] * R - u[0] * t / 2.0, c[1] + n[1] * R - u[1] * t / 2.0),
               (c[0] - n[0] * R - u[0] * t / 2.0, c[1] - n[1] * R - u[1] * t / 2.0),
               (c[0] - n[0] * R + u[0] * t / 2.0, c[1] - n[1] * R + u[1] * t / 2.0),
               (c[0] + n[0] * R + u[0] * t / 2.0, c[1] + n[1] * R + u[1] * t / 2.0)]
    _rect(sk, corners, mm)
    _extrude_all(comp, sk, z, T, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sk2 = comp.sketches.add(comp.xYConstructionPlane); sk2.name = 'Spring hole'
    _circle(sk2, g['F'], p['pin_d'], mm)
    _extrude_all(comp, sk2, z, T, adsk.fusion.FeatureOperations.CutFeatureOperation)
    return occ


def make_servo_xy(root, ctx, name, c, ang, z0, horn_z):
    """脚長サーボ: 出力軸は Z 方向。XY にフットプリント、Z に奥行 D。"""
    import adsk.fusion
    p, mm, horn = ctx['p'], ctx['mm'], ctx['horn']
    occ = _new_occ(root, name)
    comp = occ.component
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Servo body'
    _rect(sk, servo_rect(c, ang, p['sW'], p['sH'], p['sOff']), mm)
    _extrude_all(comp, sk, z0, p['sD'] * mm, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sk2 = comp.sketches.add(comp.xYConstructionPlane); sk2.name = 'Horn'
    _circle(sk2, c, horn['hornD'], mm)
    zlo, zhi = min(horn_z), max(horn_z)
    _extrude_all(comp, sk2, zlo, zhi - zlo, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    return occ


def make_servo_yz(root, ctx, name, z_mid):
    """ロールサーボ: 出力軸は X 方向で、脚面（Z = z_mid）を貫通する（要件 §4-2）。
    本体は脚と干渉しないよう後方（−X）へ rollOff だけ逃がす。YZ 平面にフットプリント、X に奥行 D。
    YZ スケッチの (u, v) は世界の (Y, Z) に対応する。"""
    import adsk.fusion
    p, mm = ctx['p'], ctx['mm']
    occ = _new_occ(root, name)
    comp = occ.component
    D, hT = p['sD'] * mm, p['hornT'] * mm
    x_horn = -p['rollOff'] * mm                  # ホーン面の X
    # 本体（ホーン面から −X 側へ D）
    sk = comp.sketches.add(comp.yZConstructionPlane); sk.name = 'Roll servo body'
    _rect(sk, servo_rect((0.0, z_mid / mm), p['rollAng'], p['sW'], p['sH'], p['sOff']), mm)
    _extrude_all(comp, sk, x_horn - D, D, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    # ホーン（ホーン面から +X 側へ hT）
    sk2 = comp.sketches.add(comp.yZConstructionPlane); sk2.name = 'Roll horn'
    _circle(sk2, (0.0, z_mid / mm), p['hornD'], mm)
    _extrude_all(comp, sk2, x_horn, hT, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    return occ


def make_roll_axis(root, ctx, z_mid):
    """ロール軸（X 方向、Z = z_mid、Y = 0）を細い棒で可視化する。"""
    import adsk.fusion
    p, mm = ctx['p'], ctx['mm']
    occ = _new_occ(root, 'Roll axis (X, 脚面を貫通)')
    comp = occ.component
    L = (p['rollOff'] + p['sD'] + 40.0) * mm
    sk = comp.sketches.add(comp.yZConstructionPlane); sk.name = 'Roll axis'
    _circle(sk, (0.0, z_mid / mm), 3.0, mm)
    _extrude_all(comp, sk, -L, L + 40.0 * mm, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    return occ


def make_body_envelope(root, ctx, z_mid):
    """胴体の慣性エンベロープ。**重心を股軸の高さ（Y = 0）と脚面（Z = z_mid）に置く**（要件 §4-1）。
    実体ではなく、バッテリー・基板・構造材をこの中に収めて重心を合わせるための参照。"""
    import adsk.fusion
    p, mm = ctx['p'], ctx['mm']
    occ = _new_occ(root, 'Body envelope (重心＝股軸高さ)')
    comp = occ.component
    x, y = p['bl'] / 2.0, p['bh'] / 2.0
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = 'Body envelope'
    _rect(sk, [(-x, -y), (x, -y), (x, y), (-x, y)], mm)
    _extrude_all(comp, sk, z_mid - p['bw'] / 2.0 * mm, p['bw'] * mm,
                 adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    return occ


# ---------- ジョイント ----------
def _circular_edge_at(occ, Pt, mm):
    import adsk.core
    best, bestd = None, 1e9
    for body in occ.bRepBodies:
        for e in body.edges:
            gm = e.geometry
            if isinstance(gm, adsk.core.Circle3D):
                d = math.hypot(gm.center.x - Pt[0] * mm, gm.center.y - Pt[1] * mm)
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
    for name, Ln in (('飛行・引込み', 145.0), ('蹴り出し最大', 195.0)):
        p = dict(DEFAULTS)
        p['Ln'] = Ln
        g = geometry(p)
        print('%s (Ln=%.0f): t1 %.1f°  t2 %.1f°  内角 %.1f°  伝達比 %.0f mm/rad  足先 (%.1f, %.1f)  接地 (%.1f, %.1f)'
              % (name, Ln, g['t1'], g['t2'], g['inner'], g['r'], g['F'][0], g['F'][1], g['S'][0], g['S'][1]))
