"""脚リンク機構の静的トルク計算（Web アプリ tools/webapp/quad-leg-linkage.html と同じ式）。

τ = −Jᵀ·F（ヤコビアンは数値微分）。リンク自重・摩擦・慣性は含まない。
単位は mm / deg / N·m。設計値の掃引に使う。
"""
import math

D2R = math.pi / 180
R2D = 180 / math.pi
G = 9.80665
KGCM = 0.0980665          # 1 kg·cm = 0.0980665 N·m


def fk(p, t1, t2):
    """順運動学。サーボ① 角 t1、サーボ② クランク角 t2 から膝 K・足先 F を求める。"""
    B = (p["bx"], p["by"])
    a1, a2 = t1 * D2R, t2 * D2R
    K = (p["L1"] * math.cos(a1), p["L1"] * math.sin(a1))
    C = (B[0] + p["Lc"] * math.cos(a2), B[1] + p["Lc"] * math.sin(a2))
    dx, dy = C[0] - K[0], C[1] - K[1]
    d = math.hypot(dx, dy)
    Le, Lr = p["Le"], p["Lr"]
    if d < 1e-9 or d > Le + Lr or d < abs(Le - Lr):
        return None                     # リンクが閉じない
    a = (Le * Le - Lr * Lr + d * d) / (2 * d)
    h = math.sqrt(max(0.0, Le * Le - a * a))
    px, py = K[0] + a * dx / d, K[1] + a * dy / d
    D = (px - p["cfg"] * h * dy / d, py + p["cfg"] * h * dx / d)
    ux, uy = -(D[0] - K[0]) / Le, -(D[1] - K[1]) / Le
    dl = p["delta"] * D2R
    rx = ux * math.cos(dl) - uy * math.sin(dl)
    ry = ux * math.sin(dl) + uy * math.cos(dl)
    return dict(K=K, C=C, D=D, F=(K[0] + p["L2"] * rx, K[1] + p["L2"] * ry))


def statics(p, t1, t2):
    """静的釣合いから各サーボの必要トルク、膝角、伝達角を求める。"""
    s = fk(p, t1, t2)
    if not s:
        return None
    h = 1e-4
    a, b = fk(p, t1 + h * R2D, t2), fk(p, t1, t2 + h * R2D)
    if not a or not b:
        return None
    J = [[(a["F"][0] - s["F"][0]) / h, (b["F"][0] - s["F"][0]) / h],
         [(a["F"][1] - s["F"][1]) / h, (b["F"][1] - s["F"][1]) / h]]
    Fy = p["mass"] * G / p["legs"] * p["sf"]
    Fx = Fy * p["fxr"] / 100
    s["tau1"] = -(J[0][0] * Fx + J[1][0] * Fy) / 1000      # mm → m
    s["tau2"] = -(J[0][1] * Fx + J[1][1] * Fy) / 1000
    tx, ty = s["K"]
    sx, sy = s["F"][0] - s["K"][0], s["F"][1] - s["K"][1]
    s["knee"] = 180 - math.acos(max(-1, min(1, (tx * sx + ty * sy) / (p["L1"] * p["L2"])))) * R2D
    # ロッドの軸力: クランク C-B とロッド方向の外積から。細長い棒なので座屈の目安になる
    cx, cy = s["C"][0] - p["bx"], s["C"][1] - p["by"]
    ux, uy = (s["D"][0] - s["C"][0]) / p["Lr"], (s["D"][1] - s["C"][1]) / p["Lr"]
    cross = cx * uy - cy * ux
    s["rodF"] = -s["tau2"] * 1000 / cross if abs(cross) > 1e-9 else float("nan")
    vx, vy = s["D"][0] - s["K"][0], s["D"][1] - s["K"][1]
    wx, wy = s["D"][0] - s["C"][0], s["D"][1] - s["C"][1]
    mu = math.acos(max(-1, min(1, (vx * wx + vy * wy) / (p["Le"] * p["Lr"])))) * R2D
    s["mu"] = min(mu, 180 - mu)         # 伝達角（90° に近いほど良い）
    return s


def solve_t2(p, t1, drop, lo=40.0, hi=180.0):
    """t1 を固定し、足先の高さが -drop になる t2 を探す。"""
    best = None
    t2 = lo
    while t2 <= hi:
        s = statics(p, t1, t2)
        if s:
            e = abs(s["F"][1] + drop)
            if best is None or e < best[0]:
                best = (e, t2)
        t2 += 1.0
    if not best:
        return None
    e, t2 = best
    for step in (0.2, 0.02):
        for i in range(-10, 11):
            c = t2 + step * i
            s = statics(p, t1, c)
            if s and abs(s["F"][1] + drop) < e:
                e, t2 = abs(s["F"][1] + drop), c
    return t2 if e < 0.3 else None


def sweep_stride(p, drop, span):
    """足先を同じ高さで前後 ±span mm 動かし、立位の姿勢と各トルクのピークを返す。"""
    stand, m1, m2, mu_min = None, 0.0, 0.0, 180.0
    t1 = -170.0
    while t1 <= -90:
        t2 = solve_t2(p, t1, drop)
        if t2 is not None:
            s = statics(p, t1, t2)
            if s and abs(s["F"][0]) <= span:
                m1, m2 = max(m1, abs(s["tau1"])), max(m2, abs(s["tau2"]))
                mu_min = min(mu_min, s["mu"])
                if stand is None or abs(s["F"][0]) < abs(stand["F"][0]):
                    stand = s
        t1 += 1.0
    return stand, m1, m2, mu_min


def knee_ratio(p, t1, t2, d=1.0):
    """サーボ② の 1° あたり膝角が何度動くか（減速比の逆数）。"""
    a, b = statics(p, t1, t2 - d), statics(p, t1, t2 + d)
    if not a or not b:
        return float("nan")
    return (b["knee"] - a["knee"]) / (2 * d)


def roll_torque(mass, legs, sf, d_mm):
    """外転軸: τ3 = 1脚あたり垂直力 × ロール軸から足先までの水平距離。"""
    return mass * G / legs * sf * d_mm / 1000


# =====================================================================
# 5 節リンク（2026-09-06 に採用）
# 2 個のサーボを距離 d0 だけ離して並べ、それぞれのクランク（長さ l1）の先から
# 下腿（長さ l2）が伸びて足先 F で合流する。膝は受動関節。d0=0 が同軸配置。
# 四節リンクと違い 2 個のサーボが荷重を分担し、脚にサーボが載らない。
# 軸を離すほど可動域全体のトルクが下がり、特異点からも遠ざかる。
# =====================================================================

def ik5(d0, l1, l2, fx, fy):
    """足先 (fx, fy) から 2 本のクランク角（rad）を解析的に求める。

    サーボ軸は P1=(-d0/2, 0)、P2=(+d0/2, 0)。各軸まわりの角度を返す（後側, 前側）。
    """
    out = []
    for px, sgn in ((-d0 / 2, -1), (d0 / 2, +1)):
        dx, dy = fx - px, fy
        d = math.hypot(dx, dy)
        if d > l1 + l2 or d < abs(l1 - l2) or d < 1e-9:
            return None                 # 到達範囲の外
        a = (l1 * l1 - l2 * l2 + d * d) / (2 * d)
        h2 = l1 * l1 - a * a
        if h2 < 0:
            return None
        h = math.sqrt(h2)
        mx, my = px + a * dx / d, a * dy / d
        out.append(math.atan2(my + sgn * h * dx / d, mx - sgn * h * dy / d - px))
    return out[0], out[1]


def fk5(d0, l1, l2, t1, t2):
    """クランク角から足先を求める（下側の交点を選ぶ）。"""
    A = (-d0 / 2 + l1 * math.cos(t1), l1 * math.sin(t1))
    B = (d0 / 2 + l1 * math.cos(t2), l1 * math.sin(t2))
    dx, dy = B[0] - A[0], B[1] - A[1]
    d = math.hypot(dx, dy)
    if d < 1e-9 or d > 2 * l2:
        return None
    h = math.sqrt(max(0.0, l2 * l2 - d * d / 4))
    mx, my = A[0] + dx / 2, A[1] + dy / 2
    return dict(A=A, B=B, F=(mx + h * dy / d, my - h * dx / d))


def stat5(d0, l1, l2, fx, fy, mass, legs, sf, fxr=0.0):
    """5 節リンクの静的トルク。内角は 2 本の下腿が足先でなす角（特異点の指標）。"""
    r = ik5(d0, l1, l2, fx, fy)
    if not r:
        return None
    t1, t2 = r
    s = fk5(d0, l1, l2, t1, t2)
    if not s or math.hypot(s["F"][0] - fx, s["F"][1] - fy) > 0.5:
        return None                     # 別の分岐に落ちた
    h = 1e-5
    a, b = fk5(d0, l1, l2, t1 + h, t2), fk5(d0, l1, l2, t1, t2 + h)
    if not a or not b:
        return None
    J = [[(a["F"][0] - s["F"][0]) / h, (b["F"][0] - s["F"][0]) / h],
         [(a["F"][1] - s["F"][1]) / h, (b["F"][1] - s["F"][1]) / h]]
    Fy = mass * G / legs * sf
    Fx = Fy * fxr / 100
    s["t1"], s["t2"] = t1, t2
    s["tau1"] = -(J[0][0] * Fx + J[1][0] * Fy) / 1000
    s["tau2"] = -(J[0][1] * Fx + J[1][1] * Fy) / 1000
    s["det"] = J[0][0] * J[1][1] - J[0][1] * J[1][0]
    v1 = (s["F"][0] - s["A"][0], s["F"][1] - s["A"][1])
    v2 = (s["F"][0] - s["B"][0], s["F"][1] - s["B"][1])
    s["inner"] = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (l2 * l2)))))
    return s


# =====================================================================
# 伝達比（可変機械利得）と並列バネ — 2026-09-07 追加
#
# 対称姿勢の 5 節リンクは自由度 1 で、鏡像対称は t1 + t2 = -180°。
# 伝達比 r = dL/dα（脚長 mm / クランク角 rad）が姿勢で連続的に変わる＝可変ギヤ。
#   足先力 F = 2τ/r、足先速度 v = r·ω。r が小さい＝ローギヤ（力が出て遅い）。
# 機械利得はパワーを増やさない（P = (2τ/r)·(r·ω) = 2τω で r が消える）。
# 変えられるのはモータがトルク─速度直線のどこに載るかだけ。
# =====================================================================

def ratio5(d0, l1, l2, L, h=1e-5):
    """対称姿勢（足先が中心軸上、脚長 L）での伝達比 r = dL/dα [mm/rad]。"""
    a = _alpha_for_L(d0, l1, l2, L)
    if a is None:
        return None
    lo, hi = _L_of_alpha(d0, l1, l2, a - h), _L_of_alpha(d0, l1, l2, a + h)
    if lo is None or hi is None:
        return None
    return (hi - lo) / (2 * h)


def _L_of_alpha(d0, l1, l2, a):
    """対称姿勢のクランク角 α（= t1）から脚長 L を求める。"""
    q = d0 / 2 - l1 * math.cos(a)
    if abs(q) > l2:
        return None
    return math.sqrt(l2 * l2 - q * q) - l1 * math.sin(a)


def _alpha_for_L(d0, l1, l2, L, lo=None, hi=None):
    """脚長 L を与えるクランク角 α を二分法で求める（L は α に対し単調な枝を使う）。"""
    lo = math.radians(95) if lo is None else lo
    hi = math.radians(265) if hi is None else hi
    for _ in range(120):
        m = (lo + hi) / 2
        v = _L_of_alpha(d0, l1, l2, m)
        if v is None or v < L:
            lo = m
        else:
            hi = m
    return (lo + hi) / 2


def ratio_profile(d0, l1, l2, n=200):
    """可動域全体の (L, r, 内角) を返す。r は山なりで両端の特異点で 0 に落ちる。"""
    out = []
    for i in range(n + 1):
        L = (l2 - l1) + 0.002 + ((l1 + l2) - (l2 - l1) - 0.004) * i / n
        r = ratio5(d0, l1, l2, L)
        s = stat5(d0, l1, l2, 0.0, -L, 1.0, 2, 1.0)
        if r is None or s is None:
            continue
        out.append((L, r, s["inner"]))
    return out


def _unwrap(x, ref):
    """ref を基準に ±180° の折返しを解く。"""
    while x - ref > math.pi:
        x -= 2 * math.pi
    while x - ref < -math.pi:
        x += 2 * math.pi
    return x


def par_spring5(d0, l1, l2, fy_stand, mass, legs, sf,
                fx_range=(-25, 25), fy_range=None, n=9):
    """並列バネ（各クランクの線形トーションバネ）を設計し、残るサーボトルクを返す。

    バネは左右で鏡像。crank1 は S1(t1) = tau0 + k·(t1 - t1s)、
    crank2 は S2(t2) = -S1(-pi - t2)（t1 + t2 = -pi の鏡像対称を使う）。
    (tau0, k) は可動域での残りトルクの最大値を最小化するように選ぶ。
    """
    fy_range = fy_range or (fy_stand - 22, fy_stand + 13)
    st = stat5(d0, l1, l2, 0.0, fy_stand, mass, legs, sf)
    if not st:
        return None
    t1s = st["t1"]

    grid = []
    for i in range(n):
        fx = fx_range[0] + (fx_range[1] - fx_range[0]) * i / (n - 1)
        for j in range(n):
            fy = fy_range[0] + (fy_range[1] - fy_range[0]) * j / (n - 1)
            s = stat5(d0, l1, l2, fx, fy, mass, legs, sf)
            if s and s["inner"] >= 20:          # 特異点近傍は使わない（HANDOFF §3）
                grid.append((fx, fy, s))

    def worst(tau0, k):
        m = 0.0
        for _, _, s in grid:
            a1 = _unwrap(s["t1"], t1s)
            a2 = _unwrap(-math.pi - s["t2"], t1s)     # crank2 を crank1 の座標へ写す
            m = max(m, abs(s["tau1"] - (tau0 + k * (a1 - t1s))),
                       abs(s["tau2"] + (tau0 + k * (a2 - t1s))))
        return m

    best = None
    for ki in range(-60, 61):
        k = ki * 0.002
        for ti in range(0, 61):
            tau0 = ti * 0.005
            w = worst(tau0, k)
            if best is None or w < best[0]:
                best = (w, k, tau0)
    res, k, tau0 = best
    bare = max(max(abs(s["tau1"]), abs(s["tau2"])) for _, _, s in grid)
    return dict(tau0=tau0, k=k, worst=res, bare=bare, t1s=t1s, n=len(grid),
                stand=max(abs(st["tau1"]), abs(st["tau2"])))
