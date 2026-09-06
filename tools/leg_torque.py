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
