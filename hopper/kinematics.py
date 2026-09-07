"""5 節リンク脚の運動学（脚面内、x 前・z 上、単位 m）。

`tools/webapp/hopper-2d.html` の ik/fk と同じ式。クランク角 t1（後、軸 x=-d0/2）・t2（前、軸 x=+d0/2）は
+x から +z へ反時計回りに測る（MJCF ではヒンジ軸を (0,-1,0) にして q = t とする）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class LegGeom:
    d0: float = 0.040   # サーボ軸間 [m]
    l1: float = 0.090   # クランク [m]
    l2: float = 0.130   # 下腿 [m]


def ik(g: LegGeom, fx: float, fz: float):
    """足先（先端）位置 → (t1, t2)。到達不能なら None。膝は外側（後側は後ろ、前側は前）に張り出す分岐。"""
    out = []
    for px, sgn in ((-g.d0 / 2, -1.0), (g.d0 / 2, 1.0)):
        dx, dz = fx - px, fz
        d = math.hypot(dx, dz)
        if d > g.l1 + g.l2 or d < abs(g.l1 - g.l2) or d < 1e-9:
            return None
        a = (g.l1 * g.l1 - g.l2 * g.l2 + d * d) / (2 * d)
        h2 = g.l1 * g.l1 - a * a
        if h2 < 0:
            return None
        h = math.sqrt(h2)
        mx, mz = px + a * dx / d, a * dz / d
        out.append(math.atan2(mz + sgn * h * dx / d, mx - sgn * h * dz / d - px))
    return out[0], out[1]


def fk(g: LegGeom, t1: float, t2: float):
    """(t1, t2) → dict(A, B, F)。A/B は膝、F は先端（下側の交点）。組めなければ None。"""
    ax, az = -g.d0 / 2 + g.l1 * math.cos(t1), g.l1 * math.sin(t1)
    bx, bz = g.d0 / 2 + g.l1 * math.cos(t2), g.l1 * math.sin(t2)
    dx, dz = bx - ax, bz - az
    d = math.hypot(dx, dz)
    if d < 1e-9 or d > 2 * g.l2:
        return None
    h = math.sqrt(max(0.0, g.l2 * g.l2 - d * d / 4))
    mx, mz = ax + dx / 2, az + dz / 2
    return {"A": (ax, az), "B": (bx, bz), "F": (mx + h * dz / d, mz - h * dx / d)}


def jacobian(g: LegGeom, t1: float, t2: float, h: float = 1e-6):
    """J = ∂F/∂(t1, t2)（2×2、m/rad）。数値微分。"""
    f0 = fk(g, t1, t2)
    fa = fk(g, t1 + h, t2)
    fb = fk(g, t1, t2 + h)
    if f0 is None or fa is None or fb is None:
        return None
    (x0, z0), (xa, za), (xb, zb) = f0["F"], fa["F"], fb["F"]
    return ((xa - x0) / h, (xb - x0) / h), ((za - z0) / h, (zb - z0) / h)


def shank_angles(g: LegGeom, t1: float, t2: float):
    """クランク角に対する下腿ヒンジの相対角 (s1, s2)（MJCF の初期 qpos 用）。"""
    f = fk(g, t1, t2)
    if f is None:
        return None
    (ax, az), (bx, bz), (fx, fz) = f["A"], f["B"], f["F"]
    s1 = wrap(math.atan2(fz - az, fx - ax) - t1)
    s2 = wrap(math.atan2(fz - bz, fx - bx) - t2)
    return s1, s2


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def unwrap_to(target: float, ref: float) -> float:
    """target を ref の ±π 以内に折り返す（±180° の飛びつき防止）。"""
    return ref + wrap(target - ref)


if __name__ == "__main__":
    g = LegGeom()
    r = ik(g, 0.0, -0.145)
    print("ik(0,-0.145) =", tuple(math.degrees(v) for v in r))
    f = fk(g, *r)
    print("fk ->", f["F"])
    print("J =", jacobian(g, *r))
    print("shank =", tuple(math.degrees(v) for v in shank_angles(g, *r)))
