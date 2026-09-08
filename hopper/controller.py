"""Raibert 3 分割制御（2D シミュレーターからの移植）。

ピッチ面: hopper-2d.html の vLoop=1（予測中立点）・電流制御と同じ。
ロール面: hopper-roll-2d.html と同じ（ロールサーボ直結、接地中は τ₃ = Kφ·φ + Kφd·φ̇）。
ホスト（本クラスの control()）は制御周期ごとに目標／トルク指令を更新し、
サーボ内蔵の位置 PD（servo_step()）は物理ステップごとに動く。

座標: MuJoCo 世界 x 前・y 左・z 上。胴体ピッチ θ は機首上げ正（2D と同じ）、ロール φ は +x 軸まわり。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import mujoco

from .kinematics import ik, fk, jacobian, unwrap_to
from .model import HopperParams

G = 9.80665


@dataclass
class ControlGains:
    rate: float = 200.0     # 制御周期 [Hz]
    hdes: float = 0.05      # 目標ホップ高（足先地上高）[m]
    vx_des: float = 0.0     # 前進速度指令 [m/s]
    vy_des: float = 0.0     # 横速度指令 [m/s]（+y = 左）
    v_ramp: float = 0.02    # 目標速度ランプ [m/s / hop]
    dlmax: float = 0.050    # 最大蹴り出し [m]
    kh: float = 0.5
    # ピッチ面（前進）
    cn: float = 0.35     # 3D の掃引（HANDOFF §8.10）で決めた値。2D は 0.5
    kv: float = 0.02
    ki: float = 0.004    # 足先配置の積分補正 [m per (m/s) per hop]。定常の速度偏差（写像の切片）を消す
    kth: float = 40.0
    kthd: float = 1.0
    kr: float = 20000.0
    dr: float = 300.0
    thrust_frac: float = 0.75  # 蹴り出し（逆方向成分）のトルク上限 [×stall]。速度飽和の余裕を残し姿勢トルクを守る
    kp: float = 30.0        # 脚長サーボ 位置 PD
    kd: float = 1.0
    # ロール面（横）
    cn_r: float = 0.7
    kv_r: float = 0.02
    ki_r: float = 0.004
    kphi: float = 40.0
    kphid: float = 1.0
    kp3: float = 30.0
    kd3: float = 1.0


@dataclass
class HopperController:
    p: HopperParams
    g: ControlGains = field(default_factory=ControlGains)

    def __post_init__(self):
        self.geom = self.p.geom
        self.k = self.p.stall / self.p.w0()
        self.k3 = self.p.stall3 / self.p.w03()
        self.reset()

    # ---------- 状態 ----------
    def reset(self):
        self.phase = "FLIGHT"
        self.Ts = 0.12
        self.dL = self.g.dlmax / 2
        self.vz_prev = 0.0
        self.delta_prev = 0.0
        self.t_td = 0.0
        self.tick = -1
        self.hops = 0
        self.apex = None
        self.ts_pred = 0.0
        self.ts_at_td = 0.0
        self.ts_corr = 1.0
        self.vx_eff = 0.0
        self.vy_eff = 0.0
        self.tmode = False
        self.tq = np.zeros(2)      # クランク位置目標
        self.tc = np.zeros(2)      # クランクトルク指令
        self.tpsi = 0.0            # ロール位置目標
        self.tc3 = 0.0             # ロールトルク指令
        self.last = None
        self.pk = self._new_pk()
        self.filt = np.zeros(4)    # 5 ms 平均（τ12, ω12, τ3, ω3）
        self.xf_cmd = 0.0
        self.yf_cmd = 0.0
        self.xbias = 0.0
        self.ybias = 0.0
        self.Mcmd = 0.0
        self.Mcmd3 = 0.0
        self.alloc = (0.0, 0.0)

    @staticmethod
    def _new_pk():
        return dict(Fn=0.0, tau=0.0, w=0.0, tauS=0.0, wS=0.0, tau3=0.0, w3=0.0, tau3S=0.0, w3S=0.0, delta=0.0, pow=0.0)

    # ---------- MuJoCo の index ----------
    def bind(self, model: mujoco.MjModel):
        self.m = model
        j = lambda n: model.joint(n).id
        self.q_root = model.jnt_qposadr[j("root")]
        self.v_root = model.jnt_dofadr[j("root")]
        self.q = {n: model.jnt_qposadr[j(n)] for n in ("roll", "c1", "s1", "c2", "s2", "bar", "cx", "fz")}
        self.v = {n: model.jnt_dofadr[j(n)] for n in ("roll", "c1", "s1", "c2", "s2", "bar", "cx", "fz")}
        self.a = {n: model.actuator(n).id for n in ("a1", "a2", "a3")}
        self.s_touch = model.sensor("foot_touch").adr[0]
        self.site_tip = model.site("tip").id
        self.site_foot = model.site("foot").id
        self.b_roll = model.body("rollframe").id

    # ---------- 観測 ----------
    def observe(self, d: mujoco.MjData):
        qp, qv = d.qpos, d.qvel
        pos = qp[self.q_root:self.q_root + 3].copy()
        quat = qp[self.q_root + 3:self.q_root + 7]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        vel = qv[self.v_root:self.v_root + 3].copy()
        omb = qv[self.v_root + 3:self.v_root + 6].copy()     # 胴体座標の角速度
        # ZYX オイラー: roll = atan2(R21, R22), pitch = -asin(R20), yaw = atan2(R10, R00)
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        yaw = math.atan2(R[1, 0], R[0, 0])
        th = -pitch            # 機首上げ正（2D と同じ向き）
        om_pitch = -omb[1]     # θ̇
        # ヨー方向の水平フレームでの速度成分
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        vx, vy, vz = float(vel @ fwd), float(vel @ left), float(vel[2])
        psi, wpsi = float(qp[self.q["roll"]]), float(qv[self.v["roll"]])
        t1, t2 = float(qp[self.q["c1"]]), float(qp[self.q["c2"]])
        w1, w2 = float(qv[self.v["c1"]]), float(qv[self.v["c2"]])
        delta = float(qp[self.q["fz"]])             # バネ圧縮（+、足が上へ押される）
        ddelta = float(qv[self.v["fz"]])
        touch = float(d.sensordata[self.s_touch])
        contact = touch > 0.5
        return dict(pos=pos, R=R, vel=vel, vx=vx, vy=vy, vz=vz, th=th, om=om_pitch, roll=roll, omr=float(omb[0]),
                    yaw=yaw, psi=psi, wpsi=wpsi, t1=t1, t2=t2, w1=w1, w2=w2, delta=delta, ddelta=ddelta,
                    contact=contact, Fn=touch)

    def stand_h(self) -> float:
        """飛行姿勢で足先が接地する胴体高さ。"""
        return self.p.hb + self.p.L0 - self.p.retract

    # ---------- ホスト制御（制御周期ごと） ----------
    def control(self, d: mujoco.MjData, o: dict):
        p, g, geom = self.p, self.g, self.geom
        now = d.time
        loaded = o["contact"] and o["delta"] > 0.001
        if self.phase == "FLIGHT":
            if self.vz_prev > 0 and o["vz"] <= 0:                     # 頂点
                self.apex = o["pos"][2] - self.stand_h()
                self.hops += 1
                self.vx_eff = float(np.clip(g.vx_des, self.vx_eff - g.v_ramp, self.vx_eff + g.v_ramp)) if g.v_ramp > 0 else g.vx_des
                self.vy_eff = float(np.clip(g.vy_des, self.vy_eff - g.v_ramp, self.vy_eff + g.v_ramp)) if g.v_ramp > 0 else g.vy_des
                self.dL = float(np.clip(self.dL + g.kh * (g.hdes - self.apex), 0.0, g.dlmax))
                self.xbias = float(np.clip(self.xbias + g.ki * (o["vx"] - self.vx_eff), -0.03, 0.03))
                self.ybias = float(np.clip(self.ybias + g.ki_r * (o["vy"] - self.vy_eff), -0.03, 0.03))
                self.last = dict(apex=self.apex, vx=o["vx"], vy=o["vy"], th=o["th"], roll=o["roll"], yaw=o["yaw"],
                                 Ts=self.Ts, dL=self.dL, **self.pk)
                self.pk = self._new_pk()
            if loaded:
                self.phase, self.t_td, self.ts_at_td = "COMP", now, self.ts_pred
        else:
            if self.phase == "COMP" and o["delta"] < self.delta_prev and o["delta"] > 0.002:
                self.phase = "THRUST"
            if not o["contact"]:
                self.Ts = float(np.clip(now - self.t_td, 0.03, 0.6))
                if self.ts_at_td > 0:
                    self.ts_corr = float(np.clip(self.ts_corr + 0.5 * (self.Ts / self.ts_at_td - self.ts_corr), 0.5, 2.0))
                self.phase = "FLIGHT"
        self.vz_prev, self.delta_prev = o["vz"], o["delta"]

        Lr = p.Ln0                                    # 引込み時のリンク先端長
        f = fk(geom, o["t1"], o["t2"])
        if f is None:
            return
        Fx, Fz = f["F"]
        Ln = math.hypot(Fx, Fz)
        self.tmode = False
        if self.phase == "FLIGHT":
            om = math.sqrt(p.ks / (p.mb + p.mf))
            v0 = math.sqrt(max(0.0, o["vz"] ** 2 + 2 * G * max(0.0, o["pos"][2] - self.stand_h())))
            self.ts_pred = float(np.clip((2 / om) * (math.pi - math.atan(v0 * om / G)) * self.ts_corr, 0.03, 0.6))
            # 足先配置（前後 x、左右 y）。世界水平（ヨー基準）
            xf = g.cn * o["vx"] * self.ts_pred / 2 + g.kv * (o["vx"] - self.vx_eff) + self.xbias
            yf = g.cn_r * o["vy"] * self.ts_pred / 2 + g.kv_r * (o["vy"] - self.vy_eff) + self.ybias
            xf = float(np.clip(xf, -0.6 * Lr, 0.6 * Lr))
            yf = float(np.clip(yf, -0.6 * Lr, 0.6 * Lr))
            self.xf_cmd, self.yf_cmd = xf, yf
            # ロール: 足を +y に置くには +x 軸まわりに asin(yf/Lr)。ロール関節は胴体基準なので φ を引く
            bw = math.asin(max(-1.0, min(1.0, yf / Lr)))
            self.tpsi = bw - o["roll"]
            # ピッチ面: 世界（x, z）→ 脚面。脚面の前後方向は胴体 x（ロールで変わらない）
            Lp = Lr * math.cos(bw)
            wx, wz = xf, -math.sqrt(max(0.0, Lp * Lp - xf * xf))
            c, s = math.cos(o["th"]), math.sin(o["th"])
            tx, tz = c * wx + s * wz, -s * wx + c * wz
            r = None
            sc = 1.0
            for _ in range(10):
                r = ik(geom, tx * sc, tz * sc)
                if r is not None:
                    break
                sc *= 0.97
            if r is not None:
                self.tq[0] = unwrap_to(r[0], o["t1"])
                self.tq[1] = unwrap_to(r[1], o["t2"])
        else:
            L = Lr + self.dL if self.phase == "THRUST" else Lr
            J = jacobian(geom, o["t1"], o["t2"])
            if J is None:
                return
            u = (Fx / Ln, Fz / Ln)
            n = (-u[1], u[0])
            self.Mcmd = g.kth * o["th"] + g.kthd * o["om"]
            er = L - Ln
            vr = (J[0][0] * o["w1"] + J[0][1] * o["w2"]) * u[0] + (J[1][0] * o["w1"] + J[1][1] * o["w2"]) * u[1]
            Fr = g.kr * er - (0.0 if (self.phase == "THRUST" and er > 0.003) else g.dr * vr)
            Ft = self.Mcmd / Ln
            F = (Fr * u[0] + Ft * n[0], Fr * u[1] + Ft * n[1])
            self.tc[0] = J[0][0] * F[0] + J[1][0] * F[1]
            self.tc[1] = J[0][1] * F[0] + J[1][1] * F[1]
            self.Mcmd3 = g.kphi * o["roll"] + g.kphid * o["omr"]
            self.tc3 = self.Mcmd3
            self.tmode = True

    # ---------- サーボ内蔵ループ（物理ステップごと） ----------
    def allocate(self, t1: float, t2: float, w1: float, w2: float):
        """姿勢優先のトルク配分。クランク①②のトルクを同方向成分 s（接線力＝ピッチモーメント）と
        逆方向成分 d（脚長方向の力）に分け、各サーボが今の速度で出せる範囲 [lo, hi]（トルク─速度直線）に
        収まるよう d を削る。蹴り出しで片方だけ速度飽和すると、削らない場合は寄生の同方向成分が出て
        機首上げになる（3D で判明、HANDOFF §8.10）。"""
        p = self.p
        st, k = p.stall, self.k
        def rng(w):
            return max(-st, -st - k * w), min(st, st - k * w)
        lo1, hi1 = rng(w1); lo2, hi2 = rng(w2)
        s = 0.5 * (t1 + t2); dd = 0.5 * (t1 - t2)
        dmax = self.g.thrust_frac * st
        dd = min(max(dd, -dmax), dmax)
        lo, hi = max(lo1, lo2), min(hi1, hi2)
        if lo > hi:
            lo = hi = 0.5 * (lo + hi)
        s = min(max(s, lo), hi)
        if dd >= 0:
            dd = min(dd, hi1 - s, s - lo2)
        else:
            dd = max(dd, lo1 - s, s - hi2)
        self.alloc = (s, dd)
        return s + dd, s - dd

    def servo_step(self, d: mujoco.MjData, o: dict):
        p, g = self.p, self.g
        if self.tmode:
            t1, t2 = self.allocate(self.tc[0], self.tc[1], o["w1"], o["w2"])
            t3 = self.tc3
        else:
            t1 = g.kp * (self.tq[0] - o["t1"]) - g.kd * o["w1"]
            t2 = g.kp * (self.tq[1] - o["t2"]) - g.kd * o["w2"]
            t3 = g.kp3 * (self.tpsi - o["psi"]) - g.kd3 * o["wpsi"]
        # アクチュエータは force = ctrl − k·q̇（forcerange ±stall）。目標トルクに逆起電力分を足して指令する
        d.ctrl[self.a["a1"]] = float(np.clip(t1 + self.k * o["w1"], -p.stall, p.stall))
        d.ctrl[self.a["a2"]] = float(np.clip(t2 + self.k * o["w2"], -p.stall, p.stall))
        d.ctrl[self.a["a3"]] = float(np.clip(t3 + self.k3 * o["wpsi"], -p.stall3, p.stall3))

    def record(self, d: mujoco.MjData, o: dict, dt: float):
        """ピーク（瞬時と 5 ms 平均）の更新。物理ステップごとに呼ぶ。"""
        f = d.actuator_force
        tau = max(abs(f[self.a["a1"]]), abs(f[self.a["a2"]]))
        w = max(abs(o["w1"]), abs(o["w2"]))
        tau3, w3 = abs(f[self.a["a3"]]), abs(o["wpsi"])
        af = dt / 0.005
        self.filt += (np.array([tau, w, tau3, w3]) - self.filt) * af
        pk = self.pk
        pk["Fn"] = max(pk["Fn"], o["Fn"]); pk["tau"] = max(pk["tau"], tau); pk["w"] = max(pk["w"], w)
        pk["tau3"] = max(pk["tau3"], tau3); pk["w3"] = max(pk["w3"], w3); pk["delta"] = max(pk["delta"], o["delta"])
        pk["tauS"] = max(pk["tauS"], self.filt[0]); pk["wS"] = max(pk["wS"], self.filt[1])
        pk["tau3S"] = max(pk["tau3S"], self.filt[2]); pk["w3S"] = max(pk["w3S"], self.filt[3])
        pk["pow"] = max(pk["pow"], abs(f[self.a["a1"]] * o["w1"]) + abs(f[self.a["a2"]] * o["w2"]) + abs(f[self.a["a3"]] * o["wpsi"]))

    def fallen(self, o: dict) -> bool:
        return abs(o["th"]) > 1.2 or abs(o["roll"]) > 1.2 or o["pos"][2] < 0.3 * self.stand_h()
