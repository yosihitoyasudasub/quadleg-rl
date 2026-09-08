"""一本脚ホッパーを MuJoCo で走らせ、2D シミュレーターと同じ指標を出す。

使い方（Colab / ローカル共通）:
    from hopper.run import simulate, summarize
    res = simulate(duration=15.0, vx_des=0.2)
    print(summarize(res))
    frames = res["frames"]     # record=True のとき（mediapy.show_video(frames, fps=50)）
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import replace

import numpy as np
import mujoco

from .model import HopperParams, build_xml, initial_qpos
from .controller import HopperController, ControlGains


def make(params: HopperParams | None = None):
    p = params or HopperParams()
    model = mujoco.MjModel.from_xml_string(build_xml(p))
    data = mujoco.MjData(model)
    data.qpos[:] = initial_qpos(p)
    mujoco.mj_forward(model, data)
    return p, model, data


def simulate(duration: float = 15.0, params: HopperParams | None = None, gains: ControlGains | None = None,
             vx_des: float | None = None, vy_des: float | None = None, hdes: float | None = None,
             vx0: float = 0.0, vy0: float = 0.0, th0: float = 0.0, roll0: float = 0.0,
             record: bool = False, fps: int = 50, camera: str = "track", width: int = 640, height: int = 360,
             log_dt: float = 0.005, verbose: bool = True):
    """duration 秒だけ走らせる。初期外乱は vx0/vy0[m/s]、th0/roll0[rad]。"""
    p, model, data = make(params)
    g = gains or ControlGains()
    if vx_des is not None: g = replace(g, vx_des=vx_des)
    if vy_des is not None: g = replace(g, vy_des=vy_des)
    if hdes is not None: g = replace(g, hdes=hdes)
    ctl = HopperController(p, g)
    ctl.bind(model)
    # 初期外乱
    data.qvel[ctl.v_root + 0] = vx0
    data.qvel[ctl.v_root + 1] = vy0
    if th0 or roll0:
        q = np.zeros(4)
        # ピッチ θ（機首上げ正）は -y 軸まわり、ロールは +x 軸まわり
        qp = np.zeros(4); mujoco.mju_axisAngle2Quat(qp, np.array([0.0, -1.0, 0.0]), th0)
        qr = np.zeros(4); mujoco.mju_axisAngle2Quat(qr, np.array([1.0, 0.0, 0.0]), roll0)
        mujoco.mju_mulQuat(q, qr, qp)
        data.qpos[ctl.q_root + 3:ctl.q_root + 7] = q
    mujoco.mj_forward(model, data)

    renderer = None
    if record:
        renderer = mujoco.Renderer(model, height=height, width=width)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance, cam.azimuth, cam.elevation = 1.2, 135, -15
    frames, log, hops_log = [], [], []
    dt = model.opt.timestep
    ctrl_dt = 1.0 / g.rate
    n_steps = int(round(duration / dt))
    next_log, next_frame = 0.0, 0.0
    fallen = False
    t0 = time.time()
    hops_seen = 0
    for i in range(n_steps):
        o = ctl.observe(data)
        tick = int(math.floor(data.time / ctrl_dt + 1e-9))
        if tick != ctl.tick:
            ctl.tick = tick
            ctl.control(data, o)
            if ctl.hops != hops_seen:
                hops_seen = ctl.hops
                hops_log.append(dict(t=data.time, **ctl.last))
        ctl.servo_step(data, o)
        mujoco.mj_step(model, data)
        ctl.record(data, o, dt)
        if data.time >= next_log:
            next_log += log_dt
            log.append((data.time, o["pos"][0], o["pos"][1], o["pos"][2], o["vx"], o["vy"], o["th"], o["roll"], o["yaw"],
                        data.actuator_force[ctl.a["a1"]], data.actuator_force[ctl.a["a2"]], data.actuator_force[ctl.a["a3"]],
                        o["w1"], o["w2"], o["wpsi"], o["delta"], o["Fn"]))
        if renderer is not None and data.time >= next_frame:
            next_frame += 1.0 / fps
            cam.lookat[:] = [o["pos"][0], o["pos"][1], 0.15]
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())
        if ctl.fallen(o):
            fallen = True
            break
    wall = time.time() - t0
    if renderer is not None:
        renderer.close()
    res = dict(params=p, gains=g, ctl=ctl, model=model, data=data, log=np.array(log), hops=hops_log,
               fallen=fallen, t_end=data.time, wall=wall, frames=frames)
    if verbose:
        print(summarize(res))
    return res


def trace(duration: float = 0.35, every: float = 0.01, params: HopperParams | None = None, gains: ControlGains | None = None,
          th0: float = 0.0, vx0: float = 0.0):
    """着地前後の内部状態を every 秒ごとに表示する診断。転倒の切り分け用。th0: 初期ピッチ [rad]（機首上げ正）"""
    p, model, data = make(params)
    g = gains or ControlGains()
    ctl = HopperController(p, g)
    ctl.bind(model)
    if th0:
        q = np.zeros(4); mujoco.mju_axisAngle2Quat(q, np.array([0.0, -1.0, 0.0]), th0)
        data.qpos[ctl.q_root + 3:ctl.q_root + 7] = q
    data.qvel[ctl.v_root] = vx0
    mujoco.mj_forward(model, data)
    dt = model.opt.timestep
    ctrl_dt = 1.0 / g.rate
    nxt = 0.0
    print("    t     z    vz    th°   om    vx   phase  cont  Fn   delta  t1°   t2°   tc1   tc2   f1    f2    Mcmd  bar°   cx   nefc")
    for i in range(int(duration / dt)):
        o = ctl.observe(data)
        tick = int(math.floor(data.time / ctrl_dt + 1e-9))
        if tick != ctl.tick:
            ctl.tick = tick
            ctl.control(data, o)
        ctl.servo_step(data, o)
        if data.time >= nxt:
            nxt += every
            f = data.actuator_force
            print(f"{data.time:6.3f} {o['pos'][2]:5.3f} {o['vz']:5.2f} {math.degrees(o['th']):5.1f} {o['om']:5.2f} {o['vx']:5.2f}  {ctl.phase:6s} {int(o['contact'])}  {o['Fn']:5.1f} {o['delta']*1000:6.1f} {math.degrees(o['t1']):6.1f} {math.degrees(o['t2']):6.1f} {ctl.tc[0]:5.2f} {ctl.tc[1]:5.2f} {f[0]:5.2f} {f[1]:5.2f} {ctl.Mcmd:6.2f} {math.degrees(data.qpos[ctl.q['bar']]):5.1f} {data.qpos[ctl.q['cx']]*1000:5.1f} {data.nefc}")
        mujoco.mj_step(model, data)
        if ctl.fallen(o):
            print("FALLEN at", round(data.time, 3))
            break
    return model, data, ctl


def check_kinematics(params: HopperParams | None = None):
    """解析 FK（hopper.kinematics）と MuJoCo の先端 site 位置が一致するか（ロールフレーム座標）。
    符号の取り違え（ヒンジ軸の向き）があればここでずれる。"""
    from .kinematics import ik, fk, shank_angles
    p, model, data = make(params)
    g = p.geom
    rf = model.body("rollframe").id
    tip = model.site("tip").id
    print(" target(x,z)[mm]   analytic F        mujoco tip(site)   diff[mm]")
    for (x, z) in [(0.0, -0.145), (0.03, -0.14), (-0.03, -0.14), (0.0, -0.18), (0.02, -0.11)]:
        r = ik(g, x, z)
        if r is None:
            print(f" ({x*1000:5.1f},{z*1000:6.1f})  unreachable"); continue
        s1, s2 = shank_angles(g, *r)
        q = initial_qpos(p)
        q[8], q[9], q[10], q[11] = r[0], s1, r[1], s2
        q[12] = math.atan2(x, -z)
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        F = fk(g, *r)["F"]
        # site の世界座標 → ロールフレーム座標
        d = data.site_xpos[tip] - data.xpos[rf]
        loc = data.xmat[rf].reshape(3, 3).T @ d
        print(f" ({x*1000:5.1f},{z*1000:6.1f})  ({F[0]*1000:6.1f},{F[1]*1000:7.1f})   ({loc[0]*1000:6.1f},{loc[1]*1000:5.1f},{loc[2]*1000:7.1f})   {np.hypot(F[0]-loc[0], F[1]-loc[2])*1000:5.2f}")


def sweep_gains(cns=(0.3, 0.4, 0.5, 0.6, 0.8), kvs=(0.01, 0.02, 0.03, 0.05), duration: float = 12.0,
                vx0: float = 0.1, th0: float = 0.03, vx_des: float = 0.0, params: HopperParams | None = None, base: ControlGains | None = None):
    """前進速度ループの cn / Kv を掃引し、外乱（vx0, th0）からの生存と直近 10 ホップの vx の平均・範囲を表にする。
    3D では接地写像が 2D と違うので、2D の値（cn 0.5 / Kv 0.03）をそのまま使わずここで決める。"""
    base = base or ControlGains()
    print(f"      {'kv':>5} " + "".join(f"{kv:>22.3f}" for kv in kvs))
    for cn in cns:
        row = [f"cn {cn:4.2f}      "]
        for kv in kvs:
            g = replace(base, cn=cn, kv=kv, vx_des=vx_des)
            r = simulate(duration, params=params, gains=g, vx0=vx0, th0=th0, verbose=False)
            h = r["hops"][-10:]
            if r["fallen"]:
                row.append(f"{'X@' + format(r['t_end'], '.1f') + 's':>22}")
            else:
                vx = [x["vx"] for x in h]
                ap = np.mean([x["apex"] for x in h]) * 100
                row.append(f"{np.mean(vx):6.2f}[{min(vx):5.2f}..{max(vx):5.2f}] {ap:4.1f}cm")
        print("".join(row))


def summarize(res) -> str:
    p, g, ctl = res["params"], res["gains"], res["ctl"]
    hops = res["hops"]
    L = [f"t {res['t_end']:.2f} s  hops {len(hops)}  {'FALLEN' if res['fallen'] else 'ok'}  (wall {res['wall']:.1f} s)"]
    if hops:
        last = hops[-10:]
        vx = np.mean([h["vx"] for h in last]); vy = np.mean([h["vy"] for h in last])
        apex = np.mean([h["apex"] for h in last])
        c = hops[-1]
        w0, w03 = p.w0(), p.w03()
        L.append(f"apex {apex*100:.1f} cm (target {g.hdes*100:.0f})  vx {vx:.2f} (target {g.vx_des})  vy {vy:.2f} (target {g.vy_des})  "
                 f"pitch {math.degrees(c['th']):.1f} deg  roll {math.degrees(c['roll']):.1f} deg  yaw {math.degrees(c['yaw']):.1f} deg  Ts {c['Ts']*1000:.0f} ms  dL {c['dL']*1000:.1f} mm")
        L.append(f"leg servo tau {c['tauS']:.2f} Nm ({c['tauS']/p.stall*100:.0f}% sustained, {c['tau']/p.stall*100:.0f}% inst)  "
                 f"w {c['wS']*60/2/math.pi:.0f} rpm ({c['wS']/w0*100:.0f}%)  |  roll servo tau {c['tau3S']:.2f} Nm ({c['tau3S']/p.stall3*100:.0f}%)  "
                 f"w {c['w3S']*60/2/math.pi:.0f} rpm ({c['w3S']/w03*100:.0f}%)  |  Fn {c['Fn']:.0f} N  delta {c['delta']*1000:.1f} mm  P {c['pow']:.1f} W")
        back = max(c['w'] / w0, c['w3'] / w03)
        L.append(f"backdrive: {'YES ' + format(back*100, '.0f') + '%' if back > 1.05 else 'none'}   Ts pred/meas {ctl.ts_pred*1000:.0f}/{ctl.Ts*1000:.0f} ms")
    return "\n".join(L)


def hop_table(res, n: int = 12) -> str:
    rows = ["hop   t[s]  apex[cm]  vx    vy    pitch  roll   yaw   Ts[ms]  dL[mm]  tau%  w%"]
    for i, h in enumerate(res["hops"][-n:]):
        p = res["params"]
        rows.append(f"{len(res['hops'])-n+i+1:>3} {h['t']:6.2f} {h['apex']*100:8.1f} {h['vx']:5.2f} {h['vy']:5.2f} {math.degrees(h['th']):6.1f} {math.degrees(h['roll']):6.1f} {math.degrees(h['yaw']):6.1f} {h['Ts']*1000:6.0f} {h['dL']*1000:7.1f} {h['tauS']/p.stall*100:5.0f} {h['wS']/p.w0()*100:4.0f}")
    return "\n".join(rows)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--hdes", type=float, default=0.05)
    ap.add_argument("--vx0", type=float, default=0.0)
    ap.add_argument("--th0", type=float, default=0.0)
    a = ap.parse_args()
    r = simulate(a.duration, vx_des=a.vx, vy_des=a.vy, hdes=a.hdes, vx0=a.vx0, th0=a.th0)
    print(hop_table(r))
