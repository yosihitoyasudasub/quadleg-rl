"""一本脚ホッパーの MJCF を生成する。

2D シミュレーター（hopper-2d.html / hopper-roll-2d.html）と同じ機体をそのまま 3D にする:
  胴体（フリージョイント）
   └ ロール軸（x 軸ヒンジ、重心から hb 下）… XH540-W150
      ├ クランク①②（−y 軸ヒンジ、軸間 d0）→ 下腿①② → 先端（5 節閉ループ、equality/connect）… XM540 ×2
      └ キャリア（先端に connect、x/z スライドで追従）→ 足（z スライドが直列バネ、圧縮で +、球の点接地）
サーボは general アクチュエータで τ = ctrl − (stall/ω₀)·q̇（forcerange ±stall）＝トルク─速度直線。
反射慣性はジョイントの armature。閉ループの連結は site 同士の connect（anchor 指定だと相手側の点が既定姿勢 qpos0 で決まり初期姿勢と食い違う）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict

from .kinematics import LegGeom, ik, fk, shank_angles


@dataclass
class HopperParams:
    # 機体
    mb: float = 1.5        # 胴体質量 [kg]
    bl: float = 0.30       # 胴体 長さ x [m]（慣性用）
    bw: float = 0.16       # 胴体 幅 y [m]
    bh: float = 0.18       # 胴体 高さ z [m]
    hb: float = 0.0        # 重心→股（ロール軸）下 [m]
    mf: float = 0.05       # 足質量 [kg]
    # 脚
    d0: float = 0.040
    l1: float = 0.090
    l2: float = 0.130
    ls0: float = 0.080     # 直列バネ自然長＝ストローク [m]
    ks: float = 900.0      # 直列バネ [N/m]
    zs: float = 0.2        # バネ減衰比
    L0: float = 0.240      # 基準脚長（股→足、伸展時）[m]
    retract: float = 0.015 # 飛行中の引込み [m]
    # 接地
    mu: float = 0.8
    # 脚長サーボ ①②（XM540-W270）
    stall: float = 10.6
    nl_rpm: float = 30.0
    jr: float = 0.02
    # ロールサーボ ③（XH540-W150）
    stall3: float = 7.1
    nl3_rpm: float = 70.0
    jr3: float = 0.0063
    # 物理
    timestep: float = 0.00025
    drop: float = 0.03     # 初期落下 [m]

    @property
    def geom(self) -> LegGeom:
        return LegGeom(self.d0, self.l1, self.l2)

    @property
    def Ln0(self) -> float:
        """飛行姿勢のリンク先端長（股→先端）。"""
        return self.L0 - self.retract - self.ls0

    def w0(self) -> float:
        return self.nl_rpm * 2 * math.pi / 60

    def w03(self) -> float:
        return self.nl3_rpm * 2 * math.pi / 60


def initial_joints(p: HopperParams):
    """飛行姿勢（先端を真下 Ln0）の (t1, t2, s1, s2, tip_x, tip_z)。"""
    g = p.geom
    r = ik(g, 0.0, -p.Ln0)
    if r is None:
        raise ValueError("初期姿勢に到達できません: Ln0=%.3f" % p.Ln0)
    t1, t2 = r
    s1, s2 = shank_angles(g, t1, t2)
    fx, fz = fk(g, t1, t2)["F"]
    return t1, t2, s1, s2, fx, fz


def build_xml(p: HopperParams) -> str:
    t1, t2, s1, s2, fx, fz = initial_joints(p)
    ixx = p.mb * (p.bw ** 2 + p.bh ** 2) / 12
    iyy = p.mb * (p.bl ** 2 + p.bh ** 2) / 12
    izz = p.mb * (p.bl ** 2 + p.bw ** 2) / 12
    k = p.stall / p.w0()          # 逆起電力相当 [N·m·s/rad]
    k3 = p.stall3 / p.w03()
    b = 0.02 * k                  # 粘性摩擦（2D と同じ 2 %）
    b3 = 0.02 * k3
    cs = 2 * p.zs * math.sqrt(p.ks * p.mf)
    z0 = p.hb - fz + p.ls0 + p.drop   # 足が地面から drop の高さになる胴体高さ
    cap = 0.004
    return f"""<mujoco model="hopper1leg">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{p.timestep}" integrator="implicitfast" gravity="0 0 -9.80665"/>
  <default>
    <geom contype="0" conaffinity="0" group="1"/>
    <joint damping="0"/>
  </default>
  <visual><global offwidth="960" offheight="540"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".85 .87 .9" rgb2=".7 .73 .78" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0 -1 2" dir="0 0.4 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="grid" contype="1" conaffinity="1" friction="{p.mu} 0.005 0.0001"/>
    <body name="torso" pos="0 0 {z0:.5f}">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="{p.mb}" diaginertia="{ixx:.6f} {iyy:.6f} {izz:.6f}"/>
      <geom type="box" size="{p.bl/2} {p.bw/2} {p.bh/2}" rgba="0.35 0.5 0.75 0.35"/>
      <site name="imu" pos="0 0 0" size="0.005"/>
      <body name="rollframe" pos="0 0 {-p.hb}">
        <joint name="roll" type="hinge" axis="1 0 0" armature="{p.jr3}" damping="{b3:.5f}" range="-0.8 0.8"/>
        <inertial pos="0 0 -0.02" mass="0.02" diaginertia="1e-5 1e-5 1e-5"/>
        <geom type="cylinder" fromto="-0.03 0 0 0.03 0 0" size="0.012" rgba="0.55 0.36 0.84 0.8"/>
        <body name="crank1" pos="{-p.d0/2} 0 0">
          <joint name="c1" type="hinge" axis="0 -1 0" armature="{p.jr}" damping="{b:.5f}"/>
          <inertial pos="{p.l1/2} 0 0" mass="0.005" diaginertia="2e-6 2e-6 1e-7"/>
          <geom type="capsule" fromto="0 0 0 {p.l1} 0 0" size="{cap}" rgba="0.29 0.44 0.65 1"/>
          <body name="shank1" pos="{p.l1} 0 0">
            <joint name="s1" type="hinge" axis="0 -1 0"/>
            <inertial pos="{p.l2/2} 0 0" mass="0.005" diaginertia="3e-6 3e-6 1e-7"/>
            <geom type="capsule" fromto="0 0 0 {p.l2} 0 0" size="{cap}" rgba="0.18 0.62 0.56 1"/>
            <body name="tip" pos="{p.l2} 0 0">
              <inertial pos="0 0 0" mass="0.005" diaginertia="5e-7 5e-7 5e-7"/>
              <site name="tip" size="0.004"/>
            </body>
          </body>
        </body>
        <body name="crank2" pos="{p.d0/2} 0 0">
          <joint name="c2" type="hinge" axis="0 -1 0" armature="{p.jr}" damping="{b:.5f}"/>
          <inertial pos="{p.l1/2} 0 0" mass="0.005" diaginertia="2e-6 2e-6 1e-7"/>
          <geom type="capsule" fromto="0 0 0 {p.l1} 0 0" size="{cap}" rgba="0.29 0.44 0.65 1"/>
          <body name="shank2" pos="{p.l1} 0 0">
            <joint name="s2" type="hinge" axis="0 -1 0"/>
            <inertial pos="{p.l2/2} 0 0" mass="0.005" diaginertia="3e-6 3e-6 1e-7"/>
            <geom type="capsule" fromto="0 0 0 {p.l2} 0 0" size="{cap}" rgba="0.18 0.62 0.56 1"/>
            <site name="s2end" pos="{p.l2} 0 0" size="0.004"/>
          </body>
        </body>
        <body name="carrier" pos="{fx:.5f} 0 {fz:.5f}">
          <joint name="cx" type="slide" axis="1 0 0"/>
          <joint name="cz" type="slide" axis="0 0 1"/>
          <inertial pos="0 0 {-p.ls0/2}" mass="0.010" diaginertia="2e-6 2e-6 2e-6"/>
          <site name="ctop" size="0.004"/>
          <geom type="capsule" fromto="0 0 0 0 0 {-p.ls0}" size="0.003" rgba="0.79 0.46 0.17 1"/>
          <body name="foot" pos="0 0 {-p.ls0}">
            <joint name="fz" type="slide" axis="0 0 1" stiffness="{p.ks}" damping="{cs:.4f}" springref="0" range="-0.02 {p.ls0}"/>
            <inertial pos="0 0 0" mass="{p.mf - 0.010}" diaginertia="2e-6 2e-6 2e-6"/>
            <geom name="foot" type="sphere" size="0.010" contype="1" conaffinity="1" friction="{p.mu} 0.005 0.0001" rgba="0.79 0.46 0.17 1"/>
            <site name="foot" type="sphere" size="0.013"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <equality>
    <!-- solref の負値は剛性・減衰の直接指定。既定（有効質量比例）だと数 g の部品では 20 N で数十 cm ずれる -->
    <connect name="loop" site1="s2end" site2="tip" solref="-30000 -300"/>
    <connect name="spring" site1="ctop" site2="tip" solref="-30000 -300"/>
  </equality>
  <actuator>
    <general name="a1" joint="c1" gaintype="fixed" biastype="affine" gainprm="1 0 0" biasprm="0 0 {-k:.5f}" ctrlrange="{-p.stall} {p.stall}" forcerange="{-p.stall} {p.stall}"/>
    <general name="a2" joint="c2" gaintype="fixed" biastype="affine" gainprm="1 0 0" biasprm="0 0 {-k:.5f}" ctrlrange="{-p.stall} {p.stall}" forcerange="{-p.stall} {p.stall}"/>
    <general name="a3" joint="roll" gaintype="fixed" biastype="affine" gainprm="1 0 0" biasprm="0 0 {-k3:.5f}" ctrlrange="{-p.stall3} {p.stall3}" forcerange="{-p.stall3} {p.stall3}"/>
  </actuator>
  <sensor>
    <framequat name="torso_quat" objtype="site" objname="imu"/>
    <gyro name="gyro" site="imu"/>
    <touch name="foot_touch" site="foot"/>
  </sensor>
</mujoco>
"""


def initial_qpos(p: HopperParams):
    """build_xml と整合する初期 qpos（root 7 + roll + c1 s1 c2 s2 + cx cz fz）。"""
    t1, t2, s1, s2, fx, fz = initial_joints(p)
    z0 = p.hb - fz + p.ls0 + p.drop
    return [0.0, 0.0, z0, 1.0, 0.0, 0.0, 0.0, 0.0, t1, s1, t2, s2, 0.0, 0.0, 0.0]


if __name__ == "__main__":
    import sys
    p = HopperParams()
    xml = build_xml(p)
    print(xml if "--print" in sys.argv else f"{len(xml)} chars, qpos0={initial_qpos(p)}")
