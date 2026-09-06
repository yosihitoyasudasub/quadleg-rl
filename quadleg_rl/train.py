"""MuJoCo Playground の環境を Brax PPO で学習する薄いラッパー。

ノートブックからは
    from quadleg_rl import train
    result = train.train("Go1JoystickFlatTerrain", num_timesteps=20_000_000, logdir="/content/drive/MyDrive/quadleg-rl/logs")
    train.render_video(result, "rollout.mp4")
の 2 行で使う。ロジックはここに置き、ノートブックには書かない。
"""
from __future__ import annotations

import datetime
import functools
import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

# JAX/MJX 向けの環境変数はインポート前に設定する
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ["XLA_FLAGS"] = os.environ.get("XLA_FLAGS", "") + " --xla_gpu_triton_gemm_any=True"

import jax  # noqa: E402
import jax.numpy as jp  # noqa: E402
import numpy as np  # noqa: E402
import mujoco  # noqa: E402

# --- 互換シム -------------------------------------------------------------
# PyPI の brax 0.14.2 は jax.device_put_replicated を呼ぶが、この API は JAX 0.10 で
# 削除されている（brax main では jax.device_put に修正済みだが未リリース）。
# brax main が入っていればここは通らない。PyPI 版しか無い環境のための保険。
if not hasattr(jax, "device_put_replicated"):
    def _device_put_replicated(x, devices):
        mesh = jax.sharding.Mesh(np.array(devices), ("_device_put_replicated",))
        sharding = jax.sharding.NamedSharding(
            mesh, jax.sharding.PartitionSpec("_device_put_replicated"))

        def put(v):
            stack = jp.stack if isinstance(v, jax.Array) else np.stack
            return jax.device_put(stack([v] * len(devices)), sharding)

        return jax.tree_util.tree_map(put, x)

    jax.device_put_replicated = _device_put_replicated
    print("[quadleg_rl] jax.device_put_replicated を補完しました（brax 0.14.2 対策）")
# --------------------------------------------------------------------------
from brax.training.agents.ppo import networks as ppo_networks  # noqa: E402
from brax.training.agents.ppo import train as ppo  # noqa: E402
from mujoco_playground import registry, wrapper  # noqa: E402
from mujoco_playground.config import locomotion_params  # noqa: E402


# Playground の一部環境（Go1 Joystick など）は既定 config が impl="warp" で、
# mujoco-warp（warp-lang）未導入の環境では mjx.put_model が AttributeError になる。
# 既定では JAX 実装に落とす。warp を入れてある環境では impl="warp" / None を渡す。
DEFAULT_IMPL: Optional[str] = os.environ.get("QUADLEG_MJX_IMPL", "jax") or None


def _with_impl(env_cfg, overrides: dict, impl: Optional[str]) -> dict:
    """config に impl キーがある Playground バージョンでのみ上書きする。"""
    out = dict(overrides)
    if impl and "impl" in env_cfg:
        out.setdefault("impl", impl)
    return out


@dataclass
class TrainResult:
    env_name: str
    env_cfg: Any
    ppo_params: Any
    make_inference_fn: Callable
    params: Any
    logdir: Path
    history: list = field(default_factory=list)   # [(steps, reward)]
    jit_time: float = 0.0
    train_time: float = 0.0
    impl: Optional[str] = None


def gpu_info() -> str:
    devs = jax.devices()
    return f"backend={jax.default_backend()} devices={[d.device_kind for d in devs]}"


def train(
    env_name: str = "Go1JoystickFlatTerrain",
    num_timesteps: int = 20_000_000,
    logdir: str | os.PathLike = "logs",
    seed: int = 1,
    num_envs: Optional[int] = None,
    num_evals: Optional[int] = None,
    domain_randomization: bool = True,
    impl: Optional[str] = DEFAULT_IMPL,
    restore_checkpoint_path: Optional[str] = None,
    env_config_overrides: Optional[dict] = None,
    progress_cb: Optional[Callable[[int, dict], None]] = None,
) -> TrainResult:
    """環境名を指定して PPO 学習。Playground 公式の推奨ハイパーパラメータをベースにする。"""
    env_cfg = registry.get_default_config(env_name)
    ppo_params = locomotion_params.brax_ppo_config(env_name)
    ppo_params.num_timesteps = num_timesteps
    if num_envs is not None:
        ppo_params.num_envs = num_envs
    if num_evals is not None:
        ppo_params.num_evals = num_evals

    overrides = _with_impl(env_cfg, env_config_overrides or {}, impl)
    env = registry.load(env_name, config=env_cfg, config_overrides=overrides)
    eval_env = registry.load(env_name, config=registry.get_default_config(env_name), config_overrides=overrides)

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    logdir = Path(logdir).expanduser().resolve() / f"{env_name}-{stamp}"
    ckpt_path = logdir / "checkpoints"
    ckpt_path.mkdir(parents=True, exist_ok=True)
    with open(ckpt_path / "config.json", "w", encoding="utf-8") as fp:
        json.dump(env_cfg.to_dict(), fp, indent=2)

    training_params = dict(ppo_params)
    network_factory = ppo_networks.make_ppo_networks
    if "network_factory" in training_params:
        network_factory = functools.partial(ppo_networks.make_ppo_networks, **training_params.pop("network_factory"))
    num_eval_envs = training_params.pop("num_eval_envs", 128)
    if domain_randomization:
        training_params["randomization_fn"] = registry.get_domain_randomizer(env_name)

    if restore_checkpoint_path:
        p = Path(restore_checkpoint_path)
        if p.is_dir() and not (p / "_METADATA").exists():
            subs = sorted([c for c in p.iterdir() if c.is_dir() and c.name.isdigit()], key=lambda c: int(c.name))
            if subs:
                p = subs[-1]
        restore_checkpoint_path = str(p)

    train_fn = functools.partial(
        ppo.train,
        **training_params,
        network_factory=network_factory,
        seed=seed,
        restore_checkpoint_path=restore_checkpoint_path,
        save_checkpoint_path=str(ckpt_path),
        wrap_env_fn=wrapper.wrap_for_brax_training,
        num_eval_envs=num_eval_envs,
    )

    history: list = []
    times = [time.monotonic()]

    def progress(num_steps, metrics):
        times.append(time.monotonic())
        r = float(metrics.get("eval/episode_reward", float("nan")))
        history.append((int(num_steps), r))
        print(f"[{num_steps:>12,d}] reward={r:8.3f}  elapsed={times[-1]-times[0]:7.0f}s", flush=True)
        if progress_cb:
            progress_cb(int(num_steps), metrics)

    print(f"{gpu_info()}\nenv={env_name}  timesteps={num_timesteps:,}  num_envs={ppo_params.num_envs}\nlogdir={logdir}")
    make_inference_fn, params, _ = train_fn(environment=env, progress_fn=progress, eval_env=eval_env)
    jit_time = times[1] - times[0] if len(times) > 1 else 0.0
    train_time = times[-1] - times[1] if len(times) > 1 else 0.0
    print(f"done. JIT {jit_time:.0f}s, train {train_time:.0f}s")
    return TrainResult(env_name, env_cfg, ppo_params, make_inference_fn, params, logdir, history,
                       jit_time, train_time, impl)


def rollout(result: TrainResult, episode_length: Optional[int] = None, seed: int = 0,
            command: Optional[tuple] = None, env_config_overrides: Optional[dict] = None):
    """学習済みポリシーでロールアウトし、レンダリング用の状態列を返す。
    command=(vx, vy, yaw_rate) を与えると Joystick 環境の指令を固定する。"""
    cfg = registry.get_default_config(result.env_name)
    env = registry.load(result.env_name, config=cfg,
                        config_overrides=_with_impl(cfg, env_config_overrides or {}, result.impl))
    ep_len = episode_length or int(result.ppo_params.episode_length)
    inference_fn = jax.jit(result.make_inference_fn(result.params, deterministic=True))
    jit_reset, jit_step = jax.jit(env.reset), jax.jit(env.step)
    rng = jax.random.PRNGKey(seed)
    state = jit_reset(rng)
    if command is not None and "command" in state.info:
        state.info["command"] = jp.array(command, dtype=jp.float32)
    states = [state]
    for _ in range(ep_len):
        rng, key = jax.random.split(rng)
        act, _ = inference_fn(state.obs, key)
        state = jit_step(state, act)
        # 環境側は一定間隔で command を再サンプリングするので、毎ステップ上書きし直す
        if command is not None and "command" in state.info:
            state.info["command"] = jp.array(command, dtype=jp.float32)
        states.append(state)
        if bool(state.done):
            print(f"[rollout] done at step {len(states)-1}/{ep_len}（転倒などで早期終了）")
            break
    return env, states


def summarize(env, states) -> dict:
    """ロールアウトが期待どおりか数値で確認する。動画が「動いていない」ときの切り分け用。"""
    qpos0, qpos1 = np.asarray(states[0].data.qpos), np.asarray(states[-1].data.qpos)
    dt = float(env.dt)
    duration = dt * (len(states) - 1)
    dx, dy = float(qpos1[0] - qpos0[0]), float(qpos1[1] - qpos0[1])
    info = {
        "steps": len(states) - 1,
        "duration_s": round(duration, 2),
        "dx_m": round(dx, 3),
        "dy_m": round(dy, 3),
        "speed_mps": round((dx**2 + dy**2) ** 0.5 / duration, 3) if duration else 0.0,
        "z_end_m": round(float(qpos1[2]), 3),
    }
    if "command" in states[-1].info:
        info["command_end"] = np.asarray(states[-1].info["command"]).round(3).tolist()
    print("[rollout]", info)
    return info


def probe(result: TrainResult, commands=((0.5, 0.0, 0.0), (1.0, 0.0, 0.0),
                                        (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)),
          episode_length: int = 300, seed: int = 0):
    """複数の指令でロールアウトし、指令に追従しているかを数値で確認する（描画なしなので速い）。

    dx が指令方向に伸びていれば追従、どの指令でも 0 付近ならポリシーが指令を無視している。
    """
    rows = []
    for cmd in commands:
        env, states = rollout(result, episode_length=episode_length, seed=seed, command=tuple(cmd))
        info = summarize(env, states)
        yaw_rate = None
        q0, q1 = np.asarray(states[0].data.qpos), np.asarray(states[-1].data.qpos)
        if len(q1) > 6:   # free joint のクォータニオンから yaw 変化を出す
            def yaw(q):
                w, x, y, z = q[3:7]
                return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            dt = float(env.dt) * (len(states) - 1)
            yaw_rate = round(float(np.unwrap([yaw(q0), yaw(q1)])[1] - yaw(q0)) / dt, 3) if dt else 0.0
        rew = {k: float(np.mean([np.asarray(s.metrics[k]) for s in states[1:]]))
               for k in states[-1].metrics if "tracking" in k}
        rows.append(dict(command=tuple(cmd), dx=info["dx_m"], dy=info["dy_m"],
                         speed=info["speed_mps"], yaw_rate=yaw_rate,
                         **{k.split("/")[-1]: round(v, 3) for k, v in rew.items()}))
    print("
=== probe ===")
    for r in rows:
        print(r)
    return rows


def render_video(result: TrainResult, out_path: str | os.PathLike = "rollout.mp4", render_every: int = 2,
                 width: int = 640, height: int = 480, camera: Optional[str] = "track",
                 **rollout_kwargs) -> Path:
    """camera="track" は Menagerie の Go1 等が持つ追従カメラ。無いモデルでは自由カメラに落とす。"""
    import mediapy as media
    env, states = rollout(result, **rollout_kwargs)
    summarize(env, states)
    traj = states[::render_every]   # MjxEnv.render は State のリストを受け取る
    fps = 1.0 / env.dt / render_every
    opt = mujoco.MjvOption()
    opt.geomgroup[2] = True    # 公式ノートブックと同じ表示設定（2=ビジュアル、3=コリジョン）
    opt.geomgroup[3] = False
    opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
    try:
        frames = env.render(traj, height=height, width=width, camera=camera, scene_option=opt)
    except Exception as e:   # カメラ名が無いモデル
        print(f"[render] camera={camera!r} を使えないので自由カメラで描画します: {e}")
        frames = env.render(traj, height=height, width=width, scene_option=opt)
    out_path = Path(out_path)
    media.write_video(out_path, frames, fps=fps)
    print(f"video: {out_path}  ({len(frames)} frames, {fps:.0f} fps)")
    return out_path


def plot_history(result: TrainResult):
    import matplotlib.pyplot as plt
    if not result.history:
        print("no history")
        return
    xs, ys = zip(*result.history)
    plt.figure(figsize=(6, 3))
    plt.plot(xs, ys, marker="o")
    plt.xlabel("env steps")
    plt.ylabel("eval episode reward")
    plt.title(result.env_name)
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.show()
