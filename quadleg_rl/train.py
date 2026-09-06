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
import mujoco  # noqa: E402
from brax.training.agents.ppo import networks as ppo_networks  # noqa: E402
from brax.training.agents.ppo import train as ppo  # noqa: E402
from mujoco_playground import registry, wrapper  # noqa: E402
from mujoco_playground.config import locomotion_params  # noqa: E402


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

    overrides = dict(env_config_overrides or {})
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
    return TrainResult(env_name, env_cfg, ppo_params, make_inference_fn, params, logdir, history, jit_time, train_time)


def rollout(result: TrainResult, episode_length: Optional[int] = None, seed: int = 0,
            command: Optional[tuple] = None, env_config_overrides: Optional[dict] = None):
    """学習済みポリシーでロールアウトし、レンダリング用の状態列を返す。
    command=(vx, vy, yaw_rate) を与えると Joystick 環境の指令を固定する。"""
    env = registry.load(result.env_name, config=registry.get_default_config(result.env_name),
                        config_overrides=env_config_overrides or {})
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
        if command is not None and "command" in state.info:
            state.info["command"] = jp.array(command, dtype=jp.float32)
        states.append(state)
        if bool(state.done):
            break
    return env, states


def render_video(result: TrainResult, out_path: str | os.PathLike = "rollout.mp4", render_every: int = 2,
                 width: int = 640, height: int = 480, **rollout_kwargs) -> Path:
    import mediapy as media
    env, states = rollout(result, **rollout_kwargs)
    traj = states[::render_every]   # MjxEnv.render は State のリストを受け取る
    fps = 1.0 / env.dt / render_every
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
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
