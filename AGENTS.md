# AGENTS.md — quadleg-rl の作業ルール（Zed エージェント向け）

まず `docs/HANDOFF.md` を読むこと。現在地・サーボ仕様・共通の実装方針・次の作業はそこにある。

## プロジェクト概要

**主線は一本脚ホッパー。** XM540-W270 × 2（ピッチ面の 5 節リンク脚）＋ XH540-W150 × 1（ロール）＋ 直列バネの 3 自由度。
2D シミュレーター（単一 HTML）で設計を詰め、MuJoCo（3D）で成立を確認済み（2026-09-08）。学習はまだしていない。

**四脚は保留。** プロジェクトは XL-320 × 12 の四脚を MuJoCo Playground で学習させることから始まり、
Colab で Go1 が歩くところまでは動いている。2026-09-06 の派生検討からホッパーが主線になった（`docs/quadruped.md`）。

実行環境: ホッパーは Colab の **CPU**（`notebooks/02_hopper_mujoco.ipynb`）、四脚の学習は Colab の **GPU**（`notebooks/01_go1_playground.ipynb`）。
ローカル（Windows）は編集と軽い検証のみ。**ローカルに mujoco は入っていない**（動力学の実行は Colab）。

## 構成

**ホッパー（主線）**
- `hopper/` — `kinematics.py`（5 節の IK/FK/ヤコビアン）、`model.py`（MJCF 生成）、`controller.py`（Raibert 制御）、`run.py`（実行・診断）
- `notebooks/02_hopper_mujoco.ipynb` — Colab(CPU) の薄いラッパー
- `tools/webapp/hopper-2d.html`（ピッチ面）／`hopper-roll-2d.html`（ロール面） — 単一 HTML の 2D シミュレーター
- `tools/fusion/HopperLeg/` — Fusion 360 アドイン。骨格・サーボ・ジョイントを生成。運動学は `hopper/kinematics.py` と同一式（一致確認済み）
- `tools/make_leg_diagram.py` — `docs/img/hopper-leg-structure.svg` を運動学から生成。寸法を変えたら再生成する
- `tools/make_shank_diagram.py` — `docs/img/hopper-shank.svg`（シャンク両端の関節構造）を生成。同上
- `tools/hopper_harness.js` / `hopper_sweeps.js` — 2D をブラウザなしで回す検証ハーネス。
  **シミュレーターを直したら必ず `node tools/hopper_sweeps.js` を走らせ、`docs/hopper-log.md` の数字が再現するか確認する**

**四脚（保留）**
- `quadleg_rl/` — 学習ロジック。ノートブックにロジックを書かない
- `notebooks/01_go1_playground.ipynb` — Colab(GPU)
- `tools/webapp/quad-leg-5bar.html`（現行）／`quad-leg-linkage.html`（旧四節版）、`tools/leg_torque.py`、`tools/fusion/QuadLegLinkage/`

**ドキュメント**（`docs/`）
- `HANDOFF.md` — 引き継ぎ。まずここ
- `hopper-report.md` — ホッパーの設計レポート（結論・物理・制御）
- `hopper-log.md` — ホッパーの検討ログ（時系列）。**コード中の「HANDOFF §8.x」はこの文書の §8.x を指す**
- `hopper-mujoco-notes.md` — MuJoCo 移行で踏んだ問題と対処、コツ
- `hopper-mechanical-spec.md` — 実機の機構設計要件（CAD の入力）。寸法・荷重・可動域・必須制約
- `quadruped.md` — 四脚（起点・保留）

## 約束事

- 単位: 長さ mm、角度 deg（Web アプリ・Fusion）。MuJoCo / MJX 側は m・rad。変換箇所にコメントを書く
- 座標系: 2D の脚設計は股関節 O 原点、+X 前方、+Y 上。MuJoCo は x 前・y 左・z 上。ピッチ θ は機首上げ正
- サーボ仕様は `docs/HANDOFF.md` §3 の値を正とする（e-Manual で確認済み）。推定値には「未検証」と明記する
- サーボ負荷は **5 ms 平均の持続ピーク**で読む。瞬時値は指令段差で必ず 100 % に触れる
- 無負荷回転数を超える設計は**失格**（減速比 152〜272:1 は逆駆動できずギヤが壊れる）
- 2D で決めた制御係数を 3D にそのまま持ち込まない。接地写像を測り直す（`hopper/run.py` の `sweep_gains()`）
- MJCF を書くときは `docs/hopper-mujoco-notes.md` §3「コツ」を先に読む（`connect` は site 同士、軽い部品は `solref` 直接指定、ほか）
- Playground / Brax の API はバージョン差が大きい。動かないときは公式 `learning/train_jax_ppo.py` と `learning/notebooks/locomotion.ipynb` の最新を参照する
- 依存追加は `pyproject.toml` とノートブックの pip セルの両方に反映
- 大きなバイナリは原則コミットしない。例外は結果を示す短い動画（`notebooks/*.mp4`、数百 KB まで）。
  チェックポイントと学習ログはコミットしない

## 次の実装対象

`docs/HANDOFF.md` §6 を正とする。要約すると、ホッパーは (1) センサの現実化、(2) 反射慣性の実測、(3) 実機設計。
四脚を再開するなら `docs/quadruped.md` §4。
