# AGENTS.md — quadleg-rl の作業ルール（Zed エージェント向け）

まず `docs/HANDOFF.md` を読むこと。目的・決定済み設計値・未検証箇所・TODO はそこにある。

## プロジェクト概要
四脚ロボット（XL-320 × 12、**4 脚 × 3 自由度**＝股外転 ＋ 股に同軸 2 サーボの **5 節リンク**）の歩行を
MuJoCo Playground（MJX / Brax PPO）で学習する。
2026-09-06 に 2 自由度案 → 3 自由度、四節リンク → 5 節リンクへ変更（経緯と根拠は HANDOFF §3）。
学習は Google Colab の GPU で実行。ローカル（Windows）は編集と軽い検証のみ。

## 構成
- `quadleg_rl/` — 学習ロジック。ノートブックにロジックを書かない
- `notebooks/` — Colab 用の薄いラッパー（clone → install → train → 動画）
- `tools/fusion/QuadLegLinkage/` — Fusion 360 アドイン（骨格・サーボ・ジョイント生成）
- `tools/webapp/` — 脚機構トルク計算 Web アプリ（`quad-leg-5bar.html` が現行、`quad-leg-linkage.html` は旧四節版）、`hopper-2d.html` は一本脚ホッパーの 2D シミュレーター（HANDOFF §8）。蓄勢方式 A / C を切替比較できる
- `tools/leg_torque.py` — 設計値検証用の静的トルク計算
- `docs/` — 引き継ぎ・設計メモ

## 約束事
- 単位: 長さ mm、角度 deg（Web アプリ・Fusion）。MuJoCo/MJX 側は m・rad。変換箇所にコメントを書く
- 座標系: 股関節 O 原点、+X 前方、+Y 上（脚の 2D 設計）。MJCF では Z 上に読み替える
- サーボ仕様は `docs/HANDOFF.md` §3 の値を正とする（図面で確認済み）。推定値を入れる場合は「未検証」と明記
- Playground / Brax の API はバージョン差が大きい。動かないときは公式 `learning/train_jax_ppo.py` と `learning/notebooks/locomotion.ipynb` の最新を参照して合わせる
- 依存追加は `pyproject.toml` とノートブックの pip セルの両方に反映
- 大きなバイナリ（動画・チェックポイント）はコミットしない（`.gitignore` 済み）

## 次の実装対象（順に）
1. ~~01 ノートブックを Colab で通す~~ **完了**（累計 1 億ステップで歩行を確認、HANDOFF §4.1）
2. ~~`go1_2dof.py`~~ **廃止**（3 自由度採用により不要）
3. `quadleg_rl/mjcf/` — 自作四脚 MJCF 生成（Web アプリ params → XML）と `02_custom_quadruped.ipynb`。
   Go1 と同じ関節構成・命名にして Playground の Go1 環境をそのまま流用する
