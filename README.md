# quadleg-rl

跳ぶ脚の設計と制御。**現在の主線は一本脚ホッパー**（DYNAMIXEL XM540 × 2 ＋ XH540 × 1、直列バネ、5 節リンク脚）。
リポジトリ名は起点だった四脚ロボット（XL-320 × 12）に由来する。四脚は保留中で、資産と再開手順は `docs/quadruped.md` にある。

**到達点**（MuJoCo 3D、2026-09-08）

| 項目 | 結果 |
| --- | --- |
| その場ホップ | 40 ホップ連続、頂点 5.0 cm、接地 165〜190 ms、脚長サーボ トルク 48 % / 速度 75 % |
| 前進 | 0.15 m/s で安定（動画 `notebooks/02_hopper_forward_0.15.mp4`）。0.2 m/s はサーボ速度の上限で転倒 |
| 横移動 | 0.2 m/s で安定。ロールサーボ 35〜56 % |
| 外乱 | 前後 ±0.3 m/s・ピッチ 0.1 rad、横 0.3 m/s・ロール 0.1 rad から回復 |
| 円運動 | 半径 0.5 m・経路速度 0.12 m/s（1 周 26 s）で周回。胴体の向きは保ったまま平行移動する方式 |

## ドキュメント

| 読む順 | 文書 | 内容 |
| --- | --- | --- |
| 1 | [`docs/HANDOFF.md`](docs/HANDOFF.md) | 引き継ぎ。現在地・サーボ仕様・共通の実装方針・進め方・次の作業 |
| 2 | [`docs/hopper-report.md`](docs/hopper-report.md) | ホッパーの設計レポート。結論・物理メカニズム・制御・設計指針 |
| 3 | [`docs/hopper-mujoco-notes.md`](docs/hopper-mujoco-notes.md) | MuJoCo 移行で踏んだ 8 件の問題と対処、MJCF・制御移植・診断のコツ |
| — | [`docs/hopper-log.md`](docs/hopper-log.md) | ホッパーの検討ログ（時系列）。コード中の「HANDOFF §8.x」はここ |
| — | [`docs/quadruped.md`](docs/quadruped.md) | 四脚（起点・保留）。設計値・学習パイプライン・再開手順 |

## 構成

```
quadleg-rl/
├─ hopper/                    【主線】一本脚ホッパー（MuJoCo）
│   ├─ kinematics.py            5 節リンクの IK / FK / ヤコビアン
│   ├─ model.py                 MJCF 生成（HopperParams で寸法・サーボ・接触を指定）
│   ├─ controller.py            Raibert 3 分割制御（電流指令、姿勢優先のトルク配分）
│   └─ run.py                   実行・指標・動画・診断（trace / sweep_gains / yaw_test）
├─ tools/
│   ├─ webapp/hopper-2d.html        ホッパー 2D シミュレーター（ピッチ面、単一 HTML）
│   ├─ webapp/hopper-roll-2d.html   同 ロール面（正面図）
│   ├─ hopper_harness.js            2D をブラウザなしで実行する検証ハーネス
│   ├─ hopper_sweeps.js             掃引と回帰（node tools/hopper_sweeps.js）
│   ├─ webapp/quad-leg-5bar.html    四脚: 5 節リンク脚のトルク計算
│   ├─ webapp/quad-leg-linkage.html 四脚: 旧四節リンク版（参考）
│   ├─ leg_torque.py                四脚: 静的トルク・伝達比・並列バネの計算
│   └─ fusion/QuadLegLinkage/       四脚: Fusion 360 アドイン
├─ quadleg_rl/                 四脚: 学習コード（MuJoCo Playground / MJX / Brax PPO）
├─ notebooks/
│   ├─ 02_hopper_mujoco.ipynb       Colab(CPU): ホッパーを MuJoCo で動かす
│   ├─ 02_hopper_forward_0.15.mp4   その動画（前進 0.15 m/s）
│   └─ 01_go1_playground.ipynb      Colab(GPU): 四脚のテスト機 Go1 で歩行学習
├─ docs/
└─ pyproject.toml
```

## 始め方

**ホッパーを動かす（GPU 不要）**

[Colab で 02 を開く](https://colab.research.google.com/github/yosihitoyasudasub/quadleg-rl/blob/main/notebooks/02_hopper_mujoco.ipynb)
→ ランタイムは CPU のまま、セル 1 → 1b → 2 → 3 → 4 → 6 の順に実行。動画はセル 7。

ローカルで動かすなら:

```sh
uv venv --python 3.12 && .venv\Scripts\activate
uv pip install -e ".[hopper]"
python -m hopper.run --duration 15 --vx 0.15
```

**2D シミュレーターを触る**

`tools/webapp/hopper-2d.html` をブラウザで開くだけ（依存なし）。寸法・バネ・サーボ機種・制御係数をその場で変えられる。
数値で掃引するときは `node tools/hopper_sweeps.js`。

**四脚の学習（保留中）**

[Colab で 01 を開く](https://colab.research.google.com/github/yosihitoyasudasub/quadleg-rl/blob/main/notebooks/01_go1_playground.ipynb)
→ ランタイムを GPU（T4 以上）にして上から実行。累計 1 億ステップで Go1 が歩く（約 40 分）。詳細は `docs/quadruped.md`。

## 参考

- MuJoCo Playground: https://github.com/google-deepmind/mujoco_playground
- BAM（同定済みアクチュエータ摩擦モデル、XL-320 / XL330 あり）: https://github.com/Rhoban/bam
- Microduck RL（XL330 の sim2real レシピ）: https://github.com/pollen-robotics/microduck_rl
