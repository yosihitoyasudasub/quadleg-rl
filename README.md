# quadleg-rl

四脚ロボット（4 脚 × 3 自由度、股外転＋股ピッチ＋膝の四節リンク脚、DYNAMIXEL XL-320 × 12）の設計と歩行学習。

```
quadleg-rl/
├─ quadleg_rl/              学習コード（MuJoCo Playground / MJX / Brax PPO）
│   └─ train.py             train() / render_video() / plot_history()
├─ notebooks/
│   └─ 01_go1_playground.ipynb   Colab: テスト機 Go1 で歩行学習を体験
├─ tools/
│   ├─ webapp/quad-leg-5bar.html        5 節リンク脚のトルク計算 Web アプリ（ブラウザで開くだけ）
│   ├─ webapp/hopper-2d.html            一本脚ホッパー 2D シミュレーター（派生テーマ）
│   ├─ webapp/hopper-roll-2d.html       同 ロール面版（正面図、段階 1）
│   ├─ webapp/quad-leg-linkage.html     旧四節リンク版（参考）
│   └─ fusion/QuadLegLinkage/           Fusion 360 アドイン（骨格・サーボ・ジョイント生成）
└─ pyproject.toml
```

## ワークフロー

1. **Zed で編集** — `quadleg_rl/` にロジックを書く。ノートブックは呼び出すだけ。
2. **GitHub に push** — `git push`
3. **Colab で学習** — `notebooks/01_go1_playground.ipynb` を Colab で開く
   （GitHub 上のファイルを `https://colab.research.google.com/github/yosihitoyasudasub/quadleg-rl/blob/main/notebooks/01_go1_playground.ipynb` で直接開ける）。
   セル 2 で `git clone`/`pull`、セル 5 で学習、セル 6 で継続学習（チェックポイント再開）、セル 7 で指令追従の診断、セル 8-9 で動画。
4. チェックポイントと動画は Google Drive `MyDrive/quadleg-rl/logs/` に残る。

## ローカルで動作確認（CPU、任意）

```sh
uv venv --python 3.12 && .venv\Scripts\activate    # Windows は WSL2 推奨
uv pip install -e .
python -c "from quadleg_rl import train; print(train.gpu_info())"
```

CPU では学習は現実的でないが、環境の読み込みや MJCF の検証はできる。

## 次の段階（予定）

- `02_custom_quadruped.ipynb`: Web アプリの寸法から自作四脚の MJCF を生成し、Go1 環境をベースに差し替えて学習
- アクチュエータモデル: XL330 は BAM（Microduck の `microduck_rl` 参照）、XL-320 は PD＋トルク上限＋遅延で近似
- Sim2Real: ONNX 書き出し → Raspberry Pi＋U2D2＋IMU で 50 Hz 実行

## 参考

- MuJoCo Playground: https://github.com/google-deepmind/mujoco_playground
- Microduck RL（XL330 の sim2real レシピ）: https://github.com/pollen-robotics/microduck_rl
