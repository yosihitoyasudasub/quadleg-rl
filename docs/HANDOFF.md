# 引き継ぎドキュメント — quadleg-rl

作成: 2026-09-06（Claude Cowork セッションから Zed エージェントへ）

## 1. プロジェクトの目的

XL-320 × 8 個（手持ち）で 4 脚 × 2 自由度の四脚ロボットを作り、MuJoCo Playground（MJX / Brax PPO）で歩行ポリシーを学習して実機に載せる。
将来は XL330 に置き換え、股外転軸を追加して 3 自由度化する構想。

## 2. 現在の状態

| 項目 | 状態 |
| --- | --- |
| 脚リンク設計ツール（Web） | 完成・公開済み（Claude アーティファクト）。`tools/webapp/quad-leg-linkage.html` に置く予定だが**未コピー** → 出力フォルダから手動配置 |
| Fusion 360 アドイン | `tools/fusion/QuadLegLinkage/` に配置済み。リボンボタン・ダイアログ・前回値記憶まで実装。Fusion 上での実行テストは**未実施** |
| 学習コード | `quadleg_rl/train.py`（Playground の `train_jax_ppo.py` を関数化）。**未実行**、構文チェックも未実施（サンドボックス障害のため） |
| Colab ノートブック | `notebooks/01_go1_playground.ipynb`。テスト機 Go1（4 脚 × 3 自由度）で流れを体験する用途。**未実行** |
| GitHub | 未 push。`git init` から |

## 3. 決定済みの設計値

### 脚リンク機構（股関節 2 サーボ＋四節リンク、Web アプリの座標系）
- 股関節 O = 原点、+X 前方、+Y 上。サーボ①がホーン軸 O で大腿を直接駆動、サーボ②（同軸・背中合わせ、B=(0,0)）がクランク Lc → ロッド Lr → 下腿レバー Le で膝を駆動
- 現行案（XL-320 / XL330 共通）: **L1=48, L2=48, Lc=6, Le=12, Lr=48 mm**, δ=0, cfg=A
  - Lc=6 はホーン穴 P.C.D 12（半径 6）にロッドを直付けする想定。クランク：膝 = 約 1:2 の減速
  - 推奨立ち姿勢 θ1=−130°, θ2=127° → 足先 ≈ (0, −73) mm、膝角 ≈ 101°、伝達角 ≈ 88°
  - 荷重条件 1 kg・2 脚支持・動的係数 1.5: サーボ① ≈ 2.3 kg·cm、サーボ② ≈ 1.2 kg·cm、ロッド軸力 ≈ 20 N
- Web アプリ貼り付け用 params:
  `{"bx":0,"by":0,"L1":48,"L2":48,"Lc":6,"Lr":48,"Le":12,"delta":0,"cfg":1,"t1":-130,"t2":127,"mass":1,"legs":2,"sf":1.5,"fxr":0,"servo":5,"stall":4.0,"sW":24,"sH":36,"sOff":9,"sPCD":12,"sN":8,"sA1":0,"sA2":0}`
- 計算モデル: 静的釣合い τ = −Jᵀ·F（ヤコビアンは数値微分）。リンク自重・摩擦・慣性は未考慮

### サーボ仕様（ROBOTIS 図面・e-Manual で確認済み）
| 型番 | ストール | 質量 | 本体 W×H×D | ホーン軸〜上端 | ホーン | 備考 |
| --- | --- | --- | --- | --- | --- | --- |
| XL-320 | 0.39 N·m @7.4 V (≈4.0 kg·cm) | 16.7 g | 24×36×27 | 9.0 | Ø17.8、4-Ø4 リベット + 4-Ø1.6 タップ、P.C.D 12 | Joint 0–300°、電流制御なし、分解能 0.29°、2S 直結 |
| XL330-M288 | 0.52 N·m @5 V (≈5.3 kg·cm) | 18 g | 20×34×23(+3) | 9.5 | Ø16、4-Ø1.6（M2 タップ）P.C.D 12 | 電流制御あり、5 V レギュレータ必要、BAM モデル公開（microduck_rl） |
| XM540-W270 | 10.6 N·m @12 V | 165 g | 33.5×58.5×44 | 13.75 | Ø26、8-M2.5 P.C.D 22、ボス Ø10 | 大型案（不採用方向） |

XL-320 での余裕: 質量 1 kg で股 58%、膝 30%。**質量 0.7 kg 目標**（→ 股 40%）。ストールは瞬時値、連続は 1/3 目安。

### 歩行・旋回の結論
- 2 脚（平面脚 × 2）は静的にも動的にも歩行不可 → 4 脚で進める
- 4 脚 × 2 自由度: 前後歩行（トロット／クロール）と**差動ステア旋回**（足の滑り前提）まで可能。その場旋回・横歩きは外転軸が必要

## 4. 学習パイプラインの方針

1. **01（テスト機）**: Go1JoystickFlatTerrain を 2,000 万ステップ（T4 20〜40 分）→ 動画確認。1 億で公式設定
2. **Go1 を 2 自由度化**（最短の検証）: Menagerie の Go1 MJCF を複製し、外転関節を削除（または range 0・アクチュエータ削除）→ 8 アクチュエータ。Playground の `Go1` 環境クラスを継承し、観測次元・アクション次元・デフォルト姿勢（`default_pose`）・報酬のうち関節数依存の箇所を修正
3. **自作四脚 MJCF**: Web アプリの寸法・XL-320 質量（16.7 g × 8）・胴体質量から生成。四節リンクは最初は膝を直接関節にして 1:2 の比を後で角度変換（MuJoCo の `connect` 等式拘束で閉ループ再現は後回し）。アクチュエータは位置 PD（kp）＋ `forcerange` でストール相当（0.39 N·m）＋コマンド遅延で近似。XL330 なら BAM
4. **観測**: 関節角・関節速度・IMU（角速度・重力方向）・前回アクション・速度指令。XL-320 は電流フィードバックなし
5. **Sim2Real**: ONNX 書き出し → Raspberry Pi 4/5 ＋ U2D2 ＋ IMU（BNO085 等）で 50 Hz。OpenRB-150 では推論不可

## 5. コード上の注意点（未検証のため要確認）

- `quadleg_rl/train.py`
  - `locomotion_params.brax_ppo_config(env_name)` の戻り値（ConfigDict）から `network_factory` / `num_eval_envs` を取り出して `ppo.train` に渡している。Playground のバージョンによりキー名が変わる可能性
  - MJX の実装選択: Playground の Go1 Joystick は既定 config が `impl="warp"`。`mujoco-warp` 未導入の環境では
    `mjx.put_model` が `AttributeError: type object 'int' has no attribute 'WARP'` で落ちるため、`train()` は
    既定で `impl="jax"` に上書きする（`DEFAULT_IMPL` / 環境変数 `QUADLEG_MJX_IMPL` で変更可）。warp を入れた環境で
    速度を狙うなら `impl="warp"` を明示する
  - `rollout()` で `state.info["command"]` を直接書き換えて指令を固定している。Joystick 環境が `info["command"]` を持つ前提
  - `env.render()` は `State` のリストを受け取る（`.data` ではない）
  - チェックポイント再開: `restore_checkpoint_path` に run の `checkpoints/` を渡すと最新の数字ディレクトリを選ぶ
- **バージョンの組み合わせ（2026-09 時点で Colab 実行して確定）**: `jax[cuda12]==0.10.*` ＋ `flax>=0.12`。
  JAX 0.11 は `jax.device_put_replicated` を削除しており brax 0.14.2 が動かない（brax の依存は `jax>=0.4.6` で上限なし）。
  古い flax（Colab プリインストール）は `jax.core.get_opaque_trace_state` を直接呼ぶため JAX 0.10 以降で落ちる。
  brax が 0.11 対応したらこの固定を外す
- ノートブック: セル 2 で `sys.path.insert(0, os.getcwd())` してリポジトリ直下から `quadleg_rl` を import。GitHub URL 未設定ならファイル欄に `quadleg_rl/` をアップロード
- Colab の pip: `jax[cuda12]`, `playground`, `mediapy`。Menagerie アセットは初回ロード時に自動ダウンロード
- Fusion アドイン: as-built 回転ジョイントを円エッジ中心から作る方式。同軸配置ではサーボ②を +Z 側に置き、クランク層を +3s に移す。ユーザーパラメータ `qll_*` は記録のみで寸法は未拘束（「パラメータを変更」で追従させる拘束版は未実装）

## 6. 直近の TODO（優先順）

1. `git init` → GitHub push → Colab で 01 を実行し、`train.py` のエラーを潰す
2. `tools/webapp/quad-leg-linkage.html` を配置
3. Go1 2 自由度版環境（`quadleg_rl/envs/go1_2dof.py`）を作り、平面脚で歩行・差動旋回が学習できるか確認
4. 自作四脚 MJCF 生成スクリプト（`quadleg_rl/mjcf/`）＋ `02_custom_quadruped.ipynb`
5. 実機側: 胴体・脚の 3D プリント設計（Fusion アドインの骨格を土台に）、IMU と計算機の選定

## 7. 参考リンク

- MuJoCo Playground: https://github.com/google-deepmind/mujoco_playground（`learning/train_jax_ppo.py`, `learning/notebooks/locomotion.ipynb`）
- Microduck RL（XL330 BAM・ドメインランダム化・sim2real の手本）: https://github.com/pollen-robotics/microduck_rl
- XM540 e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
- XL-320 e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xl320/
- XL330 商品ページ（日本）: https://e-shop.robotis.co.jp/product.php?id=417
- 図面 PDF（XM540/XH540、X330、XL-320）はユーザー手元にあり。数値は本書 §3 に転記済み
