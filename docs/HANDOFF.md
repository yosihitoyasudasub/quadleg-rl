# 引き継ぎドキュメント — quadleg-rl

作成: 2026-09-06（Claude Cowork セッションから Zed エージェントへ）
更新: 2026-09-06（Colab で 01 を実行し歩行を確認。サーボ数の判明により **3 自由度化を決定**）

## 1. プロジェクトの目的

XL-320 で四脚ロボットを作り、MuJoCo Playground（MJX / Brax PPO）で歩行ポリシーを学習して実機に載せる。

**2026-09-06 に構成を変更**: 当初は手持ち 8 個を前提に 4 脚 × 2 自由度で進める計画だったが、
XL-320 が 14 個以上あることが判明したため、**4 脚 × 3 自由度（股外転・股ピッチ・膝、計 12 サーボ）** に変更する。
理由は §4 に記載。旧計画にあった `quadleg_rl/envs/go1_2dof.py`（Go1 の外転軸を殺した 8 アクチュエータ環境）は**不要**になった。

## 2. 現在の状態

| 項目 | 状態 |
| --- | --- |
| 脚リンク設計ツール（Web） | `tools/webapp/quad-leg-linkage.html` に配置済み（単一 HTML、ブラウザで開くだけ）。**平面 2 自由度のみ**。3 自由度化に伴い外転軸・胴体・スタンス幅の追加が必要 |
| Fusion 360 アドイン | `tools/fusion/QuadLegLinkage/` に配置済み。リボンボタン・ダイアログ・前回値記憶まで実装。Fusion 上での実行テストは**未実施** |
| 学習コード | `quadleg_rl/train.py`。**Colab で動作確認済み**。`train()` / `rollout()` / `summarize()` / `probe()` / `render_video()` / `plot_history()` |
| Colab ノートブック | `notebooks/01_go1_playground.ipynb`。**通しで実行し歩行を確認済み**（結果は §4.1）。トラブルシュート表つき |
| GitHub | https://github.com/yosihitoyasudasub/quadleg-rl （public）。main ブランチ |
| 自作四脚の MJCF | **未着手**。次の主タスク（§6） |

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
| XL330-M288 | 0.52 N·m @5 V (≈5.3 kg·cm) | 18 g | 20×34×23(+3) | 9.5 | Ø16、4-Ø1.6（M2 タップ）P.C.D 12 | 電流制御あり、5 V レギュレータ必要（8〜12 個分のピーク電流に注意） |
| XM540-W270 | 10.6 N·m @12 V | 165 g | 33.5×58.5×44 | 13.75 | Ø26、8-M2.5 P.C.D 22、ボス Ø10 | 大型案（不採用方向） |

XL-320 での余裕: 質量 1 kg で股 58%、膝 30%。**質量 0.7 kg 目標**（→ 股 40%）。ストールは瞬時値、連続は 1/3 目安（XL-320 なら約 0.13 N·m）。

**3 自由度化に伴う再計算が必要（未実施）**
- サーボ 4 個追加で +67 g、外転軸のブラケット・構造材を含めて **+100〜150 g** を見込む。0.7 kg 目標の再検討が要る
- 外転（ロール）軸のトルクは「脚の横オフセット × 荷重」で決まる。スタンス幅を狭くすれば抑えられる。Web アプリは平面 2 自由度の計算なので、この軸は**別途手計算が必要**
- XL330 への移行は「歩いた後の改善策」として保留。BAM に XL-320 の同定済みモデルがあるため、移行の主な動機だった sim2real の精度差は小さくなった（§5.2）

### 歩行・旋回の結論
- 2 脚（平面脚 × 2）は静的にも動的にも歩行不可 → 4 脚で進める
- 4 脚 × 2 自由度: 前後歩行（トロット／クロール）と**差動ステア旋回**（足の滑り前提）まで可能。その場旋回・横歩きは不可
- **4 脚 × 3 自由度（採用）**: その場旋回・横歩き・ロール外乱への踏み出し復帰が可能。Go1 と同じトポロジー

## 4. 学習パイプラインの方針

### 4.1 01（テスト機 Go1）— **完了**

`Go1JoystickFlatTerrain` を Colab の T4 で学習し、歩行・速度追従・旋回を確認した。

| 段階 | ステップ | 時間 | 最終 reward | 結果 |
| --- | --- | --- | --- | --- |
| 初回 | 2,949 万 | 15 分 | 17.1 | **指令を無視してその場に立つ**（局所解） |
| 継続（チェックポイント再開） | +7,373 万（累計 約 1 億） | 23 分 | 26.9 | 前進 1.0 指令に 0.92 m/s で追従、旋回も可 |

- T4 の実効速度は約 **68,000 ステップ/秒**（JIT は毎回 150 秒前後）
- reward は 2,000 万〜6,000 万の間 18〜20 で停滞し、**6,300 万付近で 19.9 → 23.0 と階段状に跳ねて歩き出す**。
  頭打ちに見えても学習量不足を疑うこと（公式推奨は 2 億ステップ）
- 局所解の理由は報酬設計から説明できる。`feet_clearance=-2.0` や `feet_slip=-0.1` は足を振らなければ発生せず、
  `pose=+0.5` は立っているだけで満額もらえる。**歩き出すには一時的な減点を受け入れる必要がある**
- 停滞しているかどうかは `train.probe(result)` で判定する。全指令で速度がほぼ 0、
  かつ停止指令のときだけ `tracking_lin_vel` が高ければ局所解

### 4.2 自作四脚 MJCF（3 自由度・次の主タスク）

**方針変更**: 旧計画の「Go1 を 2 自由度化する検証」は、3 自由度採用により不要になった。
自作機が Go1 と同じトポロジー（4 脚 × 3 自由度 = 12 アクチュエータ）になるため、
**Playground の Go1 環境をそのまま流用し、MJCF だけ自作機に差し替える**のが最短ルートになる。

- 関節構成: 各脚に 股外転（ロール）・股ピッチ・膝ピッチ。順序と命名は Go1 の MJCF に合わせると環境側の修正が減る
- 寸法: Web アプリの脚リンク設計（L1=48, L2=48 ほか）＋ 外転軸のオフセット（未設計）
- 質量: XL-320 16.7 g × 12 ＋ 構造材 ＋ 電池・計算機
- 四節リンクは、まず膝を直接関節としてモデル化し、クランク：膝 ≈ 1:2 の比は角度変換で扱う
  （`connect` 等式拘束による閉ループ再現は後回し）
- アクチュエータ: 位置 PD（`kp`）＋ `forcerange` でストール相当 0.39 N·m ＋ コマンド遅延。BAM の適用は §5.2 参照
- 環境側で確認・修正が要る箇所: `default_pose`（立ち姿勢）、関節の `range`、
  足先の body 名（接地判定に使う）、`Kp`/`Kd`、指令のスケール

### 4.3 観測とアクション

関節角・関節速度・IMU（角速度・重力方向）・前回アクション・速度指令。XL-320 は電流フィードバックがないので観測に入れない。
アクションは関節の目標角度（位置 PD）。**トルク制御は使わない** — 減速比の高いサーボでは摩擦が支配的で、
電流制御があっても実質的なトルク制御は困難。XL330 に移行する場合も「電流基準位置制御（電流上限つきの位置制御）」が現実解。

### 4.4 Sim2Real

ONNX 書き出し → Raspberry Pi 4/5 ＋ U2D2 ＋ IMU（BNO085 等）で 50 Hz。OpenRB-150 では推論不可。
実機で出せない力を学習中に使わせないため、`forcerange` をストール値に固定することが最重要。

## 5. コード上の注意点

### 5.1 学習コードと環境（Colab で検証済み）

- `quadleg_rl/train.py`
  - `locomotion_params.brax_ppo_config(env_name)` の戻り値（ConfigDict）から `network_factory` / `num_eval_envs` を取り出して `ppo.train` に渡している。Playground のバージョンによりキー名が変わる可能性
  - MJX の実装選択: Playground の Go1 Joystick は既定 config が `impl="warp"`。`mujoco-warp` 未導入の環境では
    `mjx.put_model` が `AttributeError: type object 'int' has no attribute 'WARP'` で落ちるため、`train()` は
    既定で `impl="jax"` に上書きする（`DEFAULT_IMPL` / 環境変数 `QUADLEG_MJX_IMPL` で変更可）。warp を入れた環境で
    速度を狙うなら `impl="warp"` を明示する
  - `rollout()` で `state.info["command"]` を直接書き換えて指令を固定している。Joystick 環境が `info["command"]` を持つ前提
  - `env.render()` は `State` のリストを受け取る（`.data` ではない）
  - チェックポイント再開: `restore_checkpoint_path` に run の `checkpoints/` を渡すと最新の数字ディレクトリを選ぶ
- **バージョンの組み合わせ（2026-09 時点、Colab で実際に歩かせた構成）**: `jax[cuda12]`（最新）＋ `playground` ＋ `flax>=0.12` ＋ **brax は PyPI 版のまま**
  - PyPI の brax 0.14.2（2026-03-15）は `jax.device_put_replicated` を使うが、この API は JAX **0.10** で削除済み
    （`_deprecations` の関数が `None` ＝ 警告ではなく `AttributeError`）。JAX を 0.10 に下げても直らない
  - brax は main の 2026-03-25 コミットで `jax.device_put` に修正済みだが未リリース。
    `pip install "brax @ git+..."` は版番号が 0.14.2 のままのため pip が差し替えをスキップすることがあり、当てにならない
  - そこで **`train.py` が `jax.device_put_replicated` の互換シムを持つ**（`jax` に属性が無いときだけ定義）。
    適用されると `[quadleg_rl] jax.device_put_replicated を補完しました` と表示される。
    brax の新リリースが出たらシムごと削除してよい
  - 古い flax（Colab プリインストール）は `jax.core.get_opaque_trace_state` を直接呼ぶため要更新
- 描画は `camera="track"`（Menagerie の Go1 等が持つ追従カメラ）を指定しないと自由カメラになり、機体が画面端に寄る
- Colab: `pip` はディスク、`import` はメモリ。**install → 再起動 → import** の順を守る。
  再起動すると `result` が失われるのでノートブックのセル 6（チェックポイント再開）で復元する。
  clone より先に `sys.path` へ入れた場合は `importlib.invalidate_caches()` が必要
- Menagerie アセットは初回ロード時に自動ダウンロードされる（40 秒ほど）

### 5.2 BAM（アクチュエータの摩擦モデル）— 未検証、MJCF 作成時に調査

- **BAM = "Better Actuator Models"**: https://github.com/Rhoban/bam （Apache-2.0）。
  作者は Rhoban チーム（ボルドー大学、Duclusaud・Passault ほか）。論文は ICRA 2025
  *"Extended Friction Models for the Physics Simulation of Servo Actuators"*。
  Pollen Robotics の microduck / Open Duck は**利用している側**（当初「Pollen 製」と誤認していた）
- MuJoCo 標準のクーロン＋粘性摩擦では表せない Stribeck 効果・荷重依存摩擦・二次項を扱う
- **`bam/params/xl320/` に m1〜m6 の同定済みパラメータが存在する**（実ファイルで確認済み）。
  XL330-M288-T、MX-64、MX-106、Feetech STS3215 なども同梱。
  → **XL-320 のままでも BAM を使える**ので、XL330 に移行する主要な動機の一つが消えた
- **未確認の課題**: README には「MuJoCo CPU と MuJoCo **Warp** から使える API」とある。
  本プロジェクトは `impl="jax"`（MJX の JAX 実装）で学習しているため、
  BAM の統合コードが Python コールバック方式だと JIT の中で使えない可能性がある。対応の選択肢は
  (a) `mujoco-warp` を入れて `impl="warp"` に戻す、(b) 摩擦モデルを JAX で書き直す、
  (c) `armature` / `damping` / `frictionloss` / `forcerange` に近似的に落とす。
  `bam/mujoco.py` を読んで判断する
- XL-320 の同定がどの電圧で行われたかは要確認（Feetech は `..._7_4V` と電圧付きだが xl320 は無印）

### 5.3 その他

- Fusion アドイン: as-built 回転ジョイントを円エッジ中心から作る方式。同軸配置ではサーボ②を +Z 側に置き、クランク層を +3s に移す。ユーザーパラメータ `qll_*` は記録のみで寸法は未拘束（「パラメータを変更」で追従させる拘束版は未実装）

## 6. 直近の TODO（優先順）

1. ~~`git init` → GitHub push → Colab で 01 を実行~~ **完了**（§4.1）
2. ~~Go1 2 自由度版環境~~ **不要**（3 自由度採用のため廃止）
3. **3 自由度の脚・胴体設計**: 外転軸のオフセットとブラケット、スタンス幅の決定。
   外転軸のトルクを手計算し、質量目標（0.7 kg → 要再検討）との整合を取る
4. **自作四脚 MJCF 生成スクリプト**（`quadleg_rl/mjcf/`）＋ `02_custom_quadruped.ipynb`。
   Go1 と同じ関節構成・命名にして Playground の Go1 環境を流用する
5. `tools/webapp/quad-leg-linkage.html` を配置（未コピーのまま）
6. BAM の MJX 適用可否を調査（§5.2）
7. 実機側: 胴体・脚の 3D プリント設計（Fusion アドインの骨格を土台に）、IMU と計算機の選定。
   サーボ取り付け部はパラメータ化し、将来の XL330 載せ替えに備える

## 7. 参考リンク

- MuJoCo Playground: https://github.com/google-deepmind/mujoco_playground（`learning/train_jax_ppo.py`, `learning/notebooks/locomotion.ipynb`）
- **BAM（Rhoban、同定済み摩擦モデル。XL-320 / XL330 両方あり）**: https://github.com/Rhoban/bam / https://bam.readthedocs.io/
- Microduck RL（ドメインランダム化・sim2real の手本。BAM の利用側）: https://github.com/pollen-robotics/microduck_rl
- XM540 e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
- XL-320 e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xl320/
- XL330 商品ページ（日本）: https://e-shop.robotis.co.jp/product.php?id=417
- 図面 PDF（XM540/XH540、X330、XL-320）はユーザー手元にあり。数値は本書 §3 に転記済み
