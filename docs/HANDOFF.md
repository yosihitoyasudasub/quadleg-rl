# 引き継ぎドキュメント — 一本脚ホッパー / 四脚ロボット

作成: 2026-09-06（Claude Cowork セッションから Zed エージェントへ）
更新: 2026-09-09（**主線を四脚 → 一本脚ホッパーに変更**。ドキュメントを再編）

まずここを読み、必要な詳細は各文書へ進むこと。

| 知りたいこと | 読む文書 |
| --- | --- |
| ホッパーの結論・物理・制御（**まずこれ**） | `hopper-report.md` |
| ホッパーの検討ログ（時系列。コード中の「HANDOFF §8.x」はここ） | `hopper-log.md` |
| MuJoCo で踏んだ問題と対処、MJCF・制御移植のコツ | `hopper-mujoco-notes.md` |
| 四脚（起点、現在は保留）の設計値・学習パイプライン | `quadruped.md` |
| サーボ仕様・実機の制御器・共通の実装方針・次の作業 | **本書**（以下） |

## 1. 現在地

**主線は一本脚ホッパー。** XM540-W270 × 2（ピッチ面の 5 節リンク脚）＋ XH540-W150 × 1（ロール）＋ 直列バネの 3 自由度で、
跳びながらバランスし移動する機体。2D シミュレーターで設計を詰め、MuJoCo（3D）で成立を確認した段階。

**四脚は保留。** プロジェクトは「XL-320 × 12 の四脚ロボットを学習させる」ことから始まり、
学習パイプライン（Colab で Go1 が歩くところまで）と脚設計ツールはできている。
2026-09-06 の派生検討からホッパーが主線になった。四脚の資産と再開手順は `quadruped.md` に分離してある。

**両者は無関係ではない。** 5 節リンク脚、サーボのトルク─速度モデル、AtomS3R による Sim2Real の構成は共通で、
ホッパーで得た知見（伝達比、直列バネ、電流制御、閉ループ MJCF の書き方）はそのまま四脚に持ち帰れる。

## 2. 現在の状態

| 項目 | 状態 |
| --- | --- |
| **ホッパー MuJoCo（3D、主線）** | `hopper/`（`model.py` MJCF 生成、`controller.py` Raibert、`run.py`）＋ `notebooks/02_hopper_mujoco.ipynb`。**Colab CPU で成立を確認済み**（2026-09-08）。その場 5 cm・前進 0.15 m/s・横 0.2 m/s。動画 `notebooks/02_hopper_forward_0.15.mp4` |
| ホッパー 2D シミュレーター（ピッチ面） | `tools/webapp/hopper-2d.html`。蓄勢方式 A（着地エネルギー）/ C（飛行中蓄勢＋ラッチ）を切替比較できる |
| ホッパー 2D シミュレーター（ロール面） | `tools/webapp/hopper-roll-2d.html`。正面図、股ロールサーボ直結、脚長 1 自由度 |
| ホッパーの検証ハーネス | `tools/hopper_harness.js`（2D をブラウザなしで実行）＋ `tools/hopper_sweeps.js`。`node tools/hopper_sweeps.js` |
| ホッパーの円運動 | `ControlGains.circle_r`（速度ベクトルを世界座標で回す方式）。**成立を確認済み**（2026-09-09、半径 0.5 m・経路速度 0.12 m/s・1 周 26 s）。ノートブック 02 のセル 7b / 7c |
| ホッパーの実機 | **未着手**（§6） |
| 四脚 — 学習コード | `quadleg_rl/train.py`。Colab で動作確認済み |
| 四脚 — Colab ノートブック | `notebooks/01_go1_playground.ipynb`。通しで実行し Go1 の歩行を確認済み |
| 四脚 — 脚設計ツール | `tools/webapp/quad-leg-5bar.html`（5 節リンク）、`tools/leg_torque.py` |
| 四脚 — Fusion 360 アドイン | `tools/fusion/QuadLegLinkage/`。実装済み、Fusion 上での実行テストは未実施 |
| 四脚 — 自作 MJCF | **未着手**（保留、`quadruped.md`） |
| GitHub | https://github.com/yosihitoyasudasub/quadleg-rl （public）。main ブランチ |

## 3. サーボ仕様（共通・2026-09-06〜09-08 に ROBOTIS e-Manual で確認）

| 型番 | ストール | 無負荷速度 | 質量 | 本体 W×H×D | ホーン軸〜端 | ホーン | 備考 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **XL-320（採用）** | **0.39 N·m @7.4 V**（4.0 kg·cm、1.1 A） | 114 rpm | 16.7 g | 24×36×27 | 9.0 | Ø17.8、4-Ø4 リベット + 4-Ø1.6 タップ、P.C.D 12 | 減速比 238:1、分解能 0.29°、電流制御なし、2S 直結、**Sync Read 可 / Bulk 不可**、位置データ 2 バイト |
| XL330-M288 | 0.52 N·m @5 V（5.3 kg·cm、1.47 A） | 103 rpm | 18 g | 20×34×26 | 9.5 | Ø16、4-Ø1.6（M2 タップ）P.C.D 12 | 減速比 288:1、分解能 4096、電流制御あり、5 V レギュレータ必要、BAM モデルあり |
| XL430-W250 | 1.5 N·m @12 V（15.3 kg·cm） | 61 rpm | 57.2 g | 28.5×46.5×34 | 12（**未確認**） | — | 減速比 258:1、分解能 4096 |
| XM540-W270 | 10.6 N·m @12 V（108.1 kg·cm、4.4 A） | 30 rpm | 165 g | 33.5×58.5×44 | 13.75 | Ø26、8-M2.5 P.C.D 22、ボス Ø10 | 減速比 272:1、大型案（不採用方向） |
| XH540-W150 | 7.1 N·m @12 V（72.4 kg·cm） | 70 rpm | 165 g | 33.5×58.5×44 | 13.75 | XM540 と同一 | 減速比 152.3:1。**ホッパーのロールサーボ候補**（`hopper-log.md` §8.9、2026-09-08 e-Manual 確認） |

- ストールは瞬時値。**連続使用は 1/3 が目安**（XL-320 なら 0.130 N·m）
- サーボ 12 個の質量: XL-320 で **200 g**、XL330 で 216 g、XL430 で 686 g、XM540 で 1,980 g
- 5 節リンクでの余裕（d0=30/l1=25/l2=70、1.0 kg）: 立位 0.107（連続の 82%）、可動域の最大 0.153（ストールの 39%）
- **設計質量は 1.0 kg**。1.2 kg を超えると立位で連続定格に達する

**採用**: ホッパーは脚長 **XM540-W270 × 2** ＋ ロール **XH540-W150 × 1**（ロールは速度が要るので減速比の小さい機種）。
四脚は **XL-320 × 12**。

## 4. 共通の実装方針

### 4.1 Sim2Real — **M5Stack AtomS3R で実行**（2026-09-06 決定）

旧案（Raspberry Pi 4 ＋ U2D2 ＋ 外部 IMU、計 85 g 超）から変更。**AtomS3R 単体で 6.7 g**、IMU も内蔵で、
約 75 g（総質量の 7.5%）の軽量化になる。

| 項目 | 仕様 |
| --- | --- |
| MCU | ESP32-S3-PICO-1-N8R8（240 MHz デュアルコア、FPU あり） |
| メモリ | Flash 8 MB ＋ **PSRAM 8 MB**、内蔵 SRAM 512 KB |
| IMU | **BMI270（6 軸）内蔵** ＋ BMM150（地磁気） |
| その他 | 0.85 型 LCD、Wi-Fi/BLE、Grove、IO 6 本（G5/G6/G7/G8/G38/G39） |
| 電源 | 5 V 入力（3.3 V 出力）。サーボの 7.4 V とは別系統、DC-DC が必要 |
| サイズ・質量 | 24×24×12 mm、6.7 g、¥3,531（スイッチサイエンス） |

**ポリシーのサイズ制約（学習時に効く）**

| 構成 | パラメータ数 | float32 | 判定 |
| --- | --- | --- | --- |
| (512, 256, 128)（Go1 公式設定） | 約 192,000 | 770 KB | **不可に近い**。PSRAM 必須で 50 Hz なら 38 MB/s の読み出しになり、Octal PSRAM の実効帯域とほぼ同じ |
| **(128, 128, 128)**（Playground 既定） | 約 42,000 | 170 KB | **内蔵 SRAM に収まる**。演算 2.1 MMAC/s で余裕 |
| int8 量子化 | 約 42,000 | 42 KB | さらに高速 |

→ **学習を実機に載せるなら `network_factory.policy_hidden_layer_sizes` を (128,128,128) にすること**（四脚・ホッパー共通）。
Go1 だけが大きい設定を使っているので、それを引き継がないよう注意。

**通信（検証済み）**

- **XL-320 は Sync Read / Sync Write に対応。Bulk Read / Bulk Write は非対応**
  （根拠: DynamixelSDK の `bulk_read_write.py` 冒頭に "the XL320 does not support Bulk Read and Bulk Write" と明記され、
  機種選択肢からも除外。一方 `sync_read_write.py` には XL320 の選択肢がある）
- **XL-320 の位置データは 2 バイト**（X シリーズは 4 バイト）。SDK の X_SERIES サンプルは流用できない

  | 項目 | アドレス | バイト |
  | --- | --- | --- |
  | Return Delay Time | 5 | 1 |
  | Torque Enable | 24 | 1 |
  | Goal Position | 30 | 2 |
  | Moving Speed | 32 | 2 |
  | Torque Limit | 35 | 2 |
  | Present Position | 37 | 2 |
  | Present Speed | 39 | 2 |

- 1 Mbps・12 個で **Sync Write 約 0.5 ms ＋ Sync Read 約 1.6 ms = 3 ms 以下**。50 Hz（20 ms）に対し余裕あり
- **全サーボの Return Delay Time を 0 にすること**（初期値 250 ＝ 500 µs、12 個で 6 ms を浪費する）

**ハードウェア側の課題**

- **半二重 TTL 回路が必要**: 74LVC1G241 / 74HC126 等の三state バッファ ＋ 方向制御 GPIO。
  XL-320 の信号は 3.3 V なので ESP32-S3 と直結できる。TX・RX・DIR の 3 本で足りる
- **電源 2 系統**: サーボ 7.4 V（2S 直結）、AtomS3R 5 V。12 個同時ストールは理論上 10 A 超なので配線は太く
- Wi-Fi/BLE で観測・アクションをリアルタイムに PC へ送れる → **sim2real のギャップを実測で比較できる**

**学習側で最重要**: 実機で出せない力を使う歩容を学習させないため、`forcerange` をストール値 0.39 N·m に固定する。

### 4.2 BAM（アクチュエータの摩擦モデル）— 未検証、学習を始めるときに調査

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

## 5. 開発の進め方

**設計の順序（ホッパーで確立した型）**

1. **2D の単一 HTML シミュレーターで設計を詰める**。ピッチ面とロール面を分けて、機構寸法・バネ・サーボ機種・制御係数を決める。
   ブラウザで即座に触れることと、Node のハーネス（`tools/hopper_harness.js`）で掃引・回帰できることが効いた
2. **MuJoCo（3D）で連成を確認する**。2D では見えない項目（ヨーのドリフト、面の連成、接触の実際）がここで出る
3. **実機**。センサを真値から実測に置き換え、制御周期と通信を詰める

**守ること**

- シミュレーターを直したら `node tools/hopper_sweeps.js` を走らせ、`hopper-log.md` の数字が再現するか確認する
- サーボ負荷は **5 ms 平均の持続ピーク**で読む。瞬時値は指令段差で必ず 100 % に触れる
- サーボの回転が無負荷回転数を超える設計は**失格**（減速比 152〜272:1 は逆駆動できずギヤが壊れる）
- 2D で決めた制御係数を 3D にそのまま持ち込まない。接地写像を測り直す（`hopper/run.py` の `sweep_gains()`）
- 単位: 長さ mm・角度 deg（Web アプリ・Fusion）、m・rad（MuJoCo / MJX）。変換箇所にコメントを書く
- 推定値には「未検証」と明記する

**Colab の運用**

- コード修正は「セル 1b（`git fetch` → `reset --hard` → `importlib.reload`）」で取り込む。
  ノートブックのセル自体を変えたときだけ GitHub から開き直す
- `pip` はディスク、`import` はメモリ。**install → 再起動 → import** の順を守る
- ノートブック 02（ホッパー）は **CPU ランタイムでよい**。01（四脚の学習）は GPU が要る

## 6. 次の作業（優先順）

**ホッパー（主線）**

1. **センサの現実化**: 速度推定（IMU 積分＋脚オドメトリ）と接地検出（バネ圧縮量）を実測ベースに置き換え、
   真値なしでも制御が成り立つか MuJoCo で確認する。前進ループは速度推定誤差に直接効く
2. **反射慣性の実測**: XM540 / XH540 のロータ慣性を測って `HopperParams.jr` / `jr3` を更新する。
   足先換算 8 kg 相当と支配的なパラメータなので、ここで数値の信頼度が決まる
3. **実機設計**: 直列バネ（900 N/m・ストローク 80 mm）の実装、足パッド（半径 25 mm 前後のゴム）、
   股軸を重心高さに置く胴体、AtomS3R での 200 Hz 電流指令（§4.1）
4. 速度・跳躍の上限を上げるなら機構変更（`hopper-report.md` §6 の案 B ＝ 直列股ピッチ、または蓄勢方式 C）

**四脚（保留、再開するとき）**

- `quadruped.md` §4 を参照

## 7. 参考リンク

- MuJoCo Playground: https://github.com/google-deepmind/mujoco_playground（`learning/train_jax_ppo.py`, `learning/notebooks/locomotion.ipynb`）
- **BAM（Rhoban、同定済み摩擦モデル。XL-320 / XL330 両方あり）**: https://github.com/Rhoban/bam / https://bam.readthedocs.io/
- Microduck RL（ドメインランダム化・sim2real の手本。BAM の利用側）: https://github.com/pollen-robotics/microduck_rl
- XM540 e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
- XL-320 e-Manual: https://emanual.robotis.com/docs/en/dxl/x/xl320/
- XL330 商品ページ（日本）: https://e-shop.robotis.co.jp/product.php?id=417
- 図面 PDF（XM540/XH540、X330、XL-320）はユーザー手元にあり。数値は本書 §3 に転記済み
