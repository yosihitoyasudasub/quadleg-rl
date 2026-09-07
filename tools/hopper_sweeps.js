// HANDOFF §8.2 / §8.5 / §8.6 の数字を再現する掃引スクリプト。
//
//   node tools/hopper_sweeps.js            # 全部
//   node tools/hopper_sweeps.js regress    # モード A の回帰（§8.1 の記録と照合）
//   node tools/hopper_sweeps.js charge     # §8.2 蓄勢方式 C
//   node tools/hopper_sweeps.js ceiling    # §8.5 到達できるホップ上限（E + C）
//   node tools/hopper_sweeps.js forward    # §8.6 前進速度ループ
//
// 数字が変わったら、シミュレーターの物理か制御を変えたということ。
// HANDOFF の該当節も直すこと。

const A = require('./hopper_harness.js');

// §8.1 の標準設定
const BASE = {
  mb: 2.0, bw: 300, bh: 180, hb: 0, mf: 0.05,
  d0: 40, l1: 60, l2: 160, ls0: 80, ks: 1200, zs: 0.2, kg: 20000, mu: 0.8,
  servo: 3, stall: 10.6, nl: 30, jr: 0.02, kp: 120, kd: 2.2,
  rate: 200, hdes: 5, vdes: 0, L0: 240, dlmax: 50, retract: 15,
  kh: 0.5, kv: 0.02, cn: 1.0, kth: 40, kthd: 1,
  hipMode: 1, kr: 20000, dr: 300,
  chgMode: 0, chgPre: 40, chgR: 2.0, chgSv: 0, chgRel: 0, thrustMs: 0,
};

/** 指定パラメータで T 秒ぶん回し、直近数ホップの平均をまとめて返す。 */
function sim(over, T = 12) {
  Object.assign(A.getP(), BASE, over);
  A.setDrop(3);
  A.resetSim();
  const apex = [], vel = [];
  let hops = 0, chg = null, ratio = null, wmax = 0;
  const P = A.getP(), wnl = P.nl * 2 * Math.PI / 60;
  for (let i = 0; i < T * 400 && !A.getS().fallen; i++) {
    A.advance(0.0025);
    const C = A.getC();
    if (C.hops > hops) {
      hops = C.hops;
      if (C.last) { apex.push(C.last.apex); vel.push(C.last.v); wmax = Math.max(wmax, C.last.w / wnl * 100); }
      if (C.lastChg) chg = C.lastChg;
      if (C.lastR) ratio = C.lastR;
    }
  }
  const mean = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN;
  return {
    fallen: A.getS().fallen, hops, chg, ratio, wmax,
    apex: mean(apex.slice(-3)), vel: mean(vel.slice(-4)),
    osc: vel.length >= 6 ? Math.max(...vel.slice(-6)) - Math.min(...vel.slice(-6)) : NaN,
    c: A.getC().last,
    tau: A.getC().last ? A.getC().last.tau / P.stall * 100 : NaN,
    w: A.getC().last ? A.getC().last.w / wnl * 100 : NaN,
  };
}

/** 目標ホップ高を上げていき、追従しなくなる手前＝到達上限を返す。
 *  noBackdrive=true なら ω>105 %（サーボの逆駆動＝ギヤ破損）の設定を棄却する。 */
function ceiling(over, noBackdrive = true) {
  let best = 0, rec = null;
  for (const hdes of [3, 5, 8, 10, 12, 15, 18, 22, 26, 30]) {
    const r = sim({...over, hdes});
    if (r.fallen || !(r.apex > hdes / 100 * 0.85)) continue;
    if (noBackdrive && r.wmax > 105) continue;
    if (r.apex * 100 > best) { best = r.apex * 100; rec = r; }
  }
  return {cm: best, rec};
}

const pad = (s, n) => String(s).padEnd(n);
const num = (v, n, d = 1) => (isNaN(v) ? '—' : v.toFixed(d)).padStart(n);

// ---------------------------------------------------------------- §8.1 回帰
function regress() {
  console.log('=== モード A の回帰（HANDOFF §8.1 の記録と照合）===');
  const cases = [
    ['標準（その場ホップ）', {}],
    ['制御周期 50 Hz', {rate: 50}], ['制御周期 100 Hz', {rate: 100}], ['制御周期 500 Hz', {rate: 500}],
    ['股軸 50mm 下', {hb: 50}], ['足質量 0.15kg', {mf: 0.15}],
    ['位置指令のみ', {hipMode: 0}],
    ['前進 0.1 m/s', {vdes: 0.1}], ['前進 0.2 m/s', {vdes: 0.2}],
  ];
  for (const [lab, ov] of cases) {
    const r = sim(ov, 14);
    console.log(`  ${pad(lab, 22)} ${r.fallen ? '転倒' : 'ok  '} 頂点 ${num(r.apex * 100, 5)} cm  hops=${r.hops}`);
  }
  console.log('  ※ 標準は 5.0 cm / Ts 175 ms / τ100 % / ω98 % が期待値');
}

// ---------------------------------------------------------------- §8.2 蓄勢 C
function charge() {
  console.log('\n=== §8.2 蓄勢方式 C（飛行中蓄勢＋ラッチ）===');
  console.log(`  ${pad('条件', 30)} 頂点     τ    ω   蓄勢`);
  const rows = [
    ['A 現行（着地エネルギー）', {}],
    ['C XL-320 巻取り rc=10 δ40', {chgMode: 1, chgPre: 40, chgR: 10, chgSv: 0}],
    ['C XM540 巻取り rc=40 δ40', {chgMode: 1, chgPre: 40, chgR: 40, chgSv: 3}],
    ['C XM540 巻取り rc=60 δ40', {chgMode: 1, chgPre: 40, chgR: 60, chgSv: 3}],
  ];
  for (const [lab, ov] of rows) {
    const r = sim(ov, 14), g = r.chg;
    console.log(`  ${pad(lab, 30)}${num(r.apex * 100, 5)}cm ${num(r.tau, 4, 0)}% ${num(r.w, 4, 0)}%` +
      (g ? `  ${(g.pre * 1000).toFixed(0)}/${A.getP().chgPre}mm ${g.E.toFixed(2)}J ${g.full ? 'OK' : 'NG'}` : ''));
  }
  console.log('\n  蓄勢モータのピーク機械出力 stall·ω₀/4 と、必要パワー ½ks·δ²/飛行時間 の比較:');
  for (const s of A.SERVOS) {
    const w0 = s.nl * 2 * Math.PI / 60;
    console.log(`    ${pad(s.name, 20)} ${(s.stall * w0 / 4).toFixed(2)} W`);
  }
  const E = 0.5 * BASE.ks * 0.04 * 0.04;
  console.log(`    δpre=40mm の蓄勢 ${E.toFixed(2)} J → 飛行 200 ms なら ${(E / 0.2).toFixed(2)} W が要る`);
}

// ------------------------------------------------------- §8.5 到達上限（E + C）
function ceilings() {
  console.log('\n=== §8.5 到達できるホップ上限（ω≤105 % ＝ 逆駆動しない範囲）===');
  const rows = [
    ['A l1=60 l2=160（現行）', {}],
    ['A l1=80 l2=140（E のみ）', {l1: 80, l2: 140}],
    ['C l1=60 + XM540 rc=60 δ40', {chgMode: 1, chgPre: 40, chgR: 60, chgSv: 3}],
    ['C l1=80 + XM540 rc=60 δ40', {l1: 80, l2: 140, chgMode: 1, chgPre: 40, chgR: 60, chgSv: 3}],
    ['C l1=80 + XM540 rc=100 δ60', {l1: 80, l2: 140, chgMode: 1, chgPre: 60, chgR: 100, chgSv: 3}],
    ['C l1=80 + XL-320 rc=15 δ20', {l1: 80, l2: 140, chgMode: 1, chgPre: 20, chgR: 15, chgSv: 0}],
  ];
  for (const [lab, ov] of rows) {
    const {cm, rec} = ceiling(ov);
    console.log(`  ${pad(lab, 30)} 上限 ${num(cm, 5)} cm  ω=${rec ? num(rec.wmax, 4, 0) + '%' : '  —'}`);
  }
  // 目標高の刻みと評価時間の取り方で ±0.3 cm ほど動く。傾向（A < E < C < E+C）が保たれていればよい。
  console.log('  期待値: 約 5 / 10 / 16 / 17.6 / 19 / 13.7 cm（±0.3 cm）');
}

// ------------------------------------------------------- §8.6 前進速度ループ
function forward() {
  console.log('\n=== §8.6 前進速度ループ（調整では直らないことの確認）===');
  const VL = [0, 0.1, 0.2, 0.3, 0.4];
  const configs = [
    ['l1=60 現行 目標5cm', {hdes: 5}],
    ['l1=80 素 目標8cm', {l1: 80, l2: 140, hdes: 8}],
    ['l1=80 +C 目標15cm', {l1: 80, l2: 140, chgMode: 1, chgPre: 40, chgR: 60, chgSv: 3, hdes: 15}],
  ];
  for (const [lab, ov] of configs) {
    let best = null;
    for (const cn of [0.85, 0.90, 0.95, 1.00]) {
      for (const kv of [0.01, 0.02, 0.03]) {
        let err = 0, fell = 0, det = [];
        for (const vdes of VL) {
          const r = sim({...ov, cn, kv, vdes});
          if (r.fallen) { err += 1; fell++; det.push('  X '); }
          else { err += Math.abs(r.vel - vdes); det.push(r.vel.toFixed(2)); }
        }
        if (!best || err < best.err) best = {err, fell, cn, kv, det: det.join(' ')};
      }
    }
    console.log(`  ${pad(lab, 22)} 最良 cn=${best.cn} Kv=${best.kv}`);
    console.log(`  ${pad('', 22)} 指令 0/0.1/0.2/0.3/0.4 → ${best.det}  転倒 ${best.fell}/${VL.length}`);
  }
  console.log('\n  蹴り出し打ち切り（thrustMs）は逆効果 — 1 接地あたりの速度増幅率:');
  const OV = {l1: 80, l2: 140, chgMode: 1, chgPre: 40, chgR: 60, chgSv: 3, hdes: 15, cn: 1.0, kv: 0.01, vdes: 0.2};
  for (const thrustMs of [0, 40, 80]) {
    Object.assign(A.getP(), BASE, OV, {thrustMs});
    A.setDrop(3); A.resetSim();
    let prev = 'FLIGHT', vin = null, pairs = [];
    for (let i = 0; i < 10 * 400 && !A.getS().fallen; i++) {
      A.advance(0.0025);
      const ph = A.getC().phase, vx = A.getS().vx;
      if (prev === 'FLIGHT' && ph !== 'FLIGHT') vin = vx;
      if (prev !== 'FLIGHT' && ph === 'FLIGHT' && vin !== null) pairs.push([vin, vx]);
      prev = ph;
    }
    const use = pairs.slice(-6).filter(p => Math.abs(p[0]) > 0.005);
    const g = use.length ? use.reduce((a, p) => a + p[1] / p[0], 0) / use.length : NaN;
    console.log(`    thrustMs=${pad(thrustMs === 0 ? '無制限' : thrustMs + 'ms', 8)} 増幅率 ${num(g, 5, 2)} 倍`);
  }
  console.log('    期待値: 無制限 1.16 倍 / 打ち切ると 3.56 倍（周期 2 の発散）');
}

const which = process.argv[2];
const all = !which;
if (all || which === 'regress') regress();
if (all || which === 'charge') charge();
if (all || which === 'ceiling') ceilings();
if (all || which === 'forward') forward();
