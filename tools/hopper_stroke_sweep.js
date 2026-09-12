// 直列バネの必要ストロークとばね定数を決める掃引（docs/hopper-mechanical-spec.md §7）。
//
//   node tools/hopper_stroke_sweep.js           # 全部
//   node tools/hopper_stroke_sweep.js stroke    # 現行 ks 900 での必要ストローク
//   node tools/hopper_stroke_sweep.js ks        # ばね定数の掃引
//   node tools/hopper_stroke_sweep.js compare   # 現行 vs 採用案の条件別比較
//
// 背景: 実在の圧縮コイルばねは「自由長 ≒ 密着長 ＋ ストローク」なので、ストロークを
// 80 mm 取ると自由長が 117 mm になり、脚（L0 240 mm）に収まらない。ストロークを削れるか、
// ばね定数を上げて縮み量そのものを減らせるかを測る。
//
// 結論（2026-09-13）: ストロークは削れない（60 mm では足りない）。
// ks 900 → 1500 N/m に上げると δ が 71 → 49 mm に減り、自由長 91 mm で L0 240 mm に収まる。
// 詳細は docs/hopper-log.md §8.11。

const A = require('./hopper_harness.js');
const DEF = A.DEF;

/** 現行の既定（2D シミュレーターの DEF そのまま） */
const CURRENT = {ks: 900, ls0: 80, L0: 240, cn: 0.5};
/** 採用案: ばねを硬くして縮みを減らす。Ln = 240 − 15 − 91 = 134 mm */
const PLAN = {ks: 1500, ls0: 91, L0: 240, cn: 0.6};

/** 指定条件で T 秒回して指標を返す。ls0 は底付き限界も兼ねるので、
 *  「本当に必要な縮み量」を測るときは ls0 を十分大きくすること。 */
function sim(over, drop = 3, T = 18) {
  Object.assign(A.getP(), DEF, over);
  A.setDrop(drop);
  A.resetSim();
  const P = A.getP(), wnl = P.nl * 2 * Math.PI / 60;
  const rows = [];
  let hops = 0;
  for (let i = 0; i < T * 400 && !A.getS().fallen; i++) {
    A.advance(0.0025);
    const C = A.getC();
    if (C.hops > hops) { hops = C.hops; if (C.last) rows.push(C.last); }
  }
  const tail = rows.slice(-5);
  const mean = f => tail.length ? tail.reduce((s, r) => s + f(r), 0) / tail.length : NaN;
  const mx = f => rows.length ? Math.max(...rows.map(f)) : NaN;
  return {
    fallen: A.getS().fallen, hops,
    d: mx(r => r.delta) * 1000,            // バネ圧縮ピーク [mm]
    dSteady: mean(r => r.delta) * 1000,
    apex: mean(r => r.apex) * 100,
    v: mean(r => r.v),
    Ts: mean(r => r.Ts) * 1000,
    Fn: mx(r => r.Fn),
    tau: mx(r => r.tauS) / P.stall * 100,
    w: mx(r => r.wS) / wnl * 100,
    bottom: rows.some(r => r.delta >= P.ls0 / 1000 - 1e-4),
  };
}

const f = (x, n = 2) => Number.isFinite(x) ? x.toFixed(n) : ' -- ';

/** 全条件のリスト。[ラベル, 上書き, 落下高さ cm] */
const CASES = [
  ['その場 hdes 3', {hdes: 3}, 3],
  ['その場 hdes 5', {hdes: 5}, 3],
  ['その場 hdes 7', {hdes: 7}, 3],
  ['その場 hdes 9', {hdes: 9}, 3],
  ['前進 0.1 m/s', {vdes: 0.1}, 3],
  ['前進 0.2 m/s', {vdes: 0.2}, 3],
  ['前進 0.3 m/s', {vdes: 0.3}, 3],
  ['落下 6 cm', {}, 6],
  ['落下 10 cm', {}, 10],
  ['落下 15 cm', {}, 15],
  ['足 mf 40 g', {mf: 0.04}, 3],
  ['足 mf 90 g', {mf: 0.09}, 3],
];

function block(name, base) {
  console.log('\n=== %s（Ln = %d mm）===', name, base.L0 - 15 - base.ls0);
  for (const [lab, over, drop] of CASES) {
    const r = sim({...base, ...over}, drop);
    console.log('  %s | δ %s mm  頂点 %s cm  v %s | Ts %s ms  Fn %s N  τ %s%%  ω %s%% | %s',
      lab.padEnd(14), f(r.d, 1).padStart(5), f(r.apex, 1).padStart(4), f(r.v).padStart(5),
      f(r.Ts, 0).padStart(3), f(r.Fn, 0).padStart(3), f(r.tau, 0).padStart(3), f(r.w, 0).padStart(3),
      r.fallen ? '転倒' : (r.bottom ? '底付き' : `${r.hops} hops`));
  }
}

/** ストロークを削れるか。底付きさせない ls0 で「本当に必要な縮み量」を測る。 */
function stroke() {
  console.log('\n=== 必要ストローク（ks 900、底付きさせない ls0 140 で測る）===');
  let worst = 0, lab = '';
  for (const [l, over, drop] of CASES) {
    const r = sim({ks: 900, ls0: 140, L0: 300, cn: 0.5, ...over}, drop);
    if (r.d > worst) { worst = r.d; lab = l; }
    console.log('  %s | δ %s mm', l.padEnd(14), f(r.d, 1).padStart(5));
  }
  console.log('  → 最大 %s mm（%s）。60 mm のストッパでは %s',
    f(worst, 1), lab, worst > 60 ? '足りない' : '足りる');
}

/** ばね定数を上げて縮み量を減らせるか。 */
function ks() {
  console.log('\n=== ばね定数の掃引（mb 1.5 kg、底付きさせない ls0 140 / L0 300）===');
  console.log('  k[N/m] | 静沈 | δ hdes5  hdes9  落下10  落下15 | 前進 0.2 の τ | Ts | 頂点追従');
  for (const k of [900, 1200, 1500, 1800, 2400, 3000]) {
    const b = {ks: k, ls0: 140, L0: 300, cn: 0.6};
    const r5 = sim(b), r9 = sim({...b, hdes: 9});
    const d10 = sim(b, 10), d15 = sim(b, 15), rv = sim({...b, vdes: 0.2});
    console.log('  %s | %s | %s  %s  %s  %s | %s%% | %s ms | %s',
      String(k).padStart(6), f(1.5 * 9.81 / k * 1000, 1).padStart(4) + ' mm',
      f(r5.d, 1).padStart(5), f(r9.d, 1).padStart(5), f(d10.d, 1).padStart(5), f(d15.d, 1).padStart(5),
      f(rv.tau, 0).padStart(3), f(r5.Ts, 0).padStart(3),
      r5.fallen ? '転倒' : (Math.abs(r9.apex - 9) < 0.6 ? 'OK' : `hdes9 で ${f(r9.apex, 1)} cm`));
  }
}

function compare() {
  block('現行  ks 900 / 自由長 80 / L0 240', CURRENT);
  block('採用案 ks 1500 / 自由長 91 / L0 240', PLAN);
}

const which = process.argv[2];
if (!which || which === 'stroke') stroke();
if (!which || which === 'ks') ks();
if (!which || which === 'compare') compare();
