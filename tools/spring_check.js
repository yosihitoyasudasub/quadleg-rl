// 候補のばね（カタログ品でも特注でも）が成立するかを 2D シミュレーターで判定する。
//
//   node tools/spring_check.js --k 1.5 --lf 91 --od 15 --dmax 55
//   node tools/spring_check.js --k 1.5 --lf 122 --od 15 --ratio 0.45
//
//   --k     ばね定数 [N/mm]        （必須）
//   --lf    自由長 [mm]            （必須）
//   --od    外径 [mm]              （任意。ガイド管 内径 16.1 に入るか見る）
//   --dmax  使用可能な最大たわみ [mm]（--ratio と どちらか）
//   --ratio 許容たわみ率 0〜1       （例: ミスミ WF は 0.45）
//   --l0    脚長 [mm]（既定 240）   L0 を保つ前提で先端長 Ln = L0 − 15 − Lf を出す
//
// 判定基準（docs/hopper-mechanical-spec.md §6 / §7）
//   1. 外径 ≦ 16.0 mm            ガイド管（内径 16.1）に入ること
//   2. 先端長 Ln ≧ 125 mm        Ln 114 では後ろ振りでクランクが水平を越えてサーボと干渉
//   3. δ(落下 10 cm) ≦ 使用可能たわみ × 0.9   常用で底付きしないこと
//   4. 目標ホップ高 9 cm に届く   蹴り出しのトルクが足りること
//   5. 持続トルク ≦ 90 %、速度 ≦ 105 %        サーボが壊れないこと（速度超過は失格）

const A = require('./hopper_harness.js');
const DEF = A.DEF;

// ---------- 引数 ----------
const argv = process.argv.slice(2);
const arg = (name, def) => {
  const i = argv.indexOf('--' + name);
  return i >= 0 && argv[i + 1] !== undefined ? Number(argv[i + 1]) : def;
};
const K = arg('k'), LF = arg('lf'), OD = arg('od', NaN);
const RATIO = arg('ratio', NaN);
const L0 = arg('l0', 240);
const DMAX = arg('dmax', Number.isFinite(RATIO) ? RATIO * LF : NaN);

if (!Number.isFinite(K) || !Number.isFinite(LF)) {
  console.error('使い方: node tools/spring_check.js --k <N/mm> --lf <mm> [--od <mm>] '
    + '[--dmax <mm> | --ratio <0-1>] [--l0 <mm>]');
  process.exit(1);
}

// ---------- シミュレーション ----------
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
    fallen: A.getS().fallen,
    d: mx(r => r.delta) * 1000,
    apex: mean(r => r.apex) * 100,
    v: mean(r => r.v),
    Ts: mean(r => r.Ts) * 1000,
    tau: mx(r => r.tauS) / P.stall * 100,
    w: mx(r => r.wS) / wnl * 100,
  };
}

// 底付きの影響を除いて「素の必要たわみ」を測るため、ls0 は十分大きくして回す。
// 幾何（先端長 Ln）は判定したい条件に合わせる。
const Ln = L0 - 15 - LF;
const BASE = {ks: K * 1000, ls0: 200, L0: Ln + 15 + 200, cn: 0.6};

const f = (x, n = 1) => Number.isFinite(x) ? x.toFixed(n) : ' -- ';

console.log('\n── 候補 ──');
console.log('  ばね定数 %s N/mm   自由長 %s mm   外径 %s mm', f(K, 2), f(LF, 0),
  Number.isFinite(OD) ? f(OD, 1) : '未指定');
console.log('  使用可能たわみ %s mm%s', Number.isFinite(DMAX) ? f(DMAX, 1) : '未指定',
  Number.isFinite(RATIO) ? `（たわみ率 ${(RATIO * 100).toFixed(0)} %）` : '');
console.log('  L0 = %s mm を保つと 先端長 Ln = %s mm', f(L0, 0), f(Ln, 0));

console.log('\n── 動作 ──');
const CASES = [
  ['その場 hdes 3', {hdes: 3}, 3], ['その場 hdes 5', {hdes: 5}, 3],
  ['その場 hdes 7', {hdes: 7}, 3], ['その場 hdes 9', {hdes: 9}, 3],
  ['前進 0.1 m/s', {vdes: 0.1}, 3], ['前進 0.2 m/s', {vdes: 0.2}, 3],
  ['前進 0.3 m/s', {vdes: 0.3}, 3],
  ['落下 6 cm', {}, 6], ['落下 10 cm', {}, 10], ['落下 15 cm', {}, 15],
];
const R = {};
for (const [lab, over, drop] of CASES) {
  const r = sim({...BASE, ...over}, drop);
  R[lab] = r;
  const over_d = Number.isFinite(DMAX) && r.d > DMAX;
  console.log('  %s | δ %s mm%s  頂点 %s cm  v %s | τ %s%%  ω %s%% | %s',
    lab.padEnd(14), f(r.d).padStart(5), over_d ? ' ×' : '  ',
    f(r.apex).padStart(4), f(r.v, 2).padStart(5),
    f(r.tau, 0).padStart(3), f(r.w, 0).padStart(3),
    r.fallen ? '転倒' : 'OK');
}

// ---------- 判定 ----------
console.log('\n── 判定 ──');
const ng = [];
const ok = (cond, msg, detail) => {
  console.log('  %s %s%s', cond ? '○' : '×', msg, detail ? '  … ' + detail : '');
  if (!cond) ng.push(msg);
};

ok(!Number.isFinite(OD) || OD <= 16.0, '外径 ≦ 16.0 mm（ガイド管に入る）',
  Number.isFinite(OD) ? `外径 ${f(OD, 1)}` : '外径 未指定のため判定省略');
ok(Ln >= 125, '先端長 Ln ≧ 125 mm（クランクがサーボ本体と干渉しない）',
  `Ln = ${f(Ln, 0)} mm`);
const dDesign = Math.max(R['落下 10 cm'].d, R['その場 hdes 9'].d);
ok(!Number.isFinite(DMAX) || dDesign <= DMAX * 0.9,
  '常用で底付きしない（δ ≦ 使用可能たわみ × 0.9）',
  `必要 ${f(dDesign)} mm / 使用可能 ${Number.isFinite(DMAX) ? f(DMAX) : '未指定'} mm`);
ok(!R['その場 hdes 9'].fallen && R['その場 hdes 9'].apex >= 8.4,
  '目標ホップ高 9 cm に届く', `${f(R['その場 hdes 9'].apex)} cm`);
const tauMax = Math.max(...Object.values(R).map(r => r.tau).filter(Number.isFinite));
const wMax = Math.max(...Object.values(R).map(r => r.w).filter(Number.isFinite));
ok(tauMax <= 90, '持続トルク ≦ 90 %', `最大 ${f(tauMax, 0)} %`);
ok(wMax <= 105, '速度 ≦ 105 %（超過は失格：減速比 152〜272:1 は逆駆動できない）',
  `最大 ${f(wMax, 0)} %`);
ok(!CASES.some(([l]) => R[l].fallen), '全条件で転倒しない');

console.log('\n  %s', ng.length === 0 ? '★ 採用可' : `不可（${ng.length} 項目）`);
if (ng.length) for (const m of ng) console.log('    - ' + m);
if (Number.isFinite(DMAX) && dDesign > DMAX * 0.9) {
  console.log('    → 必要な自由長は 約 %s mm（たわみ率 %s %% のとき）',
    Number.isFinite(RATIO) ? f(dDesign / 0.9 / RATIO, 0) : '—',
    Number.isFinite(RATIO) ? (RATIO * 100).toFixed(0) : '—');
}
console.log('');
