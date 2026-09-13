// バネ軸の向きの検討（docs/hopper-log.md §8.12、機構設計要件 §7-7）。
//
//   node tools/hopper_spring_axis.js
//
// A. 現行（バネ軸 = 股→先端。ロッドを股ブッシュで受ける方式）
// B. 案 2（バネ軸 = 下腿 2 本の二等分線。ロッド無し、足先ブロックの向きを菱形リンクで決める）
// C. バネ軸を股→先端から固定角だけずらしたときの感度 → 「向きの精度」要求を決める
//
// 2D シミュレーターの P.sprAx / P.sprOff を使う（既定 0 / 0 で従来どおり）。

const A = require('./hopper_harness.js'); const DEF=A.DEF;
function sim(over, drop=3, T=18){
  Object.assign(A.getP(),DEF,over); A.setDrop(drop); A.resetSim();
  const P=A.getP(), wnl=P.nl*2*Math.PI/60; const rows=[]; let hops=0, thMax=0;
  for(let i=0;i<T*400&&!A.getS().fallen;i++){A.advance(0.0025);const C=A.getC();
    thMax=Math.max(thMax,Math.abs(A.getS().th));
    if(C.hops>hops){hops=C.hops;if(C.last)rows.push(C.last);}}
  const tail=rows.slice(-5),mean=f=>tail.length?tail.reduce((s,r)=>s+f(r),0)/tail.length:NaN;
  const mx=f=>rows.length?Math.max(...rows.map(f)):NaN;
  return {fallen:A.getS().fallen,hops,d:mx(r=>r.delta)*1000,apex:mean(r=>r.apex)*100,
          v:mean(r=>r.v),tau:mx(r=>r.tauS)/P.stall*100,w:mx(r=>r.wS)/wnl*100,
          thMax:thMax*180/Math.PI};
}
const f=(x,n=1)=>Number.isFinite(x)?x.toFixed(n):' -- ';
const BASE={ks:1560,ls0:100,L0:260,cn:0.6};
const CASES=[['その場 hdes 5',{hdes:5},3],['その場 hdes 9',{hdes:9},3],
  ['前進 0.1',{vdes:0.1},3],['前進 0.2',{vdes:0.2},3],['前進 0.3',{vdes:0.3},3],
  ['落下 10 cm',{},10],['落下 15 cm',{},15]];
function block(name,extra){
  console.log('\n=== %s ===',name);
  for(const [lab,over,drop] of CASES){
    const r=sim({...BASE,...extra,...over},drop);
    console.log('  %s | δ %s  頂点 %s cm  v %s | ピッチ最大 %s° | τ %s%%  ω %s%% | %s',
      lab.padEnd(12),f(r.d).padStart(5),f(r.apex).padStart(4),f(r.v,2).padStart(5),
      f(r.thMax).padStart(5),f(r.tau,0).padStart(3),f(r.w,0).padStart(3),r.fallen?'転倒':'OK');
  }
}
block('A. 現行: バネ軸 = 股→先端（ロッド方式）',{sprAx:0});
block('B. 案 2: バネ軸 = 下腿の二等分線（ロッド無し・菱形リンクで向き決め）',{sprAx:1});
console.log('\n=== C. バネ軸の固定ずれに対する感度（股→先端 から sprOff° 回す）===');
console.log('  どこまでの向き誤差なら成立するか。案 2〜5 の「向きの精度」要求を決める数字');
for(const off of [0,1,2,3,5,8,12,20]){
  const r0=sim({...BASE,sprAx:0,sprOff:off}), rv=sim({...BASE,sprAx:0,sprOff:off,vdes:0.2}), rd=sim({...BASE,sprAx:0,sprOff:off},10);
  console.log('  %s° | その場: 頂点 %s cm ピッチ %s° %s | 前進0.2: v %s ピッチ %s° %s | 落下10: ピッチ %s° %s',
    String(off).padStart(2), f(r0.apex).padStart(4), f(r0.thMax).padStart(5), r0.fallen?'転倒':'OK ',
    f(rv.v,2).padStart(5), f(rv.thMax).padStart(5), rv.fallen?'転倒':'OK ',
    f(rd.thMax).padStart(5), rd.fallen?'転倒':'OK ');
}
