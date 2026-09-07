// hopper-2d.html をブラウザなしで動かすためのハーネス。
//
// HTML の <script> をそのまま取り出し、最小限の DOM スタブを与えて実行する。
// シミュレーターのロジックを二重に持たないので、hopper-2d.html を直せば
// こちらも自動で追従する（コピーではなく本体を読んでいる）。
//
//   const A = require('./hopper_harness.js');
//   Object.assign(A.getP(), {l1:80, l2:140, hdes:8});   // パラメータを書き換え
//   A.setDrop(3); A.resetSim();
//   for (let i = 0; i < 12*400; i++) A.advance(0.0025); // 実時間 12 秒ぶん
//   console.log(A.getC().last);                         // 直近サイクルの結果
//
// 使用例は tools/hopper_sweeps.js（HANDOFF §8.2 / §8.5 / §8.6 の数字を再現する）。

const fs = require('fs');
const path = require('path');

const HTML = path.join(__dirname, 'webapp', 'hopper-2d.html');

function build(htmlPath = HTML) {
  const html = fs.readFileSync(htmlPath, 'utf8');
  const m = html.match(/<script>\r?\n([\s\S]*)<\/script>/);   // CRLF チェックアウトでも動くように
  if (!m) throw new Error('hopper-2d.html から <script> を取り出せませんでした');

  // IIFE の殻を外し、内部の状態と関数を返させる
  let js = m[1].trim().replace(/^\(\(\)=>\{/, '').replace(/\}\)\(\);$/, '');
  js += `
    return {getP:()=>P, getS:()=>S, getC:()=>C,
            resetSim, advance, exportText, kin, ratioPeak, ratioNow,
            setDrop:v=>{dropCm=v}, SERVOS, DEF};`;

  const noop = () => {};
  // canvas の 2D コンテキストは全メソッドを no-op にする
  const ctx = new Proxy({}, {get: (t, k) =>
    k === 'canvas' ? {width: 800, height: 400} :
    k === 'measureText' ? () => ({width: 10}) : noop});
  const el = () => ({
    textContent: '', innerHTML: '', value: '', style: {}, className: '', dataset: {},
    getContext: () => ctx,
    getBoundingClientRect: () => ({width: 800, height: 400}),
    addEventListener: noop, querySelector: () => null, querySelectorAll: () => [],
    set onclick(v) {}, set onchange(v) {},
  });
  const document = {querySelectorAll: () => [], getElementById: el, documentElement: {}};
  const window = {addEventListener: noop, devicePixelRatio: 1};

  return new Function(
    'document', 'window', 'requestAnimationFrame', 'getComputedStyle',
    'localStorage', 'navigator', 'console', js
  )(
    document, window, noop, () => ({getPropertyValue: () => '#000'}),
    {getItem: () => null, setItem: noop},          // localStorage は毎回まっさら
    {clipboard: {writeText: async () => {}}}, console
  );
}

module.exports = build();
module.exports.build = build;
