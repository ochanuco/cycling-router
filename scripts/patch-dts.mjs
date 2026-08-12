// wasm-pack が生成する router_wasm.d.ts の route_ch を、手書きの正確な型へ差し替える。
//
// route_ch は serde_wasm_bindgen 経由で JsValue を返すため、wasm-bindgen は
// 戻り値の形を知らず `buffers: Array<any>` / 戻り値 `any` になる。全文を
// 上書きすると将来 wasm-pack が新しい export を足したときに静かに消えるので、
// 該当シグネチャだけを正規表現で置換する。marker で冪等にしている。

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const MARKER = '/* patched-by: scripts/patch-dts.mjs */';

const IMPORT_LINE =
  "import type { RouteChResult } from './router_wasm_worker';";

// bundler ターゲットだけが Workers wrapper (と手書き d.ts) を伴う。
const TARGET = path.join(ROOT, 'dist', 'bundler', 'router_wasm.d.ts');

const BUFFERS_ANY = /(\bfunction route_ch\(buffers:\s*)Array<any>/;
const RETURN_ANY = /(\bexport function route_ch\([^)]*\)\s*:\s*)any(\s*;)/;

if (!fs.existsSync(TARGET)) {
  process.stderr.write(`patch-dts: ${TARGET} が無い。先に build-wasm.sh を実行すること\n`);
  process.exit(1);
}

let src = fs.readFileSync(TARGET, 'utf8');
if (src.startsWith(MARKER)) {
  process.stdout.write('patch-dts: already patched; skip\n');
  process.exit(0);
}

const before = src;
src = src.replace(BUFFERS_ANY, '$1Uint8Array[]');
const buffersPatched = src !== before;

const beforeReturn = src;
src = src.replace(RETURN_ANY, '$1RouteChResult$2');
const returnPatched = src !== beforeReturn;

if (!buffersPatched || !returnPatched) {
  process.stderr.write(
    `patch-dts: route_ch のシグネチャが想定と違う (buffers=${buffersPatched}, return=${returnPatched})。\n` +
      'wasm-pack の出力形式が変わった可能性がある。scripts/patch-dts.mjs の正規表現を見直すこと。\n'
  );
  process.exit(1);
}

src = `${MARKER}\n${IMPORT_LINE}\n${src}`;
fs.writeFileSync(TARGET, src);
process.stdout.write('patch-dts: route_ch を RouteChResult に差し替えた\n');
