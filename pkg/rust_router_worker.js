/* Cloudflare Workers (esbuild + [[rules]] CompiledWasm) 用の手書き wrapper.
 *
 * wasm-pack --target bundler の rust_router.js は
 *   import * as wasm from "./rust_router_bg.wasm";
 * という namespace-import を使う (webpack/vite 向けで、Workers の bundler は
 * 非対応で build エラーになる)。
 *
 * 代わりに Cloudflare の `[[rules]] type = "CompiledWasm"` 経由で wasm を
 * `WebAssembly.Module` として default-import → 呼び出し時に lazy で Instance
 * 化して、bg.js の `__wbg_set_wasm()` に渡す。
 *
 * lazy 化の理由: トップレベルで `new WebAssembly.Instance()` が throw すると
 * モジュール評価で Worker 全体が落ち、worker.mjs 側の JS fallback に到達
 * できない。lazy なら各呼び出しで try/catch で受け取れて JS フォール
 * バックに逃げられる (CodeRabbit PR #87 指摘)。
 */

import wasmModule from './rust_router_bg.wasm';
import * as bg from './rust_router_bg.js';

let initialized = false;
let initError = null;

function ensureInitialized() {
  if (initialized) return;
  if (initError) throw initError;
  try {
    // wasm-bindgen が生成する imports object: bg.js が export している
    // __wbg_* / __wbindgen_* 関数群を './rust_router_bg.js' namespace で渡す。
    const instance = new WebAssembly.Instance(wasmModule, {
      './rust_router_bg.js': bg
    });
    bg.__wbg_set_wasm(instance.exports);
    // wasm-pack generates an `__wbindgen_start` exported function for module
    // initialization (used by externref / start ctors). Call it explicitly.
    if (typeof instance.exports.__wbindgen_start === 'function') {
      instance.exports.__wbindgen_start();
    }
    initialized = true;
  } catch (err) {
    initError = err;
    throw err;
  }
}

export function route_ch(...args) {
  ensureInitialized();
  return bg.route_ch(...args);
}

export function astar(...args) {
  ensureInitialized();
  return bg.astar(...args);
}
