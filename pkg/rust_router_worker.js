/* Cloudflare Workers (esbuild + [[rules]] CompiledWasm) 用の手書き wrapper.
 *
 * wasm-pack --target bundler の rust_router.js は
 *   import * as wasm from "./rust_router_bg.wasm";
 * という namespace-import を使う (webpack/vite 向けで、Workers の bundler は
 * 非対応で build エラーになる)。
 *
 * 代わりに Cloudflare の `[[rules]] type = "CompiledWasm"` 経由で wasm を
 * `WebAssembly.Module` として default-import → 手動で Instance 化して、
 * bg.js の `__wbg_set_wasm()` に渡す。
 */

import wasmModule from './rust_router_bg.wasm';
import * as bg from './rust_router_bg.js';

// wasm-bindgen が生成する imports object: bg.js が export している
// __wbg_* / __wbindgen_* 関数群を全部 './rust_router_bg.js' namespace に
// マッピングする必要がある。bg.js 全 export をそのまま渡す。
const instance = new WebAssembly.Instance(wasmModule, {
  './rust_router_bg.js': bg
});

bg.__wbg_set_wasm(instance.exports);
// wasm-pack generates an `__wbindgen_start` exported function for module
// initialization (used by externref / start ctors). Call it explicitly.
if (typeof instance.exports.__wbindgen_start === 'function') {
  instance.exports.__wbindgen_start();
}

export const route_ch = bg.route_ch;
export const astar = bg.astar;
