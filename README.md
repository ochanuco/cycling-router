# cycling-router

自転車ルーティングのコア実装。CSR グラフ構築、Contraction Hierarchies (CH) クエリ、座標スナップ、ルートと補給地点の距離計算を Rust で提供する。

[ride-oasis](https://github.com/ochanuco/ride-oasis) から切り出したもので、WASM として Cloudflare Workers 上で動くことを主眼に置いている。

## クレート構成

| クレート | 役割 | 依存 |
|---|---|---|
| `router-core` | アルゴリズム本体。CSR / CH / snap / route_filter / A* | なし（wasm32 ターゲット時のみ `js-sys`） |
| `router-wasm` | wasm-bindgen adapter。JS 境界の型変換だけを担う | `router-core`, `wasm-bindgen`, `js-sys` |
| `router-cli` | CH 前計算バイナリ `ch-preprocess` | なし |

`router-core` は JS 境界を持たない。唯一 `js-sys` に触れるのは、wasm32 で `std::time::SystemTime::now()` が使えないために CH クエリの time budget 判定へ `js_sys::Date` を使う箇所（`chquery.rs` の `now_ms`）で、これは target-specific dependency なのでネイティブビルドでは一切引かれない。

### route_ch がコアではなく adapter にある理由

`route_ch` は `Uint8Array` を受け取り `JsValue` を返す、まさに JS 境界の関数で、中身は core の部品（CSR 構築 → snap → CH クエリ → shortcut 展開）を順に呼ぶオーケストレーションでしかない。そのため `router-wasm` に置いている。ネイティブからルート計算を呼ぶ必要が出た時点で、この手続きを `router-core` に引き上げる。

## ビルド

WASM（3 ターゲットを `dist/` へ出力し、Workers 用 wrapper を同梱する）:

```bash
./scripts/build-wasm.sh
```

| 出力 | 用途 |
|---|---|
| `dist/bundler/` | Cloudflare Workers。`router_wasm_worker.js` 経由で使う |
| `dist/nodejs/` | Node からの検証・ベンチ |
| `dist/web/` | ブラウザ |

ネイティブ:

```bash
cargo test --workspace
cargo build -p router-cli --release   # → target/release/ch-preprocess
```

## Cloudflare Workers から使う

wasm-pack の `--target bundler` が生成する `router_wasm.js` は `import * as wasm from "./router_wasm_bg.wasm"` という namespace-import を使い、Workers の bundler が受け付けない。このため手書きの wrapper (`crates/router-wasm/js/worker.js` → `dist/bundler/router_wasm_worker.js`) を挟み、`[[rules]] type = "CompiledWasm"` 経由で得た `WebAssembly.Module` を遅延インスタンス化する。

遅延にしているのは、トップレベルで `new WebAssembly.Instance()` が throw すると Worker 全体が落ちて呼び出し側の JS フォールバックに到達できないため。

```js
import { route_ch, route_distances } from "./router_wasm_worker.js";
```

## 配布

tag (`v*`) を push すると CI が 3 ターゲット分をビルドし、`cycling-router-wasm-<tag>.tar.gz` を GitHub Release に添付する。ビルド成果物は git に含めない。

## ライセンス

MIT
