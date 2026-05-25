# rust-router

Cycling router compiled to wasm32 + native CH preprocessor binary.

- `astar()` — forward A* (PoC, 旧)
- `route_ch()` — DNF 用 cold-path: tile decode → CSR build → snap → CH
  query → shortcut unpack を WASM 内で一気通貫 (本番稼働中)

## Build

`npm run wasm:build` がリポジトリ root から両 target を一度に出す:

```bash
# from repo root
npm run wasm:build
```

内部的に:
```bash
wasm-pack build rust-router --release --target bundler --out-dir pkg
wasm-pack build rust-router --release --target nodejs  --out-dir pkg-node
```

- `pkg/` — Workers (Cloudflare bundler 用)。worker.mjs から
  `./rust-router/pkg/rust_router_worker.js` 経由で import される
- `pkg-node/` — Node.js (ローカル bench `scripts/cycling_wasm_bench.js` 用)

## Benchmark

Tiles を `data/cycling/tiles/` に展開した状態で:

```bash
node --expose-gc scripts/cycling_wasm_bench.js \
  --from 135.49,34.69 --to 135.52,34.71 --iters 3
```

ローカル 3km route で chQuery 2-6ms / CSR build 138-183ms / 合計 ~150ms。
JS CSR 比 chQuery 5x、CSR build 2.5x、合計 ~2x 速い。

## Workers integration

本番 worker.mjs は `rust_router_worker.js` (手書き wrapper) を import し、
WASM 失敗時は JS CSR-only path (`lib/cycling/tiled_router.js` の csrOnly mode)
に自動 fallback する。wrapper は lazy 初期化で例外を握って fallback 可能に
してある (CodeRabbit PR #87 指摘対応)。

`wrangler.toml` の `[[rules]] CompiledWasm` で .wasm を WebAssembly.Module
として bundle する。

## Status

- [x] forward A* (`astar`) — PoC
- [x] CSR + chQuery + snap (`route_ch`) — production
- [x] cargo unit tests (10 cases)
- [x] Workers integration via `rust_router_worker.js`
- [x] 本番投入 (PR #87)、`alg=ch-wasm` 観測中

## 旧 PoC benchmark (synthetic grids)

| grid    | JS aStarOnView | WASM astar | speedup |
|---------|---------------:|-----------:|--------:|
| 10x10   |          0.23ms|      0.02ms|  11.5x |
| 20x20   |          0.58ms|      0.07ms|   7.9x |
| 50x50   |          1.62ms|      0.21ms|   7.7x |
| 100x100 |          4.76ms|      1.04ms|   4.6x |
