# rust-router

Cycling A\* router compiled to wasm32 — PoC for comparing Rust/WASM vs the
existing JS implementation in `lib/cycling/tiled_router.js`.

## Build

```bash
mise exec -- wasm-pack build --target nodejs --release
```

Outputs to `pkg/`:
- `rust_router.js` — JS glue (CommonJS for Node)
- `rust_router_bg.wasm` — compiled WASM module
- `rust_router.d.ts` — TypeScript declarations

For browser/Worker: use `--target web` or `--target bundler`.

## Benchmark

After building:

```bash
node scripts/bench_wasm_vs_js.mjs
```

Compares JS aStarOnView vs Rust/WASM `astar()` on the same synthetic graph
(20x20 grid + 15x15 grid) and reports timings.

## Status

- [x] forward A\* implementation in Rust (parity with JS)
- [x] cargo unit tests pass
- [x] wasm-pack build verified (24KB optimized wasm)
- [x] benchmark vs JS (`scripts/bench_wasm_vs_js.mjs`)
- [ ] decode binary tile format directly in Rust (zero-copy)
- [ ] Workers integration

This is an exploratory branch. Not for merge yet.

## Benchmark results (synthetic grids)

| grid     | JS aStarOnView | WASM astar | speedup |
|----------|---------------:|-----------:|--------:|
| 10x10    |          0.23ms|      0.02ms|  11.5x |
| 20x20    |          0.58ms|      0.07ms|   7.9x |
| 50x50    |          1.62ms|      0.21ms|   7.7x |
| 100x100  |          4.76ms|      1.04ms|   4.6x |

Both compute identical optimal distances (`match ✓`).

For real OSM workloads where A\* heuristic prunes search (settled << N), the
speedup will be smaller (likely 2-3x). The 100x100 grid is the closest to a
"settled = total" worst case; actual Kansai queries settle ~10k of millions
of nodes.

## When to consider integrating

- Worker CPU is the bottleneck even after CH integration (PR #2b 元案)
- Need >50km routes at <1s warm latency
- Cold-start cost of wasm-instantiate (~30-50ms) is acceptable

For current Kansai 18km-cap deployment, edge cache + NBA\* already meets
target. WASM is a "kept ready" optimization rather than a near-term need.
