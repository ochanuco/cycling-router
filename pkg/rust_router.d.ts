/* AUTO-PATCHED-BY: scripts/patch_wasm_dts.mjs */
/* tslint:disable */
/* eslint-disable */

import type { RouteChResult } from './rust_router_worker';

/*
 * route_ch の正確な戻り値型は ./rust_router_worker.d.ts の RouteChResult
 * (RouteChOk | RouteChErr) を参照。本ファイルは wasm-pack 再生成時に
 * scripts/patch_wasm_dts.mjs が差分 patch で再適用する。
 */

/**
 * Forward A* on the flat graph representation. Returns the path including
 * start and goal indices, prefixed by the total distance.
 */
export function astar(node_coords: Float64Array, edge_data: Float64Array, start: number, goal: number): Float64Array;

/**
 * DNF route entry point. Decodes tile buffers, builds CSR, snaps endpoints,
 * runs CH bidirectional query, unpacks shortcuts. Returns a JS object with
 * `{ distance, settled, terminated, ch_ms, snap_from_m, snap_to_m,
 * from_id, to_id, coords: [[lon,lat]...], algorithm, csr_bytes,
 * csr_node_count, csr_edge_count }`.
 *
 * `buffers` は `Array<Uint8Array>` を JS から渡す想定。各要素はタイル
 * binary (v1 or v2)。corridor + snap neighborhood 分まとめて渡す。
 *
 * 失敗時は `{ error: "..." }` を含む JS object を返す (例外を投げない)。
 */
export function route_ch(buffers: Uint8Array[], from_lon: number, from_lat: number, to_lon: number, to_lat: number, max_snap_meters: number): RouteChResult;

/**
 * Browser GPX-mode helper: for each shop point, compute the minimum
 * perpendicular distance (meters) to the route polyline. Used by
 * `frontend/app.js` to filter supply-points within N meters of the route
 * without running the O(N×M) JS loop on the main thread (5-10x faster).
 *
 * Inputs (flat typed arrays for zero-copy boundary):
 * - `route_lonlats`: Float64Array of length 2*N (lon, lat alternating)
 * - `shop_lonlats`: Float64Array of length 2*M (lon, lat alternating)
 *
 * Returns Float32Array of length M with per-shop minimum distance (m).
 * On empty/invalid inputs returns the appropriate length 0 / INF array.
 */
export function route_distances(route_lonlats: Float64Array, shop_lonlats: Float64Array): Float32Array;
