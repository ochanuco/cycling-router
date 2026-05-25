/* tslint:disable */
/* eslint-disable */

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
export function route_ch(buffers: Array<any>, from_lon: number, from_lat: number, to_lon: number, to_lat: number, max_snap_meters: number): any;
