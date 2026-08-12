/* Cloudflare Workers 用 wrapper (worker.js) の型定義。
 *
 * wasm-pack が生成する router_wasm.d.ts は route_ch を `any` にする
 * (serde_wasm_bindgen 経由で JsValue を返すため wasm-bindgen が形を知らない)。
 * 利用側は wrapper 経由で import するので、正確な型はここで与える。
 *
 * 各フィールドは crates/router-wasm/src/lib.rs の RouteOk / RouteErr /
 * RouteErrWithMeta / RouteMeta に対応する。ズレると scripts/check-dts.mjs が
 * CI で落ちる。
 */

/** route_ch が計測値として返すメタ情報。エラー時にも付くことがある。 */
export interface RouteMeta {
  csr_bytes: number;
  csr_node_count: number;
  csr_edge_count: number;
  csr_build_ms: number;
  ch_ms: number;
  /** CH で解けず bidirectional Dijkstra に落ちたときのみ設定される。 */
  fallback_ms: number | null;
}

/** 経路が引けたときの結果。coords は [lon, lat] の並び。 */
export interface RouteChOk {
  error?: undefined;
  distance: number;
  settled: number;
  terminated: string;
  ch_ms: number;
  fallback_ms: number | null;
  snap_from_m: number;
  snap_to_m: number;
  from_id: number;
  to_id: number;
  coords: Array<[number, number]>;
  algorithm: string;
  csr_bytes: number;
  csr_node_count: number;
  csr_edge_count: number;
  csr_build_ms: number;
  /** NaN 座標を除いた後の経路点数 (= coords.length)。 */
  node_count: number;
}

/** 失敗時。例外は投げず、必ずこの形のオブジェクトが返る。 */
export interface RouteChErr {
  error: string;
  meta?: RouteMeta;
}

export type RouteChResult = RouteChOk | RouteChErr;

/**
 * タイル binary から CSR を組み、端点をスナップして CH クエリを解く。
 * 失敗時も例外を投げず `{ error }` を返す。
 */
export function route_ch(
  buffers: Uint8Array[],
  from_lon: number,
  from_lat: number,
  to_lon: number,
  to_lat: number,
  max_snap_meters: number
): RouteChResult;

/** flat な [lon, lat, ...] 配列で A* を解く。先頭が総距離、以降が経路 index。 */
export function astar(
  node_coords: Float64Array,
  edge_data: Float64Array,
  start: number,
  goal: number
): Float64Array;

/** 各 shop について、ルート折れ線までの最短距離 (m) を返す。長さは shop 数。 */
export function route_distances(
  route_lonlats: Float64Array,
  shop_lonlats: Float64Array
): Float32Array;
