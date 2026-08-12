//! wasm-bindgen adapter for `router-core`.
//!
//! Cloudflare Workers / ブラウザから呼ぶ JS 境界だけを持つ薄い層。
//! アルゴリズム本体は `router-core` にあり、このクレートは型変換
//! (Uint8Array / JsValue) とオーケストレーションのみを担う。

use router_core::astar::astar as core_astar;
use router_core::{chquery, csr, route_filter, snap};
use serde::Serialize;
use wasm_bindgen::prelude::*;

/// Forward A* on the flat graph representation. Returns the path including
/// start and goal indices, prefixed by the total distance.
#[wasm_bindgen]
pub fn astar(node_coords: &[f64], edge_data: &[f64], start: u32, goal: u32) -> Vec<f64> {
    core_astar(node_coords, edge_data, start, goal)
}

/// Browser GPX-mode helper: for each shop point, compute the minimum
/// perpendicular distance (meters) to the route polyline. Used by
/// `frontend/app.js` to filter supply-points within N meters of the route
/// without running the O(N×M) JS loop on the main thread (5-10x faster).
///
/// Inputs (flat typed arrays for zero-copy boundary):
/// - `route_lonlats`: Float64Array of length 2*N (lon, lat alternating)
/// - `shop_lonlats`: Float64Array of length 2*M (lon, lat alternating)
///
/// Returns Float32Array of length M with per-shop minimum distance (m).
/// On empty/invalid inputs returns the appropriate length 0 / INF array.
#[wasm_bindgen]
pub fn route_distances(route_lonlats: &[f64], shop_lonlats: &[f64]) -> Vec<f32> {
    route_filter::route_distances(route_lonlats, shop_lonlats)
}

/// DNF route entry point. Decodes tile buffers, builds CSR, snaps endpoints,
/// runs CH bidirectional query, unpacks shortcuts. Returns a JS object with
/// `{ distance, settled, terminated, ch_ms, snap_from_m, snap_to_m,
/// from_id, to_id, coords: [[lon,lat]...], algorithm, csr_bytes,
/// csr_node_count, csr_edge_count }`.
///
/// `buffers` は `Array<Uint8Array>` を JS から渡す想定。各要素はタイル
/// binary (v1 or v2)。corridor + snap neighborhood 分まとめて渡す。
///
/// 失敗時は `{ error: "..." }` を含む JS object を返す (例外を投げない)。
#[wasm_bindgen]
pub fn route_ch(
    buffers: js_sys::Array,
    from_lon: f64,
    from_lat: f64,
    to_lon: f64,
    to_lat: f64,
    max_snap_meters: f64,
) -> JsValue {
    // Input validation (CodeRabbit PR #87 指摘):
    //  - max_snap_meters = NaN / 負値 → 全 snap 判定が false 化して上限無効化
    //  - from/to 座標 NaN → snap 内部で全 NaN 比較 → 全 INF → snap miss
    if !max_snap_meters.is_finite() || max_snap_meters <= 0.0 {
        return to_err("invalid_max_snap_meters");
    }
    if !from_lon.is_finite() || !from_lat.is_finite() || !to_lon.is_finite() || !to_lat.is_finite()
    {
        return to_err("invalid_coords");
    }

    // Validate all elements up front. The wasm path keeps JS Uint8Array handles
    // and lets CSR build copy one tile at a time into reusable scratch memory;
    // the native compile fallback keeps the previous owned Vec path.
    #[cfg(target_arch = "wasm32")]
    let mut u8_arrays: Vec<js_sys::Uint8Array> = Vec::with_capacity(buffers.length() as usize);
    #[cfg(not(target_arch = "wasm32"))]
    let mut buf_vec: Vec<Vec<u8>> = Vec::with_capacity(buffers.length() as usize);
    for i in 0..buffers.length() {
        let v = buffers.get(i);
        match v.dyn_into::<js_sys::Uint8Array>() {
            Ok(u8a) => {
                #[cfg(target_arch = "wasm32")]
                {
                    u8_arrays.push(u8a);
                }
                #[cfg(not(target_arch = "wasm32"))]
                {
                    buf_vec.push(u8a.to_vec());
                }
            }
            Err(_) => return to_err("invalid_buffer_element"),
        }
    }

    let t_csr0 = chquery_now_ms();
    #[cfg(target_arch = "wasm32")]
    let mut csr = csr::build_csr_from_uint8_arrays(&u8_arrays);
    #[cfg(not(target_arch = "wasm32"))]
    let mut csr = csr::build_csr(&buf_vec);
    let csr_build_ms = chquery_now_ms().saturating_sub(t_csr0) as u32;
    #[cfg(target_arch = "wasm32")]
    drop(u8_arrays);
    #[cfg(not(target_arch = "wasm32"))]
    drop(buf_vec);
    let csr_bytes = csr.memory_bytes() as u32;

    let from_snap = snap::snap(&csr, from_lon, from_lat);
    let to_snap = snap::snap(&csr, to_lon, to_lat);
    let (from_snap, to_snap) = match (from_snap, to_snap) {
        (Some(a), Some(b)) => (a, b),
        _ => return to_err("no_nearby_node"),
    };
    if from_snap.distance_m > max_snap_meters {
        return to_err("no_nearby_node_from");
    }
    if to_snap.distance_m > max_snap_meters {
        return to_err("no_nearby_node_to");
    }
    csr.release_ids();

    // CH 主経路 (level 制約あり)
    let t_ch0 = chquery_now_ms();
    let mut rc = chquery::ch_query(
        &csr,
        from_snap.idx,
        to_snap.idx,
        &chquery::ChQueryOpts::default(),
    );
    let ch_ms = chquery_now_ms().saturating_sub(t_ch0) as u32;
    let mut fallback_ms: Option<u32> = None;
    let mut algorithm = "ch-wasm";
    // cap 触れたら plain bidi Dijkstra fallback (level 制約なし)
    if !rc.distance.is_finite() {
        let t_fb0 = chquery_now_ms();
        rc = chquery::ch_query(
            &csr,
            from_snap.idx,
            to_snap.idx,
            &chquery::ChQueryOpts {
                settled_cap: 300_000,
                pops_cap: 800_000,
                time_budget_ms: 10_000,
                no_level_constraint: true,
            },
        );
        fallback_ms = Some(chquery_now_ms().saturating_sub(t_fb0) as u32);
        algorithm = "csr-wasm-dijkstra";
    }

    if !rc.distance.is_finite() {
        let meta = RouteMeta {
            csr_bytes,
            csr_node_count: csr.node_count,
            csr_edge_count: csr.edge_count,
            csr_build_ms,
            ch_ms,
            fallback_ms,
        };
        drop(csr);
        return to_err_with("unreachable_in_corridor", &meta);
    }

    // shortcut 展開
    let mut expanded: Vec<u32> = Vec::with_capacity(rc.path_idx.len() * 4);
    if !rc.path_idx.is_empty() {
        expanded.push(rc.path_idx[0]);
        for w in rc.path_idx.windows(2) {
            chquery::unpack_ch_edge(&csr, w[0], w[1], &mut expanded);
        }
    }
    let mut coords: Vec<(f32, f32)> = Vec::with_capacity(expanded.len());
    for &idx in &expanded {
        let i = idx as usize;
        let lon = csr.lons[i];
        let lat = csr.lats[i];
        if lon.is_nan() || lat.is_nan() {
            continue;
        }
        coords.push((lon, lat));
    }
    // NaN 座標を落とした後の点数を返す。expanded.len() のままだと coords の
    // 長さと食い違い、呼び出し側が「返ってきた経路点の数」として使うとずれる。
    let node_count = coords.len() as u32;
    let csr_node_count = csr.node_count;
    let csr_edge_count = csr.edge_count;
    drop(expanded);
    drop(csr);

    // OSM ids of from/to for caller logging (i64 → f64 で JS Number 範囲内、< 2^53)
    let result = RouteOk {
        distance: rc.distance,
        settled: rc.settled,
        terminated: rc.terminated.to_string(),
        ch_ms,
        fallback_ms,
        snap_from_m: from_snap.distance_m,
        snap_to_m: to_snap.distance_m,
        from_id: from_snap.id as f64,
        to_id: to_snap.id as f64,
        coords,
        algorithm: algorithm.to_string(),
        csr_bytes,
        csr_node_count,
        csr_edge_count,
        csr_build_ms,
        node_count,
    };
    serde_wasm_bindgen::to_value(&result).unwrap_or_else(|_| serialize_failed())
}

#[derive(Serialize)]
struct RouteOk {
    distance: f64,
    settled: u32,
    terminated: String,
    ch_ms: u32,
    fallback_ms: Option<u32>,
    snap_from_m: f64,
    snap_to_m: f64,
    from_id: f64,
    to_id: f64,
    coords: Vec<(f32, f32)>,
    algorithm: String,
    csr_bytes: u32,
    csr_node_count: u32,
    csr_edge_count: u32,
    csr_build_ms: u32,
    node_count: u32,
}

#[derive(Serialize)]
struct RouteMeta {
    csr_bytes: u32,
    csr_node_count: u32,
    csr_edge_count: u32,
    csr_build_ms: u32,
    ch_ms: u32,
    fallback_ms: Option<u32>,
}

#[derive(Serialize)]
struct RouteErr<'a> {
    error: &'a str,
}

#[derive(Serialize)]
struct RouteErrWithMeta<'a> {
    error: &'a str,
    meta: &'a RouteMeta,
}

/// serde 変換が失敗したときの最後の砦。route_ch は「常にオブジェクトを返す
/// (失敗時も `{ error: ... }`)」という契約なので、null を返すと呼び出し側の
/// `result.error` 参照が落ちる。js_sys で直接組み立てればシリアライズを
/// 経由しないため、この経路自体は失敗しない。
fn serialize_failed() -> JsValue {
    let obj = js_sys::Object::new();
    let _ = js_sys::Reflect::set(
        &obj,
        &JsValue::from_str("error"),
        &JsValue::from_str("serialize_failed"),
    );
    obj.into()
}

fn to_err(msg: &str) -> JsValue {
    serde_wasm_bindgen::to_value(&RouteErr { error: msg }).unwrap_or_else(|_| serialize_failed())
}

fn to_err_with(msg: &str, meta: &RouteMeta) -> JsValue {
    serde_wasm_bindgen::to_value(&RouteErrWithMeta { error: msg, meta })
        .unwrap_or_else(|_| serialize_failed())
}

// route_ch の各フェーズ (CSR 構築 / CH クエリ / fallback) を計測するための時刻取得。
// router-core の chquery にも同名の内部関数があるが、あちらは private なので
// proxy ではなく同じ実装を持つ。wasm32 では SystemTime が使えないため js_sys::Date、
// ネイティブでは SystemTime を使う。
#[cfg(target_arch = "wasm32")]
fn chquery_now_ms() -> u64 {
    js_sys::Date::now() as u64
}
#[cfg(not(target_arch = "wasm32"))]
fn chquery_now_ms() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
