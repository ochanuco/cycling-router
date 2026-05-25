//! Cycling router compiled to wasm32.
//!
//! ## Entry points
//!
//! - [`route_ch`] — DNF cold-path: takes raw tile binaries from R2 +
//!   from/to lon/lat, runs tile decode → CSR build → snap → CH query →
//!   shortcut unpack, returns route geometry as JS object. This is the
//!   production path.
//! - [`astar`] (legacy) — original forward A* PoC on flat typed arrays.
//!   Kept for backward compat; not used in production.

mod csr;
mod chquery;
mod snap;

use std::collections::BinaryHeap;
use std::cmp::Ordering;
use wasm_bindgen::prelude::*;
use serde::Serialize;

const MIN_COST_FACTOR: f64 = 0.7;

#[derive(Clone, Copy)]
struct HeapEntry {
    f: f64,
    idx: u32,
}

impl Eq for HeapEntry {}
impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is max-heap; invert so we get min-heap on f.
        other.f.partial_cmp(&self.f).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn haversine_m(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    // Equirectangular approximation (matches JS aStarOnView).
    let mean_lat_rad = ((lat1 + lat2) * 0.5).to_radians();
    let cos_lat = mean_lat_rad.cos();
    let dxm = (lon2 - lon1) * cos_lat * 111_320.0;
    let dym = (lat2 - lat1) * 110_540.0;
    (dxm * dxm + dym * dym).sqrt()
}

/// Forward A* on the flat graph representation. Returns the path including
/// start and goal indices, prefixed by the total distance.
#[wasm_bindgen]
pub fn astar(
    node_coords: &[f64],
    edge_data: &[f64],
    start: u32,
    goal: u32,
) -> Vec<f64> {
    if node_coords.len() < 2 || node_coords.len() % 2 != 0 {
        return vec![f64::INFINITY];
    }
    let n = (node_coords.len() / 2) as u32;
    if start >= n || goal >= n {
        return vec![f64::INFINITY];
    }
    if start == goal {
        return vec![0.0, start as f64];
    }

    // Build CSR-style adjacency: head[v] = first edge index for node v
    let edge_count = edge_data.len() / 3;
    let mut fan_out: Vec<Vec<usize>> = vec![Vec::new(); n as usize];
    for ei in 0..edge_count {
        let from = edge_data[ei * 3] as u32;
        if from < n {
            fan_out[from as usize].push(ei);
        }
    }

    let goal_lon = node_coords[(goal as usize) * 2];
    let goal_lat = node_coords[(goal as usize) * 2 + 1];

    let heuristic = |idx: u32| {
        let i = (idx as usize) * 2;
        haversine_m(node_coords[i], node_coords[i + 1], goal_lon, goal_lat) * MIN_COST_FACTOR
    };

    let mut dist = vec![f64::INFINITY; n as usize];
    let mut parent = vec![i32::MIN; n as usize];
    let mut settled = vec![false; n as usize];

    dist[start as usize] = 0.0;
    let mut heap = BinaryHeap::new();
    heap.push(HeapEntry { f: heuristic(start), idx: start });

    while let Some(HeapEntry { idx: u_idx, .. }) = heap.pop() {
        if settled[u_idx as usize] {
            continue;
        }
        settled[u_idx as usize] = true;
        if u_idx == goal {
            break;
        }
        let g = dist[u_idx as usize];
        for &ei in &fan_out[u_idx as usize] {
            let to = edge_data[ei * 3 + 1] as u32;
            if to >= n || settled[to as usize] {
                continue;
            }
            let cost = edge_data[ei * 3 + 2];
            let ng = g + cost;
            if ng < dist[to as usize] {
                dist[to as usize] = ng;
                parent[to as usize] = u_idx as i32;
                heap.push(HeapEntry { f: ng + heuristic(to), idx: to });
            }
        }
    }

    if !dist[goal as usize].is_finite() {
        return vec![f64::INFINITY];
    }

    // Walk parent back from goal to start.
    let mut path_rev: Vec<f64> = Vec::new();
    let mut cur = goal as i32;
    while cur >= 0 {
        path_rev.push(cur as f64);
        cur = parent[cur as usize];
    }
    path_rev.reverse();
    let mut out = Vec::with_capacity(path_rev.len() + 1);
    out.push(dist[goal as usize]);
    out.extend(path_rev);
    out
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
    // Copy each Uint8Array into Vec<u8> for owned access during CSR build.
    let mut buf_vec: Vec<Vec<u8>> = Vec::with_capacity(buffers.length() as usize);
    for i in 0..buffers.length() {
        let v = buffers.get(i);
        if let Ok(u8a) = v.dyn_into::<js_sys::Uint8Array>() {
            buf_vec.push(u8a.to_vec());
        }
    }

    let t_csr0 = chquery_now_ms();
    let csr = csr::build_csr(&buf_vec);
    let csr_build_ms = (chquery_now_ms() - t_csr0) as u32;
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

    // CH 主経路 (level 制約あり)
    let t_ch0 = chquery_now_ms();
    let mut rc = chquery::ch_query(&csr, from_snap.idx, to_snap.idx, &chquery::ChQueryOpts::default());
    let mut ch_ms = (chquery_now_ms() - t_ch0) as u32;
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
        fallback_ms = Some((chquery_now_ms() - t_fb0) as u32);
        algorithm = "csr-wasm-dijkstra";
    }

    if !rc.distance.is_finite() {
        return to_err_with("unreachable_in_corridor", &RouteMeta {
            csr_bytes,
            csr_node_count: csr.node_count,
            csr_edge_count: csr.edge_count,
            csr_build_ms,
            ch_ms,
            fallback_ms,
        });
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
        csr_node_count: csr.node_count,
        csr_edge_count: csr.edge_count,
        csr_build_ms,
        node_count: expanded.len() as u32,
    };
    serde_wasm_bindgen::to_value(&result).unwrap_or(JsValue::NULL)
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

fn to_err(msg: &str) -> JsValue {
    serde_wasm_bindgen::to_value(&RouteErr { error: msg }).unwrap_or(JsValue::NULL)
}

fn to_err_with(msg: &str, meta: &RouteMeta) -> JsValue {
    serde_wasm_bindgen::to_value(&RouteErrWithMeta { error: msg, meta }).unwrap_or(JsValue::NULL)
}

// timing helper proxying to chquery module's now_ms
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_chain() {
        // 3 nodes: 0 → 1 → 2 (each segment ~111m apart in lon)
        let nodes: Vec<f64> = vec![135.0, 34.0, 135.001, 34.0, 135.002, 34.0];
        let edges: Vec<f64> = vec![
            0.0, 1.0, 100.0,
            1.0, 0.0, 100.0,
            1.0, 2.0, 100.0,
            2.0, 1.0, 100.0,
        ];
        let r = astar(&nodes, &edges, 0, 2);
        assert_eq!(r[0], 200.0);
        assert_eq!(r[1..], [0.0, 1.0, 2.0]);
    }

    #[test]
    fn start_eq_goal() {
        let nodes: Vec<f64> = vec![135.0, 34.0];
        let edges: Vec<f64> = vec![];
        let r = astar(&nodes, &edges, 0, 0);
        assert_eq!(r, vec![0.0, 0.0]);
    }

    #[test]
    fn unreachable() {
        let nodes: Vec<f64> = vec![135.0, 34.0, 135.001, 34.0];
        let edges: Vec<f64> = vec![];
        let r = astar(&nodes, &edges, 0, 1);
        assert_eq!(r, vec![f64::INFINITY]);
    }
}
