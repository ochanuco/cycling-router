//! Forward A* on a flat graph representation.
//!
//! 旧 `rust-router/src/lib.rs` から分離。JS/WASM 境界を持たない純 Rust 実装で、
//! wasm_bindgen wrapper は `router-wasm` 側にある。

use std::cmp::Ordering;
use std::collections::BinaryHeap;

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
pub fn astar(node_coords: &[f64], edge_data: &[f64], start: u32, goal: u32) -> Vec<f64> {
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
    heap.push(HeapEntry {
        f: heuristic(start),
        idx: start,
    });

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
                heap.push(HeapEntry {
                    f: ng + heuristic(to),
                    idx: to,
                });
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
