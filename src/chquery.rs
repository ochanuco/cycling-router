//! CH bidirectional query on CSR. Port of `lib/cycling/chquery_csr.js`.
//!
//! - Float64 dist (precision required; Float32 で経路選択が劣化する)
//! - Per-direction termination: topF >= best && topB >= best
//! - UNKNOWN_LEVEL ノードへの relax は skip (cross-tile target 対応)
//! - core-core lateral relax 許可 (`noLevelConstraint` で全部緩める fallback も)
//! - settled / pops / time の caps で hard bail-out → caller が fallback

use crate::csr::{Csr, NO_VIA, UNKNOWN_LEVEL};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

const INF: f64 = f64::INFINITY;
const NO_PARENT: u32 = u32::MAX;
const SETTLED_F: u8 = 1;
const SETTLED_B: u8 = 2;

#[derive(Clone, Copy)]
struct HeapEntry {
    key: f64,
    idx: u32,
}

impl Eq for HeapEntry {}
impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.key == other.key
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is max-heap; invert to get min-heap on key.
        other.key.partial_cmp(&self.key).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct ChQueryOpts {
    pub settled_cap: u32,
    pub pops_cap: u32,
    pub time_budget_ms: u32,
    pub no_level_constraint: bool,
}

impl Default for ChQueryOpts {
    fn default() -> Self {
        Self {
            settled_cap: 80_000,
            pops_cap: 200_000,
            time_budget_ms: 5_000,
            no_level_constraint: false,
        }
    }
}

pub struct ChQueryResult {
    pub distance: f64,
    pub path_idx: Vec<u32>,
    pub settled: u32,
    /// "ok" / "cap" / "time" / "noMeet" / "same" / "oob"
    pub terminated: &'static str,
}

pub fn ch_query(csr: &Csr, start_idx: u32, goal_idx: u32, opts: &ChQueryOpts) -> ChQueryResult {
    let n = csr.node_count;
    if start_idx == goal_idx {
        return ChQueryResult {
            distance: 0.0,
            path_idx: vec![start_idx],
            settled: 0,
            terminated: "same",
        };
    }
    if start_idx >= n || goal_idx >= n {
        return ChQueryResult {
            distance: INF,
            path_idx: vec![],
            settled: 0,
            terminated: "oob",
        };
    }

    let nc = n as usize;
    let mut dist_f: Vec<f64> = vec![INF; nc];
    let mut dist_b: Vec<f64> = vec![INF; nc];
    dist_f[start_idx as usize] = 0.0;
    dist_b[goal_idx as usize] = 0.0;
    let mut parent_f: Vec<u32> = vec![NO_PARENT; nc];
    let mut parent_b: Vec<u32> = vec![NO_PARENT; nc];
    let mut settled: Vec<u8> = vec![0; nc];

    let mut heap_f: BinaryHeap<HeapEntry> = BinaryHeap::with_capacity(1024);
    let mut heap_b: BinaryHeap<HeapEntry> = BinaryHeap::with_capacity(1024);
    heap_f.push(HeapEntry { key: 0.0, idx: start_idx });
    heap_b.push(HeapEntry { key: 0.0, idx: goal_idx });

    let mut best: f64 = INF;
    let mut meeting: i32 = -1;

    let t0_ms = now_ms();
    let mut pops: u32 = 0;
    let mut settled_count: u32 = 0;

    // We pre-borrow CSR fields to avoid repeated indirections.
    let levels = &csr.levels;
    let cores = &csr.cores;
    let fwd_off = &csr.fwd_offsets;
    let fwd_to = &csr.fwd_to;
    let fwd_cost = &csr.fwd_cost;
    let rev_off = &csr.rev_offsets;
    let rev_from = &csr.rev_from;
    let rev_cost = &csr.rev_cost;
    let no_level = opts.no_level_constraint;

    loop {
        if settled_count > opts.settled_cap || pops > opts.pops_cap {
            return ChQueryResult {
                distance: INF,
                path_idx: vec![],
                settled: settled_count,
                terminated: "cap",
            };
        }
        if (pops & 0x3FF) == 0 && now_ms() - t0_ms > opts.time_budget_ms as u64 {
            return ChQueryResult {
                distance: INF,
                path_idx: vec![],
                settled: settled_count,
                terminated: "time",
            };
        }
        pops += 1;

        let top_f = heap_f.peek().map(|e| e.key).unwrap_or(INF);
        let top_b = heap_b.peek().map(|e| e.key).unwrap_or(INF);
        if top_f >= best && top_b >= best {
            break;
        }
        let expand_f = top_f < best && (top_b >= best || top_f <= top_b);

        if expand_f {
            let ent = heap_f.pop();
            let HeapEntry { key: d, idx: u } = match ent {
                Some(e) => e,
                None => break,
            };
            let u_usize = u as usize;
            if settled[u_usize] & SETTLED_F != 0 {
                continue;
            }
            if d > dist_f[u_usize] {
                continue;
            }
            settled[u_usize] |= SETTLED_F;
            settled_count += 1;
            let db = dist_b[u_usize];
            if db != INF {
                let sum = d + db;
                if sum < best {
                    best = sum;
                    meeting = u as i32;
                }
            }
            let u_level = levels[u_usize];
            let u_is_core = cores[u_usize] == 1;
            let so = fwd_off[u_usize] as usize;
            let eo = fwd_off[u_usize + 1] as usize;
            for e in so..eo {
                let v = fwd_to[e];
                let v_usize = v as usize;
                let v_level = levels[v_usize];
                if v_level == UNKNOWN_LEVEL {
                    continue;
                }
                if !no_level {
                    let core_core = u_is_core && cores[v_usize] == 1;
                    if !core_core && v_level <= u_level {
                        continue;
                    }
                }
                let nd = d + fwd_cost[e] as f64;
                if nd < dist_f[v_usize] {
                    dist_f[v_usize] = nd;
                    parent_f[v_usize] = u;
                    heap_f.push(HeapEntry { key: nd, idx: v });
                    let dbv = dist_b[v_usize];
                    if dbv != INF {
                        let sum = nd + dbv;
                        if sum < best {
                            best = sum;
                            meeting = v as i32;
                        }
                    }
                }
            }
        } else {
            let ent = heap_b.pop();
            let HeapEntry { key: d, idx: u } = match ent {
                Some(e) => e,
                None => break,
            };
            let u_usize = u as usize;
            if settled[u_usize] & SETTLED_B != 0 {
                continue;
            }
            if d > dist_b[u_usize] {
                continue;
            }
            settled[u_usize] |= SETTLED_B;
            settled_count += 1;
            let df = dist_f[u_usize];
            if df != INF {
                let sum = df + d;
                if sum < best {
                    best = sum;
                    meeting = u as i32;
                }
            }
            let u_level = levels[u_usize];
            let u_is_core = cores[u_usize] == 1;
            let so = rev_off[u_usize] as usize;
            let eo = rev_off[u_usize + 1] as usize;
            for e in so..eo {
                let v = rev_from[e];
                let v_usize = v as usize;
                let v_level = levels[v_usize];
                if v_level == UNKNOWN_LEVEL {
                    continue;
                }
                if !no_level {
                    let core_core = u_is_core && cores[v_usize] == 1;
                    if !core_core && v_level <= u_level {
                        continue;
                    }
                }
                let nd = d + rev_cost[e] as f64;
                if nd < dist_b[v_usize] {
                    dist_b[v_usize] = nd;
                    parent_b[v_usize] = u;
                    heap_b.push(HeapEntry { key: nd, idx: v });
                    let dfv = dist_f[v_usize];
                    if dfv != INF {
                        let sum = dfv + nd;
                        if sum < best {
                            best = sum;
                            meeting = v as i32;
                        }
                    }
                }
            }
        }
    }

    if meeting < 0 || !best.is_finite() {
        return ChQueryResult {
            distance: INF,
            path_idx: vec![],
            settled: settled_count,
            terminated: "noMeet",
        };
    }

    // Reconstruct path: start → ... → meeting → ... → goal (forward direction).
    let m = meeting as u32;
    let mut fwd_chain: Vec<u32> = vec![m];
    let mut cur = m;
    while parent_f[cur as usize] != NO_PARENT {
        cur = parent_f[cur as usize];
        fwd_chain.push(cur);
    }
    fwd_chain.reverse();
    let mut back_chain: Vec<u32> = Vec::new();
    cur = m;
    while parent_b[cur as usize] != NO_PARENT {
        cur = parent_b[cur as usize];
        back_chain.push(cur);
    }
    let mut path_idx = fwd_chain;
    path_idx.extend(back_chain);

    ChQueryResult {
        distance: best,
        path_idx,
        settled: settled_count,
        terminated: "ok",
    }
}

/// Walk a (u_idx → v_idx) edge expanding any shortcut via recursively.
/// Appends intermediate node indices (excluding u_idx) into `out`.
pub fn unpack_ch_edge(csr: &Csr, u_idx: u32, v_idx: u32, out: &mut Vec<u32>) {
    let mut stack: Vec<(u32, u32)> = vec![(u_idx, v_idx)];
    let mut safety: u32 = 0;
    while let Some((a, b)) = stack.pop() {
        safety += 1;
        if safety > 1_000_000 {
            break;
        }
        // Linear scan over fwd edges of `a` to find one ending at `b`.
        let so = csr.fwd_offsets[a as usize] as usize;
        let eo = csr.fwd_offsets[(a as usize) + 1] as usize;
        let mut via: i64 = -2; // -2 not found, -1 original, >=0 via idx
        for e in so..eo {
            if csr.fwd_to[e] == b {
                let v = csr.fwd_via_id[e];
                via = if v == NO_VIA { -1 } else { v as i64 };
                break;
            }
        }
        if via == -1 || via == -2 {
            out.push(b);
            continue;
        }
        // Push reversed so (a → via) processed first.
        stack.push((via as u32, b));
        stack.push((a, via as u32));
    }
}

// time helpers ---------------------------------------------------------------

#[cfg(target_arch = "wasm32")]
fn now_ms() -> u64 {
    // js_sys::Date::now() returns f64 ms since epoch.
    js_sys::Date::now() as u64
}

#[cfg(not(target_arch = "wasm32"))]
fn now_ms() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::{build_csr, NO_VIA};

    // Minimal v2 tile builder for tests (mirrors csr.rs tests).
    fn enc_node(id: u64, lon: f32, lat: f32, level: u32, core: u8) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(id as f64).to_le_bytes());
        v.extend_from_slice(&lon.to_le_bytes());
        v.extend_from_slice(&lat.to_le_bytes());
        let word = level | if core != 0 { 1u32 << 31 } else { 0 };
        v.extend_from_slice(&word.to_le_bytes());
        v
    }
    fn enc_edge(from: u64, to: u64, to_lon: f32, to_lat: f32, cost: f32, via: u64) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(from as f64).to_le_bytes());
        v.extend_from_slice(&(to as f64).to_le_bytes());
        v.extend_from_slice(&to_lon.to_le_bytes());
        v.extend_from_slice(&to_lat.to_le_bytes());
        v.extend_from_slice(&cost.to_le_bytes());
        v.extend_from_slice(&0u32.to_le_bytes());
        v.extend_from_slice(&(via as f64).to_le_bytes());
        v
    }
    fn make_tile(nodes: Vec<Vec<u8>>, edges: Vec<Vec<u8>>) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&0x45444952u32.to_le_bytes());
        b.push(2);
        b.push(0);
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&(nodes.len() as u32).to_le_bytes());
        b.extend_from_slice(&(edges.len() as u32).to_le_bytes());
        for n in nodes { b.extend(n); }
        for e in edges { b.extend(e); }
        b
    }

    fn idx_of(csr: &crate::csr::Csr, id: u64) -> u32 {
        csr.ids.iter().position(|&x| x == id).unwrap() as u32
    }

    #[test]
    fn chain_returns_total_distance() {
        // 0 → 1 → 2 → 3, levels 0..3, costs 100 each
        let nodes = vec![
            enc_node(10, 0.0, 0.0, 0, 0),
            enc_node(11, 0.001, 0.0, 1, 0),
            enc_node(12, 0.002, 0.0, 2, 0),
            enc_node(13, 0.003, 0.0, 3, 0),
        ];
        let edges = vec![
            enc_edge(10, 11, 0.001, 0.0, 100.0, 0),
            enc_edge(11, 12, 0.002, 0.0, 100.0, 0),
            enc_edge(12, 13, 0.003, 0.0, 100.0, 0),
        ];
        let csr = build_csr(&[make_tile(nodes, edges)]);
        let start = idx_of(&csr, 10);
        let goal = idx_of(&csr, 13);
        let r = ch_query(&csr, start, goal, &ChQueryOpts::default());
        assert!((r.distance - 300.0).abs() < 1e-6, "dist={}", r.distance);
        assert_eq!(r.path_idx.len(), 4);
        assert_eq!(r.terminated, "ok");
    }

    #[test]
    fn unreachable_returns_inf() {
        let nodes = vec![enc_node(0, 0.0, 0.0, 0, 0), enc_node(1, 0.001, 0.0, 1, 0)];
        let csr = build_csr(&[make_tile(nodes, vec![])]);
        let r = ch_query(&csr, 0, 1, &ChQueryOpts::default());
        assert!(r.distance.is_infinite());
    }

    #[test]
    fn core_core_lateral_relax_allows_downhill() {
        // 0(L10,core) - 1(L5,core) - 2(L10,non-core), edge weights 100 each
        let nodes = vec![
            enc_node(0, 0.0, 0.0, 10, 1),
            enc_node(1, 0.001, 0.0, 5, 1),
            enc_node(2, 0.002, 0.0, 10, 0),
        ];
        let edges = vec![
            enc_edge(0, 1, 0.001, 0.0, 100.0, 0),
            enc_edge(1, 2, 0.002, 0.0, 100.0, 0),
        ];
        let csr = build_csr(&[make_tile(nodes, edges)]);
        let r = ch_query(&csr, idx_of(&csr, 0), idx_of(&csr, 2), &ChQueryOpts::default());
        assert!((r.distance - 200.0).abs() < 1e-6, "dist={}", r.distance);
    }

    #[test]
    fn shortcut_unpack_iterative() {
        let nodes = vec![
            enc_node(0, 0.0, 0.0, 0, 0),
            enc_node(1, 0.001, 0.0, 1, 0),
            enc_node(2, 0.002, 0.0, 2, 0),
        ];
        let edges = vec![
            enc_edge(0, 1, 0.001, 0.0, 100.0, 0),
            enc_edge(1, 2, 0.002, 0.0, 100.0, 0),
            enc_edge(0, 2, 0.002, 0.0, 200.0, 1),
        ];
        let csr = build_csr(&[make_tile(nodes, edges)]);
        let i0 = idx_of(&csr, 0);
        let i1 = idx_of(&csr, 1);
        let i2 = idx_of(&csr, 2);
        let mut out = Vec::new();
        unpack_ch_edge(&csr, i0, i2, &mut out);
        assert_eq!(out, vec![i1, i2]);
        // suppress unused warnings for NO_VIA in test scope
        let _ = NO_VIA;
    }
}
