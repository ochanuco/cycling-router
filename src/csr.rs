//! CH-mode ephemeral CSR built from tile binary buffers in WASM context.
//!
//! Port of `lib/cycling/ch_csr.js` with the same data layout: Uint32 offsets
//! + Uint32 to / Float32 cost / Uint32 viaId per direction. Nodes carry
//! ids/lons/lats/levels/cores as parallel typed arrays.
//!
//! Memory profile: pre-sized typed-array allocations only. No intermediate
//! Vec growth (mirrors the JS leaner-build pattern).

use std::collections::HashMap;

pub const HEADER_BYTES: usize = 16;
pub const NODE_BYTES_V1: usize = 16;
pub const NODE_BYTES_V2: usize = 20;
pub const EDGE_BYTES_V1: usize = 28;
pub const EDGE_BYTES_V2: usize = 40;
pub const MAGIC: u32 = 0x45444952; // "RIDE"
const CORE_BIT_V2: u32 = 1 << 31;

pub const NO_VIA: u32 = u32::MAX;
pub const UNKNOWN_LEVEL: u32 = u32::MAX - 1;

#[derive(Clone, Copy)]
struct TileHeader {
    version: u8,
    node_count: u32,
    edge_count: u32,
}

fn read_header(buf: &[u8]) -> Option<TileHeader> {
    if buf.len() < HEADER_BYTES {
        return None;
    }
    let magic = u32::from_le_bytes(buf[0..4].try_into().ok()?);
    if magic != MAGIC {
        return None;
    }
    let version = buf[4];
    if version != 1 && version != 2 {
        return None;
    }
    let node_count = u32::from_le_bytes(buf[8..12].try_into().ok()?);
    let edge_count = u32::from_le_bytes(buf[12..16].try_into().ok()?);
    Some(TileHeader { version, node_count, edge_count })
}

#[inline]
fn read_f64_le(buf: &[u8], off: usize) -> f64 {
    f64::from_le_bytes(buf[off..off + 8].try_into().unwrap_or([0u8; 8]))
}

#[inline]
fn read_f32_le(buf: &[u8], off: usize) -> f32 {
    f32::from_le_bytes(buf[off..off + 4].try_into().unwrap_or([0u8; 4]))
}

#[inline]
fn read_u32_le(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes(buf[off..off + 4].try_into().unwrap_or([0u8; 4]))
}

pub struct Csr {
    pub node_count: u32,
    pub edge_count: u32,
    pub id_to_idx: HashMap<u64, u32>,
    pub ids: Vec<u64>,
    pub lons: Vec<f32>,
    pub lats: Vec<f32>,
    pub levels: Vec<u32>,
    pub cores: Vec<u8>,
    pub fwd_offsets: Vec<u32>,
    pub fwd_to: Vec<u32>,
    pub fwd_cost: Vec<f32>,
    pub fwd_via_id: Vec<u32>,
    pub rev_offsets: Vec<u32>,
    pub rev_from: Vec<u32>,
    pub rev_cost: Vec<f32>,
    pub rev_via_id: Vec<u32>,
}

impl Csr {
    pub fn memory_bytes(&self) -> usize {
        self.ids.len() * 8
            + self.lons.len() * 4
            + self.lats.len() * 4
            + self.levels.len() * 4
            + self.cores.len()
            + self.fwd_offsets.len() * 4
            + self.fwd_to.len() * 4
            + self.fwd_cost.len() * 4
            + self.fwd_via_id.len() * 4
            + self.rev_offsets.len() * 4
            + self.rev_from.len() * 4
            + self.rev_cost.len() * 4
            + self.rev_via_id.len() * 4
    }
}

/// Build CSR from a slice of tile binary buffers.
pub fn build_csr(buffers: &[Vec<u8>]) -> Csr {
    // Phase 0: scan headers for upper bound sizing.
    let mut headers: Vec<Option<TileHeader>> = Vec::with_capacity(buffers.len());
    let mut node_upper: usize = 0;
    for buf in buffers {
        let h = read_header(buf);
        if let Some(ref hh) = h {
            node_upper += hh.node_count as usize;
        }
        headers.push(h);
    }

    // Phase A: pre-allocate node arrays sized to node_upper (sum of tile
    // node sections). Cross-tile target / via node の追加登録は意図的に
    // 行わない (corridor 境界 edges/shortcuts は idx undefined → スキップ)。
    let mut ids: Vec<u64> = vec![0; node_upper];
    let mut lons: Vec<f32> = vec![0.0; node_upper];
    let mut lats: Vec<f32> = vec![0.0; node_upper];
    let mut levels: Vec<u32> = vec![UNKNOWN_LEVEL; node_upper];
    let mut cores: Vec<u8> = vec![0; node_upper];
    let mut node_count: u32 = 0;
    let mut id_to_idx: HashMap<u64, u32> = HashMap::with_capacity(node_upper);

    // Phase B: ingest tile node sections.
    for (t, buf) in buffers.iter().enumerate() {
        let h = match headers[t] {
            Some(h) => h,
            None => continue,
        };
        let mut off = HEADER_BYTES;
        if h.version == 2 {
            for _ in 0..h.node_count {
                let id = read_f64_le(buf, off) as u64;
                let lon = read_f32_le(buf, off + 8);
                let lat = read_f32_le(buf, off + 12);
                let word = read_u32_le(buf, off + 16);
                let level = if word >= CORE_BIT_V2 { word - CORE_BIT_V2 } else { word };
                let core = if word >= CORE_BIT_V2 { 1u8 } else { 0u8 };
                if !id_to_idx.contains_key(&id) {
                    let idx = node_count;
                    id_to_idx.insert(id, idx);
                    let i = idx as usize;
                    ids[i] = id;
                    lons[i] = lon;
                    lats[i] = lat;
                    levels[i] = level;
                    cores[i] = core;
                    node_count += 1;
                }
                off += NODE_BYTES_V2;
            }
        } else {
            for _ in 0..h.node_count {
                let id = read_f64_le(buf, off) as u64;
                let lon = read_f32_le(buf, off + 8);
                let lat = read_f32_le(buf, off + 12);
                if !id_to_idx.contains_key(&id) {
                    let idx = node_count;
                    id_to_idx.insert(id, idx);
                    let i = idx as usize;
                    ids[i] = id;
                    lons[i] = lon;
                    lats[i] = lat;
                    // levels[i] stays at default UNKNOWN_LEVEL (relax 禁止)
                    node_count += 1;
                }
                off += NODE_BYTES_V1;
            }
        }
    }

    // Phase C: precompute edge section offsets.
    let mut edge_offsets: Vec<usize> = vec![0; buffers.len()];
    for t in 0..buffers.len() {
        if let Some(h) = headers[t] {
            let nb = if h.version == 2 { NODE_BYTES_V2 } else { NODE_BYTES_V1 };
            edge_offsets[t] = HEADER_BYTES + (h.node_count as usize) * nb;
        }
    }

    // Phase D: count fwd/rev degrees, total edges.
    let nc = node_count as usize;
    let mut fwd_deg: Vec<u32> = vec![0; nc];
    let mut rev_deg: Vec<u32> = vec![0; nc];
    let mut total_edges: u32 = 0;
    for t in 0..buffers.len() {
        let h = match headers[t] {
            Some(h) => h,
            None => continue,
        };
        let eb = if h.version == 2 { EDGE_BYTES_V2 } else { EDGE_BYTES_V1 };
        let mut off = edge_offsets[t];
        for _ in 0..h.edge_count {
            let from = read_f64_le(&buffers[t], off) as u64;
            let to = read_f64_le(&buffers[t], off + 8) as u64;
            if let (Some(&f_idx), Some(&t_idx)) = (id_to_idx.get(&from), id_to_idx.get(&to)) {
                fwd_deg[f_idx as usize] += 1;
                rev_deg[t_idx as usize] += 1;
                total_edges += 1;
            }
            off += eb;
        }
    }

    // Phase E: prefix sums.
    let mut fwd_offsets: Vec<u32> = vec![0; nc + 1];
    let mut rev_offsets: Vec<u32> = vec![0; nc + 1];
    for i in 0..nc {
        fwd_offsets[i + 1] = fwd_offsets[i] + fwd_deg[i];
        rev_offsets[i + 1] = rev_offsets[i] + rev_deg[i];
    }

    // Phase F: fill CSR.
    let te = total_edges as usize;
    let mut fwd_to: Vec<u32> = vec![0; te];
    let mut fwd_cost: Vec<f32> = vec![0.0; te];
    let mut fwd_via_id: Vec<u32> = vec![NO_VIA; te];
    let mut rev_from: Vec<u32> = vec![0; te];
    let mut rev_cost: Vec<f32> = vec![0.0; te];
    let mut rev_via_id: Vec<u32> = vec![NO_VIA; te];
    let mut fwd_cursor: Vec<u32> = fwd_offsets.clone();
    let mut rev_cursor: Vec<u32> = rev_offsets.clone();
    for t in 0..buffers.len() {
        let h = match headers[t] {
            Some(h) => h,
            None => continue,
        };
        let eb = if h.version == 2 { EDGE_BYTES_V2 } else { EDGE_BYTES_V1 };
        let mut off = edge_offsets[t];
        for _ in 0..h.edge_count {
            let from = read_f64_le(&buffers[t], off) as u64;
            let to = read_f64_le(&buffers[t], off + 8) as u64;
            let cost = read_f32_le(&buffers[t], off + 24);
            let mut via_idx = NO_VIA;
            if h.version == 2 {
                let via_osm = read_f64_le(&buffers[t], off + 32) as u64;
                if via_osm != 0 {
                    if let Some(&vi) = id_to_idx.get(&via_osm) {
                        via_idx = vi;
                    }
                }
            }
            if let (Some(&f_idx), Some(&t_idx)) = (id_to_idx.get(&from), id_to_idx.get(&to)) {
                let fp = fwd_cursor[f_idx as usize] as usize;
                fwd_cursor[f_idx as usize] += 1;
                fwd_to[fp] = t_idx;
                fwd_cost[fp] = cost;
                fwd_via_id[fp] = via_idx;
                let rp = rev_cursor[t_idx as usize] as usize;
                rev_cursor[t_idx as usize] += 1;
                rev_from[rp] = f_idx;
                rev_cost[rp] = cost;
                rev_via_id[rp] = via_idx;
            }
            off += eb;
        }
    }

    // Truncate node arrays to actual nodeCount (Vec::truncate drops tail).
    ids.truncate(nc);
    lons.truncate(nc);
    lats.truncate(nc);
    levels.truncate(nc);
    cores.truncate(nc);

    Csr {
        node_count,
        edge_count: total_edges,
        id_to_idx,
        ids,
        lons,
        lats,
        levels,
        cores,
        fwd_offsets,
        fwd_to,
        fwd_cost,
        fwd_via_id,
        rev_offsets,
        rev_from,
        rev_cost,
        rev_via_id,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode_node_v2(id: u64, lon: f32, lat: f32, level: u32, core: u8) -> Vec<u8> {
        let mut v = Vec::with_capacity(NODE_BYTES_V2);
        v.extend_from_slice(&(id as f64).to_le_bytes());
        v.extend_from_slice(&lon.to_le_bytes());
        v.extend_from_slice(&lat.to_le_bytes());
        let word = level | if core != 0 { CORE_BIT_V2 } else { 0 };
        v.extend_from_slice(&word.to_le_bytes());
        v
    }

    fn encode_edge_v2(from: u64, to: u64, to_lon: f32, to_lat: f32, cost: f32, via: u64) -> Vec<u8> {
        let mut v = Vec::with_capacity(EDGE_BYTES_V2);
        v.extend_from_slice(&(from as f64).to_le_bytes());
        v.extend_from_slice(&(to as f64).to_le_bytes());
        v.extend_from_slice(&to_lon.to_le_bytes());
        v.extend_from_slice(&to_lat.to_le_bytes());
        v.extend_from_slice(&cost.to_le_bytes());
        v.extend_from_slice(&0u32.to_le_bytes()); // pad
        v.extend_from_slice(&(via as f64).to_le_bytes());
        v
    }

    fn make_tile_v2(nodes: Vec<Vec<u8>>, edges: Vec<Vec<u8>>) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&MAGIC.to_le_bytes());
        buf.push(2); // version
        buf.push(0); // flags
        buf.extend_from_slice(&0u16.to_le_bytes()); // pad
        buf.extend_from_slice(&(nodes.len() as u32).to_le_bytes());
        buf.extend_from_slice(&(edges.len() as u32).to_le_bytes());
        for n in nodes {
            buf.extend(n);
        }
        for e in edges {
            buf.extend(e);
        }
        buf
    }

    #[test]
    fn empty_tile_builds_empty_csr() {
        let tile = make_tile_v2(vec![], vec![]);
        let csr = build_csr(&[tile]);
        assert_eq!(csr.node_count, 0);
        assert_eq!(csr.edge_count, 0);
    }

    #[test]
    fn single_node_single_edge_roundtrip() {
        let nodes = vec![
            encode_node_v2(100, 135.0, 34.0, 10, 0),
            encode_node_v2(200, 135.001, 34.0, 20, 0),
        ];
        let edges = vec![encode_edge_v2(100, 200, 135.001, 34.0, 100.0, 0)];
        let tile = make_tile_v2(nodes, edges);
        let csr = build_csr(&[tile]);
        assert_eq!(csr.node_count, 2);
        assert_eq!(csr.edge_count, 1);
        let i100 = *csr.id_to_idx.get(&100).unwrap();
        let i200 = *csr.id_to_idx.get(&200).unwrap();
        let s = csr.fwd_offsets[i100 as usize] as usize;
        let e = csr.fwd_offsets[(i100 as usize) + 1] as usize;
        assert_eq!(e - s, 1);
        assert_eq!(csr.fwd_to[s], i200);
        assert!((csr.fwd_cost[s] - 100.0).abs() < 0.01);
        assert_eq!(csr.fwd_via_id[s], NO_VIA);
        assert_eq!(csr.levels[i100 as usize], 10);
        assert_eq!(csr.levels[i200 as usize], 20);
    }

    #[test]
    fn shortcut_via_id_resolved_to_local_idx() {
        let nodes = vec![
            encode_node_v2(1, 0.0, 0.0, 0, 0),
            encode_node_v2(2, 0.001, 0.0, 1, 0),
            encode_node_v2(3, 0.002, 0.0, 2, 0),
        ];
        let edges = vec![
            encode_edge_v2(1, 2, 0.001, 0.0, 100.0, 0),
            encode_edge_v2(2, 3, 0.002, 0.0, 100.0, 0),
            encode_edge_v2(1, 3, 0.002, 0.0, 200.0, 2),
        ];
        let tile = make_tile_v2(nodes, edges);
        let csr = build_csr(&[tile]);
        let i1 = *csr.id_to_idx.get(&1).unwrap();
        let i2 = *csr.id_to_idx.get(&2).unwrap();
        let i3 = *csr.id_to_idx.get(&3).unwrap();
        let s = csr.fwd_offsets[i1 as usize] as usize;
        let e = csr.fwd_offsets[(i1 as usize) + 1] as usize;
        assert_eq!(e - s, 2);
        let mut found_shortcut = false;
        for k in s..e {
            if csr.fwd_to[k] == i3 {
                assert_eq!(csr.fwd_via_id[k], i2);
                assert!((csr.fwd_cost[k] - 200.0).abs() < 0.01);
                found_shortcut = true;
            }
        }
        assert!(found_shortcut);
    }
}
