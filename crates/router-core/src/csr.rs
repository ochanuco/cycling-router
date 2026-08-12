//! CH-mode ephemeral CSR built from tile binary buffers in WASM context.
//!
//! Port of `lib/cycling/ch_csr.js` with the same data layout: Uint32 offsets,
//! plus Uint32 to / Float32 cost / Uint32 viaId per direction. Nodes carry
//! ids/lons/lats/levels/cores as parallel typed arrays.
//!
//! Memory profile: pre-sized CSR allocations only. The wasm entry path can
//! stream JS tile buffers through one reusable scratch Vec to avoid holding a
//! second copy of every input tile.

use std::collections::{hash_map::Entry, HashMap};
use std::hash::{BuildHasherDefault, Hasher};

pub const HEADER_BYTES: usize = 16;
pub const NODE_BYTES_V1: usize = 16;
pub const NODE_BYTES_V2: usize = 20;
pub const EDGE_BYTES_V1: usize = 28;
pub const EDGE_BYTES_V2: usize = 40;
pub const MAGIC: u32 = 0x45444952; // "RIDE"
const CORE_BIT_V2: u32 = 1 << 31;

pub const NO_VIA: u32 = u32::MAX;
pub const UNKNOWN_LEVEL: u32 = CORE_BIT_V2 - 1;
const LEVEL_MASK: u32 = CORE_BIT_V2 - 1;
const IDX_FILLED_BIT: u32 = 1 << 31;
const IDX_MASK: u32 = IDX_FILLED_BIT - 1;

#[derive(Clone, Copy)]
struct TileHeader {
    version: u8,
    node_count: u32,
    edge_count: u32,
}

#[derive(Clone, Copy)]
struct TileMeta {
    buf_index: usize,
    header: TileHeader,
    edge_offset: usize,
}

#[derive(Default)]
struct U64Hasher {
    state: u64,
}

#[inline(always)]
fn mix_u64(mut x: u64) -> u64 {
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

impl Hasher for U64Hasher {
    #[inline(always)]
    fn finish(&self) -> u64 {
        self.state
    }

    #[inline(always)]
    fn write_u64(&mut self, i: u64) {
        self.state = mix_u64(i);
    }

    #[inline(always)]
    fn write(&mut self, bytes: &[u8]) {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        for &b in bytes {
            h ^= b as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        self.state = mix_u64(h);
    }
}

type IdMap = HashMap<u64, u32, BuildHasherDefault<U64Hasher>>;

trait TileSource {
    fn tile_count(&self) -> usize;

    fn with_tile<R, F>(&mut self, index: usize, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R;
}

#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
struct BorrowedTileSource<'a> {
    buffers: &'a [Vec<u8>],
}

impl TileSource for BorrowedTileSource<'_> {
    #[inline]
    fn tile_count(&self) -> usize {
        self.buffers.len()
    }

    #[inline]
    fn with_tile<R, F>(&mut self, index: usize, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(&self.buffers[index])
    }
}

#[cfg(target_arch = "wasm32")]
struct Uint8ArrayTileSource<'a> {
    arrays: &'a [js_sys::Uint8Array],
    scratch: Vec<u8>,
}

#[cfg(target_arch = "wasm32")]
impl TileSource for Uint8ArrayTileSource<'_> {
    #[inline]
    fn tile_count(&self) -> usize {
        self.arrays.len()
    }

    #[inline]
    fn with_tile<R, F>(&mut self, index: usize, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        let src = &self.arrays[index];
        self.scratch.resize(src.length() as usize, 0);
        src.copy_to(&mut self.scratch);
        f(&self.scratch)
    }
}

fn read_header(buf: &[u8]) -> Option<TileHeader> {
    if buf.len() < HEADER_BYTES {
        return None;
    }
    let magic = read_u32_le(buf, 0);
    if magic != MAGIC {
        return None;
    }
    let version = buf[4];
    if version != 1 && version != 2 {
        return None;
    }
    let node_count = read_u32_le(buf, 8);
    let edge_count = read_u32_le(buf, 12);
    // 宣言サイズと実バッファ長が一致するか検証。短いバッファのまま固定長
    // スライス読みすると panic するため、不一致なら None でスキップ
    // (CodeRabbit PR #87 指摘)。
    let nb = if version == 2 {
        NODE_BYTES_V2
    } else {
        NODE_BYTES_V1
    };
    let eb = if version == 2 {
        EDGE_BYTES_V2
    } else {
        EDGE_BYTES_V1
    };
    let expected = HEADER_BYTES
        .checked_add((node_count as usize).checked_mul(nb)?)?
        .checked_add((edge_count as usize).checked_mul(eb)?)?;
    if buf.len() < expected {
        return None;
    }
    Some(TileHeader {
        version,
        node_count,
        edge_count,
    })
}

#[inline(always)]
fn read_u64_le(buf: &[u8], off: usize) -> u64 {
    debug_assert!(off + 8 <= buf.len());
    // read_header() validates tile section sizes before these hot loops run.
    unsafe { u64::from_le_bytes((buf.as_ptr().add(off) as *const [u8; 8]).read_unaligned()) }
}

#[inline(always)]
fn read_f64_le(buf: &[u8], off: usize) -> f64 {
    f64::from_bits(read_u64_le(buf, off))
}

#[inline(always)]
fn read_f32_le(buf: &[u8], off: usize) -> f32 {
    f32::from_bits(read_u32_le(buf, off))
}

#[cfg(target_arch = "wasm32")]
#[inline(always)]
fn read_f32_pair_le(buf: &[u8], off: usize) -> (f32, f32) {
    use core::arch::wasm32::{f32x4_extract_lane, v128_load64_zero};

    debug_assert!(off + 8 <= buf.len());
    // wasm loads are explicitly 1-aligned here; the tile format only
    // guarantees byte alignment inside Uint8Array buffers.
    let v = unsafe { v128_load64_zero(buf.as_ptr().add(off) as *const u64) };
    (f32x4_extract_lane::<0>(v), f32x4_extract_lane::<1>(v))
}

#[cfg(not(target_arch = "wasm32"))]
#[inline(always)]
fn read_f32_pair_le(buf: &[u8], off: usize) -> (f32, f32) {
    let bits = read_u64_le(buf, off);
    (
        f32::from_bits(bits as u32),
        f32::from_bits((bits >> 32) as u32),
    )
}

#[inline(always)]
fn read_u32_le(buf: &[u8], off: usize) -> u32 {
    debug_assert!(off + 4 <= buf.len());
    // read_header() validates tile section sizes before these hot loops run.
    unsafe { u32::from_le_bytes((buf.as_ptr().add(off) as *const [u8; 4]).read_unaligned()) }
}

#[inline]
fn prefix_sum_offsets(offsets: &mut [u32]) {
    let mut sum = 0u32;
    for v in offsets.iter_mut().skip(1) {
        sum += *v;
        *v = sum;
    }
}

#[inline]
fn restore_tail_offsets(offsets: &mut [u32], total_edges: u32) {
    if offsets.is_empty() {
        return;
    }
    let n = offsets.len() - 1;
    for i in 1..n {
        offsets[i] = offsets[i + 1];
    }
    offsets[n] = total_edges;
}

#[inline(always)]
pub fn level_word_level(word: u32) -> u32 {
    word & LEVEL_MASK
}

#[inline(always)]
pub fn level_word_is_core(word: u32) -> bool {
    (word & CORE_BIT_V2) != 0
}

#[inline(always)]
pub fn level_word_is_unknown(word: u32) -> bool {
    level_word_level(word) == UNKNOWN_LEVEL
}

#[inline(always)]
fn idx_from_map(value: u32) -> u32 {
    value & IDX_MASK
}

pub struct Csr {
    pub node_count: u32,
    pub edge_count: u32,
    pub ids: Vec<u64>,
    pub lons: Vec<f32>,
    pub lats: Vec<f32>,
    /// Lower 31 bits are CH level; high bit is the core flag.
    pub levels: Vec<u32>,
    pub fwd_offsets: Vec<u32>,
    pub fwd_to: Vec<u32>,
    pub fwd_cost: Vec<f32>,
    pub fwd_via_id: Vec<u32>,
    pub rev_offsets: Vec<u32>,
    pub rev_from: Vec<u32>,
    pub rev_cost: Vec<f32>,
}

impl Csr {
    pub fn memory_bytes(&self) -> usize {
        self.ids.len() * 8
            + self.lons.len() * 4
            + self.lats.len() * 4
            + self.levels.len() * 4
            + self.fwd_offsets.len() * 4
            + self.fwd_to.len() * 4
            + self.fwd_cost.len() * 4
            + self.fwd_via_id.len() * 4
            + self.rev_offsets.len() * 4
            + self.rev_from.len() * 4
            + self.rev_cost.len() * 4
    }

    pub fn release_ids(&mut self) {
        self.ids = Vec::new();
    }
}

/// Build CSR from a slice of tile binary buffers.
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
pub fn build_csr(buffers: &[Vec<u8>]) -> Csr {
    let mut source = BorrowedTileSource { buffers };
    build_csr_from_source(&mut source)
}

/// Build CSR from JS Uint8Array handles without retaining a wasm-side copy of
/// every tile. Each pass copies one tile at a time into a reusable scratch Vec.
#[cfg(target_arch = "wasm32")]
pub fn build_csr_from_uint8_arrays(arrays: &[js_sys::Uint8Array]) -> Csr {
    let mut source = Uint8ArrayTileSource {
        arrays,
        scratch: Vec::new(),
    };
    build_csr_from_source(&mut source)
}

fn build_csr_from_source<S: TileSource>(source: &mut S) -> Csr {
    // Phase 0: scan headers for upper bound sizing and fixed section offsets.
    let mut metas: Vec<TileMeta> = Vec::with_capacity(source.tile_count());
    let mut node_upper: usize = 0;
    for buf_index in 0..source.tile_count() {
        source.with_tile(buf_index, |buf| {
            if let Some(header) = read_header(buf) {
                node_upper += header.node_count as usize;
                let nb = if header.version == 2 {
                    NODE_BYTES_V2
                } else {
                    NODE_BYTES_V1
                };
                let edge_offset = HEADER_BYTES + (header.node_count as usize) * nb;
                metas.push(TileMeta {
                    buf_index,
                    header,
                    edge_offset,
                });
            }
        });
    }

    // Phase A: register unique node ids. Cross-tile target / via node の追加
    // 登録は意図的に行わない (corridor 境界 edges/shortcuts は idx undefined
    // → スキップ)。Node payload arrays are allocated after this pass with the
    // exact unique count instead of the summed tile-node upper bound.
    let mut node_count: u32 = 0;
    let mut id_to_idx: IdMap =
        HashMap::with_capacity_and_hasher(node_upper, BuildHasherDefault::<U64Hasher>::default());

    for meta in &metas {
        source.with_tile(meta.buf_index, |buf| {
            let h = meta.header;
            let nb = if h.version == 2 {
                NODE_BYTES_V2
            } else {
                NODE_BYTES_V1
            };
            let mut off = HEADER_BYTES;
            for _ in 0..h.node_count {
                let id = read_f64_le(buf, off) as u64;
                if let Entry::Vacant(e) = id_to_idx.entry(id) {
                    let idx = node_count;
                    debug_assert!(idx < IDX_FILLED_BIT);
                    e.insert(idx);
                    node_count += 1;
                }
                off += nb;
            }
        });
    }

    // Phase B: fill exact-sized node arrays, preserving first occurrence wins.
    let nc = node_count as usize;
    let mut ids: Vec<u64> = vec![0; nc];
    let mut lons: Vec<f32> = vec![0.0; nc];
    let mut lats: Vec<f32> = vec![0.0; nc];
    let mut levels: Vec<u32> = vec![UNKNOWN_LEVEL; nc];
    for meta in &metas {
        source.with_tile(meta.buf_index, |buf| {
            let h = meta.header;
            let mut off = HEADER_BYTES;
            if h.version == 2 {
                for _ in 0..h.node_count {
                    let id = read_f64_le(buf, off) as u64;
                    let (lon, lat) = read_f32_pair_le(buf, off + 8);
                    let word = read_u32_le(buf, off + 16);
                    if let Some(raw_idx) = id_to_idx.get_mut(&id) {
                        if (*raw_idx & IDX_FILLED_BIT) == 0 {
                            let i = idx_from_map(*raw_idx) as usize;
                            *raw_idx |= IDX_FILLED_BIT;
                            ids[i] = id;
                            lons[i] = lon;
                            lats[i] = lat;
                            levels[i] = word;
                        }
                    }
                    off += NODE_BYTES_V2;
                }
            } else {
                for _ in 0..h.node_count {
                    let id = read_f64_le(buf, off) as u64;
                    let (lon, lat) = read_f32_pair_le(buf, off + 8);
                    if let Some(raw_idx) = id_to_idx.get_mut(&id) {
                        if (*raw_idx & IDX_FILLED_BIT) == 0 {
                            let i = idx_from_map(*raw_idx) as usize;
                            *raw_idx |= IDX_FILLED_BIT;
                            ids[i] = id;
                            lons[i] = lon;
                            lats[i] = lat;
                        }
                    }
                    off += NODE_BYTES_V1;
                }
            }
        });
    }

    // Phase C: count fwd/rev degrees directly into offset tails.
    let mut fwd_offsets: Vec<u32> = vec![0; nc + 1];
    let mut rev_offsets: Vec<u32> = vec![0; nc + 1];
    let mut total_edges: u32 = 0;
    for meta in &metas {
        source.with_tile(meta.buf_index, |buf| {
            let h = meta.header;
            let eb = if h.version == 2 {
                EDGE_BYTES_V2
            } else {
                EDGE_BYTES_V1
            };
            let mut off = meta.edge_offset;
            for _ in 0..h.edge_count {
                let from = read_f64_le(buf, off) as u64;
                let to = read_f64_le(buf, off + 8) as u64;
                if let (Some(&f_raw), Some(&t_raw)) = (id_to_idx.get(&from), id_to_idx.get(&to)) {
                    let f_idx = idx_from_map(f_raw);
                    let t_idx = idx_from_map(t_raw);
                    fwd_offsets[f_idx as usize + 1] += 1;
                    rev_offsets[t_idx as usize + 1] += 1;
                    total_edges += 1;
                }
                off += eb;
            }
        });
    }

    // Phase D: prefix sums.
    prefix_sum_offsets(&mut fwd_offsets);
    prefix_sum_offsets(&mut rev_offsets);

    // Phase E: fill CSR. Iterate edge records backwards and decrement offset
    // tails in-place, preserving the original per-node edge order without
    // allocating cursor arrays.
    let te = total_edges as usize;
    let mut fwd_to: Vec<u32> = vec![0; te];
    let mut fwd_cost: Vec<f32> = vec![0.0; te];
    let mut fwd_via_id: Vec<u32> = vec![NO_VIA; te];
    let mut rev_from: Vec<u32> = vec![0; te];
    let mut rev_cost: Vec<f32> = vec![0.0; te];
    for meta in metas.iter().rev() {
        source.with_tile(meta.buf_index, |buf| {
            let h = meta.header;
            let eb = if h.version == 2 {
                EDGE_BYTES_V2
            } else {
                EDGE_BYTES_V1
            };
            let base = meta.edge_offset;
            for edge_i in (0..h.edge_count as usize).rev() {
                let off = base + edge_i * eb;
                let id = read_f64_le(buf, off) as u64;
                let to = read_f64_le(buf, off + 8) as u64;
                let (f_idx, t_idx) = match (id_to_idx.get(&id), id_to_idx.get(&to)) {
                    (Some(&f_raw), Some(&t_raw)) => (idx_from_map(f_raw), idx_from_map(t_raw)),
                    _ => continue,
                };
                let cost = read_f32_le(buf, off + 24);
                let mut via_idx = NO_VIA;
                if h.version == 2 {
                    let via_osm = read_f64_le(buf, off + 32) as u64;
                    if via_osm != 0 {
                        if let Some(&vi_raw) = id_to_idx.get(&via_osm) {
                            via_idx = idx_from_map(vi_raw);
                        }
                    }
                }

                let fp_tail = &mut fwd_offsets[f_idx as usize + 1];
                *fp_tail -= 1;
                let fp = *fp_tail as usize;
                fwd_to[fp] = t_idx;
                fwd_cost[fp] = cost;
                fwd_via_id[fp] = via_idx;

                let rp_tail = &mut rev_offsets[t_idx as usize + 1];
                *rp_tail -= 1;
                let rp = *rp_tail as usize;
                rev_from[rp] = f_idx;
                rev_cost[rp] = cost;
            }
        });
    }
    drop(id_to_idx);
    restore_tail_offsets(&mut fwd_offsets, total_edges);
    restore_tail_offsets(&mut rev_offsets, total_edges);

    Csr {
        node_count,
        edge_count: total_edges,
        ids,
        lons,
        lats,
        levels,
        fwd_offsets,
        fwd_to,
        fwd_cost,
        fwd_via_id,
        rev_offsets,
        rev_from,
        rev_cost,
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

    fn encode_edge_v2(
        from: u64,
        to: u64,
        to_lon: f32,
        to_lat: f32,
        cost: f32,
        via: u64,
    ) -> Vec<u8> {
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

    fn idx_of(csr: &Csr, id: u64) -> u32 {
        csr.ids.iter().position(|&x| x == id).unwrap() as u32
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
        let i100 = idx_of(&csr, 100);
        let i200 = idx_of(&csr, 200);
        let s = csr.fwd_offsets[i100 as usize] as usize;
        let e = csr.fwd_offsets[(i100 as usize) + 1] as usize;
        assert_eq!(e - s, 1);
        assert_eq!(csr.fwd_to[s], i200);
        assert!((csr.fwd_cost[s] - 100.0).abs() < 0.01);
        assert_eq!(csr.fwd_via_id[s], NO_VIA);
        assert_eq!(level_word_level(csr.levels[i100 as usize]), 10);
        assert_eq!(level_word_level(csr.levels[i200 as usize]), 20);
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
        let i1 = idx_of(&csr, 1);
        let i2 = idx_of(&csr, 2);
        let i3 = idx_of(&csr, 3);
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
