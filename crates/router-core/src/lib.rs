//! Cycling router core.
//!
//! CSR グラフ構築、Contraction Hierarchies クエリ、座標スナップ、
//! ルート/補給地点間の距離計算を提供する純 Rust クレート。
//!
//! JS/WASM 境界は持たない。Cloudflare Workers 向けの wasm_bindgen wrapper は
//! `router-wasm`、CH 前計算バイナリは `router-cli` にある。

pub mod astar;
pub mod chquery;
pub mod csr;
pub mod route_filter;
pub mod snap;

#[cfg(test)]
mod tests {
    use crate::astar::astar;
    use crate::{chquery, csr, route_filter, snap};
    use std::time::Instant;

    #[test]
    fn straight_chain() {
        // 3 nodes: 0 → 1 → 2 (each segment ~111m apart in lon)
        let nodes: Vec<f64> = vec![135.0, 34.0, 135.001, 34.0, 135.002, 34.0];
        let edges: Vec<f64> = vec![
            0.0, 1.0, 100.0, 1.0, 0.0, 100.0, 1.0, 2.0, 100.0, 2.0, 1.0, 100.0,
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
    fn invalid_edge_endpoints_are_ignored_not_folded_into_node_zero() {
        // f64 -> u32 は saturating cast なので、負値 / NaN / 非整数を通すと
        // 「ノード 0 への辺」に化ける。0 と 2 は本来つながっていないので、
        // これらの辺が採用されると 0 -> 2 が到達可能になってしまう。
        let nodes: Vec<f64> = vec![135.0, 34.0, 135.001, 34.0, 135.002, 34.0];
        let bad_edges: Vec<f64> = vec![
            -1.0,
            2.0,
            10.0, // from が負 -> 0 に化ける
            f64::NAN,
            2.0,
            10.0, // from が NaN -> 0 に化ける
            0.5,
            2.0,
            10.0, // from が非整数 -> 0 に化ける
            0.0,
            -1.0,
            10.0, // to が負 -> 0 に化ける
            0.0,
            2.0,
            f64::NAN, // コストが NaN
            0.0,
            2.0,
            -5.0, // コストが負
        ];
        let r = astar(&nodes, &bad_edges, 0, 2);
        assert_eq!(r, vec![f64::INFINITY], "不正な辺は採用されないこと");
    }

    #[test]
    fn unreachable() {
        let nodes: Vec<f64> = vec![135.0, 34.0, 135.001, 34.0];
        let edges: Vec<f64> = vec![];
        let r = astar(&nodes, &edges, 0, 1);
        assert_eq!(r, vec![f64::INFINITY]);
    }

    fn push_node_v2(buf: &mut Vec<u8>, id: u64, lon: f32, lat: f32, level: u32, core: u8) {
        buf.extend_from_slice(&(id as f64).to_le_bytes());
        buf.extend_from_slice(&lon.to_le_bytes());
        buf.extend_from_slice(&lat.to_le_bytes());
        let word = level | if core != 0 { 1 << 31 } else { 0 };
        buf.extend_from_slice(&word.to_le_bytes());
    }

    fn push_edge_v2(buf: &mut Vec<u8>, from: u64, to: u64, to_lon: f32, to_lat: f32, cost: f32) {
        buf.extend_from_slice(&(from as f64).to_le_bytes());
        buf.extend_from_slice(&(to as f64).to_le_bytes());
        buf.extend_from_slice(&to_lon.to_le_bytes());
        buf.extend_from_slice(&to_lat.to_le_bytes());
        buf.extend_from_slice(&cost.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
        buf.extend_from_slice(&0f64.to_le_bytes());
    }

    fn synthetic_tile(width: usize, height: usize) -> Vec<u8> {
        let node_count = width * height;
        let edge_count = (width - 1) * height + width * (height - 1);
        let mut buf = Vec::with_capacity(
            csr::HEADER_BYTES + node_count * csr::NODE_BYTES_V2 + edge_count * csr::EDGE_BYTES_V2,
        );
        buf.extend_from_slice(&csr::MAGIC.to_le_bytes());
        buf.push(2);
        buf.push(0);
        buf.extend_from_slice(&0u16.to_le_bytes());
        buf.extend_from_slice(&(node_count as u32).to_le_bytes());
        buf.extend_from_slice(&(edge_count as u32).to_le_bytes());

        let id_at = |x: usize, y: usize| (y * width + x + 1) as u64;
        for y in 0..height {
            for x in 0..width {
                let id = id_at(x, y);
                let lon = 135.0 + x as f32 * 0.0001;
                let lat = 34.0 + y as f32 * 0.0001;
                push_node_v2(&mut buf, id, lon, lat, id as u32, 0);
            }
        }
        for y in 0..height {
            for x in 0..width {
                let from = id_at(x, y);
                if x + 1 < width {
                    let to = id_at(x + 1, y);
                    push_edge_v2(
                        &mut buf,
                        from,
                        to,
                        135.0 + (x + 1) as f32 * 0.0001,
                        34.0 + y as f32 * 0.0001,
                        10.0,
                    );
                }
                if y + 1 < height {
                    let to = id_at(x, y + 1);
                    push_edge_v2(
                        &mut buf,
                        from,
                        to,
                        135.0 + x as f32 * 0.0001,
                        34.0 + (y + 1) as f32 * 0.0001,
                        10.0,
                    );
                }
            }
        }
        buf
    }

    #[test]
    #[ignore = "timing demo; run with --ignored --nocapture"]
    fn perf_timing_demo() {
        let tile = synthetic_tile(96, 96);
        let buffers = vec![tile];

        let t0 = Instant::now();
        let csr = csr::build_csr(&buffers);
        let csr_ms = t0.elapsed().as_secs_f64() * 1000.0;

        let t0 = Instant::now();
        let a = snap::snap(&csr, 135.001, 34.001).unwrap();
        let b = snap::snap(&csr, 135.008, 34.008).unwrap();
        let snap_ms = t0.elapsed().as_secs_f64() * 1000.0;

        let t0 = Instant::now();
        let ch = chquery::ch_query(&csr, a.idx, b.idx, &chquery::ChQueryOpts::default());
        let ch_ms = t0.elapsed().as_secs_f64() * 1000.0;

        let t0 = Instant::now();
        let fallback = chquery::ch_query(
            &csr,
            a.idx,
            b.idx,
            &chquery::ChQueryOpts {
                settled_cap: 300_000,
                pops_cap: 800_000,
                time_budget_ms: 10_000,
                no_level_constraint: true,
            },
        );
        let fallback_ms = t0.elapsed().as_secs_f64() * 1000.0;

        let mut route = Vec::with_capacity(512 * 2);
        for i in 0..512 {
            route.push(135.0 + i as f64 * 0.00001);
            route.push(34.0 + (i % 32) as f64 * 0.000005);
        }
        let mut shops = Vec::with_capacity(4096 * 2);
        for i in 0..4096 {
            shops.push(135.0 + (i % 128) as f64 * 0.00002);
            shops.push(34.0 + (i / 128) as f64 * 0.00002);
        }
        let t0 = Instant::now();
        let distances = route_filter::route_distances(&route, &shops);
        let route_dist_ms = t0.elapsed().as_secs_f64() * 1000.0;

        eprintln!(
            "perf_timing_demo nodes={} edges={} csr_ms={:.3} snap2_ms={:.3} ch_ms={:.3} fallback_ms={:.3} route_dist_ms={:.3} ch_dist={:.1} fb_dist={:.1} route_out={}",
            csr.node_count,
            csr.edge_count,
            csr_ms,
            snap_ms,
            ch_ms,
            fallback_ms,
            route_dist_ms,
            ch.distance,
            fallback.distance,
            distances.len()
        );
    }
}
