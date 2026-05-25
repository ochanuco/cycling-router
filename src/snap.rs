//! O(N) nearest-node snap on CSR (port of `lib/cycling/snap_csr.js`).
//!
//! Equirectangular comparison for the inner loop (cheap, monotone-ordered
//! with true distance), then haversine for the final reported distance.

use crate::csr::Csr;

const EARTH_R: f64 = 6378137.0;

pub fn haversine_m(a_lon: f64, a_lat: f64, b_lon: f64, b_lat: f64) -> f64 {
    let to_rad = std::f64::consts::PI / 180.0;
    let d_lat = (b_lat - a_lat) * to_rad;
    let d_lon = (b_lon - a_lon) * to_rad;
    let lat1 = a_lat * to_rad;
    let lat2 = b_lat * to_rad;
    let s = (d_lat * 0.5).sin().powi(2) + lat1.cos() * lat2.cos() * (d_lon * 0.5).sin().powi(2);
    2.0 * EARTH_R * s.sqrt().asin()
}

pub struct SnapResult {
    pub idx: u32,
    pub id: u64,
    pub distance_m: f64,
}

pub fn snap(csr: &Csr, lon: f64, lat: f64) -> Option<SnapResult> {
    if csr.node_count == 0 {
        return None;
    }
    let cos_lat = (lat * std::f64::consts::PI / 180.0).cos();
    let mut best_idx: i64 = -1;
    let mut best_sq: f64 = f64::INFINITY;
    let n = csr.node_count as usize;
    for i in 0..n {
        let ln = csr.lons[i] as f64;
        if ln.is_nan() {
            continue;
        }
        let la = csr.lats[i] as f64;
        let dlon = (ln - lon) * cos_lat;
        let dlat = la - lat;
        let sq = dlon * dlon + dlat * dlat;
        if sq < best_sq {
            best_sq = sq;
            best_idx = i as i64;
        }
    }
    if best_idx < 0 {
        return None;
    }
    let i = best_idx as usize;
    let id = csr.ids[i];
    let d = haversine_m(lon, lat, csr.lons[i] as f64, csr.lats[i] as f64);
    Some(SnapResult { idx: best_idx as u32, id, distance_m: d })
}
