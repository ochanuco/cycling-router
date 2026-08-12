//! O(N) nearest-node snap on CSR (port of `lib/cycling/snap_csr.js`).
//!
//! Equirectangular comparison for the inner loop (cheap, monotone-ordered
//! with true distance), then haversine for the final reported distance.

use crate::csr::Csr;

const EARTH_R: f64 = 6378137.0;

#[inline]
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

#[inline]
pub fn snap(csr: &Csr, lon: f64, lat: f64) -> Option<SnapResult> {
    if csr.node_count == 0 {
        return None;
    }
    let n = csr.node_count as usize;
    // release_ids() を呼んだ後の Csr では ids が空になる。その状態で
    // get_unchecked すると UB なので、長さを実際に確認して足りなければ
    // スナップ不能として None を返す (0 などで埋めると誤った node id が
    // 呼び出し側に流れる)。
    if n > csr.lons.len() || n > csr.lats.len() || n > csr.ids.len() {
        return None;
    }
    let cos_lat = (lat * std::f64::consts::PI / 180.0).cos();
    let mut best_idx = u32::MAX;
    let mut best_sq: f64 = f64::INFINITY;
    // get_unchecked の安全性根拠: 直前の長さ検査で n <= 各配列長 が保証されている。
    debug_assert!(n <= csr.lons.len() && n <= csr.lats.len() && n <= csr.ids.len());
    let mut i = 0usize;
    while i < n {
        let ln = unsafe { *csr.lons.get_unchecked(i) } as f64;
        if ln.is_nan() {
            i += 1;
            continue;
        }
        let la = unsafe { *csr.lats.get_unchecked(i) } as f64;
        let dlon = (ln - lon) * cos_lat;
        let dlat = la - lat;
        let sq = dlon * dlon + dlat * dlat;
        if sq < best_sq {
            best_sq = sq;
            best_idx = i as u32;
        }
        i += 1;
    }
    if best_idx == u32::MAX {
        return None;
    }
    let i = best_idx as usize;
    let id = unsafe { *csr.ids.get_unchecked(i) };
    let d = haversine_m(
        lon,
        lat,
        unsafe { *csr.lons.get_unchecked(i) } as f64,
        unsafe { *csr.lats.get_unchecked(i) } as f64,
    );
    Some(SnapResult { idx: best_idx, id, distance_m: d })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::build_csr;

    fn enc_node(id: u64, lon: f32, lat: f32) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(id as f64).to_le_bytes());
        v.extend_from_slice(&lon.to_le_bytes());
        v.extend_from_slice(&lat.to_le_bytes());
        v.extend_from_slice(&1u32.to_le_bytes());
        v
    }

    fn make_tile(nodes: Vec<Vec<u8>>) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&0x45444952u32.to_le_bytes());
        b.push(2);
        b.push(0);
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&(nodes.len() as u32).to_le_bytes());
        b.extend_from_slice(&0u32.to_le_bytes());
        for n in nodes {
            b.extend(n);
        }
        b
    }

    #[test]
    fn snaps_to_nearest_node() {
        let csr = build_csr(&[make_tile(vec![
            enc_node(10, 135.0, 34.0),
            enc_node(20, 135.01, 34.0),
        ])]);
        let r = snap(&csr, 135.009, 34.0).expect("should snap");
        assert_eq!(r.id, 20);
        assert!(r.distance_m < 200.0, "distance_m = {}", r.distance_m);
    }

    #[test]
    fn empty_csr_returns_none() {
        let csr = build_csr(&[make_tile(vec![])]);
        assert!(snap(&csr, 135.0, 34.0).is_none());
    }

    #[test]
    fn after_release_ids_returns_none_instead_of_reading_out_of_bounds() {
        // release_ids() は ids を空にする。長さ検査が無いと get_unchecked が
        // 範囲外を読む (UB)。0 などで埋めず None を返すこと。
        let mut csr = build_csr(&[make_tile(vec![
            enc_node(10, 135.0, 34.0),
            enc_node(20, 135.01, 34.0),
        ])]);
        assert!(snap(&csr, 135.0, 34.0).is_some());
        csr.release_ids();
        assert!(snap(&csr, 135.0, 34.0).is_none());
    }
}
