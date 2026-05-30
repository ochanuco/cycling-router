//! Point-to-route distance batch compute (browser WASM).
//!
//! Port of `frontend/route_math.js` pointToRouteDistanceMeters with the same
//! equirectangular-projection-to-meters approach. Used by frontend GPX mode
//! to filter shops within N meters of the route.
//!
//! Throughput target: 5M point-segment ops in ~30ms on smartphone (vs
//! 200-500ms with the JS implementation). Tight numeric loop, zero
//! allocations in hot path.

const EARTH_RADIUS_M: f64 = 6371008.8;

#[inline]
fn project_to_meters(lon: f64, lat: f64, cos_ref_lat: f64) -> (f64, f64) {
    let x = EARTH_RADIUS_M * lon.to_radians() * cos_ref_lat;
    let y = EARTH_RADIUS_M * lat.to_radians();
    (x, y)
}

/// For each shop, compute the minimum perpendicular distance (meters) to any
/// segment of the route polyline.
///
/// - `route_lonlats`: flat Float64Array `[lon0, lat0, lon1, lat1, ...]` of
///   length 2*N (N route vertices, N-1 segments)
/// - `shop_lonlats`: flat Float64Array `[lon0, lat0, lon1, lat1, ...]` of
///   length 2*M (M shop points)
///
/// Returns a Float32 vector of length M; each entry is the minimum distance
/// in meters (or +Infinity if route has < 2 points).
///
/// Algorithm matches `frontend/route_math.js`:
///  - equirectangular projection at reference = mean(route latitudes)
///  - segment distance = distance from projected shop point to the projected
///    segment using parameterized closest-point clamped to [0, 1].
///  - returns Float32 (sub-mm precision at this scale, halves payload to JS)
pub fn route_distances(route_lonlats: &[f64], shop_lonlats: &[f64]) -> Vec<f32> {
    if shop_lonlats.len() < 2 || shop_lonlats.len() % 2 != 0 {
        return Vec::new();
    }
    let m = shop_lonlats.len() / 2;
    let mut out: Vec<f32> = vec![f32::INFINITY; m];

    if route_lonlats.len() < 4 || route_lonlats.len() % 2 != 0 {
        return out;
    }
    let n = route_lonlats.len() / 2;

    // Reference latitude = mean of route lats (matches JS meanLatitude).
    let mut lat_sum = 0.0;
    for i in 0..n {
        lat_sum += route_lonlats[i * 2 + 1];
    }
    let ref_lat = lat_sum / n as f64;
    let cos_ref_lat = ref_lat.to_radians().cos();

    // Pre-project route vertices once.
    let mut rx: Vec<f64> = Vec::with_capacity(n);
    let mut ry: Vec<f64> = Vec::with_capacity(n);
    for i in 0..n {
        let (x, y) = project_to_meters(
            route_lonlats[i * 2],
            route_lonlats[i * 2 + 1],
            cos_ref_lat,
        );
        rx.push(x);
        ry.push(y);
    }

    // For each shop, scan all (n-1) segments.
    for k in 0..m {
        let (px, py) = project_to_meters(
            shop_lonlats[k * 2],
            shop_lonlats[k * 2 + 1],
            cos_ref_lat,
        );
        let mut min_d2 = f64::INFINITY;
        for i in 1..n {
            let ax = rx[i - 1];
            let ay = ry[i - 1];
            let bx = rx[i];
            let by = ry[i];
            let abx = bx - ax;
            let aby = by - ay;
            let ab2 = abx * abx + aby * aby;
            let d2 = if ab2 == 0.0 {
                let dx = px - ax;
                let dy = py - ay;
                dx * dx + dy * dy
            } else {
                let apx = px - ax;
                let apy = py - ay;
                let mut t = (apx * abx + apy * aby) / ab2;
                if t < 0.0 {
                    t = 0.0;
                } else if t > 1.0 {
                    t = 1.0;
                }
                let cx = ax + abx * t;
                let cy = ay + aby * t;
                let dx = px - cx;
                let dy = py - cy;
                dx * dx + dy * dy
            };
            if d2 < min_d2 {
                min_d2 = d2;
            }
        }
        out[k] = min_d2.sqrt() as f32;
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn empty_shops_returns_empty() {
        let route = vec![135.0, 34.0, 135.001, 34.0];
        let shops = vec![];
        let out = route_distances(&route, &shops);
        assert_eq!(out.len(), 0);
    }

    #[test]
    fn empty_route_returns_inf_per_shop() {
        let route = vec![];
        let shops = vec![135.0, 34.0, 135.001, 34.0];
        let out = route_distances(&route, &shops);
        assert_eq!(out.len(), 2);
        assert!(out[0].is_infinite());
        assert!(out[1].is_infinite());
    }

    #[test]
    fn shop_on_route_yields_near_zero() {
        // straight east-west route from (135, 34) → (135.01, 34)
        let route = vec![135.0, 34.0, 135.01, 34.0];
        // shop exactly on the midpoint (135.005, 34.0)
        let shops = vec![135.005, 34.0];
        let out = route_distances(&route, &shops);
        assert!(out[0] < 1.0, "expected ~0, got {}", out[0]);
    }

    #[test]
    fn shop_perpendicular_offset_matches_haversine() {
        // route: (135, 34) → (135.01, 34) (about 920m east at lat 34)
        // shop: (135.005, 34.001) — should be ~111m north of midpoint
        let route = vec![135.0, 34.0, 135.01, 34.0];
        let shops = vec![135.005, 34.001];
        let out = route_distances(&route, &shops);
        // Expected ~111m (1 deg lat ≈ 111000m, 0.001 deg = 111m)
        assert!(approx_eq(out[0], 111.0, 5.0), "got {}", out[0]);
    }

    #[test]
    fn multi_segment_picks_nearest() {
        // L-shape route: (135,34) → (135.01,34) → (135.01,34.01)
        let route = vec![135.0, 34.0, 135.01, 34.0, 135.01, 34.01];
        // Shop near the corner (135.0099, 34.0001)
        let shops = vec![135.0099, 34.0001];
        let out = route_distances(&route, &shops);
        // Should be very close (~11m due to 0.0001° = ~11m)
        assert!(out[0] < 20.0, "got {}", out[0]);
    }

    #[test]
    fn many_shops_one_call() {
        let route = vec![135.0, 34.0, 135.01, 34.0];
        let shops = vec![
            135.005, 34.0,      // on route ~0
            135.005, 34.001,    // ~111m north
            135.005, 34.01,     // ~1110m north
        ];
        let out = route_distances(&route, &shops);
        assert_eq!(out.len(), 3);
        assert!(out[0] < 1.0);
        assert!(approx_eq(out[1], 111.0, 5.0));
        assert!(approx_eq(out[2], 1110.0, 50.0));
    }
}
