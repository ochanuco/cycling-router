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

#[inline(always)]
fn segment_distance2(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let abx = bx - ax;
    let aby = by - ay;
    let ab2 = abx * abx + aby * aby;
    if ab2 == 0.0 {
        let dx = px - ax;
        let dy = py - ay;
        dx * dx + dy * dy
    } else {
        let apx = px - ax;
        let apy = py - ay;
        let t = ((apx * abx + apy * aby) / ab2).clamp(0.0, 1.0);
        let cx = ax + abx * t;
        let cy = ay + aby * t;
        let dx = px - cx;
        let dy = py - cy;
        dx * dx + dy * dy
    }
}

/// Scalar reference implementation.
///
/// wasm32 では下の SIMD 版が使われるが、こちらも常にコンパイルしておく。SIMD 版の
/// 索引計算 (2 本ずつ処理する境界と末尾の端数) は目視では確かめにくいので、wasm32 の
/// テストで両者の出力を突き合わせる基準として必要になる。
#[cfg_attr(target_arch = "wasm32", allow(dead_code))]
#[inline(always)]
fn min_distance2_to_route_scalar(px: f64, py: f64, rx: &[f64], ry: &[f64]) -> f64 {
    let mut min_d2 = f64::INFINITY;
    for i in 1..rx.len() {
        let d2 = segment_distance2(px, py, rx[i - 1], ry[i - 1], rx[i], ry[i]);
        if d2 < min_d2 {
            min_d2 = d2;
        }
    }
    min_d2
}

/// SIMD (simd128) 実装。1 反復で 2 セグメントを処理する。挙動の基準は
/// `min_distance2_to_route_scalar`。
#[cfg(target_arch = "wasm32")]
#[inline(always)]
fn min_distance2_to_route_simd(px: f64, py: f64, rx: &[f64], ry: &[f64]) -> f64 {
    use core::arch::wasm32::{
        f64x2, f64x2_add, f64x2_div, f64x2_extract_lane, f64x2_max, f64x2_min, f64x2_mul,
        f64x2_splat, f64x2_sub,
    };

    let mut min_d2 = f64::INFINITY;
    let pxv = f64x2_splat(px);
    let pyv = f64x2_splat(py);
    let zero = f64x2_splat(0.0);
    let one = f64x2_splat(1.0);

    let mut i = 1usize;
    while i + 1 < rx.len() {
        let ax0 = rx[i - 1];
        let ay0 = ry[i - 1];
        let bx0 = rx[i];
        let by0 = ry[i];
        let ax1 = rx[i];
        let ay1 = ry[i];
        let bx1 = rx[i + 1];
        let by1 = ry[i + 1];

        let abx0 = bx0 - ax0;
        let aby0 = by0 - ay0;
        let abx1 = bx1 - ax1;
        let aby1 = by1 - ay1;
        if abx0 * abx0 + aby0 * aby0 == 0.0 || abx1 * abx1 + aby1 * aby1 == 0.0 {
            let d20 = segment_distance2(px, py, ax0, ay0, bx0, by0);
            if d20 < min_d2 {
                min_d2 = d20;
            }
            let d21 = segment_distance2(px, py, ax1, ay1, bx1, by1);
            if d21 < min_d2 {
                min_d2 = d21;
            }
            i += 2;
            continue;
        }

        let ax = f64x2(ax0, ax1);
        let ay = f64x2(ay0, ay1);
        let abx = f64x2(abx0, abx1);
        let aby = f64x2(aby0, aby1);
        let ab2 = f64x2_add(f64x2_mul(abx, abx), f64x2_mul(aby, aby));
        let apx = f64x2_sub(pxv, ax);
        let apy = f64x2_sub(pyv, ay);
        let dot = f64x2_add(f64x2_mul(apx, abx), f64x2_mul(apy, aby));
        let t = f64x2_min(one, f64x2_max(zero, f64x2_div(dot, ab2)));
        let cx = f64x2_add(ax, f64x2_mul(abx, t));
        let cy = f64x2_add(ay, f64x2_mul(aby, t));
        let dx = f64x2_sub(pxv, cx);
        let dy = f64x2_sub(pyv, cy);
        let d2 = f64x2_add(f64x2_mul(dx, dx), f64x2_mul(dy, dy));

        let d20 = f64x2_extract_lane::<0>(d2);
        if d20 < min_d2 {
            min_d2 = d20;
        }
        let d21 = f64x2_extract_lane::<1>(d2);
        if d21 < min_d2 {
            min_d2 = d21;
        }
        i += 2;
    }

    while i < rx.len() {
        let d2 = segment_distance2(px, py, rx[i - 1], ry[i - 1], rx[i], ry[i]);
        if d2 < min_d2 {
            min_d2 = d2;
        }
        i += 1;
    }
    min_d2
}

#[cfg(not(target_arch = "wasm32"))]
#[inline(always)]
fn min_distance2_to_route(px: f64, py: f64, rx: &[f64], ry: &[f64]) -> f64 {
    min_distance2_to_route_scalar(px, py, rx, ry)
}

#[cfg(target_arch = "wasm32")]
#[inline(always)]
fn min_distance2_to_route(px: f64, py: f64, rx: &[f64], ry: &[f64]) -> f64 {
    min_distance2_to_route_simd(px, py, rx, ry)
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
    if shop_lonlats.len() < 2 || !shop_lonlats.len().is_multiple_of(2) {
        return Vec::new();
    }
    let m = shop_lonlats.len() / 2;
    let mut out: Vec<f32> = vec![f32::INFINITY; m];

    if route_lonlats.len() < 4 || !route_lonlats.len().is_multiple_of(2) {
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
        let (x, y) = project_to_meters(route_lonlats[i * 2], route_lonlats[i * 2 + 1], cos_ref_lat);
        rx.push(x);
        ry.push(y);
    }

    // For each shop, scan all (n-1) segments.
    for k in 0..m {
        let (px, py) = project_to_meters(shop_lonlats[k * 2], shop_lonlats[k * 2 + 1], cos_ref_lat);
        let min_d2 = min_distance2_to_route(px, py, &rx, &ry);
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
            135.005, 34.0, // on route ~0
            135.005, 34.001, // ~111m north
            135.005, 34.01, // ~1110m north
        ];
        let out = route_distances(&route, &shops);
        assert_eq!(out.len(), 3);
        assert!(out[0] < 1.0);
        assert!(approx_eq(out[1], 111.0, 5.0));
        assert!(approx_eq(out[2], 1110.0, 50.0));
    }
}

/// SIMD 実装とスカラー実装の差分テスト。
///
/// `min_distance2_to_route_simd` は 1 反復で 2 セグメントを畳むため、
/// 索引の刻み方・末尾の端数・退化セグメント (長さ 0) のときのスカラー
/// 退避経路が読みでは確かめにくい。ここでスカラー実装を基準に突き合わせる。
///
/// wasm32 でしかコンパイルされないので `wasm-pack test --node` で走らせる
/// (CI の "Test (wasm32 SIMD)" ステップ)。
#[cfg(all(test, target_arch = "wasm32"))]
mod simd_tests {
    use super::*;
    use wasm_bindgen_test::wasm_bindgen_test;

    /// 決定的な擬似乱数 (テストを再現可能にするため rand は入れない)。
    struct Lcg(u64);

    impl Lcg {
        fn next_unit(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
        }

        /// -50000.0 ..= 50000.0 (投影後の平面座標を想定したスケール)
        fn next_coord(&mut self) -> f64 {
            self.next_unit() * 100_000.0 - 50_000.0
        }
    }

    fn assert_same(label: &str, px: f64, py: f64, rx: &[f64], ry: &[f64]) {
        let simd = min_distance2_to_route_simd(px, py, rx, ry);
        let scalar = min_distance2_to_route_scalar(px, py, rx, ry);

        if simd.is_infinite() && scalar.is_infinite() {
            return;
        }
        let diff = (simd - scalar).abs();
        let tol = scalar.abs() * 1e-12 + 1e-6;
        assert!(
            diff <= tol,
            "{label}: simd={simd} scalar={scalar} diff={diff} (n={})",
            rx.len()
        );
    }

    #[wasm_bindgen_test]
    fn simdはセグメント数の偶奇によらずスカラーと一致する() {
        let mut rng = Lcg(0x5EED_1234);
        // 頂点 2..=17 = セグメント 1..=16。偶数本・奇数本の両方を通す。
        for n in 2..=17usize {
            let rx: Vec<f64> = (0..n).map(|_| rng.next_coord()).collect();
            let ry: Vec<f64> = (0..n).map(|_| rng.next_coord()).collect();
            for k in 0..8 {
                let px = rng.next_coord();
                let py = rng.next_coord();
                assert_same(&format!("n={n} k={k}"), px, py, &rx, &ry);
            }
        }
    }

    #[wasm_bindgen_test]
    fn 退化セグメントがどの位置にあってもスカラーと一致する() {
        let mut rng = Lcg(0xC0FF_EE01);
        // 長さ 0 のセグメントは SIMD 側で除算を避けるためスカラーへ退避する。
        // その分岐は 2 本ペアの片方だけが退化しているときにも入るので、
        // 退化位置を全パターン試す。
        for n in 3..=12usize {
            for dup_at in 1..n {
                let mut rx: Vec<f64> = (0..n).map(|_| rng.next_coord()).collect();
                let mut ry: Vec<f64> = (0..n).map(|_| rng.next_coord()).collect();
                rx[dup_at] = rx[dup_at - 1];
                ry[dup_at] = ry[dup_at - 1];
                let px = rng.next_coord();
                let py = rng.next_coord();
                assert_same(&format!("n={n} dup_at={dup_at}"), px, py, &rx, &ry);
            }
        }
    }

    #[wasm_bindgen_test]
    fn 全頂点が同一でもスカラーと一致する() {
        for n in 2..=9usize {
            let rx = vec![100.0; n];
            let ry = vec![-250.0; n];
            assert_same(&format!("all-dup n={n}"), 0.0, 0.0, &rx, &ry);
        }
    }

    #[wasm_bindgen_test]
    fn 頂点が1点以下なら無限大を返す() {
        assert!(min_distance2_to_route_simd(0.0, 0.0, &[], &[]).is_infinite());
        assert!(min_distance2_to_route_simd(0.0, 0.0, &[1.0], &[2.0]).is_infinite());
    }

    #[wasm_bindgen_test]
    fn 点がセグメント上にあるとほぼ0になる() {
        // (0,0) - (100,0) の中点
        let rx = vec![0.0, 100.0, 100.0];
        let ry = vec![0.0, 0.0, 50.0];
        let d2 = min_distance2_to_route_simd(50.0, 0.0, &rx, &ry);
        assert!(d2 < 1e-9, "got {d2}");
    }
}
