//! Native CH preprocessor for ride-oasis cycling graph.
//!
//! Reads `<dir>/nodes.ndjson` and `<dir>/edges.ndjson` (the output of
//! `scripts/cycling_build_graph.js`) and writes:
//!   - `<dir>/ch_levels.ndjson`  one `{"id":N,"level":L,"core":0|1}` per node
//!   - `<dir>/ch_edges.ndjson`   originals + shortcuts:
//!     `{"from":U,"to":W,"cost":C,"via":V|null}`
//!
//! The format is forward-compatible with the previous JS implementation
//! (`scripts/cycling_ch_build.js`); the new `core` field indicates whether
//! a node is in the uncontracted core (top-fraction skipped + degree-skipped).
//! `scripts/cycling_ch_tile_split.js` treats missing `core` as 0, so old
//! ch_levels.ndjson can still be consumed.
//!
//! Algorithm: bounded-hop witness search + degree-based ordering.
//! Per-iteration shortcut count is bounded by a Pareto-style "skip if shortcut
//! cost >= existing witness path" rule. Naive but fast enough for Kansai-scale
//! when implemented natively (vs the GC-heavy JS version that OOMs at 12 GB).

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

const HOP_LIMIT: u32 = 5;

#[derive(Clone, Copy)]
struct Edge {
    other: u32, // for fwd: target; for rev: source
    cost: f32,
    via: i32, // -1 = original, else dense index of via node
}

struct Graph {
    n: usize,
    ids: Vec<u64>, // dense index -> original ID
    fwd: Vec<Vec<Edge>>,
    rev: Vec<Vec<Edge>>,
    contracted: Vec<bool>,
}

fn load_nodes_ndjson(
    path: &Path,
) -> std::io::Result<(Vec<u64>, std::collections::HashMap<u64, u32>)> {
    let f = File::open(path)?;
    let r = BufReader::with_capacity(8 * 1024 * 1024, f);
    let mut ids = Vec::new();
    let mut id_to_idx = std::collections::HashMap::new();
    for line in r.lines() {
        let line = line?;
        if line.is_empty() {
            continue;
        }
        // Manual parse: nodes look like {"id":123,"lon":135.5,"lat":34.7}
        // Use a tiny extractor to avoid a serde dep for now.
        // 必須フィールドは fail-fast。0 fallback だと壊れた入力をそのまま
        // 前処理して ch_levels.ndjson / ch_edges.ndjson に不正データを書き
        // 出すリスクがあるため、parse 失敗は line 内容付きで panic させる。
        let id = parse_required::<u64>(&line, "\"id\"");
        // 座標は CH 前計算では使わない (次数だけで順序を決める) が、欠損・不正な
        // 行をここで弾くために検証は通す。座標のないノードはタイル生成側で問題に
        // なるので、前処理を通してしまわない。parse_required は構文しか見ないため
        // "NaN" は素通りする。NaN 座標のノードは route_ch 側で経路点から除外され、
        // distance と返却座標列が食い違う原因になるので、ここで弾く。
        let lon = parse_required::<f64>(&line, "\"lon\"");
        let lat = parse_required::<f64>(&line, "\"lat\"");
        if !lon.is_finite() || !lat.is_finite() {
            panic!("ch-preprocess: lon/lat must be finite: {line}");
        }
        if id_to_idx.contains_key(&id) {
            continue;
        }
        let idx = ids.len() as u32;
        id_to_idx.insert(id, idx);
        ids.push(id);
    }
    Ok((ids, id_to_idx))
}

/// Required-field parser: extract + parse, panicking with the offending line
/// content on failure. Used for id/lon/lat/from/to/cost_m where a 0 fallback
/// would silently produce corrupt CH output.
fn parse_required<T: std::str::FromStr>(line: &str, key: &str) -> T
where
    <T as std::str::FromStr>::Err: std::fmt::Display,
{
    let raw = extract_field(line, key).unwrap_or_else(|| {
        panic!(
            "ch-preprocess: missing required field {} in line: {}",
            key, line
        );
    });
    raw.parse::<T>().unwrap_or_else(|e| {
        panic!(
            "ch-preprocess: failed to parse {} (value={:?}) in line: {} — {}",
            key, raw, line, e
        );
    })
}

/// Extracts the substring following `key":` (a number) up to `,` or `}`.
fn extract_field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let p = line.find(key)?;
    let after = &line[p + key.len()..];
    let colon = after.find(':')?;
    let mut s = after[colon + 1..].trim_start();
    // strip trailing "," or "}"
    let end = s
        .find(|c: char| c == ',' || c == '}' || c.is_whitespace())
        .unwrap_or(s.len());
    s = &s[..end];
    Some(s)
}

fn load_edges_ndjson(
    path: &Path,
    id_to_idx: &std::collections::HashMap<u64, u32>,
    graph: &mut Graph,
) -> std::io::Result<u64> {
    let f = File::open(path)?;
    let r = BufReader::with_capacity(8 * 1024 * 1024, f);
    let mut edge_count = 0u64;
    for line in r.lines() {
        let line = line?;
        if line.is_empty() {
            continue;
        }
        let from = parse_required::<u64>(&line, "\"from\"");
        let to = parse_required::<u64>(&line, "\"to\"");
        let cost = parse_required::<f32>(&line, "\"cost_m\"");
        // NaN はダイクストラの比較を壊し、負のコストは最短路の前提を崩す。
        // 他の必須フィールドと同じくその場で落とす。
        if !cost.is_finite() || cost < 0.0 {
            panic!(
                "ch-preprocess: cost_m must be finite and non-negative (value={}) in line: {}",
                cost, line
            );
        }
        // oneway: "oneway":true / false / null. We look for the literal.
        let oneway = line.contains("\"oneway\":true");
        let from_idx = match id_to_idx.get(&from) {
            Some(i) => *i,
            None => continue,
        };
        let to_idx = match id_to_idx.get(&to) {
            Some(i) => *i,
            None => continue,
        };
        graph.fwd[from_idx as usize].push(Edge {
            other: to_idx,
            cost,
            via: -1,
        });
        graph.rev[to_idx as usize].push(Edge {
            other: from_idx,
            cost,
            via: -1,
        });
        edge_count += 1;
        if !oneway {
            graph.fwd[to_idx as usize].push(Edge {
                other: from_idx,
                cost,
                via: -1,
            });
            graph.rev[from_idx as usize].push(Edge {
                other: to_idx,
                cost,
                via: -1,
            });
            edge_count += 1;
        }
    }
    Ok(edge_count)
}

fn allocate_graph(n: usize, ids: Vec<u64>) -> Graph {
    let fwd = (0..n).map(|_| Vec::new()).collect();
    let rev = (0..n).map(|_| Vec::new()).collect();
    Graph {
        n,
        ids,
        fwd,
        rev,
        contracted: vec![false; n],
    }
}

// Min-heap entry for witness search
#[derive(Clone, Copy)]
struct WSEntry {
    dist: f32,
    hops: u32,
    node: u32,
}

impl Eq for WSEntry {}
impl PartialEq for WSEntry {
    fn eq(&self, other: &Self) -> bool {
        self.dist == other.dist
    }
}
impl Ord for WSEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .dist
            .partial_cmp(&self.dist)
            .unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for WSEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Multi-target bounded-hop Dijkstra from `start`, computing witness costs to
/// all `targets` (typically the out-neighbors of the node being contracted).
/// Skips `excluded` (the contracted node itself) and `contracted` nodes.
/// Returns a vector aligned with `targets`: best cost found, or +inf.
///
/// Stopping: when heap top exceeds `max_overall` (max of per-target sc costs)
/// or every target has been settled.
///
/// 引数が多いのは承知の上。`dist` / `touched` / `result` は呼び出し側で使い回す
/// スクラッチバッファで、収縮のたびに確保し直さないためにここへ渡している。
/// 構造体にまとめると所有権の取り回しが変わり、Kansai 規模の前計算で効いている
/// バッファ再利用の性質を壊しかねないため、計測なしでは触らない。
#[allow(clippy::too_many_arguments)]
fn witness_costs_multi(
    graph: &Graph,
    start: u32,
    excluded: u32,
    targets: &[u32],
    target_caps: &[f32], // per-target shortcut cost upper bound
    hop_limit: u32,
    dist: &mut [f32],
    touched: &mut Vec<u32>,
    result: &mut Vec<f32>,
) {
    result.clear();
    result.resize(targets.len(), f32::INFINITY);

    // Index targets by node for O(1) hit check during search.
    let mut target_idx_by_node: std::collections::HashMap<u32, usize> =
        std::collections::HashMap::with_capacity(targets.len());
    let mut max_cap = 0.0_f32;
    for (i, &t) in targets.iter().enumerate() {
        target_idx_by_node.insert(t, i);
        if target_caps[i] > max_cap {
            max_cap = target_caps[i];
        }
    }
    let mut remaining = targets.len();

    let mut heap: BinaryHeap<WSEntry> = BinaryHeap::new();
    dist[start as usize] = 0.0;
    touched.push(start);
    heap.push(WSEntry {
        dist: 0.0,
        hops: 0,
        node: start,
    });

    while let Some(WSEntry {
        dist: d,
        hops,
        node,
    }) = heap.pop()
    {
        if d > max_cap {
            break;
        }
        if d > dist[node as usize] {
            continue;
        }
        if let Some(&ti) = target_idx_by_node.get(&node) {
            if result[ti].is_infinite() {
                result[ti] = d;
                remaining -= 1;
                if remaining == 0 {
                    break;
                }
            }
        }
        if hops >= hop_limit {
            continue;
        }
        for e in &graph.fwd[node as usize] {
            if e.other == excluded {
                continue;
            }
            if graph.contracted[e.other as usize] {
                continue;
            }
            let nd = d + e.cost;
            if nd > max_cap {
                continue;
            }
            if nd < dist[e.other as usize] {
                if dist[e.other as usize] == f32::INFINITY {
                    touched.push(e.other);
                }
                dist[e.other as usize] = nd;
                heap.push(WSEntry {
                    dist: nd,
                    hops: hops + 1,
                    node: e.other,
                });
            }
        }
    }
}

fn reset_dist(dist: &mut [f32], touched: &mut Vec<u32>) {
    for &t in touched.iter() {
        dist[t as usize] = f32::INFINITY;
    }
    touched.clear();
}

/// Naive degree-based node ordering (low degree first). Future enhancement:
/// edge-difference with lazy priority queue updates.
///
/// 次数だけをキーにすると sort_unstable が同次数ノードの順序を保証せず、同じ
/// 入力から違う CH が出てしまう。生成したタイルと再生成物が食い違うと原因の
/// 特定が難しいので、ノード index を第 2 キーにして順序を固定する。
fn compute_order(graph: &Graph) -> Vec<u32> {
    let mut order: Vec<(u32, u32)> = (0..graph.n as u32)
        .map(|v| {
            (
                v,
                (graph.fwd[v as usize].len() + graph.rev[v as usize].len()) as u32,
            )
        })
        .collect();
    order.sort_unstable_by_key(|x| (x.1, x.0));
    order.into_iter().map(|x| x.0).collect()
}

#[derive(Clone, Copy)]
struct ShortcutRec {
    from: u32,
    to: u32,
    cost: f32,
    via: u32,
}

const MAX_ACTIVE_DEGREE: usize = 64;

fn build_ch(graph: &mut Graph, contract_fraction: f64) -> (Vec<u32>, Vec<bool>, Vec<ShortcutRec>) {
    let n = graph.n;
    let order = compute_order(graph);
    let mut levels = vec![0u32; n];
    // core[v] = !contracted[v] after build. partial CH query (CH side) では
    // core ノード同士の edge は level 比較を緩める (上下方向制約を外す)
    // 必要があるため、tile に明示 bit として書き出す。
    let mut core = vec![false; n];
    let limit = ((n as f64 * contract_fraction) as usize).min(n);

    // Pre-allocated scratch for witness searches.
    let mut dist: Vec<f32> = vec![f32::INFINITY; n];
    let mut touched: Vec<u32> = Vec::with_capacity(4096);
    let mut shortcuts: Vec<ShortcutRec> = Vec::new();
    let mut targets: Vec<u32> = Vec::with_capacity(64);
    let mut target_caps: Vec<f32> = Vec::with_capacity(64);
    let mut witness_results: Vec<f32> = Vec::with_capacity(64);

    let t0 = Instant::now();
    let mut last_report = Instant::now();
    let mut shortcut_inserts = 0u64;
    let mut witness_calls = 0u64;
    let mut skipped_degree = 0u64;

    for (lvl, &v) in order.iter().enumerate() {
        if lvl >= limit {
            levels[v as usize] = lvl as u32;
            // top-fraction (default 5%) は意図的に contract せず core 扱い。
            // query 側で core-core edge を level 制約なしに relax できるよう
            // marker を立てる。contracted=false のまま (skipped_degree と同じ)。
            core[v as usize] = true;
            continue;
        }
        levels[v as usize] = lvl as u32;

        let ins: Vec<Edge> = graph.rev[v as usize]
            .iter()
            .filter(|e| !graph.contracted[e.other as usize])
            .copied()
            .collect();
        let outs: Vec<Edge> = graph.fwd[v as usize]
            .iter()
            .filter(|e| !graph.contracted[e.other as usize])
            .copied()
            .collect();

        if ins.is_empty() || outs.is_empty() {
            graph.contracted[v as usize] = true;
            continue;
        }

        // Naive ordering is degree-based, so we should contract low-degree
        // nodes first. If active degree is huge (= node accumulated many
        // shortcuts from earlier contractions), skip contraction and leave
        // it as "core" — queries will use plain bidi Dijkstra here. This
        // avoids the classic CH explosion where every (in × out) pair adds
        // a shortcut and the graph grows to gigabytes.
        if ins.len() > MAX_ACTIVE_DEGREE || outs.len() > MAX_ACTIVE_DEGREE {
            skipped_degree += 1;
            core[v as usize] = true;
            continue;
        }

        // For each in-edge u -> v, do ONE multi-target witness search to all
        // outs. This batches the heap setup cost across N targets.
        for in_e in &ins {
            let u = in_e.other;
            // Build target list (skip u itself) with their shortcut cost caps.
            targets.clear();
            target_caps.clear();
            for out_e in &outs {
                if out_e.other == u {
                    continue;
                }
                targets.push(out_e.other);
                target_caps.push(in_e.cost + out_e.cost);
            }
            if targets.is_empty() {
                continue;
            }
            reset_dist(&mut dist, &mut touched);
            witness_costs_multi(
                graph,
                u,
                v,
                &targets,
                &target_caps,
                HOP_LIMIT,
                &mut dist,
                &mut touched,
                &mut witness_results,
            );
            witness_calls += 1;
            for (ti, &w) in targets.iter().enumerate() {
                let sc_cost = target_caps[ti];
                if witness_results[ti] <= sc_cost {
                    continue;
                }
                graph.fwd[u as usize].push(Edge {
                    other: w,
                    cost: sc_cost,
                    via: v as i32,
                });
                graph.rev[w as usize].push(Edge {
                    other: u,
                    cost: sc_cost,
                    via: v as i32,
                });
                shortcuts.push(ShortcutRec {
                    from: u,
                    to: w,
                    cost: sc_cost,
                    via: v,
                });
                shortcut_inserts += 1;
            }
        }

        graph.contracted[v as usize] = true;

        if last_report.elapsed().as_secs() >= 5 {
            let pct = (lvl as f64 / n as f64) * 100.0;
            eprintln!(
                "  contraction: {:>10}/{} ({:.1}%) shortcuts={} witnesses={} elapsed={:.1}s",
                lvl + 1,
                n,
                pct,
                shortcut_inserts,
                witness_calls,
                t0.elapsed().as_secs_f64()
            );
            last_report = Instant::now();
        }
    }
    let core_count = core.iter().filter(|x| **x).count();
    eprintln!(
        "  contraction done: shortcuts={} witnesses={} skipped_high_degree={} core={} elapsed={:.1}s (limit={}/{})",
        shortcut_inserts, witness_calls, skipped_degree, core_count, t0.elapsed().as_secs_f64(), limit, n
    );
    (levels, core, shortcuts)
}

fn write_levels(path: &Path, ids: &[u64], levels: &[u32], core: &[bool]) -> std::io::Result<()> {
    let f = File::create(path)?;
    let mut w = BufWriter::with_capacity(8 * 1024 * 1024, f);
    for (i, &id) in ids.iter().enumerate() {
        let core_bit = if core[i] { 1 } else { 0 };
        writeln!(
            w,
            "{{\"id\":{},\"level\":{},\"core\":{}}}",
            id, levels[i], core_bit
        )?;
    }
    w.flush()
}

fn write_edges(path: &Path, graph: &Graph, shortcuts: &[ShortcutRec]) -> std::io::Result<()> {
    let f = File::create(path)?;
    let mut w = BufWriter::with_capacity(8 * 1024 * 1024, f);
    // Originals
    for u in 0..graph.n {
        for e in &graph.fwd[u] {
            if e.via != -1 {
                continue;
            }
            writeln!(
                w,
                "{{\"from\":{},\"to\":{},\"cost\":{},\"via\":null}}",
                graph.ids[u], graph.ids[e.other as usize], e.cost
            )?;
        }
    }
    // Shortcuts
    for sc in shortcuts {
        writeln!(
            w,
            "{{\"from\":{},\"to\":{},\"cost\":{},\"via\":{}}}",
            graph.ids[sc.from as usize],
            graph.ids[sc.to as usize],
            sc.cost,
            graph.ids[sc.via as usize]
        )?;
    }
    w.flush()
}

fn main() -> std::io::Result<()> {
    let mut args = std::env::args().skip(1);
    let mut dir: Option<PathBuf> = None;
    let mut contract_fraction = 0.95_f64; // skip top 5% (expensive tail)
    while let Some(a) = args.next() {
        match a.as_str() {
            "--dir" => dir = args.next().map(PathBuf::from),
            // パース失敗や範囲外を黙って 0.95 に戻すと、`--contract-fraction 1.5`
            // のような打ち間違いに気づけないまま数時間の前計算が走る。省略時のみ
            // 既定値を使い、明示指定が不正なら他の引数エラーと同じく即終了する。
            "--contract-fraction" => {
                let raw = args.next().unwrap_or_default();
                match raw.parse::<f64>() {
                    Ok(v) if v > 0.0 && v <= 1.0 => contract_fraction = v,
                    _ => {
                        eprintln!(
                            "--contract-fraction must be a number in (0, 1]; got {:?}",
                            raw
                        );
                        std::process::exit(1);
                    }
                }
            }
            "-h" | "--help" => {
                println!("Usage: ch-preprocess --dir <graphDir> [--contract-fraction 0.95]");
                println!("  reads <dir>/nodes.ndjson + edges.ndjson");
                println!("  writes <dir>/ch_levels.ndjson + ch_edges.ndjson");
                println!(
                    "  --contract-fraction (0,1]: portion of nodes to contract (default 0.95,"
                );
                println!("    leaving top 5% uncontracted to avoid expensive tail)");
                return Ok(());
            }
            other => {
                eprintln!("unknown arg: {}", other);
                std::process::exit(1);
            }
        }
    }
    let dir = dir.unwrap_or_else(|| {
        eprintln!("--dir required");
        std::process::exit(1);
    });

    let t0 = Instant::now();

    eprintln!("[1/4] loading nodes from {}/nodes.ndjson", dir.display());
    let (ids, id_to_idx) = load_nodes_ndjson(&dir.join("nodes.ndjson"))?;
    let n = ids.len();
    eprintln!("  nodes: {} in {:.1}s", n, t0.elapsed().as_secs_f64());

    let mut graph = allocate_graph(n, ids);

    let t1 = Instant::now();
    eprintln!("[2/4] loading edges from {}/edges.ndjson", dir.display());
    let e = load_edges_ndjson(&dir.join("edges.ndjson"), &id_to_idx, &mut graph)?;
    eprintln!(
        "  directed edges: {} in {:.1}s",
        e,
        t1.elapsed().as_secs_f64()
    );

    let t2 = Instant::now();
    eprintln!(
        "[3/4] building CH (hop_limit={}, contract_fraction={:.2})",
        HOP_LIMIT, contract_fraction
    );
    let (levels, core, shortcuts) = build_ch(&mut graph, contract_fraction);
    eprintln!(
        "  shortcuts: {} in {:.1}s",
        shortcuts.len(),
        t2.elapsed().as_secs_f64()
    );

    let t3 = Instant::now();
    eprintln!("[4/4] writing output");
    write_levels(&dir.join("ch_levels.ndjson"), &graph.ids, &levels, &core)?;
    write_edges(&dir.join("ch_edges.ndjson"), &graph, &shortcuts)?;
    eprintln!("  written in {:.1}s", t3.elapsed().as_secs_f64());

    eprintln!("done in {:.1}s", t0.elapsed().as_secs_f64());
    let _ = fs::metadata(dir.join("ch_levels.ndjson"))?;
    Ok(())
}
