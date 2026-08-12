// 手書きの worker.d.ts と Rust 側の #[derive(Serialize)] 構造体を突き合わせる。
//
// route_ch は serde_wasm_bindgen 経由で JsValue を返すため、wasm-bindgen は
// 戻り値の形を知らず d.ts では any になる。正確な型は手で書くしかないが、
// 手書きは Rust 側の変更に追従し損ねる。フィールド名の集合だけでも機械的に
// 突き合わせておけば、追加・削除・改名を CI で捕まえられる。
//
// 型の一致までは見ない (Rust の u32 と TS の number は 1 対 1 ではない)。
// あくまで「フィールドの過不足」を検出する。

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const RUST = path.join(ROOT, 'crates', 'router-wasm', 'src', 'lib.rs');
const DTS = path.join(ROOT, 'crates', 'router-wasm', 'js', 'worker.d.ts');

// Rust 側: `struct Name<'a> { field: Type, ... }` からフィールド名を拾う。
function rustStructFields(src, name) {
  const re = new RegExp(`struct\\s+${name}(?:<[^>]*>)?\\s*\\{([^}]*)\\}`, 'm');
  const m = src.match(re);
  if (!m) throw new Error(`Rust struct not found: ${name}`);
  return new Set(
    m[1]
      .split('\n')
      .map((l) => l.trim())
      .filter((l) => l && !l.startsWith('//') && !l.startsWith('#['))
      .map((l) => l.split(':')[0].trim())
      .filter(Boolean)
  );
}

// TS 側: `interface Name { field: Type; ... }` からフィールド名を拾う。
// オプショナル (`field?`) と undefined 専用の番兵は除く。
function tsInterfaceFields(src, name) {
  const re = new RegExp(`interface\\s+${name}\\s*\\{([\\s\\S]*?)\\n\\}`, 'm');
  const m = src.match(re);
  if (!m) throw new Error(`TS interface not found: ${name}`);
  const fields = new Set();
  for (const raw of m[1].split('\n')) {
    const line = raw.trim();
    if (!line || line.startsWith('//') || line.startsWith('*') || line.startsWith('/*')) continue;
    const hit = line.match(/^([A-Za-z_][A-Za-z0-9_]*)\??\s*:/);
    if (hit) fields.add(hit[1]);
  }
  return fields;
}

const rust = fs.readFileSync(RUST, 'utf8');
const dts = fs.readFileSync(DTS, 'utf8');

// TS 側のみに存在してよいもの (union 判別用の番兵)
const TS_ONLY = { RouteChOk: new Set(['error']) };

const PAIRS = [
  ['RouteOk', 'RouteChOk'],
  ['RouteMeta', 'RouteMeta']
];

let failed = false;
for (const [rustName, tsName] of PAIRS) {
  const r = rustStructFields(rust, rustName);
  const t = tsInterfaceFields(dts, tsName);
  const allowed = TS_ONLY[tsName] || new Set();

  const missingInTs = [...r].filter((f) => !t.has(f));
  const extraInTs = [...t].filter((f) => !r.has(f) && !allowed.has(f));

  if (missingInTs.length || extraInTs.length) {
    failed = true;
    process.stderr.write(`\n${rustName} (Rust) <-> ${tsName} (TS) が一致しません\n`);
    if (missingInTs.length) process.stderr.write(`  d.ts に無い: ${missingInTs.join(', ')}\n`);
    if (extraInTs.length) process.stderr.write(`  d.ts にだけある: ${extraInTs.join(', ')}\n`);
  } else {
    process.stdout.write(`${rustName} <-> ${tsName}: ${r.size} フィールド一致\n`);
  }
}

// 失敗時の戻り値は Rust 側が RouteErr と RouteErrWithMeta の 2 種類あり、TS 側は
// meta を optional にした RouteChErr 1 つで両方を表す。したがって RouteChErr の
// フィールド集合は 2 つの Rust 構造体の和集合と一致するはず。存在確認だけだと
// 例えば Rust に error_code を足しても気づけないので、ここも突き合わせる。
try {
  const rustErrUnion = new Set([
    ...rustStructFields(rust, 'RouteErr'),
    ...rustStructFields(rust, 'RouteErrWithMeta')
  ]);
  const tsErr = tsInterfaceFields(dts, 'RouteChErr');

  const missingInTs = [...rustErrUnion].filter((f) => !tsErr.has(f));
  const extraInTs = [...tsErr].filter((f) => !rustErrUnion.has(f));

  if (missingInTs.length || extraInTs.length) {
    failed = true;
    process.stderr.write('\nRouteErr + RouteErrWithMeta (Rust) <-> RouteChErr (TS) が一致しません\n');
    if (missingInTs.length) process.stderr.write(`  d.ts に無い: ${missingInTs.join(', ')}\n`);
    if (extraInTs.length) process.stderr.write(`  d.ts にだけある: ${extraInTs.join(', ')}\n`);
  } else {
    process.stdout.write(
      `RouteErr + RouteErrWithMeta <-> RouteChErr: ${rustErrUnion.size} フィールド一致\n`
    );
  }
} catch (e) {
  failed = true;
  process.stderr.write(`${e.message}\n`);
}

if (failed) {
  process.stderr.write('\nworker.d.ts を crates/router-wasm/src/lib.rs に合わせて更新してください。\n');
  process.exit(1);
}
process.stdout.write('worker.d.ts は Rust の構造体と一致しています\n');
