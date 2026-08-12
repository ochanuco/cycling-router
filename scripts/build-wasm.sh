#!/usr/bin/env bash
# router-wasm を 3 ターゲット分ビルドし、Cloudflare Workers 用の手書き wrapper を
# 同梱する。出力は dist/ 配下。GitHub Release にはこの dist/ を固めて添付する。
#
#   bundler → Cloudflare Workers (worker.js wrapper 経由で使う)
#   nodejs  → Node からの検証・ベンチ
#   web     → ブラウザ (frontend/wasm)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CRATE="$ROOT/crates/router-wasm"
DIST="$ROOT/dist"

rm -rf "$DIST"
mkdir -p "$DIST"

for target in bundler nodejs web; do
  echo "==> wasm-pack build --target $target"
  wasm-pack build "$CRATE" --release --target "$target" --out-dir "$DIST/$target"
done

# Workers 用 wrapper は bundler 出力の隣に置く。wasm-pack が生成する
# router_wasm.js は namespace-import を使い Workers の bundler が通らないため、
# worker.mjs 側はこの wrapper を import する。
cp "$CRATE/js/worker.js" "$DIST/bundler/router_wasm_worker.js"
# wasm-pack は route_ch を any にしか型付けできない (serde_wasm_bindgen 経由で
# JsValue を返すため)。正確な型は手書きの d.ts で与える。中身が Rust 側と
# ずれていないかは scripts/check-dts.mjs が CI で検証する。
cp "$CRATE/js/worker.d.ts" "$DIST/bundler/router_wasm_worker.d.ts"

# wasm-pack 出力側の route_ch も any のままなので、手書き型を参照する形に
# 差分 patch する (全文上書きはしない)。
node "$ROOT/scripts/patch-dts.mjs"

# 配布物に MIT ライセンスを同梱する (wasm-pack は crate ディレクトリ直下しか
# 見ないため、workspace ルートの LICENSE を出力へコピーする)。
for target in bundler nodejs web; do
  cp "$ROOT/LICENSE" "$DIST/$target/LICENSE"
done

echo "==> built:"
find "$DIST" -maxdepth 2 -type f \( -name "*.wasm" -o -name "*worker.js" \) | sort
