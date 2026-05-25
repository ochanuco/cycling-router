/**
 * Forward A* on the flat graph representation. Returns the path including
 * start and goal indices, prefixed by the total distance.
 * @param {Float64Array} node_coords
 * @param {Float64Array} edge_data
 * @param {number} start
 * @param {number} goal
 * @returns {Float64Array}
 */
export function astar(node_coords, edge_data, start, goal) {
    const ptr0 = passArrayF64ToWasm0(node_coords, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ptr1 = passArrayF64ToWasm0(edge_data, wasm.__wbindgen_malloc);
    const len1 = WASM_VECTOR_LEN;
    const ret = wasm.astar(ptr0, len0, ptr1, len1, start, goal);
    var v3 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
    wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
    return v3;
}

/**
 * DNF route entry point. Decodes tile buffers, builds CSR, snaps endpoints,
 * runs CH bidirectional query, unpacks shortcuts. Returns a JS object with
 * `{ distance, settled, terminated, ch_ms, snap_from_m, snap_to_m,
 * from_id, to_id, coords: [[lon,lat]...], algorithm, csr_bytes,
 * csr_node_count, csr_edge_count }`.
 *
 * `buffers` は `Array<Uint8Array>` を JS から渡す想定。各要素はタイル
 * binary (v1 or v2)。corridor + snap neighborhood 分まとめて渡す。
 *
 * 失敗時は `{ error: "..." }` を含む JS object を返す (例外を投げない)。
 * @param {Array<any>} buffers
 * @param {number} from_lon
 * @param {number} from_lat
 * @param {number} to_lon
 * @param {number} to_lat
 * @param {number} max_snap_meters
 * @returns {any}
 */
export function route_ch(buffers, from_lon, from_lat, to_lon, to_lat, max_snap_meters) {
    const ret = wasm.route_ch(buffers, from_lon, from_lat, to_lon, to_lat, max_snap_meters);
    return ret;
}

/**
 * Browser GPX-mode helper: for each shop point, compute the minimum
 * perpendicular distance (meters) to the route polyline. Used by
 * `frontend/app.js` to filter supply-points within N meters of the route
 * without running the O(N×M) JS loop on the main thread (5-10x faster).
 *
 * Inputs (flat typed arrays for zero-copy boundary):
 * - `route_lonlats`: Float64Array of length 2*N (lon, lat alternating)
 * - `shop_lonlats`: Float64Array of length 2*M (lon, lat alternating)
 *
 * Returns Float32Array of length M with per-shop minimum distance (m).
 * On empty/invalid inputs returns the appropriate length 0 / INF array.
 * @param {Float64Array} route_lonlats
 * @param {Float64Array} shop_lonlats
 * @returns {Float32Array}
 */
export function route_distances(route_lonlats, shop_lonlats) {
    const ptr0 = passArrayF64ToWasm0(route_lonlats, wasm.__wbindgen_malloc);
    const len0 = WASM_VECTOR_LEN;
    const ptr1 = passArrayF64ToWasm0(shop_lonlats, wasm.__wbindgen_malloc);
    const len1 = WASM_VECTOR_LEN;
    const ret = wasm.route_distances(ptr0, len0, ptr1, len1);
    var v3 = getArrayF32FromWasm0(ret[0], ret[1]).slice();
    wasm.__wbindgen_free(ret[0], ret[1] * 4, 4);
    return v3;
}
export function __wbg___wbindgen_throw_1506f2235d1bdba0(arg0, arg1) {
    throw new Error(getStringFromWasm0(arg0, arg1));
}
export function __wbg_get_2b48c7d0d006a781(arg0, arg1) {
    const ret = arg0[arg1 >>> 0];
    return ret;
}
export function __wbg_instanceof_Uint8Array_86f30649f63ef9c2(arg0) {
    let result;
    try {
        result = arg0 instanceof Uint8Array;
    } catch (_) {
        result = false;
    }
    const ret = result;
    return ret;
}
export function __wbg_length_4a591ecaa01354d9(arg0) {
    const ret = arg0.length;
    return ret;
}
export function __wbg_length_66f1a4b2e9026940(arg0) {
    const ret = arg0.length;
    return ret;
}
export function __wbg_new_ce1ab61c1c2b300d() {
    const ret = new Object();
    return ret;
}
export function __wbg_new_d90091b82fdf5b91() {
    const ret = new Array();
    return ret;
}
export function __wbg_now_190933fa139cc119() {
    const ret = Date.now();
    return ret;
}
export function __wbg_prototypesetcall_3249fc62a0fafa30(arg0, arg1, arg2) {
    Uint8Array.prototype.set.call(getArrayU8FromWasm0(arg0, arg1), arg2);
}
export function __wbg_set_6be42768c690e380(arg0, arg1, arg2) {
    arg0[arg1] = arg2;
}
export function __wbg_set_dca99999bba88a9a(arg0, arg1, arg2) {
    arg0[arg1 >>> 0] = arg2;
}
export function __wbindgen_cast_0000000000000001(arg0) {
    // Cast intrinsic for `F64 -> Externref`.
    const ret = arg0;
    return ret;
}
export function __wbindgen_cast_0000000000000002(arg0, arg1) {
    // Cast intrinsic for `Ref(String) -> Externref`.
    const ret = getStringFromWasm0(arg0, arg1);
    return ret;
}
export function __wbindgen_init_externref_table() {
    const table = wasm.__wbindgen_externrefs;
    const offset = table.grow(4);
    table.set(0, undefined);
    table.set(offset + 0, undefined);
    table.set(offset + 1, null);
    table.set(offset + 2, true);
    table.set(offset + 3, false);
}
function getArrayF32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat32ArrayMemory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

function getArrayU8FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint8ArrayMemory0().subarray(ptr / 1, ptr / 1 + len);
}

let cachedFloat32ArrayMemory0 = null;
function getFloat32ArrayMemory0() {
    if (cachedFloat32ArrayMemory0 === null || cachedFloat32ArrayMemory0.byteLength === 0) {
        cachedFloat32ArrayMemory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32ArrayMemory0;
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function passArrayF64ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 8, 8) >>> 0;
    getFloat64ArrayMemory0().set(arg, ptr / 8);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

let WASM_VECTOR_LEN = 0;


let wasm;
export function __wbg_set_wasm(val) {
    wasm = val;
}
