/* @ts-self-types="./rust_router.d.ts" */
import * as wasm from "./rust_router_bg.wasm";
import { __wbg_set_wasm } from "./rust_router_bg.js";

__wbg_set_wasm(wasm);
wasm.__wbindgen_start();
export {
    astar, route_ch
} from "./rust_router_bg.js";
