// @ts-ignore
import wasmBase64 from "../pkg/vectorg_engine_wasm3d_bg.wasm";
import wasmInit from "../pkg/vectorg_engine_wasm3d";
import base64 from "base64-js";

/**
 * Initializes VectorG Engine.
 * Has to be called and awaited before using any library methods.
 */
export async function init() {
    await wasmInit(base64.toByteArray(wasmBase64 as unknown as string).buffer);
}
