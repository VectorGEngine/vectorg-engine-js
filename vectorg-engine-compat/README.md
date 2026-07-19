# VectorG Engine JS compatibility builds

This directory builds the `-compat` VectorG Engine JS packages. These packages inline WebAssembly as base64 for bundlers that cannot load a separate `.wasm` file.

The scripts generate 2D and 3D standard, deterministic, and SIMD variants from the corresponding `@vectorg/vectorg-engine-2d` and `@vectorg/vectorg-engine-3d` packages.

VectorG Engine JS targets VectorG Engine and is not compatible with upstream Rapier.js.
