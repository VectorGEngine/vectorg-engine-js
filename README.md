# VectorG Engine JS

[![VectorG Engine JS CI](https://github.com/VectorGEngine/vectorg-engine-js/actions/workflows/vectorg-engine-js-ci.yml/badge.svg)](https://github.com/VectorGEngine/vectorg-engine-js/actions/workflows/vectorg-engine-js-ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

**JavaScript and WebAssembly bindings for VectorG Engine.**

VectorG Engine JS exposes the real-time vehicle dynamics engine that powers the VectorG driving simulator. It includes the engine’s tire modeling, suspension, drivetrain, differential, realistic vehicle controller, and racing-simulation features.

VectorG Engine JS targets VectorG Engine and is not compatible with upstream Rapier.js.

## Packages

- `@vectorg/vectorg-engine-3d`: primary 3D bindings;
- `@vectorg/vectorg-engine-3d-simd`: SIMD-enabled 3D bindings;
- `@vectorg/vectorg-engine-3d-deterministic`: deterministic 3D bindings;
- `@vectorg/vectorg-engine-2d`: 2D bindings; and
- corresponding `-compat` packages with inlined WebAssembly for broader bundler support.

## Building packages

From the repository root:

```shell
./builds/prepare_builds/prepare_all_projects.sh
./builds/prepare_builds/build_all_projects.sh
```

Generated packages are written under `builds/`. To build WebAssembly-inlined packages, run `npm run build` in `vectorg-engine-compat/` after preparing the standard builds.

## Origin and attribution

VectorG Engine JS is derived from Rapier.js and Rapier, originally developed by [Dimforge](https://dimforge.com). It preserves the original license, copyright notices, and attribution.

See [LICENSE](LICENSE) and [NOTICE](NOTICE).
