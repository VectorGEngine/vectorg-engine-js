import commonjs from "@rollup/plugin-commonjs";
import {nodeResolve} from "@rollup/plugin-node-resolve";
import typescript from "@rollup/plugin-typescript";
import terser from "@rollup/plugin-terser";
import path from "path";
import {base64} from "rollup-plugin-base64";
import copy from "rollup-plugin-copy";
import filesize from "rollup-plugin-filesize";

const config = (dim, features_postfix) => {
    const featureSuffix = features_postfix.replace(dim, "");
    const dimensionSuffix = `-${dim}`;
    const sourcePackage = `vectorg-engine${dimensionSuffix}${featureSuffix}`;

    return {
        input: `builds/${features_postfix}/gen${dim}/vectorg-engine.ts`,
        output: [
            {
                file: `builds/${features_postfix}/pkg/vectorg-engine.es.js`,
                format: "es",
                sourcemap: true,
                exports: "named",
            },
            {
                file: `builds/${features_postfix}/pkg/vectorg-engine.cjs.js`,
                format: "cjs",
                sourcemap: true,
                exports: "named",
            },
        ],
        plugins: [
            copy({
                targets: [
                    {
                        src: `builds/${features_postfix}/wasm-build/package.json`,
                        dest: `builds/${features_postfix}/pkg/`,
                        transform(content) {
                            let config = JSON.parse(content.toString());
                            config.name = `@vectorg/vectorg-engine${dimensionSuffix}${featureSuffix}-compat`;
                            config.description +=
                                " Compatibility package with inlined webassembly as base64.";
                            config.types = "vectorg-engine.d.ts";
                            config.main = "vectorg-engine.cjs.js";
                            config.module = "vectorg-engine.es.js";
                            // delete config.module;
                            config.files = ["*"];
                            return JSON.stringify(config, undefined, 2);
                        },
                    },
                    {
                        src: `../builds/${sourcePackage}/LICENSE`,
                        dest: `builds/${features_postfix}/pkg`,
                    },
                    {
                        src: `../builds/${sourcePackage}/README.md`,
                        dest: `builds/${features_postfix}/pkg`,
                    },
                    {
                        src: `../builds/${sourcePackage}/NOTICE`,
                        dest: `builds/${features_postfix}/pkg`,
                    },
                ],
            }),
            base64({include: "**/*.wasm"}),
            terser(),
            nodeResolve(),
            commonjs(),
            typescript({
                tsconfig: path.resolve(
                    __dirname,
                    `builds/${features_postfix}/tsconfig.pkg.json`,
                ),
                sourceMap: true,
                inlineSources: true,
            }),
            filesize(),
        ],
    };
};

export default [
    config("2d", "2d"),
    config("2d", "2d-deterministic"),
    config("2d", "2d-simd"),
    config("3d", "3d"),
    config("3d", "3d-deterministic"),
    config("3d", "3d-simd"),
];
