#!/bin/bash


help()
{
    printf "Usage: %s: [-d 2|3] [-f deterministim|non-deterministic|simd]\n" $0
}

while getopts :d:f: name
do
    case $name in
    d)  
        dimension="$OPTARG";;
    f)  
        feature="$OPTARG";;
    ?)  help ; exit 1;;
    esac
done

if [[ -z "$dimension" ]]; then
    help; exit 2;
fi
if [[ -z "$feature" ]]; then
    help; exit 3;
fi

if [[ $feature == "non-deterministic" ]]; then
    feature_postfix=""
else
    feature_postfix="-${feature}"
fi

if [[ $dimension == "2" ]]; then
    package_name="vectorg-engine-2d${feature_postfix}"
else
    package_name="vectorg-engine-3d${feature_postfix}"
fi

rust_source_directory="../builds/${package_name}"

if [ ! -d "$rust_source_directory" ]; then
    echo "Directory $rust_source_directory does not exist";
    echo "You may want to generate rust projects first.";
    help
    exit 4;
fi

# Working dir in wasm-pack is the project root so we need that "../../"

if [[ $feature == "simd" ]]; then
    export additional_rustflags='-C target-feature=+simd128'
else
    export additional_rustflags=''
fi

RUSTFLAGS="${additional_rustflags}" wasm-pack --verbose build --target web --out-dir "../../vectorg-engine-compat/builds/${dimension}d${feature_postfix}/wasm-build" "$rust_source_directory"
