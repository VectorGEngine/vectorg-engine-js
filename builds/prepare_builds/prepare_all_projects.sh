#!/bin/bash

features=(non-deterministic)
dims=(dim3)

for feature in ${features[@]}; do
    for dim in ${dims[@]}; do
        echo "preparing dimension $dim with feature $feature"
        cargo run -p prepare_builds -- -d ${dim} -f ${feature}
    done
done
