#!/bin/bash

npm run build
rsync -av --delete-after dist/ crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/vectorg-engine/demos3d
# rsync -av dist/ammo.wasm.wasm crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/vectorg-engine/ammo.wasm.wasm
# rsync -av dist/physx.release.wasm crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/vectorg-engine/physx.release.wasm
