#!/usr/bin/env bash
# Build 3DSWiggle inside the official devkitPro Docker image (works with docker or podman).
#   ./build.sh          -> 3DSWiggle.3dsx + 3DSWiggle.cia
#   ./build.sh clean
set -e
cd "$(dirname "$0")"
ENGINE=${ENGINE:-$(command -v podman || command -v docker)}
IMAGE=docker.io/devkitpro/devkitarm:latest
"$ENGINE" run --rm -v "$PWD":/src:Z -w /src \
    -e PATH=/opt/devkitpro/devkitARM/bin:/opt/devkitpro/tools/bin:/src/tools/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    "$IMAGE" make "${@:-cia}"
