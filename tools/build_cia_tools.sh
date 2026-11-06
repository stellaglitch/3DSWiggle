#!/usr/bin/env bash
# Builds makerom (3DSGuy/Project_CTR) and bannertool (carstene1ns/3ds-bannertool)
# from source inside the devkitPro image, into tools/bin. Run once before ./build.sh.
set -e
cd "$(dirname "$0")/.."
ENGINE=${ENGINE:-$(command -v podman || command -v docker)}
SRC=${SRC:-$HOME/3dstools}
mkdir -p "$SRC" tools/bin
[ -d "$SRC/Project_CTR" ] || git clone --depth 1 https://github.com/3DSGuy/Project_CTR.git "$SRC/Project_CTR"
[ -d "$SRC/bannertool" ]  || git clone --depth 1 --recursive https://github.com/carstene1ns/3ds-bannertool.git "$SRC/bannertool"

"$ENGINE" run --rm -v "$SRC":/t:Z -v "$PWD/tools/bin":/out:Z docker.io/devkitpro/devkitarm:latest bash -ec '
cd /t/Project_CTR/makerom
make -j"$(nproc)" deps && make -j"$(nproc)" program
cp "$(find . -type f -name makerom -perm -u+x | head -1)" /out/makerom
cd /t/bannertool && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j"$(nproc)"
cp "$(find . -type f -name bannertool -perm -u+x | head -1)" /out/bannertool
'
ls -la tools/bin
