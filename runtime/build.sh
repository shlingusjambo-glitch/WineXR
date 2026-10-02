#!/bin/sh
# Builds runtime/build/vr4mac_openxr.dll (x64 Windows DLL, runs inside Wine) + the smoke test.
set -e
cd "$(dirname "$0")"
mkdir -p build
CC=x86_64-w64-mingw32-gcc
$CC -O2 -Wall -Wno-unused-function -Wno-format-truncation -shared -Iinclude -o build/vr4mac_openxr.dll vr4mac_openxr.c -ld3d11 -ldxgi -ldxguid -static-libgcc
$CC -O2 -Wall -Iinclude -o build/test_xr.exe test_xr.c -ld3d11 -ld3d12 -ldxgi -ldxguid -static-libgcc
echo "built build/vr4mac_openxr.dll"
