#!/bin/sh
# Builds runtime/build/vr4mac_openxr.dll (x64 Windows DLL, runs inside Wine) + the smoke test.
set -eu
cd "$(dirname "$0")"
mkdir -p build
CC=${CC:-x86_64-w64-mingw32-gcc}
command -v "$CC" >/dev/null 2>&1 || { echo "error: $CC not found (install mingw-w64)" >&2; exit 127; }
$CC -O2 -Wall -Wno-unused-function -Wno-format-truncation -shared -Iinclude -o build/vr4mac_openxr.dll vr4mac_openxr.c -ld3d11 -ldxgi -ldxguid -static-libgcc
$CC -O2 -Wall -Iinclude -o build/test_xr.exe test_xr.c -ld3d11 -ld3d12 -ldxgi -ldxguid -static-libgcc
echo "built build/vr4mac_openxr.dll"
