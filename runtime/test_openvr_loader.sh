#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p build
CC=${CC:-x86_64-w64-mingw32-gcc}
command -v "$CC" >/dev/null 2>&1 || { echo "error: $CC not found (install mingw-w64)" >&2; exit 127; }
"$CC" -O2 -Wall -Wextra -o build/test_openvr_loader.exe test_openvr_loader.c -lshell32 -static-libgcc
printf '%s\n' 'Built build/test_openvr_loader.exe; run under active game Wine environment with the game openvr_api.dll path.'
