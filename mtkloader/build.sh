#!/bin/bash
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

LLD="/opt/local/bin/ld.lld-mp-15"
CC="/opt/local/bin/clang-mp-15"

CFLAGS="--target=aarch64-linux-gnu -ffreestanding -nostdlib -fno-builtin -mgeneral-regs-only -mstrict-align -Os -Wall -Wextra"

# Touch firmware header (cached / gitignored)
TOUCH_IMG="${TOUCH_IMG:-$ROOT/fw-touch/hdl_firmware.img}"
if [ ! -f "$HERE/touchfw.h" ]; then
	[ -f "$TOUCH_IMG" ] || { echo "mtkloader: no $TOUCH_IMG -- run pull-firmware.sh first" >&2; exit 1; }
	python3 "$HERE/../tools/mktouchfw.py" "$TOUCH_IMG" ROMBOOT_APP_CODE "$HERE/touchfw.h"
fi
[ -f "$HERE/font8x16.h" ] || swift "$HERE/../tools/mkfont8x16.swift" > "$HERE/font8x16.h"

$CC $CFLAGS "$@" -c "$HERE/start.S" -o "$HERE/start.o"
$CC $CFLAGS "$@" -c "$HERE/mtkloader.c" -o "$HERE/mtkloader.o"
$LLD -T "$HERE/link.ld" --oformat binary "$HERE/start.o" "$HERE/mtkloader.o" -o "$HERE/mtkloader.bin"
ls -la "$HERE/mtkloader.bin"

