#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# Build the Rubens vendor_boot image whose ramdisk carries only /lib/firmware.
#
#   tools/build-rubens-vendor-ramdisk.sh <stock-vendor_boot.img> <out.img>
#
# Why a separate firmware ramdisk:
#   LK refuses to boot when the boot image ramdisk grows past roughly its
#   stock size (~4 MB); a 5.3 MB boot ramdisk made the device reset before
#   Linux ever started.  The vendor_boot ramdisk is much larger (the stock
#   one is 44 MB), and the kernel unpacks both ramdisks into the same
#   initramfs, so the firmware can live in the vendor_boot ramdisk while the
#   boot ramdisk stays small.
#
# The dtb and bootconfig are taken from the stock vendor_boot image so the
# LK-visible header matches the factory layout (page size 4096, stock load
# addresses and the MediaTek bootopt cmdline).
set -eu

srcdir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
stock=${1:?usage: $0 <stock-vendor_boot.img> <out.img> [firmware-dir]}
out=${2:?usage: $0 <stock-vendor_boot.img> <out.img> [firmware-dir]}
fwdir=${3:-$srcdir/rubens-firmware}

gen_init_cpio=${GEN_INIT_CPIO:-$srcdir/../usr/gen_init_cpio}
if [ ! -x "$gen_init_cpio" ]; then
	printf '%s\n' "gen_init_cpio not found; set GEN_INIT_CPIO" >&2
	exit 1
fi
if [ ! -d "$fwdir" ]; then
	printf '%s\n' "firmware dir $fwdir not found" >&2
	exit 1
fi

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

unpack_bootimg --boot_img "$stock" --out "$work/unpack" >/dev/null

spec=$work/spec
: > "$spec"
printf 'dir /lib 0755 0 0\n' >> "$spec"
printf 'dir /lib/firmware 0755 0 0\n' >> "$spec"
(
	cd "$fwdir"
	find . -mindepth 1 -type d | while read -r d; do
		printf 'dir /lib/firmware/%s 0755 0 0\n' "${d#./}" >> "$spec"
	done
	find . -type f | while read -r f; do
		printf 'file /lib/firmware/%s %s 0644 0 0\n' \
			"${f#./}" "$fwdir/${f#./}" >> "$spec"
	done
)

"$gen_init_cpio" "$spec" > "$work/firmware.cpio"
gzip -9 "$work/firmware.cpio"

mkbootimg --header_version 4 --pagesize 4096 \
	--vendor_boot "$out" \
	--vendor_ramdisk "$work/firmware.cpio.gz" \
	--dtb "$work/unpack/dtb" \
	--vendor_bootconfig "$work/unpack/bootconfig" \
	--base 0x40000000 --kernel_offset 0 \
	--ramdisk_offset 0x26f00000 --tags_offset 0x7c80000 \
	--dtb_offset 0x7c80000 \
	--vendor_cmdline "bootopt=64S3,32N2,64N2" \
	--os_version 12.0.0 --os_patch_level 2023-11

printf 'built %s\n' "$out"
