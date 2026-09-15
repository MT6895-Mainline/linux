#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# Build the Alpine-based Rubens initramfs used as the boot/vendor_boot ramdisk.
#
#   tools/build-rubens-rootfs.sh <output.cpio.gz> [alpine-minirootfs.tar.gz]
#
# Downloads the Alpine aarch64 minirootfs when the tarball is not cached,
# installs tools/rubens-rootfs-init.sh as /init and packs everything with the
# kernel's gen_init_cpio (no root required, device nodes come from the spec).

set -eu

srcdir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
out=${1:?usage: $0 output.cpio.gz [alpine-tarball]}
tarball=${2:-/tmp/opencode/alpine-minirootfs-3.20.3-aarch64.tar.gz}
sha_url="$tarball.sha256"
work=/tmp/opencode/rubens-rootfs
url=https://dl-cdn.alpinelinux.org/alpine/v3.20/releases/aarch64/alpine-minirootfs-3.20.3-aarch64.tar.gz

gen_init_cpio=${GEN_INIT_CPIO:-$srcdir/../usr/gen_init_cpio}
if [ ! -x "$gen_init_cpio" ]; then
	gen_init_cpio=/tmp/opencode/rubens-independent-build/usr/gen_init_cpio
fi
if [ ! -x "$gen_init_cpio" ]; then
	printf '%s\n' "gen_init_cpio not found; build the kernel first" >&2
	exit 1
fi

if [ ! -f "$tarball" ]; then
	printf '%s\n' "downloading $url"
	curl -fL --retry 3 -o "$tarball" "$url"
fi
if [ ! -f "$sha_url" ]; then
	curl -fsL --retry 3 -o "$sha_url" "$url.sha256" || true
fi
if [ -f "$sha_url" ]; then
	expected=$(awk '{print $1}' "$sha_url")
	actual=$(sha256sum "$tarball" | awk '{print $1}')
	if [ "$expected" != "$actual" ]; then
		printf '%s\n' "sha256 mismatch for $tarball" >&2
		exit 1
	fi
fi

rm -rf "$work"
mkdir -p "$work"
tar -xzf "$tarball" -C "$work"
cp "$srcdir/rubens-rootfs-init.sh" "$work/init"
chmod 0755 "$work/init"
mkdir -p "$work/dev" "$work/proc" "$work/sys" "$work/run" "$work/tmp"

if [ -d "$srcdir/rubens-firmware" ]; then
	mkdir -p "$work/lib/firmware"
	cp "$srcdir/rubens-firmware/"* "$work/lib/firmware/"
fi

spec=/tmp/opencode/rubens-rootfs.spec
: > "$spec"
printf 'nod /dev/console 0600 0 0 c 5 1\n' >> "$spec"
printf 'nod /dev/null 0666 0 0 c 1 3\n' >> "$spec"

find "$work" -mindepth 1 -printf '%y %m %p %l\n' | while read -r type mode path link; do
	rel=${path#"$work"}
	case "$type" in
	d)
		printf 'dir %s %s 0 0\n' "$rel" "$mode"
		;;
	f)
		printf 'file %s %s %s 0 0\n' "$rel" "$path" "$mode"
		;;
	l)
		printf 'slink %s %s %s 0 0\n' "$rel" "$link" "$mode"
		;;
	esac
done >> "$spec"

"$gen_init_cpio" "$spec" | gzip -9 > "$out"
printf 'built %s (%s bytes, %s files)\n' "$out" "$(stat -c %s "$out")" "$(wc -l < "$spec")"
