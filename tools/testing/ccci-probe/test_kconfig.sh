#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
set -eu

root=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
conf=${KCONFIG_CONF:-"$root/scripts/kconfig/conf"}
if [ ! -x "$conf" ]; then
	echo "Build scripts/kconfig/conf with the kernel's host toolchain first." >&2
	exit 1
fi
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT HUP INT TERM
cd "$tmp"
count=0

# Only temporary configurations are written; the real .config is untouched.
# ccci_probe and ccci_smem_dump share the same dependency contract:
# module-only, and only when modules/arch/embedded-dtb/util are all usable.
for modules in y n; do
	for mediatek in y n; do
		for embedded in y n; do
			for util in y m n; do
				for request in y m n; do
					printf 'CONFIG_MODULES=%s\nCONFIG_ARCH_MEDIATEK=%s\nCONFIG_MTK_EMBED_BOARD_DTB=%s\nCONFIG_MTK_CCCI_UTIL=%s\nCONFIG_MTK_CCCI_PROBE_MODULE=%s\nCONFIG_MTK_CCCI_SMEM_DUMP=%s\nCONFIG_MTK_CCCI_CCIF=%s\n' \
						"$modules" "$mediatek" "$embedded" "$util" "$request" "$request" "$request" > config
					srctree="$root" KCONFIG_CONFIG="$tmp/config" \
						"$conf" --olddefconfig \
						"$root/tools/testing/ccci-probe/Kconfig" > conf.log 2>&1
					expected=n
					if [ "$modules$mediatek$embedded" = yyy ] &&
					   [ "$util" != n ] && [ "$request" != n ]; then
						expected=m
					fi
					for sym in CONFIG_MTK_CCCI_PROBE_MODULE CONFIG_MTK_CCCI_SMEM_DUMP CONFIG_MTK_CCCI_CCIF; do
						actual=$(sed -n "s/^$sym=//p" config)
						actual=${actual:-n}
						if [ "$actual" != "$expected" ]; then
							echo "FAIL: modules=$modules arch=$mediatek embed=$embedded util=$util request=$request: $sym: $actual != $expected" >&2
							exit 1
						fi
					done
					count=$((count + 1))
				done
			done
		done
	done
done
echo "PASS: $count Kconfig combinations; probe and smem_dump are never built-in"
