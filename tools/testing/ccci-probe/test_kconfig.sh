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
for modules in y n; do
	for mediatek in y n; do
		for embedded in y n; do
			for request in y m n; do
				printf 'CONFIG_MODULES=%s\nCONFIG_ARCH_MEDIATEK=%s\nCONFIG_MTK_EMBED_BOARD_DTB=%s\nCONFIG_MTK_CCCI_PROBE_MODULE=%s\n' \
					"$modules" "$mediatek" "$embedded" "$request" > config
				srctree="$root" KCONFIG_CONFIG="$tmp/config" \
					"$conf" --olddefconfig \
					"$root/tools/testing/ccci-probe/Kconfig" > conf.log 2>&1
				expected=n
				if [ "$modules$mediatek$embedded" = yyy ] &&
				   [ "$request" != n ]; then
					expected=m
				fi
				actual=$(sed -n 's/^CONFIG_MTK_CCCI_PROBE_MODULE=//p' config)
				actual=${actual:-n}
				if [ "$actual" != "$expected" ]; then
					echo "FAIL: modules=$modules arch=$mediatek embed=$embedded request=$request: $actual != $expected" >&2
					exit 1
				fi
				count=$((count + 1))
			done
		done
	done
done
echo "PASS: $count Kconfig combinations; probe is never built-in"
