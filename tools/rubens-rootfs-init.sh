#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# Rubens Alpine initramfs init.
#
# Runs entirely from the initramfs (no storage rootfs yet). It mounts the
# pseudo filesystems and puts a root shell on the USB CDC ACM port (ttyGS0,
# provided by the built-in g_serial gadget) and on the panel VT (tty1).

export PATH=/sbin:/bin:/usr/sbin:/usr/bin

mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev
mkdir -p /dev/pts /run /tmp
mount -t devpts devpts /dev/pts 2>/dev/null
mount -t tmpfs -o mode=0755 tmpfs /run
mount -t tmpfs -o mode=1777 tmpfs /tmp
mount -t debugfs debugfs /sys/kernel/debug 2>/dev/null

hostname rubens

# Keep the panel console awake and show the panel shell instead of the
# boot log tail (tty0). The fbdev blank timer otherwise powers the display
# down after 10 idle minutes and the panel keeps the last frame.
echo 0 > /sys/class/graphics/fb0/blank 2>/dev/null

{
	echo "rubens Linux $(uname -r) - Alpine initramfs"
	echo "USB serial shell: /dev/ttyGS0, panel shell: /dev/tty1"
} > /dev/kmsg 2>/dev/null

spawn_shell() {
	dev="$1"
	banner="$2"

	while [ ! -c "$dev" ]; do
		sleep 1
	done
	while :; do
		{
			echo ""
			echo "$banner"
			echo "type 'exit' to respawn the shell"
		} > "$dev" 2>/dev/null
		sh -i < "$dev" > "$dev" 2>&1
		sleep 1
	done
}

# The bootlog record window only holds the last 61KB of printk, so the early
# boot messages (PHY probes etc.) are long gone by the time the oops partition
# is read back. Dump the full dmesg plus the device state to a fixed offset
# (4 MiB) of the oops partition instead; the host can read it from Android.
(
	sleep 12
	{
		echo "=== rubens debug dump $(date 2>/dev/null) ==="
		echo "=== dmesg ==="
		dmesg
		echo "=== deferred ==="
		cat /sys/kernel/debug/devices_deferred 2>/dev/null
		echo "=== udc ==="
		ls -l /sys/class/udc 2>/dev/null
		echo "=== phys ==="
		for p in /sys/class/phy/phy*; do
			echo "--- $p"
			cat "$p/name" 2>/dev/null
		done
		echo "=== platform devices matching phy ==="
		ls -l /sys/bus/platform/devices/ 2>/dev/null | grep -i phy
		echo "=== regulators ==="
		for r in /sys/class/regulator/regulator.*; do
			echo "$r: $(cat "$r/name" 2>/dev/null) state=$(cat "$r/state" 2>/dev/null) uV=$(cat "$r/microvolts" 2>/dev/null)"
		done
		echo "=== regulator summary (usb) ==="
		grep -iE 'vusb|vrf18|vibr' /sys/kernel/debug/regulator/regulator_summary 2>/dev/null
		echo "=== u2 phy regs (BC11/DTM) ==="
		grep -iE 'xsphy|t-phy' /sys/kernel/debug/devices_deferred 2>/dev/null
		echo "=== mtk-xsphy driver ==="
		ls -l /sys/bus/platform/drivers/mtk-xsphy/ 2>/dev/null
		echo "=== platform devices matching 112/dwc/usb ==="
		ls -l /sys/bus/platform/devices/ 2>/dev/null | grep -iE '1120|dwc|usb'
	} > /tmp/rubens-dump.txt 2>&1

	dump_dev=""
	for b in /sys/class/block/*; do
		[ -f "$b/partition" ] || continue
		case "$(cat "$b/uevent" 2>/dev/null)" in
		*PARTNAME=oops*)
			dump_dev="/dev/$(basename "$b")"
			break
			;;
		esac
	done

	if [ -n "$dump_dev" ]; then
		dd if=/tmp/rubens-dump.txt of="$dump_dev" bs=4096 seek=1024 conv=notrunc 2>/dev/null
		status=$?
		sync
		echo "rubens-diag: dump $dump_dev status=$status size=$(wc -c < /tmp/rubens-dump.txt)" > /dev/kmsg
	else
		echo "rubens-diag: oops partition not found" > /dev/kmsg
	fi
) &

spawn_shell /dev/ttyGS0 "rubens Linux shell on USB CDC ACM" &
spawn_shell /dev/tty1 "rubens Linux shell on the panel VT" &
chvt 1 2>/dev/null

# PID 1: reap orphans forever.
while :; do
	wait
	sleep 3600
done
