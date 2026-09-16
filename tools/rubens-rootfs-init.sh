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

# Boot the Debian rootfs from the "cust" partition when it is present and
# mountable. Anything unexpected (no partition, bad superblock, no init)
# falls through to the rescue shells below so the device always has a
# console.
# Find a block device by its GPT partition name.
find_part_by_name() {
	want="$1"
	for b in /sys/class/block/*; do
		[ -f "$b/partition" ] || continue
		case "$(cat "$b/uevent" 2>/dev/null)" in
		*PARTNAME="$want"*)
			echo "/dev/$(basename "$b")"
			return 0
			;;
		esac
	done
	return 1
}

# Firmware the drivers may request again after switch_root (WiFi chip
# reset, touch resume, ...).  The per-device NVRAM blobs are taken from
# nvdata below instead, so they are not listed here.
ROOTFS_FW="
BT_FW.cfg
conninfra.cfg
wifi.cfg
regulatory.db
regulatory.db.p7s
WIFI_RAM_CODE_soc7_0_1b_t_1.bin
soc7_0_ram_mcu_1b_t_1_hdr.bin
soc7_0_ram_bt_1b_t_1_hdr.bin
soc7_0_ram_wmmcu_1b_t_1_hdr.bin
aw8697_haptic.bin
tfa98xx.cnt
st_fts_L11a.ftb
stm_fts_production_limits.csv
arm/mali/arch10.8/mali_csffw.bin
"

copy_fw_to_rootfs() {
	cd /lib/firmware 2>/dev/null || return 0
	for f in $ROOTFS_FW; do
		[ -f "$f" ] || continue
		mkdir -p "/mnt/root/lib/firmware/$(dirname "$f")"
		[ -f "/mnt/root/lib/firmware/$f" ] &&
			cmp -s "$f" "/mnt/root/lib/firmware/$f" && continue
		cp "$f" "/mnt/root/lib/firmware/$f"
	done
	cd /
}

# Copy the per-device NVRAM blobs (WiFi calibration/MAC, BT address) out of
# the nvdata partition and mirror the runtime firmware onto the rootfs, so
# every boot uses the real data stored on the device instead of whatever was
# baked into an image.  The same copies are placed in the initramfs so even
# a firmware request that races the switch_root finds them.
provision_rootfs() {
	mkdir -p /mnt/root/lib/firmware
	copy_fw_to_rootfs

	part=$(find_part_by_name nvdata) || return 0
	mkdir -p /mnt/nvdata
	mount -t ext4 -o ro "$part" /mnt/nvdata 2>/dev/null || return 0
	src=/mnt/nvdata/APCFG/APRDEB
	dst=/mnt/root/lib/firmware/mediatek/mt6895
	mkdir -p "$dst" /lib/firmware/mediatek/mt6895

	# WiFi NVRAM blob (calibration data + MAC): copied verbatim.
	if [ -f "$src/WIFI" ]; then
		cp "$src/WIFI" "$dst/WIFI"
		cp "$src/WIFI" /lib/firmware/mediatek/mt6895/WIFI
		echo "rubens: copied WiFi NVRAM from nvdata" > /dev/kmsg
	fi
	if [ -f "$src/WIFI_CUSTOM" ]; then
		cp "$src/WIFI_CUSTOM" "$dst/WIFI_CUSTOM"
		cp "$src/WIFI_CUSTOM" /lib/firmware/mediatek/mt6895/WIFI_CUSTOM
	fi

	# BT address: the driver copies the first six bytes straight into
	# bdaddr_t (least significant byte first), so reverse them here to get
	# the address in the usual MSB-first textual order.
	if [ -f "$src/BT_Addr" ]; then
		od -An -tx1 -N6 "$src/BT_Addr" | awk '
			function h2d(s,   i, c, v) {
				v = 0
				for (i = 1; i <= length(s); i++) {
					c = substr(s, i, 1)
					v = v * 16 + index("0123456789abcdef", c) - 1
				}
				return v
			}
			{ for (i = 6; i >= 1; i--) printf "%c", h2d($i) }
		' > "$dst/BT_Addr"
		cp "$dst/BT_Addr" /lib/firmware/mediatek/mt6895/BT_Addr
		echo "rubens: copied BT address from nvdata" > /dev/kmsg
	fi

	umount /mnt/nvdata
}

CUST_PART=$(find_part_by_name cust)
if [ -n "$CUST_PART" ]; then
	mkdir -p /mnt/root
	if mount -t ext4 "$CUST_PART" /mnt/root 2>/dev/null &&
	   [ -x /mnt/root/sbin/init ]; then
		provision_rootfs
		mount --move /dev /mnt/root/dev
		mount --move /proc /mnt/root/proc
		mount --move /sys /mnt/root/sys
		mount --move /run /mnt/root/run
		echo "rubens: switching root to $CUST_PART (Debian)" > /dev/kmsg
		exec switch_root /mnt/root /sbin/init
	fi
	umount /mnt/root 2>/dev/null
	echo "rubens: no bootable Debian rootfs on $CUST_PART, rescue shell" > /dev/kmsg
fi

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
