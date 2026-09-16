// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal PID 1 for the Rubens flash-test image.
 *
 * Proves userspace was reached by writing a marker into the kernel log (which
 * rubens-bootlog mirrors to the oops partition), then dumps the MIPI-DSI bus,
 * driver and deferred-probe state so display bring-up can be diagnosed from
 * the persisted boot log alone. It finally sleeps forever.
 */

#define __NR_getdents64	61
#define __NR_read	63
#define __NR_mount	40
#define __NR_openat	56
#define __NR_close	57
#define __NR_write	64
#define __NR_readlinkat	78
#define __NR_nanosleep	101

#define AT_FDCWD	(-100)
#define O_RDONLY	0
#define O_WRONLY	1
#define O_DIRECTORY	00200000

static long syscall5(long nr, long a0, long a1, long a2, long a3, long a4)
{
	register long x8 __asm__("x8") = nr;
	register long x0 __asm__("x0") = a0;
	register long x1 __asm__("x1") = a1;
	register long x2 __asm__("x2") = a2;
	register long x3 __asm__("x3") = a3;
	register long x4 __asm__("x4") = a4;

	__asm__ volatile("svc #0"
			 : "+r"(x0)
			 : "r"(x1), "r"(x2), "r"(x3), "r"(x4), "r"(x8)
			 : "memory");

	return x0;
}

static long slen(const char *s)
{
	long n = 0;

	while (s[n])
		n++;
	return n;
}

static void build_path(char *dst, const char *a, const char *b, const char *c)
{
	while (*a)
		*dst++ = *a++;
	while (b && *b)
		*dst++ = *b++;
	while (c && *c)
		*dst++ = *c++;
	*dst = 0;
}

static void emit(const char *tag, const char *text);

static void check_path(const char *path)
{
	char tag[280];
	long fd;

	build_path(tag, "rubens-check: ", path, "");
	fd = syscall5(__NR_openat, AT_FDCWD, (long)path, O_RDONLY, 0, 0);
	if (fd >= 0) {
		syscall5(__NR_close, fd, 0, 0, 0, 0);
		emit(tag, "EXISTS");
	} else {
		emit(tag, "missing");
	}
}

static int kmsg_fd = -1;

static void emit(const char *tag, const char *text)
{
	char buf[512];
	long i = 0;

	if (kmsg_fd < 0)
		return;
	while (tag[i] && i < 400)
		buf[i] = tag[i], i++;
	if (text && text[0]) {
		long j = 0;

		while (text[j] && i < 508)
			buf[i++] = text[j++];
	}
	if (i == 0 || buf[i - 1] != '\n')
		buf[i++] = '\n';
	syscall5(__NR_write, kmsg_fd, (long)buf, i, 0, 0);
}

static void list_dir(const char *tag, const char *path)
{
	char buf[1024];
	long fd, n, off;

	emit(tag, "(dir)");
	fd = syscall5(__NR_openat, AT_FDCWD, (long)path, O_RDONLY | O_DIRECTORY,
		      0, 0);
	if (fd < 0)
		return;
	for (;;) {
		n = syscall5(__NR_getdents64, fd, (long)buf, sizeof(buf), 0, 0);
		if (n <= 0)
			break;
		off = 0;
		while (off < n) {
			unsigned short reclen = *(unsigned short *)(buf + off + 16);
			const char *name = buf + off + 19;

			if (name[0] != '.')
				emit("  ", name);
			off += reclen;
		}
	}
	syscall5(__NR_close, fd, 0, 0, 0, 0);
}

static void read_file(const char *tag, const char *path)
{
	char buf[512];
	long fd, n;

	fd = syscall5(__NR_openat, AT_FDCWD, (long)path, O_RDONLY, 0, 0);
	if (fd < 0)
		return;
	n = syscall5(__NR_read, fd, (long)buf, sizeof(buf) - 1, 0, 0);
	syscall5(__NR_close, fd, 0, 0, 0, 0);
	if (n <= 0)
		return;
	buf[n] = 0;
	emit(tag, buf);
}

static void read_link(const char *tag, const char *path)
{
	char buf[256];
	long n;

	n = syscall5(__NR_readlinkat, AT_FDCWD, (long)path, (long)buf,
		     sizeof(buf) - 1, 0);
	if (n <= 0)
		return;
	buf[n] = 0;
	emit(tag, buf);
}

static void diag_checks(void)
{
	check_path("/sys/bus/mipi-dsi");
	check_path("/sys/bus/mipi-dsi/devices");
	check_path("/sys/bus/mipi-dsi/drivers");
	check_path("/sys/bus/mipi-dsi/devices/lcm.0");
	check_path("/sys/bus/mipi-dsi/devices/lcm.0/modalias");
	check_path("/sys/bus/mipi-dsi/devices/lcm.0/driver");
	check_path("/sys/bus/mipi-dsi/drivers/panel-l11a-38-0a-0a-dsc-cmd");
	check_path("/sys/bus/mipi-dsi/drivers/panel-l11a-38-0a-0a-dsc-cmd/lcm.0");
	check_path("/sys/class/regulator");
	check_path("/sys/kernel/debug/regulator");
	check_path("/sys/bus/platform/devices/14017000.dsi");

	read_file("rubens-diag: lcm modalias:",
		  "/sys/bus/mipi-dsi/devices/lcm.0/modalias");
	read_file("rubens-diag: lcm uevent:",
		  "/sys/bus/mipi-dsi/devices/lcm.0/uevent");
	read_file("rubens-diag: panel driver uevent:",
		  "/sys/bus/mipi-dsi/drivers/panel-l11a-38-0a-0a-dsc-cmd/uevent");
	read_file("rubens-diag: regulator summary:",
		  "/sys/kernel/debug/regulator/regulator_summary");
	read_file("rubens-diag: deferred:",
		  "/sys/kernel/debug/devices_deferred");
}

void _start(void)
{
	static const char marker[] =
		"rubens-flash-test: userspace init running\n";
	struct { long sec; long nsec; } delay = { 5, 0 };

	syscall5(__NR_mount, (long)"devtmpfs", (long)"/dev",
		 (long)"devtmpfs", 0, 0);
	kmsg_fd = syscall5(__NR_openat, AT_FDCWD, (long)"/dev/kmsg", O_WRONLY,
			   0, 0);
	if (kmsg_fd >= 0)
		syscall5(__NR_write, kmsg_fd, (long)marker, sizeof(marker) - 1,
			 0, 0);

	syscall5(__NR_mount, (long)"sysfs", (long)"/sys", (long)"sysfs", 0, 0);
	syscall5(__NR_mount, (long)"debugfs", (long)"/sys/kernel/debug",
		 (long)"debugfs", 0, 0);
	syscall5(__NR_mount, (long)"proc", (long)"/proc", (long)"proc", 0, 0);

	read_file("rubens-diag: panel compatible:",
		  "/proc/device-tree/soc@0/dsi@14017000/panel1@0/compatible");
	read_file("rubens-diag: panel status:",
		  "/proc/device-tree/soc@0/dsi@14017000/panel1@0/status");
	diag_checks();

	for (;;)
		syscall5(__NR_nanosleep, (long)&delay, 0, 0, 0, 0);
}
