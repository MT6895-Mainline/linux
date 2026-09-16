// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hold DRM master on the rubens display.
 *
 * The kernel's fbdev refresh worker commits console frames through the DRM
 * fb helper, but drm_master_internal_acquire() refuses to commit while no
 * userspace DRM master exists. Stay alive holding master so the console keeps
 * refreshing on the panel.
 */

#define __NR_ioctl	29
#define __NR_openat	56
#define __NR_close	57
#define __NR_write	64
#define __NR_nanosleep	101

#define AT_FDCWD	(-100)
#define O_RDWR		2

/* DRM_IOCTL_SET_MASTER = _IO('d', 0x1e) */
#define DRM_IOCTL_SET_MASTER	0x0000641e

static long sc(long nr, long a0, long a1, long a2, long a3)
{
	register long x8 __asm__("x8") = nr;
	register long x0 __asm__("x0") = a0;
	register long x1 __asm__("x1") = a1;
	register long x2 __asm__("x2") = a2;
	register long x3 __asm__("x3") = a3;

	__asm__ volatile("svc #0"
			 : "+r"(x0)
			 : "r"(x1), "r"(x2), "r"(x3), "r"(x8)
			 : "memory");
	return x0;
}

static void puts(const char *s)
{
	long n = 0;

	while (s[n])
		n++;
	sc(__NR_write, 1, (long)s, n, 0);
}

static void kmsg(const char *s)
{
	long fd = sc(__NR_openat, AT_FDCWD, (long)"/dev/kmsg", 1, 0);

	if (fd >= 0) {
		long n = 0;

		while (s[n])
			n++;
		sc(__NR_write, fd, (long)s, n, 0);
		sc(__NR_close, fd, 0, 0, 0);
	}
	puts(s);
}

void _start(void)
{
	struct { long sec; long nsec; } delay = { 60, 0 };
	long fd, ret;

	fd = sc(__NR_openat, AT_FDCWD, (long)"/dev/dri/card0", O_RDWR, 0);
	if (fd < 0) {
		kmsg("rubens-drm-master: open card0 failed\n");
		for (;;)
			sc(__NR_nanosleep, (long)&delay, 0, 0, 0);
	}

	ret = sc(__NR_ioctl, fd, DRM_IOCTL_SET_MASTER, 0, 0);
	if (ret)
		kmsg("rubens-drm-master: SET_MASTER failed\n");
	else
		kmsg("rubens-drm-master: master acquired\n");

	for (;;)
		sc(__NR_nanosleep, (long)&delay, 0, 0, 0);
}
