// SPDX-License-Identifier: GPL-2.0-only
/*
 * Trigger one DRM commit through the fbdev pan_display path and report the
 * result. Used to validate the rubens panel commit path from userspace.
 */

#define __NR_ioctl	29
#define __NR_openat	56
#define __NR_close	57
#define __NR_write	64

#define AT_FDCWD	(-100)
#define O_RDWR		2

#define FBIOPAN_DISPLAY	0x4606

struct fb_var_screeninfo_min {
	unsigned int xres;
	unsigned int yres;
	unsigned int xres_virtual;
	unsigned int yres_virtual;
	unsigned int xoffset;
	unsigned int yoffset;
	unsigned int bits_per_pixel;
	unsigned int grayscale;
	unsigned int red[4];
	unsigned int green[4];
	unsigned int blue[4];
	unsigned int transp[4];
	unsigned int nonstd;
	unsigned int activate;
	unsigned int height;
	unsigned int width;
	unsigned int accel_flags;
	unsigned int pixclock;
	unsigned int left_margin;
	unsigned int right_margin;
	unsigned int upper_margin;
	unsigned int lower_margin;
	unsigned int hsync_len;
	unsigned int vsync_len;
	unsigned int sync;
	unsigned int vmode;
	unsigned int rotate;
	unsigned int colorspace;
	unsigned int reserved[4];
};

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

static void put_num(long v)
{
	char buf[24];
	int i = 0;
	int neg = v < 0;

	if (neg)
		v = -v;
	if (!v) {
		puts("0");
		return;
	}
	while (v) {
		buf[i++] = '0' + v % 10;
		v /= 10;
	}
	if (neg)
		buf[i++] = '-';
	while (i--)
		sc(__NR_write, 1, (long)&buf[i], 1, 0);
}

void _start(void)
{
	static struct fb_var_screeninfo_min var;
	long fd, ret;

	fd = sc(__NR_openat, AT_FDCWD, (long)"/dev/fb0", O_RDWR, 0);
	if (fd < 0) {
		puts("open /dev/fb0 failed\n");
		return;
	}

	/* zeroed var: pan to offset 0/0, same as the current position */
	ret = sc(__NR_ioctl, fd, FBIOPAN_DISPLAY, (long)&var, 0);
	puts("FBIOPAN_DISPLAY ret=");
	put_num(ret);
	puts("\n");
	sc(__NR_close, fd, 0, 0, 0);
}
