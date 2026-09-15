// SPDX-License-Identifier: GPL-2.0-only
/*
 * Cycle the mi_disp primary brightness through /dev/mi_display so the panel
 * brightness path can be verified from the serial console.
 */

#define __NR_ioctl	29
#define __NR_openat	56
#define __NR_close	57
#define __NR_write	64
#define __NR_nanosleep	101

#define AT_FDCWD	(-100)
#define O_RDWR		2

struct disp_base {
	unsigned int flag;
	unsigned int disp_id;
};

struct disp_brightness_req {
	struct disp_base base;
	unsigned int brightness;
	unsigned int brightness_clone;
};

#define MI_DISP_IOCTL_SET_BRIGHTNESS \
	((1u << 30) | ((sizeof(struct disp_brightness_req) & 0x3fff) << 16) | \
	 ('D' << 8) | 0x0c)

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

static void put_num(unsigned int v)
{
	char buf[12];
	int i = 0;

	if (!v) {
		puts("0");
		return;
	}
	while (v) {
		buf[i++] = '0' + v % 10;
		v /= 10;
	}
	while (i--)
		sc(__NR_write, 1, (long)&buf[i], 1, 0);
}

void _start(void)
{
	static const unsigned int levels[] = {
		4095, 500, 8191, 100, 16383, 2000, 300, 0,
	};
	struct { long sec; long nsec; } delay = { 1, 0 };
	struct disp_brightness_req req = { };
	long fd;
	unsigned int i;

	fd = sc(__NR_openat, AT_FDCWD, (long)"/dev/mi_display/disp_feature", O_RDWR, 0);
	if (fd < 0) {
		puts("open /dev/mi_display failed\n");
		for (;;)
			sc(__NR_nanosleep, (long)&delay, 0, 0, 0);
	}
	req.base.flag = 1;	/* MI_DISP_FLAG_NONBLOCK */
	req.base.disp_id = 0;	/* MI_DISP_PRIMARY */
	puts("midisp-bright start\n");

	for (;;) {
		for (i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
			long ret;

			req.brightness = levels[i];
			ret = sc(__NR_ioctl, fd, MI_DISP_IOCTL_SET_BRIGHTNESS,
				 (long)&req, 0);
			puts("brightness=");
			put_num(levels[i]);
			puts(" ioctl=");
			put_num((unsigned int)ret);
			puts("\n");
			sc(__NR_nanosleep, (long)&delay, 0, 0, 0);
		}
	}
}
