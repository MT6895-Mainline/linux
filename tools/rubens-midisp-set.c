// SPDX-License-Identifier: GPL-2.0-only
/* Set the mi_disp primary brightness once. */

#define __NR_ioctl	29
#define __NR_openat	56
#define __NR_write	64

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

void _start(void)
{
	struct disp_brightness_req req = { };
	long fd, ret;

	fd = sc(__NR_openat, AT_FDCWD,
		(long)"/dev/mi_display/disp_feature", O_RDWR, 0);
	if (fd < 0)
		return;

	req.base.flag = 1;	/* MI_DISP_FLAG_NONBLOCK */
	req.base.disp_id = 0;	/* MI_DISP_PRIMARY */
	req.brightness = 2048;
	ret = sc(__NR_ioctl, fd, MI_DISP_IOCTL_SET_BRIGHTNESS, (long)&req, 0);
	(void)ret;
}
