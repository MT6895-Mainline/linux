// SPDX-License-Identifier: GPL-2.0
/* Device test: reservations without queued images, both codec orders and races. */
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

static int decoder(const char *path)
{
	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	struct v4l2_requestbuffers r = { .type = f.type,
		.memory = V4L2_MEMORY_MMAP, .count = 2 };
	int fd = open(path, O_RDWR | O_NONBLOCK);

	assert(fd >= 0);
	f.fmt.pix_mp.width = 320;
	f.fmt.pix_mp.height = 256;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f.fmt.pix_mp.num_planes = 1;
	f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
	assert(!ioctl(fd, VIDIOC_S_FMT, &f));
	assert(!ioctl(fd, VIDIOC_REQBUFS, &r));
	return fd;
}

static int start(int fd)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

	return ioctl(fd, VIDIOC_STREAMON, &type) ? errno : 0;
}

static int encfmt(int fd)
{
	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };

	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f.fmt.pix_mp.num_planes = 1;
	f.fmt.pix_mp.plane_fmt[0].sizeimage = 1024 * 1024;
	return ioctl(fd, VIDIOC_S_FMT, &f) ? errno : 0;
}

static void race(const char *dec)
{
	int ready[2], gate[2], hold[2], result[2], status, values[2];
	pid_t children[2];
	char byte = 0;

	assert(!pipe(ready) && !pipe(gate) && !pipe(hold) && !pipe(result));
	for (int i = 0; i < 2; i++) {
		children[i] = fork();
		assert(children[i] >= 0);
		if (!children[i]) {
			int fd = decoder(dec), ret;

			assert(write(ready[1], &byte, 1) == 1);
			assert(read(gate[0], &byte, 1) == 1);
			ret = start(fd);
			assert(write(result[1], &ret, sizeof(ret)) == sizeof(ret));
			assert(read(hold[0], &byte, 1) == 1);
			close(fd);
			_exit(0);
		}
	}
	for (int i = 0; i < 2; i++)
		assert(read(ready[0], &byte, 1) == 1);
	assert(write(gate[1], "xx", 2) == 2);
	for (int i = 0; i < 2; i++)
		assert(read(result[0], &values[i], sizeof(int)) == sizeof(int));
	assert((!values[0] && values[1] == EBUSY) ||
	       (!values[1] && values[0] == EBUSY));
	assert(write(hold[1], "xx", 2) == 2);
	for (int i = 0; i < 2; i++) {
		assert(waitpid(children[i], &status, 0) == children[i]);
		assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
		close(ready[i]); close(gate[i]); close(hold[i]); close(result[i]);
	}
}

int main(int argc, char **argv)
{
	int a, b, e;

	assert(argc == 3);
	a = decoder(argv[2]); b = decoder(argv[2]);
	assert(!start(a));
	assert(start(b) == EBUSY);
	close(a);
	assert(!start(b)); /* Retry the same handle after the owner leaves. */
	close(b);
	puts("PASS decoder reservation, EBUSY and same-handle retry");
	a = decoder(argv[2]);
	assert(!start(a));
	e = open(argv[1], O_RDWR | O_NONBLOCK);
	assert(e >= 0 && encfmt(e) == EBUSY);
	close(a);
	assert(!encfmt(e)); /* Encoder retries without reopening. */
	b = decoder(argv[2]);
	assert(start(b) == EBUSY);
	close(e);
	assert(!start(b));
	close(b);
	puts("PASS decoder-to-encoder and encoder-to-decoder exclusion, retry");
	for (int i = 0; i < 10; i++)
		race(argv[2]);
	puts("PASS 10 simultaneous decoder reservations: exactly one winner");
	return 0;
}
