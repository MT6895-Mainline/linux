/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Userspace API for the FPC1540/FPC15xx fingerprint sensor SPI driver.
 *
 * The sensor is a 112x88 capacitive array.  A READ_IMAGE streams two bytes per
 * pixel; the second byte carries the 2-bit noise-shaped pixel signal which the
 * driver low-pass decodes into an 8-bit 112x88 image.
 */
#ifndef _UAPI_LINUX_FPC1540_H
#define _UAPI_LINUX_FPC1540_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define FPC1540_IMG_W		112
#define FPC1540_IMG_H		88
#define FPC1540_IMG_BYTES	(FPC1540_IMG_W * FPC1540_IMG_H)
#define FPC1540_RAW_FRAME	(FPC1540_IMG_BYTES * 2)
/* Largest raw frame the driver will read when probing the readout format. */
#define FPC1540_RAW_MAX		65536

#define FPC1540_IOC_MAGIC	0xfc

/**
 * struct fpc1540_reg_io - register read/write payload
 * @reg:   register address
 * @len:   number of data bytes (1..64)
 * @dummy: non-zero to send one dummy byte between address and data
 * @data:  data bytes
 */
struct fpc1540_reg_io {
	__u8 reg;
	__u8 len;
	__u8 dummy;
	__u8 pad;
	__u8 data[64];
};

/**
 * struct fpc1540_cmd_io - send a command and wait for an IRQ status
 * @cmd:        command byte
 * @irq_mask:   IRQ bits to wait for (0 = do not wait)
 * @timeout_ms: wait timeout in milliseconds
 * @status:     IRQ register value on return
 */
struct fpc1540_cmd_io {
	__u8  cmd;
	__u8  irq_mask;
	__u8  pad[2];
	__u32 timeout_ms;
	__u8  status;
	__u8  pad2[3];
};

/**
 * struct fpc1540_image_io - decoded image description
 * @width:  image width in pixels
 * @height: image height in pixels
 * @bytes:  decoded size in bytes
 */
struct fpc1540_image_io {
	__u16 width;
	__u16 height;
	__u16 bytes;
	__u16 pad;
};

/* Power modes for FPC1540_IOC_POWER */
#define FPC1540_POWER_OFF	0
#define FPC1540_POWER_ON	1
#define FPC1540_POWER_CYCLE	2

#define FPC1540_IOC_GET_HWID		_IOR(FPC1540_IOC_MAGIC, 0x00, __u16)
#define FPC1540_IOC_POWER		_IOW(FPC1540_IOC_MAGIC, 0x01, __u32)
#define FPC1540_IOC_RESET		_IO(FPC1540_IOC_MAGIC, 0x02)
#define FPC1540_IOC_READ_REG		_IOWR(FPC1540_IOC_MAGIC, 0x03, struct fpc1540_reg_io)
#define FPC1540_IOC_WRITE_REG		_IOW(FPC1540_IOC_MAGIC, 0x04, struct fpc1540_reg_io)
#define FPC1540_IOC_SEND_CMD		_IOWR(FPC1540_IOC_MAGIC, 0x05, struct fpc1540_cmd_io)
#define FPC1540_IOC_CAPTURE		_IOR(FPC1540_IOC_MAGIC, 0x06, struct fpc1540_image_io)
#define FPC1540_IOC_WAIT_FINGER		_IOWR(FPC1540_IOC_MAGIC, 0x07, __u32)
#define FPC1540_IOC_FINGER_STATE	_IOR(FPC1540_IOC_MAGIC, 0x08, __u32)
/* Copy the last raw FPC1540_RAW_FRAME-byte stream (2 bytes/pixel). */
#define FPC1540_IOC_GET_RAW		_IOR(FPC1540_IOC_MAGIC, 0x09, __u8)

/**
 * struct fpc1540_raw_xfer - raw SPI transfer: transmit then receive
 * @tx_len: bytes to transmit (may be 0)
 * @rx_len: bytes to receive after the transmit (may be 0)
 * @tx:     user pointer to tx_len bytes
 * @rx:     user pointer to rx_len bytes
 *
 * Lets a userspace trustlet emulator drive the sensor at the SPI level, the
 * same primitive the vendor TEE platform layer (ut_pf_spi_send_and_receive)
 * would use.
 */
struct fpc1540_raw_xfer {
	__u32 tx_len;
	__u32 rx_len;
	__u64 tx;
	__u64 rx;
};

#define FPC1540_IOC_RAW_XFER		_IOWR(FPC1540_IOC_MAGIC, 0x0a, struct fpc1540_raw_xfer)

#define FPC1540_IOC_MAXNR		0x0a

#endif /* _UAPI_LINUX_FPC1540_H */
