/*
 * yavta --  Yet Another V4L2 Test Application
 *
 * Copyright (C) 2005-2010 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 */

#define __STDC_FORMAT_MACROS

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <sched.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <linux/videodev2.h>

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "bcm_host.h"
#include "user-vcsm.h"



#ifndef V4L2_BUF_FLAG_ERROR
#define V4L2_BUF_FLAG_ERROR	0x0040
#endif

#define ARRAY_SIZE(a)	(sizeof(a)/sizeof((a)[0]))

#define VCSM_IMPORT_DMABUF MMAL_TRUE

int debug = 1;
#define print(...) do { if (debug) printf(__VA_ARGS__); }  while (0)

enum buffer_fill_mode
{
	BUFFER_FILL_NONE = 0,
	BUFFER_FILL_FRAME = 1 << 0,
	BUFFER_FILL_PADDING = 1 << 1,
};

struct buffer
{
	unsigned int idx;
	unsigned int padding[VIDEO_MAX_PLANES];
	unsigned int size[VIDEO_MAX_PLANES];
	void *mem[VIDEO_MAX_PLANES];
	MMAL_BUFFER_HEADER_T *mmal;
	int dma_fd;
	unsigned int vcsm_handle;
};

struct device
{
	int fd;
	int opened;

	enum v4l2_buf_type type;
	enum v4l2_memory memtype;
	unsigned int nbufs;
	struct buffer *buffers;

	MMAL_COMPONENT_T *isp;
	MMAL_COMPONENT_T *render;
	MMAL_COMPONENT_T *encoder;
	MMAL_POOL_T *isp_output_pool;
	MMAL_POOL_T *render_pool;
	MMAL_POOL_T *encode_pool;

	/* V4L2 to MMAL interface */
	MMAL_QUEUE_T *isp_queue;
	MMAL_POOL_T *mmal_pool;
	/* Encoded data */
	MMAL_POOL_T *output_pool;



	unsigned int width;
	unsigned int height;
	unsigned int fps;
	unsigned int frame_time_usec;
	uint32_t buffer_output_flags;
	uint32_t timestamp_type;
	struct timeval starttime;
	int64_t lastpts;

	unsigned char num_planes;
	struct v4l2_plane_pix_format plane_fmt[VIDEO_MAX_PLANES];

	void *pattern[VIDEO_MAX_PLANES];
	unsigned int patternsize[VIDEO_MAX_PLANES];

	bool write_data_prefix;


	VCOS_THREAD_T save_thread;
	MMAL_QUEUE_T *save_queue;
	int thread_quit;
	FILE *h264_fd;
	FILE *pts_fd;
};

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static bool video_is_capture(struct device *dev)
{
	return dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

static bool video_is_output(struct device *dev)
{
	return  dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT;
}

#define MMAL_ENCODING_UNUSED 0

static struct v4l2_format_info {
	const char *name;
	unsigned int fourcc;
	unsigned char n_planes;
	MMAL_FOURCC_T mmal_encoding;
} pixel_formats[] = {
	{ "RGB332", V4L2_PIX_FMT_RGB332, 1, 	MMAL_ENCODING_UNUSED },
	{ "RGB444", V4L2_PIX_FMT_RGB444, 1,	MMAL_ENCODING_UNUSED },
	{ "ARGB444", V4L2_PIX_FMT_ARGB444, 1,	MMAL_ENCODING_UNUSED },
	{ "XRGB444", V4L2_PIX_FMT_XRGB444, 1,	MMAL_ENCODING_UNUSED },
	{ "RGB555", V4L2_PIX_FMT_RGB555, 1,	MMAL_ENCODING_UNUSED },
	{ "ARGB555", V4L2_PIX_FMT_ARGB555, 1,	MMAL_ENCODING_UNUSED },
	{ "XRGB555", V4L2_PIX_FMT_XRGB555, 1,	MMAL_ENCODING_UNUSED },
	{ "RGB565", V4L2_PIX_FMT_RGB565, 1,	MMAL_ENCODING_UNUSED },
	{ "RGB555X", V4L2_PIX_FMT_RGB555X, 1,	MMAL_ENCODING_UNUSED },
	{ "RGB565X", V4L2_PIX_FMT_RGB565X, 1,	MMAL_ENCODING_RGB16 },
	{ "BGR666", V4L2_PIX_FMT_BGR666, 1,	MMAL_ENCODING_UNUSED },
	{ "BGR24", V4L2_PIX_FMT_BGR24, 1,	MMAL_ENCODING_RGB24 },
	{ "RGB24", V4L2_PIX_FMT_RGB24, 1,	MMAL_ENCODING_BGR24 },
	{ "BGR32", V4L2_PIX_FMT_BGR32, 1,	MMAL_ENCODING_BGR32 },
	{ "ABGR32", V4L2_PIX_FMT_ABGR32, 1,	MMAL_ENCODING_BGRA },
	{ "XBGR32", V4L2_PIX_FMT_XBGR32, 1,	MMAL_ENCODING_BGR32 },
	{ "RGB32", V4L2_PIX_FMT_RGB32, 1,	MMAL_ENCODING_RGB32 },
	{ "ARGB32", V4L2_PIX_FMT_ARGB32, 1,	MMAL_ENCODING_ARGB },
	{ "XRGB32", V4L2_PIX_FMT_XRGB32, 1,	MMAL_ENCODING_UNUSED },
	{ "HSV24", V4L2_PIX_FMT_HSV24, 1,	MMAL_ENCODING_UNUSED },
	{ "HSV32", V4L2_PIX_FMT_HSV32, 1,	MMAL_ENCODING_UNUSED },
	{ "Y8", V4L2_PIX_FMT_GREY, 1,		MMAL_ENCODING_UNUSED },
	{ "Y10", V4L2_PIX_FMT_Y10, 1,		MMAL_ENCODING_UNUSED },
	{ "Y12", V4L2_PIX_FMT_Y12, 1,		MMAL_ENCODING_UNUSED },
	{ "Y16", V4L2_PIX_FMT_Y16, 1,		MMAL_ENCODING_UNUSED },
	{ "UYVY", V4L2_PIX_FMT_UYVY, 1,		MMAL_ENCODING_UYVY },
	{ "VYUY", V4L2_PIX_FMT_VYUY, 1,		MMAL_ENCODING_VYUY },
	{ "YUYV", V4L2_PIX_FMT_YUYV, 1,		MMAL_ENCODING_YUYV },
	{ "YVYU", V4L2_PIX_FMT_YVYU, 1,		MMAL_ENCODING_YVYU },
	{ "NV12", V4L2_PIX_FMT_NV12, 1,		MMAL_ENCODING_NV12 },
	{ "NV12M", V4L2_PIX_FMT_NV12M, 2,	MMAL_ENCODING_UNUSED },
	{ "NV21", V4L2_PIX_FMT_NV21, 1,		MMAL_ENCODING_NV21 },
	{ "NV21M", V4L2_PIX_FMT_NV21M, 2,	MMAL_ENCODING_UNUSED },
	{ "NV16", V4L2_PIX_FMT_NV16, 1,		MMAL_ENCODING_UNUSED },
	{ "NV16M", V4L2_PIX_FMT_NV16M, 2,	MMAL_ENCODING_UNUSED },
	{ "NV61", V4L2_PIX_FMT_NV61, 1,		MMAL_ENCODING_UNUSED },
	{ "NV61M", V4L2_PIX_FMT_NV61M, 2,	MMAL_ENCODING_UNUSED },
	{ "NV24", V4L2_PIX_FMT_NV24, 1,		MMAL_ENCODING_UNUSED },
	{ "NV42", V4L2_PIX_FMT_NV42, 1,		MMAL_ENCODING_UNUSED },
	{ "YUV420M", V4L2_PIX_FMT_YUV420M, 3,	MMAL_ENCODING_UNUSED },
	{ "YUV422M", V4L2_PIX_FMT_YUV422M, 3,	MMAL_ENCODING_UNUSED },
	{ "YUV444M", V4L2_PIX_FMT_YUV444M, 3,	MMAL_ENCODING_UNUSED },
	{ "YVU420M", V4L2_PIX_FMT_YVU420M, 3,	MMAL_ENCODING_UNUSED },
	{ "YVU422M", V4L2_PIX_FMT_YVU422M, 3,	MMAL_ENCODING_UNUSED },
	{ "YVU444M", V4L2_PIX_FMT_YVU444M, 3,	MMAL_ENCODING_UNUSED },
	{ "SBGGR8", V4L2_PIX_FMT_SBGGR8, 1,	MMAL_ENCODING_BAYER_SBGGR8 },
	{ "SGBRG8", V4L2_PIX_FMT_SGBRG8, 1,	MMAL_ENCODING_BAYER_SGBRG8 },
	{ "SGRBG8", V4L2_PIX_FMT_SGRBG8, 1,	MMAL_ENCODING_BAYER_SGRBG8 },
	{ "SRGGB8", V4L2_PIX_FMT_SRGGB8, 1,	MMAL_ENCODING_BAYER_SRGGB8 },
	{ "SBGGR10_DPCM8", V4L2_PIX_FMT_SBGGR10DPCM8, 1,	MMAL_ENCODING_UNUSED },
	{ "SGBRG10_DPCM8", V4L2_PIX_FMT_SGBRG10DPCM8, 1,	MMAL_ENCODING_UNUSED },
	{ "SGRBG10_DPCM8", V4L2_PIX_FMT_SGRBG10DPCM8, 1,	MMAL_ENCODING_UNUSED },
	{ "SRGGB10_DPCM8", V4L2_PIX_FMT_SRGGB10DPCM8, 1,	MMAL_ENCODING_UNUSED },
	{ "SBGGR10", V4L2_PIX_FMT_SBGGR10, 1,	MMAL_ENCODING_UNUSED },
	{ "SGBRG10", V4L2_PIX_FMT_SGBRG10, 1,	MMAL_ENCODING_UNUSED },
	{ "SGRBG10", V4L2_PIX_FMT_SGRBG10, 1,	MMAL_ENCODING_UNUSED },
	{ "SRGGB10", V4L2_PIX_FMT_SRGGB10, 1,	MMAL_ENCODING_UNUSED },
	{ "SBGGR10P", V4L2_PIX_FMT_SBGGR10P, 1,	MMAL_ENCODING_BAYER_SBGGR10P },
	{ "SGBRG10P", V4L2_PIX_FMT_SGBRG10P, 1,	MMAL_ENCODING_BAYER_SGBRG10P },
	{ "SGRBG10P", V4L2_PIX_FMT_SGRBG10P, 1,	MMAL_ENCODING_BAYER_SGRBG10P },
	{ "SRGGB10P", V4L2_PIX_FMT_SRGGB10P, 1,	MMAL_ENCODING_BAYER_SRGGB10P },
	{ "SBGGR12", V4L2_PIX_FMT_SBGGR12, 1,	MMAL_ENCODING_UNUSED },
	{ "SGBRG12", V4L2_PIX_FMT_SGBRG12, 1,	MMAL_ENCODING_UNUSED },
	{ "SGRBG12", V4L2_PIX_FMT_SGRBG12, 1,	MMAL_ENCODING_UNUSED },
	{ "SRGGB12", V4L2_PIX_FMT_SRGGB12, 1,	MMAL_ENCODING_UNUSED },
	{ "DV", V4L2_PIX_FMT_DV, 1,		MMAL_ENCODING_UNUSED },
	{ "MJPEG", V4L2_PIX_FMT_MJPEG, 1,	MMAL_ENCODING_UNUSED },
	{ "MPEG", V4L2_PIX_FMT_MPEG, 1,		MMAL_ENCODING_UNUSED },
};


static const struct v4l2_format_info *v4l2_format_by_fourcc(unsigned int fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pixel_formats); ++i) {
		if (pixel_formats[i].fourcc == fourcc)
			return &pixel_formats[i];
	}

	return NULL;
}


static void video_set_buf_type(struct device *dev, enum v4l2_buf_type type)
{
	dev->type = type;
}

static bool video_has_valid_buf_type(struct device *dev)
{
	return (int)dev->type != -1;
}

static void video_init(struct device *dev)
{
	memset(dev, 0, sizeof *dev);
	dev->fd = -1;
	dev->memtype = V4L2_MEMORY_MMAP;
	dev->buffers = NULL;
	dev->type = (enum v4l2_buf_type)-1;
}

static bool video_has_fd(struct device *dev)
{
	return dev->fd != -1;
}



static int video_open(struct device *dev, const char *devname)
{
	if (video_has_fd(dev)) {
		print("Can't open device (already open).\n");
		return -1;
	}

	dev->fd = open(devname, O_RDWR);
	if (dev->fd < 0) {
		print("Error opening device %s: %s (%d).\n", devname,
		       strerror(errno), errno);
		return dev->fd;
	}

	print("Device %s opened.\n", devname);

	dev->opened = 1;

	return 0;
}

static int video_querycap(struct device *dev, unsigned int *capabilities)
{
	struct v4l2_capability cap;
	unsigned int caps;
	int ret;

	memset(&cap, 0, sizeof cap);
	ret = ioctl(dev->fd, VIDIOC_QUERYCAP, &cap);
	if (ret < 0)
		return 0;

	caps = cap.capabilities & V4L2_CAP_DEVICE_CAPS
	     ? cap.device_caps : cap.capabilities;

	print("Device `%s' on `%s' (driver '%s') is a video %s (%s mplanes) device.\n",
		cap.card, cap.bus_info, cap.driver,
		caps & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_CAPTURE) ? "capture" : "output",
		caps & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE) ? "with" : "without");

	*capabilities = caps;

	return 0;
}

static int cap_get_buf_type(unsigned int capabilities)
{
	if (capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
		return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	} else if (capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE) {
		return V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	} else if (capabilities & V4L2_CAP_VIDEO_CAPTURE) {
		return  V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else if (capabilities & V4L2_CAP_VIDEO_OUTPUT) {
		return V4L2_BUF_TYPE_VIDEO_OUTPUT;
	} else {
		print("Device supports neither capture nor output.\n");
		return -EINVAL;
	}

	return 0;
}

static void video_close(struct device *dev)
{
	free(dev->buffers);
	if (dev->opened)
		close(dev->fd);
}

static int video_get_format(struct device *dev)
{
	struct v4l2_format fmt;
	int ret;

	memset(&fmt, 0, sizeof fmt);
	fmt.type = dev->type;

	ret = ioctl(dev->fd, VIDIOC_G_FMT, &fmt);
	if (ret < 0) {
		print("Unable to get format: %s (%d).\n", strerror(errno),
			errno);
		return ret;
	}


		dev->width = fmt.fmt.pix.width;
		dev->height = fmt.fmt.pix.height;
		dev->num_planes = 1;

		dev->plane_fmt[0].bytesperline = fmt.fmt.pix.bytesperline;
		dev->plane_fmt[0].sizeimage = fmt.fmt.pix.bytesperline ? fmt.fmt.pix.sizeimage : 0;

	return 0;
}

static int format_bpp(__u32 pixfmt)
{
	switch(pixfmt)
	{
		case V4L2_PIX_FMT_BGR24:
		case V4L2_PIX_FMT_RGB24:
			return 4;
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_VYUY:
			return 2;
		case V4L2_PIX_FMT_SRGGB8:
		case V4L2_PIX_FMT_SBGGR8:
		case V4L2_PIX_FMT_SGRBG8:
		case V4L2_PIX_FMT_SGBRG8:
			return 1;
		default:
			return 1;
	}
}

static int video_set_format(struct device *dev, unsigned int w, unsigned int h,
			    unsigned int format, unsigned int stride,
			    unsigned int buffer_size, enum v4l2_field field,
			    unsigned int flags)
{
	struct v4l2_format fmt;
	int ret;

	memset(&fmt, 0, sizeof fmt);
	fmt.type = dev->type;

{
		fmt.fmt.pix.width = w;
		fmt.fmt.pix.height = h;
		fmt.fmt.pix.pixelformat = format;
		fmt.fmt.pix.field = field;
		print("stride is %d\n",stride);
		if (!stride)
			stride = ((w+31) &~31)*format_bpp(format);
		print("stride is now %d\n",stride);
		fmt.fmt.pix.bytesperline = stride;
		fmt.fmt.pix.sizeimage = buffer_size;
		fmt.fmt.pix.priv = V4L2_PIX_FMT_PRIV_MAGIC;
		fmt.fmt.pix.flags = flags;
	}

	ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt);
	if (ret < 0) {
		print("Unable to set format: %s (%d).\n", strerror(errno),
			errno);
		return ret;
	}

	return 0;
}

static int buffer_export(int v4l2fd, enum v4l2_buf_type bt, int index, int *dmafd, unsigned int *vcsm_hdl)
{
	struct v4l2_exportbuffer expbuf;
	unsigned int vcsm_handle;

	memset(&expbuf, 0, sizeof(expbuf));
	expbuf.type = bt;
	expbuf.index = index;
	if (ioctl(v4l2fd, VIDIOC_EXPBUF, &expbuf))
	{
		print("Failed to EXPBUF\n");
		return -1;
	}
	*dmafd = expbuf.fd;

	print("Importing DMABUF %d into VCSM...\n", expbuf.fd);
	vcsm_handle = vcsm_import_dmabuf(expbuf.fd, "V4L2 buf");
	print("...done. vcsm_handle %u\n", vcsm_handle);
	*vcsm_hdl = vcsm_handle;
	return 0;
}

static int video_buffer_mmap(struct device *dev, struct buffer *buffer,
			     struct v4l2_buffer *v4l2buf)
{
	unsigned int length;
	unsigned int offset;
	unsigned int i;

	for (i = 0; i < dev->num_planes; i++) {
{
			length = v4l2buf->length;
			offset = v4l2buf->m.offset;
		}

		buffer->mem[i] = mmap(0, length, PROT_READ | PROT_WRITE, MAP_SHARED,
				      dev->fd, offset);
		if (buffer->mem[i] == MAP_FAILED) {
			print("Unable to map buffer %u/%u: %s (%d)\n",
			       buffer->idx, i, strerror(errno), errno);
			return -1;
		}

		buffer->size[i] = length;
		buffer->padding[i] = 0;

		print("Buffer %u/%u mapped at address %p.\n",
		       buffer->idx, i, buffer->mem[i]);
	}

	return 0;
}

static int video_buffer_munmap(struct device *dev, struct buffer *buffer)
{
	unsigned int i;
	int ret;

	for (i = 0; i < dev->num_planes; i++) {
		ret = munmap(buffer->mem[i], buffer->size[i]);
		if (ret < 0) {
			print("Unable to unmap buffer %u/%u: %s (%d)\n",
			       buffer->idx, i, strerror(errno), errno);
		}

		buffer->mem[i] = NULL;
	}

	return 0;
}

static int video_buffer_alloc_userptr(struct device *dev, struct buffer *buffer,
				      struct v4l2_buffer *v4l2buf,
				      unsigned int offset, unsigned int padding)
{
	int page_size = getpagesize();
	unsigned int length;
	unsigned int i;
	int ret;

	for (i = 0; i < dev->num_planes; i++) {

			length = v4l2buf->length;

		ret = posix_memalign(&buffer->mem[i], page_size,
				     length + offset + padding);
		if (ret < 0) {
			print("Unable to allocate buffer %u/%u (%d)\n",
			       buffer->idx, i, ret);
			return -ENOMEM;
		}

		buffer->mem[i] += offset;
		buffer->size[i] = length;
		buffer->padding[i] = padding;

		print("Buffer %u/%u allocated at address %p.\n",
		       buffer->idx, i, buffer->mem[i]);
	}

	return 0;
}

static void video_buffer_free_userptr(struct device *dev, struct buffer *buffer)
{
	unsigned int i;

	for (i = 0; i < dev->num_planes; i++) {
		free(buffer->mem[i]);
		buffer->mem[i] = NULL;
	}
}

static void video_buffer_fill_userptr(struct device *dev, struct buffer *buffer,
				      struct v4l2_buffer *v4l2buf)
{
	unsigned int i;

	{
		v4l2buf->m.userptr = (unsigned long)buffer->mem[0];
		return;
	}

	for (i = 0; i < dev->num_planes; i++)
		v4l2buf->m.planes[i].m.userptr = (unsigned long)buffer->mem[i];
}

static void get_ts_flags(uint32_t flags, const char **ts_type, const char **ts_source)
{
	switch (flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) {
	case V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
		*ts_type = "unk";
		break;
	case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
		*ts_type = "mono";
		break;
	case V4L2_BUF_FLAG_TIMESTAMP_COPY:
		*ts_type = "copy";
		break;
	default:
		*ts_type = "inv";
	}
	switch (flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) {
	case V4L2_BUF_FLAG_TSTAMP_SRC_EOF:
		*ts_source = "EoF";
		break;
	case V4L2_BUF_FLAG_TSTAMP_SRC_SOE:
		*ts_source = "SoE";
		break;
	default:
		*ts_source = "inv";
	}
}

static int video_alloc_buffers(struct device *dev, int nbufs,
	unsigned int offset, unsigned int padding)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct v4l2_requestbuffers rb;
	struct v4l2_buffer buf;
	struct buffer *buffers;
	unsigned int i;
	int ret;

	memset(&rb, 0, sizeof rb);
	rb.count = nbufs;
	rb.type = dev->type;
	rb.memory = dev->memtype;

	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		print("Unable to request buffers: %s (%d).\n", strerror(errno),
			errno);
		return ret;
	}

	print("%u buffers requested.\n", rb.count);

	buffers = malloc(rb.count * sizeof buffers[0]);
	if (buffers == NULL)
		return -ENOMEM;

	/* Map the buffers. */
	for (i = 0; i < rb.count; ++i) {
		const char *ts_type, *ts_source;

		memset(&buf, 0, sizeof buf);
		memset(planes, 0, sizeof planes);

		buf.index = i;
		buf.type = dev->type;
		buf.memory = dev->memtype;
		buf.length = VIDEO_MAX_PLANES;
		buf.m.planes = planes;

		ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			print("Unable to query buffer %u: %s (%d).\n", i,
				strerror(errno), errno);
			return ret;
		}
		get_ts_flags(buf.flags, &ts_type, &ts_source);
		print("length: %u offset: %u timestamp type/source: %s/%s\n",
		       buf.length, buf.m.offset, ts_type, ts_source);

		buffers[i].idx = i;

		switch (dev->memtype)
		{
		case V4L2_MEMORY_MMAP:
			ret = video_buffer_mmap(dev, &buffers[i], &buf);
			break;

		case V4L2_MEMORY_USERPTR:
			ret = video_buffer_alloc_userptr(dev, &buffers[i], &buf, offset, padding);
			break;

		default:
			break;
		}

		if (ret < 0)
			return ret;

		if (VCSM_IMPORT_DMABUF && !buffer_export(dev->fd, dev->type, i, &buffers[i].dma_fd, &buffers[i].vcsm_handle))
		{
			print("Exported buffer %d to dmabuf %d, vcsm handle %u\n", i, buffers[i].dma_fd, buffers[i].vcsm_handle);
		}
		if (dev->mmal_pool) {
			MMAL_BUFFER_HEADER_T *mmal_buf;
			mmal_buf = mmal_queue_get(dev->mmal_pool->queue);
			if (!mmal_buf) {
				print("Failed to get a buffer from the pool. Queue length %d\n", mmal_queue_length(dev->mmal_pool->queue));
				return -1;
			}
			mmal_buf->user_data = &buffers[i];

			if (VCSM_IMPORT_DMABUF)
				mmal_buf->data = (uint8_t*)vcsm_vc_hdl_from_hdl(buffers[i].vcsm_handle);
			else
				mmal_buf->data = buffers[i].mem[0];
			mmal_buf->alloc_size = buf.length;
			buffers[i].mmal = mmal_buf;
			print("Linking V4L2 buffer index %d ptr %p to MMAL header %p. mmal->data 0x%X\n",
				i, &buffers[i], mmal_buf, (uint32_t)mmal_buf->data);
			/* Put buffer back in the pool */
			mmal_buffer_header_release(mmal_buf);
		}
	}

	dev->timestamp_type = buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
	dev->buffers = buffers;
	dev->nbufs = rb.count;
	return 0;
}

static int video_free_buffers(struct device *dev)
{
	struct v4l2_requestbuffers rb;
	unsigned int i;
	int ret;

	if (dev->nbufs == 0)
		return 0;

	for (i = 0; i < dev->nbufs; ++i) {
		switch (dev->memtype) {
		case V4L2_MEMORY_MMAP:
			if (dev->buffers[i].vcsm_handle)
			{
				print("Releasing vcsm handle %u\n", dev->buffers[i].vcsm_handle);
				vcsm_free(dev->buffers[i].vcsm_handle);
			}
			if (dev->buffers[i].dma_fd)
			{
				print("Closing dma_buf %d\n", dev->buffers[i].dma_fd);
				close(dev->buffers[i].dma_fd);
			}
			ret = video_buffer_munmap(dev, &dev->buffers[i]);
			if (ret < 0)
				return ret;
			break;
		case V4L2_MEMORY_USERPTR:
			video_buffer_free_userptr(dev, &dev->buffers[i]);
			break;
		default:
			break;
		}
	}

	memset(&rb, 0, sizeof rb);
	rb.count = 0;
	rb.type = dev->type;
	rb.memory = dev->memtype;

	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		print("Unable to release buffers: %s (%d).\n",
			strerror(errno), errno);
		return ret;
	}

	print("%u buffers released.\n", dev->nbufs);

	free(dev->buffers);
	dev->nbufs = 0;
	dev->buffers = NULL;

	return 0;
}

static int video_queue_buffer(struct device *dev, int index, enum buffer_fill_mode fill)
{
	struct v4l2_buffer buf;
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	int ret;
	unsigned int i;

	memset(&buf, 0, sizeof buf);
	memset(&planes, 0, sizeof planes);

	buf.index = index;
	buf.type = dev->type;
	buf.memory = dev->memtype;

	if (video_is_output(dev)) {
		buf.flags = dev->buffer_output_flags;
		if (dev->timestamp_type == V4L2_BUF_FLAG_TIMESTAMP_COPY) {
			struct timespec ts;

			clock_gettime(CLOCK_MONOTONIC, &ts);
			buf.timestamp.tv_sec = ts.tv_sec;
			buf.timestamp.tv_usec = ts.tv_nsec / 1000;
		}
	}


	if (dev->memtype == V4L2_MEMORY_USERPTR) {
	 {
			buf.m.userptr = (unsigned long)dev->buffers[index].mem[0];
			buf.length = dev->buffers[index].size[0];
		}
	}

	for (i = 0; i < dev->num_planes; i++) {
		if (video_is_output(dev)) {

				buf.bytesused = dev->patternsize[i];

			memcpy(dev->buffers[buf.index].mem[i], dev->pattern[i],
			       dev->patternsize[i]);
		} else {
			if (fill & BUFFER_FILL_FRAME)
				memset(dev->buffers[buf.index].mem[i], 0x55,
				       dev->buffers[index].size[i]);
			if (fill & BUFFER_FILL_PADDING)
				memset(dev->buffers[buf.index].mem[i] +
					dev->buffers[index].size[i],
				       0x55, dev->buffers[index].padding[i]);
		}
	}

	ret = ioctl(dev->fd, VIDIOC_QBUF, &buf);
	if (ret < 0)
		print("Unable to queue buffer: %s (%d).\n",
			strerror(errno), errno);

	return ret;
}

static int video_enable(struct device *dev, int enable)
{
	int type = dev->type;
	int ret;

	ret = ioctl(dev->fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type);
	if (ret < 0) {
		print("Unable to %s streaming: %s (%d).\n",
			enable ? "start" : "stop", strerror(errno), errno);
		return ret;
	}

	return 0;
}

static int video_prepare_capture(struct device *dev, int nbufs, unsigned int offset, enum buffer_fill_mode fill)
{
	unsigned int padding;
	int ret;

	/* Allocate and map buffers. */
	padding = (fill & BUFFER_FILL_PADDING) ? 4096 : 0;
	if ((ret = video_alloc_buffers(dev, nbufs, offset, padding)) < 0)
		return ret;

	return 0;
}

static int video_queue_all_buffers(struct device *dev, enum buffer_fill_mode fill)
{
	unsigned int i;
	int ret;

	/* Queue the buffers. */
	for (i = 0; i < dev->nbufs; ++i) {
		ret = video_queue_buffer(dev, i, fill);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void video_verify_buffer(struct device *dev, struct v4l2_buffer *buf)
{
	struct buffer *buffer = &dev->buffers[buf->index];
	unsigned int plane;
	unsigned int i;

	for (plane = 0; plane < dev->num_planes; ++plane) {
		const uint8_t *data = buffer->mem[plane] + buffer->size[plane];
		unsigned int errors = 0;
		unsigned int dirty = 0;

		if (buffer->padding[plane] == 0)
			continue;

		for (i = 0; i < buffer->padding[plane]; ++i) {
			if (data[i] != 0x55) {
				errors++;
				dirty = i + 1;
			}
		}

		if (errors) {
			print("Warning: %u bytes overwritten among %u first padding bytes for plane %u\n",
			       errors, dirty, plane);

			dirty = (dirty + 15) & ~15;
			dirty = dirty > 32 ? 32 : dirty;

			for (i = 0; i < dirty; ++i) {
				print("%02x ", data[i]);
				if (i % 16 == 15)
					print("\n");
			}
		}
	}
}

static void isp_ip_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	struct device *dev = (struct device *)port->userdata;
	unsigned int i;
//	print("Buffer %p (->data %p) returned\n", buffer, buffer->data);
	for (i = 0; i < dev->nbufs; i++) {
		if (dev->buffers[i].mmal == buffer) {
//			print("Matches V4L2 buffer index %d / %d\n", i, dev->buffers[i].idx);
			video_queue_buffer(dev, dev->buffers[i].idx, BUFFER_FILL_NONE);
			mmal_buffer_header_release(buffer);
			buffer = NULL;
			break;
		}
	}
	if (buffer) {
		print("Failed to find matching V4L2 buffer for mmal buffer %p\n", buffer);
		mmal_buffer_header_release(buffer);
	}
}

static void * save_thread(void *arg)
{
	struct device *dev = (struct device *)arg;
	MMAL_BUFFER_HEADER_T *buffer;
	MMAL_STATUS_T status;
	unsigned int bytes_written;

	while (!dev->thread_quit)
	{
		//Being lazy and using a timed wait instead of setting up a
		//mechanism for skipping this when destroying the thread
		buffer = mmal_queue_timedwait(dev->save_queue, 50000);
		if (!buffer)
			continue;

		//print("Buffer %p saving, filled %d, timestamp %llu, flags %04X\n", buffer, buffer->length, buffer->pts, buffer->flags);
		if (dev->h264_fd)
		{
			bytes_written = fwrite(buffer->data, 1, buffer->length, dev->h264_fd);
			fflush(dev->h264_fd);

			if (bytes_written != buffer->length)
			{
				print("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
			}
		}
		else
		{
			print("No file to save to\n");
		}
		if (dev->pts_fd &&
		    !(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
		    buffer->pts != MMAL_TIME_UNKNOWN)
			fprintf(dev->pts_fd, "%lld.%03lld\n", buffer->pts/1000, buffer->pts%1000);

		buffer->length = 0;
		status = mmal_port_send_buffer(dev->encoder->output[0], buffer);
		if(status != MMAL_SUCCESS)
		{
			print("mmal_port_send_buffer failed on buffer %p, status %d", buffer, status);
		}
	}
	return NULL;
}

static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	struct device *dev = (struct device *)port->userdata;

	//print("Buffer %p returned, filled %d, timestamp %llu, flags %04X\n", buffer, buffer->length, buffer->pts, buffer->flags);
	//vcos_log_error("File handle: %p", port->userdata);

	if (port->is_enabled)
		mmal_queue_put(dev->save_queue, buffer);
	else
		mmal_buffer_header_release(buffer);
}

static void buffers_to_isp(struct device *dev)
{
	MMAL_BUFFER_HEADER_T *buffer;

	while ((buffer = mmal_queue_get(dev->isp_output_pool->queue)) != NULL)
	{
		mmal_port_send_buffer(dev->isp->output[0], buffer);
	}

}
static void isp_output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	//print("Buffer %p from isp, filled %d, timestamp %llu, flags %04X\n", buffer, buffer->length, buffer->pts, buffer->flags);
	//vcos_log_error("File handle: %p", port->userdata);
	struct device *dev = (struct device*)port->userdata;

	if (dev->render)
	{
		MMAL_BUFFER_HEADER_T *out = mmal_queue_get(dev->render_pool->queue);
		if (out)
		{
			mmal_buffer_header_replicate(out, buffer);
			mmal_port_send_buffer(dev->render->input[0], out);
		}
	}
	if (dev->encoder)
	{
		MMAL_BUFFER_HEADER_T *out = mmal_queue_get(dev->encode_pool->queue);
		if (out)
		{
			mmal_buffer_header_replicate(out, buffer);
			mmal_port_send_buffer(dev->encoder->input[0], out);
		}
	}
	mmal_buffer_header_release(buffer);

	buffers_to_isp(dev);
}

static void render_encoder_input_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	//print("Buffer %p returned from %s, filled %d, timestamp %llu, flags %04X\n", buffer, port->name, buffer->length, buffer->pts, buffer->flags);
	//vcos_log_error("File handle: %p", port->userdata);
	struct device *dev = (struct device*)port->userdata;

	mmal_buffer_header_release(buffer);

	buffers_to_isp(dev);
}

#define LOG_DEBUG print


int video_set_dv_timings(struct device *dev);

 int handle_event(struct device *dev)
{
	struct v4l2_event ev;

	while (!ioctl(dev->fd, VIDIOC_DQEVENT, &ev))
	{
		switch (ev.type)
		{
		case V4L2_EVENT_SOURCE_CHANGE:
			//fprintf(stderr, "Source changed\n");

			 video_enable(dev, 0);
			 video_free_buffers(dev);
			 destroy_mmal(dev);
			 video_close(dev);

			return start_video("/dev/video0");


			break;
		case V4L2_EVENT_EOS:
			fprintf(stderr, "EOS\n");
			break;
		}
	}

	return 0;
}

static int setup_mmal(struct device *dev, int nbufs, int do_encode)
{
	MMAL_STATUS_T status;
	VCOS_STATUS_T vcos_status;
	MMAL_PORT_T *port;
	const struct v4l2_format_info *info;
	struct v4l2_format fmt;
	int ret;
	MMAL_PORT_T *isp_output, *encoder_input = NULL, *encoder_output = NULL;

	//FIXME: Clean up after errors

	status = mmal_component_create("vc.ril.isp", &dev->isp);
	if(status != MMAL_SUCCESS)
	{
		print("Failed to create isp\n");
		return -1;
	}

	if (do_encode)
	{
		status = mmal_component_create("vc.ril.video_encode", &dev->encoder);
		if(status != MMAL_SUCCESS)
		{
			print("Failed to create encoder");
			return -1;
		}
	}

	if (1)
	{
		status = mmal_component_create("vc.ril.video_render", &dev->render);
		if(status != MMAL_SUCCESS)
		{
			print("Failed to create render\n");
			return -1;
		}
	}

	port = dev->isp->input[0];

	memset(&fmt, 0, sizeof fmt);
	fmt.type = dev->type;

	ret = ioctl(dev->fd, VIDIOC_G_FMT, &fmt);
	if (ret < 0) {
		print("Unable to get format: %s (%d).\n", strerror(errno), errno);
		return ret;
	}

	info = v4l2_format_by_fourcc(fmt.fmt.pix.pixelformat);

	if (!info || info->mmal_encoding == MMAL_ENCODING_UNUSED)
	{
		print("Unsupported encoding\n");
		return -1;
	}

	port->format->encoding = info->mmal_encoding;
	port->format->es->video.crop.width = fmt.fmt.pix.width;
	port->format->es->video.crop.height = fmt.fmt.pix.height;
	port->format->es->video.width = (port->format->es->video.crop.width+31) & ~31;

	port->format->es->video.height = (fmt.fmt.pix.height+15) & ~15;
	port->buffer_num = nbufs;

	if (dev->fps) {
		dev->frame_time_usec = 1000000/dev->fps;
	}

	status = mmal_port_format_commit(port);
	if (status != MMAL_SUCCESS)
	{
		print("Commit failed\n");
		return -1;
	}


	unsigned int mmal_stride = mmal_encoding_width_to_stride(info->mmal_encoding, port->format->es->video.width);
	if (mmal_stride != fmt.fmt.pix.bytesperline) {
		if (video_set_format(dev, fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat, mmal_stride,
				     fmt.fmt.pix.sizeimage, fmt.fmt.pix.field, fmt.fmt.pix.flags) < 0)
			print("Failed to adjust stride\n");
		else
			// Retrieve settings again so local state is correct
			video_get_format(dev);
	}


	if (mmal_port_parameter_set_boolean(port, MMAL_PARAMETER_ZERO_COPY, VCSM_IMPORT_DMABUF) != MMAL_SUCCESS)
	{
		print("Failed to set zero copy\n");
		return -1;
	}
	dev->mmal_pool = mmal_pool_create(nbufs, 0);
	if (!dev->mmal_pool) {
		print("Failed to create pool\n");
		return -1;
	}
	print("Created pool of length %d, size %d\n", nbufs, 0);

	port->userdata = (struct MMAL_PORT_USERDATA_T *)dev;
	status = mmal_port_enable(port, isp_ip_cb);
	if (status != MMAL_SUCCESS)
	{
		print("ISP input enable failed\n");
		return -1;
	}

	mmal_format_copy(dev->isp->output[0]->format, port->format);
	port = dev->isp->output[0];
	port->format->encoding = MMAL_ENCODING_I420;
	port->buffer_num = 3;

	status = mmal_port_format_commit(port);
	if (status != MMAL_SUCCESS)
	{
		print("ISP o/p commit failed\n");
		return -1;
	}

	isp_output = dev->isp->output[0];

	if (dev->render)
	{
		status = mmal_format_full_copy(dev->render->input[0]->format, isp_output->format);
		dev->render->input[0]->buffer_num = 3;
		if (status == MMAL_SUCCESS)
			status = mmal_port_format_commit(dev->render->input[0]);
	}

	//  Encoder setup
	if (dev->encoder)
	{
		encoder_input = dev->encoder->input[0];
		encoder_output = dev->encoder->output[0];

		status = mmal_format_full_copy(encoder_input->format, isp_output->format);
		encoder_input->buffer_num = 3;
		if (status == MMAL_SUCCESS)
			status = mmal_port_format_commit(encoder_input);

		// Only supporting H264 at the moment
		encoder_output->format->encoding = MMAL_ENCODING_H264;

		encoder_output->format->bitrate = 10000000;
		encoder_output->buffer_size = 256<<10;//encoder_output->buffer_size_recommended;

		if (encoder_output->buffer_size < encoder_output->buffer_size_min)
			encoder_output->buffer_size = encoder_output->buffer_size_min;

		encoder_output->buffer_num = 8; //encoder_output->buffer_num_recommended;

		if (encoder_output->buffer_num < encoder_output->buffer_num_min)
			encoder_output->buffer_num = encoder_output->buffer_num_min;

		// We need to set the frame rate on output to 0, to ensure it gets
		// updated correctly from the input framerate when port connected
		encoder_output->format->es->video.frame_rate.num = 0;
		encoder_output->format->es->video.frame_rate.den = 1;

		// Commit the port changes to the output port
		status = mmal_port_format_commit(encoder_output);

		if (status != MMAL_SUCCESS)
		{
			print("Unable to set format on encoder output port\n");
		}

		{
			MMAL_PARAMETER_VIDEO_PROFILE_T  param;
			param.hdr.id = MMAL_PARAMETER_PROFILE;
			param.hdr.size = sizeof(param);

			param.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;//state->profile;
			param.profile[0].level = MMAL_VIDEO_LEVEL_H264_4;

			status = mmal_port_parameter_set(encoder_output, &param.hdr);
			if (status != MMAL_SUCCESS)
			{
				print("Unable to set H264 profile\n");
			}
		}

		mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1);

		//set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
		mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 1);

		//set INLINE VECTORS flag to request motion vector estimates
		mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, 0);

		if (status != MMAL_SUCCESS)
		{
			print("Unable to set format on video encoder input port\n");
		}

		print("Enable encoder....\n");
		status = mmal_component_enable(dev->encoder);
		if(status != MMAL_SUCCESS)
		{
			print("Failed to enable\n");
			return -1;
		}
		status = mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
		status += mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
		if(status != MMAL_SUCCESS)
		{
			print("Failed to set zero copy\n");
			return -1;
		}
		encoder_input->userdata = (struct MMAL_PORT_USERDATA_T *)dev;
	}

	status = mmal_port_parameter_set_boolean(isp_output, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);

	isp_output->userdata = (struct MMAL_PORT_USERDATA_T *)dev;

	if (dev->render)
	{
		status += mmal_port_parameter_set_boolean(dev->render->input[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
		if (status != MMAL_SUCCESS)
		{
			return -1;
		}

		dev->render->input[0]->userdata = (struct MMAL_PORT_USERDATA_T *)dev;

		status = mmal_port_enable(dev->render->input[0], render_encoder_input_callback);
		if (status != MMAL_SUCCESS)
			return -1;

		print("Create pool of %d buffers of size %d for render\n", dev->render->input[0]->buffer_num, 0);
		dev->render_pool = mmal_port_pool_create(dev->render->input[0], dev->render->input[0]->buffer_num, dev->render->input[0]->buffer_size);
		if(!dev->render_pool)
		{
			print("Failed to create render pool\n");
			return -1;
		}
	}

	if (dev->encoder)
	{
		status = mmal_port_enable(encoder_input, render_encoder_input_callback);
		if (status != MMAL_SUCCESS)
			return -1;

		print("Create pool of %d buffers of size %d for encode ip\n", encoder_input->buffer_num, 0);
		dev->encode_pool = mmal_port_pool_create(encoder_input, isp_output->buffer_num, isp_output->buffer_size);
		if (!dev->encode_pool)
		{
			print("Failed to create encode ip pool\n");
			return -1;
		}
	}

	status = mmal_port_enable(isp_output, isp_output_callback);
	if (status != MMAL_SUCCESS)
		return -1;

	print("Create pool of %d buffers of size %d for encode/render\n", isp_output->buffer_num, isp_output->buffer_size);
	dev->isp_output_pool = mmal_port_pool_create(isp_output, isp_output->buffer_num, isp_output->buffer_size);
	if(!dev->isp_output_pool)
	{
		print("Failed to create pool\n");
		return -1;
	}

	buffers_to_isp(dev);

	// open h264 file and put the file handle in userdata for the encoder output port
	if (do_encode)
	{

		dev->h264_fd = stdout;
		debug = 0;

		encoder_output->userdata = (void*)dev;

		//Create encoder output buffers

		print("Create pool of %d buffers of size %d\n", encoder_output->buffer_num, encoder_output->buffer_size);
		dev->output_pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
		if(!dev->output_pool)
		{
			print("Failed to create pool\n");
			return -1;
		}

		dev->save_queue = mmal_queue_create();
		if(!dev->save_queue)
		{
			print("Failed to create queue\n");
			return -1;
		}

		vcos_status = vcos_thread_create(&dev->save_thread, "save-thread",
					NULL, save_thread, dev);
		if(vcos_status != VCOS_SUCCESS)
		{
			print("Failed to create save thread\n");
			return -1;
		}

		status = mmal_port_enable(encoder_output, encoder_buffer_callback);
		if(status != MMAL_SUCCESS)
		{
			print("Failed to enable port\n");
			return -1;
		}

		unsigned int i;
		for(i=0; i<encoder_output->buffer_num; i++)
		{
			MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(dev->output_pool->queue);

			if (!buffer)
			{
				print("Where'd my buffer go?!\n");
				return -1;
			}
			status = mmal_port_send_buffer(encoder_output, buffer);
			if(status != MMAL_SUCCESS)
			{
				print("mmal_port_send_buffer failed on buffer %p, status %d\n", buffer, status);
				return -1;
			}
			print("Sent buffer %p", buffer);
		}
	}

	return 0;
}

 void destroy_mmal(struct device *dev)
{
	//FIXME: Clean up everything properly
	dev->thread_quit = 1;
	vcos_thread_join(&dev->save_thread, NULL);

	mmal_component_destroy(dev->render);
	mmal_component_destroy(dev->encoder);
	mmal_component_destroy(dev->isp);
}

 int video_do_capture(struct device *dev, enum buffer_fill_mode fill)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct v4l2_buffer buf;
	struct timespec start;
	struct timeval last;
	struct timespec ts;
	unsigned int size;
	unsigned int i = 0;
	double bps;
	double fps;
	int ret;
	int dropped_frames = 0;

	/* Start streaming. */
	ret = video_enable(dev, 1);
	if (ret < 0)
		goto done;

	size = 0;
	clock_gettime(CLOCK_MONOTONIC, &start);
	last.tv_sec = start.tv_sec;
	last.tv_usec = start.tv_nsec / 1000;

	for (;;)
	{
		fd_set fds[3];
		fd_set *rd_fds = &fds[0]; /* for capture */
		fd_set *ex_fds = &fds[1]; /* for capture */
		fd_set *wr_fds = &fds[2]; /* for output */
		struct timeval tv;
		int r;

		if (rd_fds)
		{
			FD_ZERO(rd_fds);
			FD_SET(dev->fd, rd_fds);
		}

		if (ex_fds)
		{
			FD_ZERO(ex_fds);
			FD_SET(dev->fd, ex_fds);
		}

		/* Timeout. */
		tv.tv_sec = 10;
		tv.tv_usec = 0;

		r = select(dev->fd + 1, rd_fds, wr_fds, ex_fds, &tv);

		if (-1 == r)
		{
			if (EINTR == errno)
				continue;
			errno_exit("select");
		}

		if (0 == r)
		{
			fprintf(stderr, "select timeout\n");
			exit(EXIT_FAILURE);
		}

		if (rd_fds && FD_ISSET(dev->fd, rd_fds))
		{
			const char *ts_type, *ts_source;
			int queue_buffer = 1;
			/* Dequeue a buffer. */
			memset(&buf, 0, sizeof buf);
			memset(planes, 0, sizeof planes);

			buf.type = dev->type;
			buf.memory = dev->memtype;
			buf.length = VIDEO_MAX_PLANES;
			buf.m.planes = planes;

			ret = ioctl(dev->fd, VIDIOC_DQBUF, &buf);
			if (ret < 0)
			{
				if (errno != EIO)
				{
					print("Unable to dequeue buffer: %s (%d).\n",
						  strerror(errno), errno);
					goto done;
				}
				buf.type = dev->type;
				buf.memory = dev->memtype;
				if (dev->memtype == V4L2_MEMORY_USERPTR)
					video_buffer_fill_userptr(dev, &dev->buffers[i], &buf);
			}

			if (video_is_capture(dev))
				video_verify_buffer(dev, &buf);
			//print("bytesused in buffer is %d\n", buf.bytesused);
			size += buf.bytesused;

			fps = (buf.timestamp.tv_sec - last.tv_sec) * 1000000 + buf.timestamp.tv_usec - last.tv_usec;
			fps = fps ? 1000000.0 / fps : 0.0;

			clock_gettime(CLOCK_MONOTONIC, &ts);
			get_ts_flags(buf.flags, &ts_type, &ts_source);

			last = buf.timestamp;

			if (dev->mmal_pool)
			{
				MMAL_BUFFER_HEADER_T *mmal = mmal_queue_get(dev->mmal_pool->queue);
				MMAL_STATUS_T status;
				if (!mmal)
				{
					print("Failed to get MMAL buffer\n");
				}
				else
				{
					/* Need to wait for MMAL to be finished with the buffer before returning to V4L2 */
					queue_buffer = 0;
					if (((struct buffer *)mmal->user_data)->idx != buf.index)
					{
						print("Mismatch in expected buffers. V4L2 gave idx %d, MMAL expecting %d\n",
							  buf.index, ((struct buffer *)mmal->user_data)->idx);
					}

					mmal->length = buf.length; //Deliberately use length as MMAL wants the padding

					if (!dev->starttime.tv_sec)
						dev->starttime = buf.timestamp;

					struct timeval pts;
					timersub(&buf.timestamp, &dev->starttime, &pts);
					//MMAL PTS is in usecs, so convert from struct timeval
					mmal->pts = (pts.tv_sec * 1000000) + pts.tv_usec;
					if (mmal->pts > (dev->lastpts + dev->frame_time_usec + 1000))
					{
						print("DROPPED FRAME - %lld and %lld, delta %lld\n", dev->lastpts, mmal->pts, mmal->pts - dev->lastpts);
						dropped_frames++;
					}
					dev->lastpts = mmal->pts;

					mmal->flags = MMAL_BUFFER_HEADER_FLAG_FRAME_END;
					//mmal->pts = buf.timestamp;
					status = mmal_port_send_buffer(dev->isp->input[0], mmal);
					if (status != MMAL_SUCCESS)
						print("mmal_port_send_buffer failed %d\n", status);
				}
			}

			/* Requeue the buffer. */
			// if (delay > 0)
			// 	usleep(delay * 1000);

			fflush(stdout);

			if (!queue_buffer)
				continue;

			ret = video_queue_buffer(dev, buf.index, fill);
			if (ret < 0)
			{
				print("Unable to requeue buffer: %s (%d).\n",
					  strerror(errno), errno);
				goto done;
			}
		}
		if (wr_fds && FD_ISSET(dev->fd, wr_fds))
		{
			fprintf(stderr, "Writing?!?!?\n");
		}
		if (ex_fds && FD_ISSET(dev->fd, ex_fds))
		{
			if (handle_event(dev) == 50) {
				return 50;
			}
		}
		/* EAGAIN - continue select loop. */
	}

	/* Stop streaming. */
	ret = video_enable(dev, 0);
	if (ret < 0)
		return ret;

	// if (nframes == 0) {
	// 	print("No frames captured.\n");
	// 	goto done;
	// }

	if (ts.tv_sec == start.tv_sec && ts.tv_nsec == start.tv_nsec) {
		print("Captured %u frames (%u bytes) 0 seconds\n", i, size);
		goto done;
	}

	ts.tv_sec -= start.tv_sec;
	ts.tv_nsec -= start.tv_nsec;
	if (ts.tv_nsec < 0) {
		ts.tv_sec--;
		ts.tv_nsec += 1000000000;
	}

	bps = size/(ts.tv_nsec/1000.0+1000000.0*ts.tv_sec)*1000000.0;
	fps = i/(ts.tv_nsec/1000.0+1000000.0*ts.tv_sec)*1000000.0;

	print("Captured %u frames in %lu.%06lu seconds (%f fps, %f B/s).\n",
		i, ts.tv_sec, ts.tv_nsec/1000, fps, bps);
	print("Total number of frames dropped %d\n", dropped_frames);
done:
	return video_free_buffers(dev);
}

int video_set_dv_timings(struct device *dev)
{
	struct v4l2_dv_timings timings;
	v4l2_std_id std;
	int ret;

	memset(&timings, 0, sizeof timings);
	ret = ioctl(dev->fd, VIDIOC_QUERY_DV_TIMINGS, &timings);
	if (ret >= 0) {
		print("QUERY_DV_TIMINGS returned %ux%u pixclk %llu\n", timings.bt.width, timings.bt.height, timings.bt.pixelclock);
		//Can read DV timings, so set them.
		ret = ioctl(dev->fd, VIDIOC_S_DV_TIMINGS, &timings);
		if (ret < 0) {
			print("Failed to set DV timings\n");
			return -1;
		} else {
			double tot_height, tot_width;
			const struct v4l2_bt_timings *bt = &timings.bt;

			tot_height = bt->height +
				bt->vfrontporch + bt->vsync + bt->vbackporch +
				bt->il_vfrontporch + bt->il_vsync + bt->il_vbackporch;
			tot_width = bt->width +
				bt->hfrontporch + bt->hsync + bt->hbackporch;
			dev->fps = (unsigned int)((double)bt->pixelclock /
				(tot_width * tot_height));
			print("Framerate is %u\n", dev->fps);
		}
	} else {
		memset(&std, 0, sizeof std);
		ret = ioctl(dev->fd, VIDIOC_QUERYSTD, &std);
		if (ret >= 0) {
			//Can read standard, so set it.
			ret = ioctl(dev->fd, VIDIOC_S_STD, &std);
			if (ret < 0) {
				print("Failed to set standard\n");
				return -1;
			} else {
				// SD video - assume 50Hz / 25fps
				dev->fps = 25;
			}
		}
	}
	return 0;
}

int video_get_fps(struct device *dev)
{
	struct v4l2_streamparm parm;
	int ret;

	memset(&parm, 0, sizeof parm);
	parm.type = dev->type;

	ret = ioctl(dev->fd, VIDIOC_G_PARM, &parm);
	if (ret < 0) {
		print("Unable to get frame rate: %s (%d).\n",
			strerror(errno), errno);
		/* Make a wild guess at the frame rate */
		dev->fps = 15;
		return ret;
	}

	print("Current frame rate: %u/%u\n",
		parm.parm.capture.timeperframe.numerator,
		parm.parm.capture.timeperframe.denominator);

	dev->fps = parm.parm.capture.timeperframe.numerator/
			parm.parm.capture.timeperframe.denominator;

	return 0;
}

#define V4L_BUFFERS_DEFAULT	8
#define V4L_BUFFERS_MAX		32

int start_video( char* devname)
{
	struct device dev;

	int ret;

	/* Use video capture by default if query isn't done. */
	unsigned int capabilities = V4L2_CAP_VIDEO_CAPTURE;

	int do_encode = 0;

	/* Video buffers */
	enum v4l2_memory memtype = V4L2_MEMORY_MMAP;
	unsigned int pixelformat = V4L2_PIX_FMT_YUYV;
	unsigned int fmt_flags = 0;
	unsigned int width = 640;
	unsigned int height = 480;
	unsigned int stride = 0;
	unsigned int buffer_size = 0;
	unsigned int nbufs = V4L_BUFFERS_DEFAULT;

	unsigned int userptr_offset = 0;
	enum v4l2_field field = V4L2_FIELD_ANY;

	/* Capture loop */
	enum buffer_fill_mode fill_mode = BUFFER_FILL_NONE;

	video_init(&dev);

	if (!video_has_fd(&dev))
	{

		ret = video_open(&dev, devname);
		if (ret < 0)
			return 1;
	}

	{
		struct v4l2_event_subscription sub;

		memset(&sub, 0, sizeof(sub));

		sub.type = V4L2_EVENT_SOURCE_CHANGE;
		ioctl(dev.fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
	}


	ret = video_querycap(&dev, &capabilities);
	if (ret < 0)
		return 1;

	ret = cap_get_buf_type(capabilities);
	if (ret < 0)
		return 1;

	if (!video_has_valid_buf_type(&dev))
		video_set_buf_type(&dev, ret);

	dev.memtype = memtype;

	video_set_format(&dev, width, height, pixelformat, stride, buffer_size, field, fmt_flags);
	video_set_dv_timings(&dev);
	video_get_format(&dev);


	if (!dev.fps)
	{
		video_get_fps(&dev);
	}

	setup_mmal(&dev, nbufs, do_encode);

	video_prepare_capture(&dev, nbufs, userptr_offset, fill_mode);
	video_queue_all_buffers(&dev, fill_mode);

	ret  = video_do_capture(&dev, fill_mode);

	if (ret < 0)
	{
		video_close(&dev);
		return 1;
	}

	destroy_mmal(&dev);
	video_close(&dev);

	if (ret == 50) {
		return 50;
	}

	return 0;
}

int main()
{
	struct sched_param sched;
	unsigned int rt_priority = 1;

	memset(&sched, 0, sizeof sched);
	sched.sched_priority = rt_priority;

	if (sched_setscheduler(0, SCHED_RR, &sched) < 0)
		print("Failed to select RR scheduler: %s (%d)\n", strerror(errno), errno);

	int ret = start_video("/dev/video0");

	while(ret == 50) {
			ret = start_video("/dev/video0");
	}

	return 0;
}
