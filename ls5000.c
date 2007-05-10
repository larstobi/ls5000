/*
 *   ls5000.c - Coolscan 5000 ED SANE backend
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License as
 *   published by the Free Software Foundation; either version 2 of the
 *   License, or (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston,
 *   MA 02111-1307, USA.
 *
 *   As a special exception, the authors of SANE give permission for
 *   additional uses of the libraries contained in this release of SANE.
 *
 *   The exception is that, if you link a SANE library with other files
 *   to produce an executable, this does not by itself cause the
 *   resulting executable to be covered by the GNU General Public
 *   License.  Your use of that executable is in no way restricted on
 *   account of linking the SANE library code into it.
 *
 *   This exception does not, however, invalidate any other reasons why
 *   the executable file might be covered by the GNU General Public
 *   License.
 *
 *   If you submit changes to SANE to the maintainers to be included in
 *   a subsequent release, you agree by submitting the changes that
 *   those changes may be distributed with this exception intact.
 *   
 *   If you write modifications of your own for SANE, it is your choice
 *   whether to permit this exception to apply to your modifications.
 *   If you do not wish that, delete this exception notice.
 *
 *   This file implements a SANE backend for Nikon Coolscan LS5000ED film
 *   scanner.
 *
 *
 *   Copyright 2007	Johannes Berg <johannes@sipsolutions.net>
 *
 *   This code is based on coolscan2.c,
 *	Copyright 2001-2002, Andras Major <andras@users.sourceforge.net>
 *
 *   We would like to thank Nikon Corporation for providing technical
 *   information.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>
#include <arpa/inet.h>
#include <sane/sane.h>
#include <sane/saneopts.h>

#define BACKEND_NAME ls5000
#include "config.h"
#include "sanei_usb.h"

/* typedefs */

#define free(p) free((void*)p)

typedef enum {
	LS5000_PHASE_NONE = 0x00,
	LS5000_PHASE_STATUS = 0x01,
	LS5000_PHASE_OUT = 0x02,
	LS5000_PHASE_IN = 0x03,
	LS5000_PHASE_BUSY = 0x04
} ls5000_phase_t;

typedef enum {
	LS5000_SCAN_NORMAL,
	LS5000_SCAN_AE,
	LS5000_SCAN_AE_WB
} ls5000_scan_t;

typedef enum {
	LS5000_SCAN_STAGE_IDLE,
	LS5000_SCAN_STAGE_ACQUIRE,
} ls5000_scan_stage_t;

typedef enum {
	LS5000_ERROR,
	LS5000_GOOD,
	LS5000_BECOMING_READY,
	LS5000_MECHANICAL_ERROR,
	LS5000_INITIALISATION_REQUIRED,
	LS5000_ADAPTER_ERROR,
	LS5000_NOT_RESPONDING,
	LS5000_NO_MEDIUM,
	LS5000_CMD_SEQ_ERR,
	LS5000_UNIT_ATTENTION,
	LS5000_DATA_PHASE_ERROR,
	LS5000_OVERLAPPED_CMDS,
} ls500O_unit_ready_t;

typedef enum {
	LS5000_STATUS_READY,
	LS5000_STATUS_BUSY,
	LS5000_STATUS_PROCESSING,
	LS5000_STATUS_ERROR,
	LS5000_STATUS_REISSUE,
} ls5000_status;

typedef enum {
	LS5000_OPTION_NUM = 0,

	/* info options */
	LS5000_OPTION_ADAPTER,
	
	/* actual settings */
	LS5000_OPTION_NEGATIVE,
	LS5000_OPTION_PREVIEW,
	LS5000_OPTION_INFRARED,

	LS5000_OPTION_SCAN_MODE,

	LS5000_OPTION_EXPOSURE,
	LS5000_OPTION_EXPOSURE_R,
	LS5000_OPTION_EXPOSURE_G,
	LS5000_OPTION_EXPOSURE_B,
	LS5000_OPTION_SCAN_AE,
	LS5000_OPTION_SCAN_AE_WB,

	LS5000_OPTION_LUT_R,
	LS5000_OPTION_LUT_G,
	LS5000_OPTION_LUT_B,

	LS5000_OPTION_RES,

	LS5000_OPTION_FRAME,
	LS5000_OPTION_SUBFRAME,
	LS5000_OPTION_XMIN,
	LS5000_OPTION_XMAX,
	LS5000_OPTION_YMIN,
	LS5000_OPTION_YMAX,

	LS5000_OPTION_LOAD,
	LS5000_OPTION_EJECT,
	LS5000_OPTION_RESET,
	LS5000_OPTION_INQUIRE,

	LS5000_OPTION_FOCUS,
	LS5000_OPTION_AUTOFOCUS,
	LS5000_OPTION_FOCUS_ON_CENTRE,
	LS5000_OPTION_FOCUSX,
	LS5000_OPTION_FOCUSY,

	/* keep last */
	LS5000_N_OPTIONS
} ls5000_option_t;

#define LS5000_CMD_INQUIRY 0x12
#define LS5000_CMD_TEST_UNIT_READY 0x00
#define LS5000_CMD_PHASE_CHECK 0xd0
#define LS5000_CMD_STATUS 0x06
#define LS5000_CMD_GET_WINDOW 0x25
#define LS5000_CMD_SET_WINDOW 0x24
#define LS5000_CMD_RESERVE_UNIT 0x16
#define LS5000_CMD_RELEASE_UNIT 0x17
#define LS5000_CMD_READ 0x28
#define LS5000_CMD_MODE_SELECT 0x15
#define LS5000_CMD_GET_PARAM 0xe1
#define LS5000_CMD_SET_PARAM 0xe0
#define LS5000_CMD_EXECUTE 0xc1
#define LS5000_CMD_SCAN 0x1b
#define LS5000_CMD_SEND 0x2a
#define LS5000_CMD_ABORT 0xc0

typedef unsigned int ls5000_pixel_t;

typedef struct {
	int fd;
	SANE_Byte *recv_buf;
	size_t recv_buf_size;

	/* device characteristics */
	char vendor_string[9], product_string[17], revision_string[5];
	int maxbits;
	unsigned int res_optical, res_min, res_max, *res_list, res_n_list;
	unsigned long boundaryx, boundaryy;
	unsigned long frame_offset;
	unsigned int unit_dpi;
	double unit_mm;
	int n_frames;
	char adapter[50];

	int focus_min, focus_max;

	/* settings */
	SANE_Bool negative, infrared, preview, pinfrared, gray_scan;
	ls5000_pixel_t n_lut;
	ls5000_pixel_t *lut_r, *lut_g, *lut_b, *lut_neutral;
	unsigned long res;
	unsigned long xmin, xmax, ymin, ymax;
	int i_frame;
	double subframe;

	unsigned long real_xoffset, real_yoffset, real_width, real_height,
	    logical_width, logical_height;

	double exposure, exposure_r, exposure_g, exposure_b;
	unsigned long real_exposure[4];

	SANE_Bool focus_on_centre;
	unsigned long focusx, focusy;
	int focus;

	/* read block information */
	int line_padding, block_lines;
	size_t line_bytes, line;
	SANE_Byte *block, *ordered_block;
	size_t block_read_pos;
	SANE_Byte *ir_data;
	size_t ir_data_len;

	/* status */
	ls5000_scan_stage_t scan_stage;
	unsigned long sense_key, sense_asc, sense_ascq, sense_info;
	int status;
	SANE_Bool must_read_now;

	/* SANE stuff */
	SANE_Option_Descriptor option_list[LS5000_N_OPTIONS];
} ls5000_t;

static SANE_Device **device_list = NULL;
static int n_device_list = 0;
static int open_devices = 0;

/* command handling */
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

static ls5000_phase_t ls5000_phase_check(ls5000_t *s)
{
	static SANE_Byte phase_send_buf[1] = { LS5000_CMD_PHASE_CHECK };
	SANE_Byte phase_recv_buf[1];
	SANE_Status status = 0;
	size_t n = 1;
	static char *phases[] = { "NO", "STATUS IN", "DATA OUT", "DATA IN", "BUSY", };
	char *phase = "UNKNOWN";

	DBG(20, "PHASE_CHECK\n");
	status = sanei_usb_write_bulk(s->fd, phase_send_buf, &n);
	status |= sanei_usb_read_bulk(s->fd, phase_recv_buf, &n);

	if (phase_recv_buf[0] < ARRAY_SIZE(phases))
		phase = phases[phase_recv_buf[0]];

	DBG(20, "PHASE_CHECK: %s (0x%02x)\n", phase, phase_recv_buf[0]);

	if (status)
		return -1;
	else
		return phase_recv_buf[0];
}

static SANE_Status ls5000_parse_status(ls5000_t *s)
{
	if (s->sense_key == 0)
		DBG(20, "STATUS: GOOD\n\n");
	else
		DBG(20, "STATUS: CHECK CONDITION, %02lx-%02lx-%02lx-%02lx\n\n",
		    s->sense_key, s->sense_asc, s->sense_ascq, s->sense_info);

	if (s->sense_key == 0x09 && s->sense_asc == 0x80 && s->sense_ascq == 0x06 &&
	    (s->sense_info == 0 || s->sense_info == 1)) {
		s->status = LS5000_STATUS_REISSUE;
		return SANE_STATUS_GOOD;
	}

	if (s->sense_key == 0) {
		s->status = LS5000_STATUS_READY;
		return SANE_STATUS_GOOD;
	}

	s->status = LS5000_STATUS_ERROR;
	return SANE_STATUS_IO_ERROR;
}

static SANE_Status ls5000_check_status(ls5000_t *s)
{
	size_t n_status;
	static SANE_Byte status_buf[8];

	DBG(20, "STATUS\n");
	status_buf[0] = LS5000_CMD_STATUS;
	n_status = 1;
	sanei_usb_write_bulk(s->fd, status_buf, &n_status);
	n_status = 8;
	sanei_usb_read_bulk(s->fd, status_buf, &n_status);
	if (n_status != 8) {
		DBG(4, "ls5000_check_status: Failed to read 8 status bytes from USB.\n");
		return SANE_STATUS_IO_ERROR;
	}
	s->sense_key = status_buf[1] & 0x0f;
	s->sense_asc = status_buf[2] & 0xff;
	s->sense_ascq = status_buf[3] & 0xff;
	s->sense_info = status_buf[4] & 0xff;
	return ls5000_parse_status(s);
}

#define ls5000_issue_cmd(s, n_recv, bytes...)				\
	({ SANE_Byte data[] = {bytes};					\
	   _ls5000_issue_cmd(s, n_recv, ARRAY_SIZE(data), data);	\
	})
static SANE_Status _ls5000_issue_cmd(ls5000_t *s, int n_recv,
				     int n_send, SANE_Byte *send_buf)
{
	SANE_Status status = SANE_STATUS_INVAL;
	size_t n_cmd, n_written;
	int status_only = 0;
	char cmdbuf[10000];
	char *cmd;

	switch (send_buf[0]) {
	case LS5000_CMD_TEST_UNIT_READY: cmd = "TEST_UNIT_READY"; break;
	case LS5000_CMD_PHASE_CHECK: cmd = "PHASE_CHECK"; break;
	case LS5000_CMD_INQUIRY: cmd = "INQUIRY"; break;
	case LS5000_CMD_STATUS: cmd = "STATUS"; break;
	case LS5000_CMD_GET_WINDOW: cmd = "GET_WINDOW"; break;
	case LS5000_CMD_SET_WINDOW: cmd = "SET_WINDOW"; break;
	case LS5000_CMD_RESERVE_UNIT: cmd = "RESERVE_UNIT"; break;
	case LS5000_CMD_RELEASE_UNIT: cmd = "RELEASE_UNIT"; break;
	case LS5000_CMD_READ: cmd = "READ"; break;
	case LS5000_CMD_MODE_SELECT: cmd = "MODE_SELECT"; break;
	case LS5000_CMD_GET_PARAM: cmd = "GET_PARAM"; break;
	case LS5000_CMD_SET_PARAM: cmd = "SET_PARAM"; break;
	case LS5000_CMD_EXECUTE: cmd = "EXECUTE"; break;
	case LS5000_CMD_SCAN: cmd = "SCAN"; break;
	case LS5000_CMD_ABORT: cmd = "ABORT"; break;
	case LS5000_CMD_SEND: cmd = "SEND"; break;
	default:
		snprintf(cmdbuf, sizeof(cmdbuf), "UNKNOWN_%.2x", send_buf[0]);
		cmd = cmdbuf;
	}

	DBG(20, "%s\n", cmd);

	s->status = LS5000_STATUS_READY;

	switch (send_buf[0]) {
	case LS5000_CMD_TEST_UNIT_READY:
	case LS5000_CMD_INQUIRY:
	case LS5000_CMD_MODE_SELECT:
	case LS5000_CMD_RESERVE_UNIT:
	case LS5000_CMD_RELEASE_UNIT:
	case 0x1a /*MODE SENSE*/:
	case LS5000_CMD_SCAN:
	case 0x1c /*RCVR DIAG RESULTS */:
	case 0x1d /*SEND DIAG */:
	case LS5000_CMD_ABORT:
	case LS5000_CMD_EXECUTE:
		n_cmd = 6;
		break;
	case LS5000_CMD_SET_WINDOW:
	case LS5000_CMD_GET_WINDOW:
	case LS5000_CMD_READ:
	case LS5000_CMD_SEND:
	case LS5000_CMD_SET_PARAM:
	case LS5000_CMD_GET_PARAM:
		n_cmd = 10;
		break;
	default:
		DBG(1, "BUG: ls5000_issue_cmd: Unknown command opcode 0x%02x.\n",
		    send_buf[0]);
		break;
	}

	if (n_send < n_cmd) {
		DBG(1, "BUG: ls5000_issue_cmd: Not enough command bytes!\n");
		return SANE_STATUS_INVAL;
	}

	if (n_recv != 0 && (n_send - n_cmd > 0)) {
		DBG(1, "BUG: ls5000_issue_cmd: Both data in and data out requested.\n");
		return SANE_STATUS_INVAL;
	}

	if (n_recv > 0) {
		s->recv_buf = realloc(s->recv_buf, n_recv);
		if (!s->recv_buf) {
			DBG(1, "BUG: failed to alloc recv_buf\n");
			return SANE_STATUS_NO_MEM;
		}
	}

	status = sanei_usb_write_bulk(s->fd, send_buf, &n_cmd);
	if (status != SANE_STATUS_GOOD) {
		DBG(1, "Error: ls5000_issue_cmd: Could not write command.\n");
		return SANE_STATUS_IO_ERROR;
	}
	switch (ls5000_phase_check(s)) {
	case LS5000_PHASE_OUT:
		if (n_send - n_cmd == 0) {
			DBG(4, "Error: ls5000_issue_cmd: Unexpected data out phase.\n");
			return SANE_STATUS_IO_ERROR;
		}
		n_written = n_send - n_cmd;
		status = sanei_usb_write_bulk(s->fd, send_buf + n_cmd, &n_written);
		if (n_written != n_send - n_cmd) {
			DBG(4, "Error: ls5000_issue_cmd: scanner only took %d bytes\n", n_written);
			return SANE_STATUS_IO_ERROR;
		}
		{
		int i, pos=0,l;
		snprintf(cmdbuf, sizeof(cmdbuf), "%s_DATA:", cmd);
		l=strlen(cmdbuf);
		for (i=0;(i<n_written) && (sizeof(cmdbuf)-pos>3);i++,pos+=3)
			snprintf(cmdbuf+l+pos, sizeof(cmdbuf)-pos, " %.2x", (send_buf + n_cmd)[i]);
		DBG(20, "%s\n", cmdbuf);
		}
		break;
	case LS5000_PHASE_IN:
		if (n_recv == 0) {
			DBG(4, "Error: ls5000_issue_cmd: Unexpected data in phase.\n");
			return SANE_STATUS_IO_ERROR;
		}
		if (n_recv < 0)
			return SANE_STATUS_DEVICE_BUSY;
		status = sanei_usb_read_bulk(s->fd, s->recv_buf, (size_t*)&n_recv);
		break;
	case LS5000_PHASE_NONE:
		DBG(4, "Error: ls5000_issue_cmd: No command received!\n");
		return SANE_STATUS_IO_ERROR;
	default:
		if (n_recv) {
			DBG(4, "Error: ls5000_issue_cmd: Unexpected non-data phase, but n_recv != 0.\n");
			status_only = 1;
		}
		break;
	}
	status = ls5000_check_status(s);

	if (status_only)
		return SANE_STATUS_IO_ERROR;
	else
		return status;
}

static ls500O_unit_ready_t ls5000_get_status(ls5000_t *s)
{
	int status;

	/* issue_cmd fills the sense_key stuff from the STATUS reply */
	ls5000_issue_cmd(s, 0,
			 LS5000_CMD_TEST_UNIT_READY, 0, 0, 0, 0, 0);
	if (s->sense_key == 0)
		return LS5000_GOOD;

	if (s->sense_key == 6)
		return LS5000_UNIT_ATTENTION;

	status = (s->sense_key << 24) |
		 (s->sense_asc << 16) |
		 (s->sense_ascq << 8); /* not sense_info */

	switch (status) {
	case 0x02040100:
		return LS5000_BECOMING_READY;
	case 0x02040200:
		return LS5000_MECHANICAL_ERROR;
	case 0x02040000:
		return LS5000_INITIALISATION_REQUIRED;
	case 0x02040300:
		return LS5000_ADAPTER_ERROR;
	case 0x02050000:
		return LS5000_NOT_RESPONDING;
	case 0x023a0000:
		return LS5000_NO_MEDIUM;
	case 0x052c0000:
		return LS5000_CMD_SEQ_ERR;
	case 0x0b4b0000:
		return LS5000_DATA_PHASE_ERROR;
	case 0x0b4e0000:
		return LS5000_OVERLAPPED_CMDS;
	default:
		return LS5000_ERROR;
	}
}

static SANE_Status ls5000_get_error(ls5000_t *s)
{
	int stat = ls5000_get_status(s);
	switch (stat) {
	case LS5000_GOOD:
		return SANE_STATUS_GOOD;
	case LS5000_NO_MEDIUM:
		return SANE_STATUS_NO_DOCS;
	case LS5000_MECHANICAL_ERROR:
		return SANE_STATUS_JAMMED;
	case LS5000_ADAPTER_ERROR:
		return SANE_STATUS_COVER_OPEN;
	case LS5000_BECOMING_READY:
		return SANE_STATUS_DEVICE_BUSY;
	default:
		return SANE_STATUS_IO_ERROR;
	}
}

static inline void order_values(
	unsigned long imin, unsigned long imax,
	unsigned long *min, unsigned long *max)
{
	if (imin > imax) {
		*min = imax;
		*max = imin;
	} else {
		*min = imin;
		*max = imax;
	}
}

static void ls5000_convert_options(ls5000_t *s)
{
	int i_colour, pitch;
	unsigned long xmin, xmax, ymin, ymax;

	/*
	 * The prefix "real_" refers to data in device units (1/maxdpi),
	 * "logical_" refers to resolution-dependent data.
	 */

	pitch = s->res_max / s->res;

	order_values(s->xmin, s->xmax, &xmin, &xmax);
	order_values(s->ymin, s->ymax, &ymin, &ymax);

	s->real_xoffset = xmin;
	s->real_yoffset =
	    ymin + (s->i_frame - 1) * s->frame_offset +
	    s->subframe / s->unit_mm;
	s->logical_width = (xmax - xmin + 1) / pitch;
	s->logical_height = (ymax - ymin + 1) / pitch;
	s->real_width = s->logical_width * pitch;
	s->real_height = s->logical_height * pitch;

	s->real_exposure[0] = s->exposure * s->exposure_r * 100.;
	s->real_exposure[1] = s->exposure * s->exposure_g * 100.;
	s->real_exposure[2] = s->exposure * s->exposure_b * 100.;

	for (i_colour = 0; i_colour < 3; i_colour++)
		if (s->real_exposure[i_colour] < 1)
			s->real_exposure[i_colour] = 1;
}

static SANE_Status ls5000_execute(ls5000_t *s)
{
	return ls5000_issue_cmd(s, 0, LS5000_CMD_EXECUTE, 0, 0, 0, 0, 0);
}

static SANE_Status ls5000_eject(ls5000_t *s)
{
	SANE_Status status;

	DBG(20, "EJECT\n");
	status = ls5000_issue_cmd(s, 0,
			LS5000_CMD_SET_PARAM,
			0,		/* LUN */
			0xd0,		/* Operation "Unload object" */
			0, 0, 0,	/* reserved */
			0, 0, 13,	/* parameter length, must be 13 */
			0,		/* ctrl */
			0,0,0,0,0,0,0,0,0,0,0,0,0 /* 13 dummy parameters */
			);
	if (status)
		return status;

	return ls5000_execute(s);
}

static SANE_Status ls5000_focus(ls5000_t *s)
{
	SANE_Status status;

	DBG(20, "FOCUS\n");
	status = ls5000_issue_cmd(s, 0,
			LS5000_CMD_SET_PARAM,
			0,		/* LUN */
			0xc1,		/* Operation "Focus move" */
			0, 0, 0,	/* reserved */
			0, 0, 13,	/* parameter length */
			0,		/* ctrl */
			0,		/* color (dummy) */
			(s->focus >> 24) & 0xff,
			(s->focus >> 16) & 0xff,
			(s->focus >> 8) & 0xff,
			s->focus & 0xff,
			0,0,0,0,0,0,0,0	/* rest dummy */);
	if (status)
		return status;

	return ls5000_execute(s);
}

static SANE_Status ls5000_autofocus(ls5000_t *s)
{
	SANE_Status status;
	int real_focusx, real_focusy, tmo = 15, stat;

	ls5000_convert_options(s);

	if (s->focus_on_centre) {
		real_focusx = s->real_xoffset + s->real_width / 2;
		real_focusy = s->real_yoffset + s->real_height / 2;
	} else {
		real_focusx = s->focusx;
		real_focusy = s->focusy + (s->i_frame - 1) * s->frame_offset + s->subframe / s->unit_mm;
	}

	DBG(20, "AUTOFOCUS\n");
	status = ls5000_issue_cmd(s, 0,
			LS5000_CMD_SET_PARAM,
			0,		/* LUN */
			0xa0,		/* Operation "autofocus" */
			0, 0, 0,	/* reserved */
			0, 0, 13,	/* parameter length */
			0,		/* ctrl */
			0,		/* color specification, ignored */
			(real_focusx >> 24) & 0xff,
			(real_focusx >> 16) & 0xff,
			(real_focusx >> 8) & 0xff,
			real_focusx & 0xff,
			(real_focusy >> 24) & 0xff,
			(real_focusy >> 16) & 0xff,
			(real_focusy >> 8) & 0xff,
			real_focusy & 0xff,
			0, 0, 0, 0);	/* dummy */
	if (status)
		return status;

	stat = ls5000_get_status(s);
	while (stat == LS5000_BECOMING_READY) {
		usleep(500000);
		stat = ls5000_get_status(s);
		if (!tmo--)
			return SANE_STATUS_IO_ERROR;
	}


	status = ls5000_execute(s);
	if (status)
		return status;

	status = ls5000_issue_cmd(s, 13,
			LS5000_CMD_GET_PARAM,
			0,		/* LUN */
			0xc1,		/* Operation "Focus move" */
			0, 0, 0,	/* reserved */
			0, 0, 13,	/* parameter length */
			0		/* ctrl */);
	if (status)
		return status;

	s->focus = s->recv_buf[4] +
		   256 * (s->recv_buf[3] +
			  256 * (s->recv_buf[2] +
				 256 * s->recv_buf[1]));
	return SANE_STATUS_INVAL;
}

static SANE_Status ls5000_get_exposure(ls5000_t *s)
{
	SANE_Status status;
	int i_colour;

	for (i_colour = 0; i_colour < 3; i_colour++) {
		status = ls5000_issue_cmd(s, 58,
				LS5000_CMD_GET_WINDOW,
				1,		/* LUN, single = 1 */
				0, 0, 0,	/* reserved */
				i_colour + 1,	/* window ID */
				0, 0, 58,	/* transfer length */
				0);		/* control */
		if (status)
			return status;

		s->real_exposure[i_colour] =
		    65536 * (256 * s->recv_buf[54] + s->recv_buf[55]) +
		    256 * s->recv_buf[56] + s->recv_buf[57];
	}

	return SANE_STATUS_GOOD;
}


static SANE_Status ls5000_scan(ls5000_t *s, ls5000_scan_t type)
{
	SANE_Status status;
	int i_colour, n_colour, window, stat, tmo = 15;
	unsigned char scan_kind, exp_b0, exp_b1, exp_b2, exp_b3;

	stat = ls5000_get_status(s);
	while (stat == LS5000_BECOMING_READY ||
	       stat == LS5000_UNIT_ATTENTION ||
	       stat == LS5000_CMD_SEQ_ERR) {
		usleep(500000);
		stat = ls5000_get_status(s);
		if (!tmo--)
			return SANE_STATUS_IO_ERROR;
	}

	status = ls5000_issue_cmd(s, 0,
			LS5000_CMD_MODE_SELECT,
			0x10 /* PF=1 */,
			0x00, 0x00, /* reserved */
			20, /* len */
			0, /* ctrl */
			0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 1, 3, 6, 0, 0,
			(s->unit_dpi >> 8) & 0xff,
			(s->unit_dpi >> 0) & 0xff,
			0, 0);
	if (status)
		return status;

	ls5000_convert_options(s);

	s->line_padding = 0;

	n_colour = 3 + !!s->infrared;
	if (s->gray_scan)
		n_colour = 1 + !!s->infrared;

	DBG(1, "n_colour = %d\n", n_colour);

	switch (type) {
	case LS5000_SCAN_NORMAL:
		scan_kind = 0x01;
		break;
	case LS5000_SCAN_AE:
		scan_kind = 0x20;
		break;
	case LS5000_SCAN_AE_WB:
		scan_kind = 0x40;
		break;
	/* thumbnail: 2, scanning setup: 4, scanning2 setup: 8 */
	default:
		DBG(1, "BUG: ls5000_scan: Unknown scanning type.\n");
		return SANE_STATUS_INVAL;
	}

	for (i_colour = 0; i_colour < n_colour; i_colour++) {
		/* if infrared requested set parameters for it first */
		if (s->infrared && i_colour == 0) {
			/* automatic exposure for infrared */
			exp_b0 = 0;
			exp_b1 = 0;
			exp_b2 = 0;
			exp_b3 = 0;
			window = 9; /* IR window */
		} else {
			exp_b0 = (s->real_exposure[i_colour] >> 24) & 0xff;
			exp_b1 = (s->real_exposure[i_colour] >> 16) & 0xff;
			exp_b2 = (s->real_exposure[i_colour] >> 8) & 0xff;
			exp_b3 = s->real_exposure[i_colour] & 0xff;
			/*
			 * Windows are 1,2,3, if infrared came first we don't
			 * need to adjust
			 */
			window = i_colour + !s->infrared;
		}

		status = ls5000_issue_cmd(s, 0,
				LS5000_CMD_SET_WINDOW,
				0,		/* LUN */
				0, 0, 0, 0,	/* reserved */
				0, 0, 58,	/* transfer length */
				0x80,		/* control */
				0, 0, 0,	/* reserved */
				0, 0, 0,	/* reserved */
				0, 50,		/* window descriptor length */
				window,		/* window ID */
				0,		/* reserved */
				s->res >> 8,
				s->res & 0xff,	/* X resolution */
				s->res >> 8,
				s->res & 0xff,	/* Y resolution */
				(s->real_xoffset >> 24) & 0xff,
				(s->real_xoffset >> 16) & 0xff,
				(s->real_xoffset >> 8) & 0xff,
				s->real_xoffset & 0xff, /* Upper Left X offset */
				(s->real_yoffset >> 24) & 0xff,
				(s->real_yoffset >> 16) & 0xff,
				(s->real_yoffset >> 8) & 0xff,
				s->real_yoffset & 0xff, /* Upper Left Y offset */
				(s->real_width >> 24) & 0xff,
				(s->real_width >> 16) & 0xff,
				(s->real_width >> 8) & 0xff,
				s->real_width & 0xff, /* Window width */
				(s->real_height >> 24) & 0xff,
				(s->real_height >> 16) & 0xff,
				(s->real_height >> 8) & 0xff,
				s->real_height & 0xff, /* Window height */
				0,		/* brightness */
				0,		/* threshold */
				0,		/* contrast */
				s->gray_scan?2:5,/* image composition: rgb/gray */
				16,		/* pixel composition */
				0, 0,		/* halftone pattern */
				0,		/* reserved, padding type */
				0, 0,		/* bit ordering */
				0, 0,		/* compression type, args */
				0, 0, 0,
				0, 0, 0,	/* reserved */
				0,		/* multiread, color ordering */
				0 | (s->negative ? 0 : 1), /* averaging, pos/neg */
				scan_kind,	/* scanning kind */
				0x02,		/* scanning mode (normal quality) */
				0x02,		/* colour interleaving (line w/o ccd distance) */
				0xff,		/* auto exposure target */
				exp_b0,
				exp_b1,
				exp_b2,
				exp_b3);	/* exposure (32-bit value) */
		if (status)
			return status;
	}

	status = ls5000_focus(s);
	if (status)
		return status;

 scan:
	switch (n_colour) {
	case 1:
		status = ls5000_issue_cmd(s, 0,
				LS5000_CMD_SCAN,
				0,		/* LUN */
				0, 0,		/* reserved */
				1,		/* transfer length (#windows) */
				0,		/* control */
				1);		/* windows: red */
		break;
	case 2:
		status = ls5000_issue_cmd(s, 0,
				LS5000_CMD_SCAN,
				0,		/* LUN */
				0, 0,		/* reserved */
				2,		/* transfer length (#windows) */
				0,		/* control */
				1, 9);		/* windows: red,ir */
		break;
	case 3:
		status = ls5000_issue_cmd(s, 0,
				LS5000_CMD_SCAN,
				0,		/* LUN */
				0, 0,		/* reserved */
				3,		/* transfer length (#windows) */
				0,		/* control */
				1, 2, 3);	/* windows: red,green,blue */
		break;
	case 4:
		status = ls5000_issue_cmd(s, 0,
				LS5000_CMD_SCAN,
				0,		/* LUN */
				0, 0,		/* reserved */
				4,		/* transfer length (#windows) */
				0,		/* control */
				1, 2, 3, 9);	/* windows: red,green,blue,ir */
		break;
	default:
		DBG(1, "BUG: ls5000_scan: Unknown number of input colours (%d).\n", n_colour);
		break;
	}
	if (s->status == LS5000_STATUS_REISSUE) {
		status = ls5000_issue_cmd(s, 6,
				LS5000_CMD_READ,
				0,		/* LUN */
				0x87,		/* Initiator cooperative action parameter */
				0,		/* reserved */
				0, 0,		/* data type qualifier */
				0, 0, 6,	/* transfer length */
				0x80);		/* control */
		if (status)
			return status;
		/*
		 * received data is now
		 * byte 0: data type code
		 *	1: data item bits
		 *	2-5: data len
		 */
		status = ls5000_issue_cmd(s, s->recv_buf[5] + 6,
				LS5000_CMD_READ,
				0,		/* LUN */
				0x87,		/* Initiator cooperative action parameter */
				0,		/* reserved */
				0, 0,		/* data type qualifier */
				0, 0, s->recv_buf[5] + 6, /* transfer length */
				0x80);		/* control */
		if (status)
			return status;
		/*
		 * received data is now
		 * byte 0: data type code
		 *	1: data item bits
		 *	2-5: data len
		 *	6-: data
		 */
		if ((s->recv_buf[11] != 0x08) || (s->recv_buf[12] != 0x00))
			DBG(1, "BUG: ls5000_scan: Unexpected line_padding position.\n");
		s->line_padding = 256 * s->recv_buf[19] + s->recv_buf[20];
		goto scan;
	}

	tmo = 15;
	stat = ls5000_get_status(s);
	while (stat == LS5000_BECOMING_READY ||
	       stat == LS5000_UNIT_ATTENTION ||
	       stat == LS5000_CMD_SEQ_ERR) {
		usleep(500000);
		stat = ls5000_get_status(s);
		if (!tmo--)
			return SANE_STATUS_IO_ERROR;
	}

	return ls5000_get_error(s);
}

/* use page == -1 to query evpd 0/page 0 */
static SANE_Status ls5000_page_inquiry(ls5000_t *s, int page)
{
	SANE_Status status;
	size_t len;
	char evpd = 0;

	if (page >= 0) {
		/* check length of requested page */
		evpd = 1;
		status = ls5000_issue_cmd(s, 4,
				LS5000_CMD_INQUIRY,
				evpd,		/* EVPD */
				page,		/* page code */
				0,		/* reserved */
				4,		/* size */
				0);		/* control */
		if (status) {
			DBG(4, "Error: ls5000_page_inquiry: retrieving size of page 0x%x failed: %s.\n",
			    page, sane_strstatus(status));
			return status;
		}
		/* the required length is at byte 3 (w/o header len) */
		len = s->recv_buf[3] + 4;
	} else {
		/*
		 * the only page with evpd (vital product data) == 0 is 0,
		 * it always has length 36
		 */
		len = 36;
		evpd = 0;
		page = 0;
	}

	status = ls5000_issue_cmd(s, len,
			LS5000_CMD_INQUIRY,
			evpd,		/* EVPD */
			page,		/* page code */
			0,		/* reserved */
			len,		/* size */
			0);		/* control */
	if (status) {
		DBG(4, "Error: ls5000_page_inquiry failed: %s.\n",
		    sane_strstatus(status));
		return status;
	}

	return SANE_STATUS_GOOD;
}

static void ls5000_close(ls5000_t *s)
{
	free(s->lut_r);
	free(s->lut_g);
	free(s->lut_b);
	free(s->lut_neutral);
	free(s->ir_data);

	sanei_usb_close(s->fd);
	open_devices--;
	free(s);
}

static SANE_Status
ls5000_open(const char *device, ls5000_t **sp)
{
	SANE_Status status;
	ls5000_t *s;
	char *line;
	SANE_Device **device_list_new;

	s = malloc(sizeof(ls5000_t));
	if (!s)
		return SANE_STATUS_NO_MEM;
	memset(s, 0, sizeof(ls5000_t));

	status = sanei_usb_open(device, &s->fd);
	if (status) {
		free(s);
		return status;
	}

	open_devices++;

	/* identify scanner */
	status = ls5000_page_inquiry(s, -1);
	if (status) {
		DBG(4, "Error: ls5000_open: failed to get page 0: %s.\n",
		    sane_strstatus(status));
		ls5000_close(s);
		return status;
	}

	strncpy(s->vendor_string, (char*)s->recv_buf + 8, 8);
	s->vendor_string[8] = '\0';
	strncpy(s->product_string, (char*)s->recv_buf + 16, 16);
	s->product_string[16] = '\0';
	strncpy(s->revision_string, (char*)s->recv_buf + 32, 4);
	s->revision_string[4] = '\0';

	DBG(10, "ls5000_open: Inquiry reveals: vendor = '%s', product = '%s', revision = '%s'.\n",
	    s->vendor_string, s->product_string, s->revision_string);

	if (strncmp(s->product_string, "LS-5000 ED      ", 16) != 0) {
		ls5000_close(s);
		return SANE_STATUS_UNSUPPORTED;
	}

	if (sp) {
		*sp = s;
		return SANE_STATUS_GOOD;
	}

	device_list_new = realloc(device_list, (n_device_list + 2) * sizeof(SANE_Device *));
	if (!device_list_new)
		return SANE_STATUS_NO_MEM;
	device_list = device_list_new;

	device_list[n_device_list] = malloc(sizeof(SANE_Device));
	if (!device_list[n_device_list])
		return SANE_STATUS_NO_MEM;

	line = malloc(strlen(device) + 1);
	if (!line)
		goto error;
	strcpy(line, device);
	device_list[n_device_list]->name = line;

	line = malloc(strlen(s->vendor_string) + 1);
	if (!line)
		goto error;
	strcpy(line, s->vendor_string);
	device_list[n_device_list]->vendor = line;

	line = malloc(strlen(s->product_string) + 1);
	if (!line)
		goto error;

	strcpy(line, s->product_string);
	device_list[n_device_list]->model = line;

	device_list[n_device_list]->type = "film scanner";
	n_device_list++;
	device_list[n_device_list] = NULL;

	ls5000_close(s);

	return SANE_STATUS_GOOD;
 error:
	free(device_list[n_device_list]->name);
	free(device_list[n_device_list]->vendor);
	free(device_list[n_device_list]->model);
	free(device_list[n_device_list]);
	return SANE_STATUS_NO_MEM;
}

static SANE_Status ls5000_full_inquiry(ls5000_t *s)
{
	SANE_Status status;
	int pitch, pitch_max, asciilen;
	ls5000_pixel_t pixel;

	status = ls5000_page_inquiry(s, 0xc1);
	if (status)
		return status;

	s->maxbits = s->recv_buf[82];
	s->n_lut = 1;
	s->n_lut <<= s->maxbits;
	if (s->n_lut > 1) {
		s->lut_r = realloc(s->lut_r, s->n_lut * sizeof(ls5000_pixel_t));
		s->lut_g = realloc(s->lut_g, s->n_lut * sizeof(ls5000_pixel_t));
		s->lut_b = realloc(s->lut_b, s->n_lut * sizeof(ls5000_pixel_t));
		s->lut_neutral = realloc(s->lut_neutral, s->n_lut * sizeof(ls5000_pixel_t));

		if (!s->lut_r || !s->lut_g || !s->lut_b || !s->lut_neutral) {
			free(s->lut_r);
			free(s->lut_g);
			free(s->lut_b);
			free(s->lut_neutral);
			return SANE_STATUS_NO_MEM;
		}

		for (pixel = 0; pixel < s->n_lut; pixel++)
			s->lut_r[pixel] = s->lut_g[pixel] = s->lut_b[pixel] =
			    s->lut_neutral[pixel] = pixel;
	} else {
		free(s->lut_r);
		free(s->lut_g);
		free(s->lut_b);
		free(s->lut_neutral);
		s->lut_r = NULL;
		s->lut_g = NULL;
		s->lut_b = NULL;
		s->lut_neutral = NULL;
	}
	s->res_optical = 256 * s->recv_buf[18] + s->recv_buf[19];
	s->res_max = 256 * s->recv_buf[20] + s->recv_buf[21];
	s->res_min = 256 * s->recv_buf[22] + s->recv_buf[23];
	s->boundaryx =
	    65536 * (256 * s->recv_buf[36] + s->recv_buf[37]) +
	    256 * s->recv_buf[38] + s->recv_buf[39];

	s->boundaryy =
	    65536 * (256 * s->recv_buf[58] + s->recv_buf[59]) +
	    256 * s->recv_buf[60] + s->recv_buf[61];

	s->focus_min = 256 * s->recv_buf[76] + s->recv_buf[77];
	s->focus_max = 256 * s->recv_buf[78] + s->recv_buf[79];

	s->n_frames = s->recv_buf[75];

	/* XXX */
	s->frame_offset = s->res_max * 1.5 + 1;

	/* generate resolution list */
	s->res_n_list = pitch_max = floor(s->res_max / (double)s->res_min);
	s->res_list = realloc(s->res_list, pitch_max * sizeof(unsigned int));
	for (pitch = 1; pitch <= pitch_max; pitch++)
		s->res_list[pitch - 1] = s->res_max / pitch;

	s->unit_dpi = s->res_max;
	s->unit_mm = 25.4 / s->unit_dpi;

	/* get adapter information */
	status = ls5000_page_inquiry(s, 1);
	if (status) {
		strncpy(s->adapter, "(none)", sizeof(s->adapter) - 1);
		return SANE_STATUS_GOOD;
	}

	asciilen = s->recv_buf[4];
	if (asciilen > sizeof(s->adapter) - 1)
		asciilen = sizeof(s->adapter) - 1;
	strncpy(s->adapter, (char*)s->recv_buf + 5, asciilen);

/*	This doesn't seem to work as documented, I get
	"Mount" in the second string as well.

	if (strcmp(s->adapter, "Mount") == 0) {
		DBG(10, "mount adapter\n");
		status = ls5000_page_inquiry(s, 0x10);
		if (status)
			return SANE_STATUS_GOOD;
		asciilen = s->recv_buf[4];
		snprintf(s->adapter, sizeof(s->adapter),
			 "Mount: %*s", asciilen-1, (char*)s->recv_buf + 5);
	}
*/

	return SANE_STATUS_GOOD;
}

static SANE_Status ls5000_load(ls5000_t *s)
{
	SANE_Status status;

	DBG(20, "LOAD\n");
	status = ls5000_issue_cmd(s, 0,
			LS5000_CMD_SET_PARAM,
			0,		/* LUN */
			0xd1,		/* Operation "Load object" */
			0, 0, 0,	/* reserved */
			0, 0, 13,	/* parameter length */
			0,		/* ctrl */
			0,0,0,0,0,0,0,0,0,0,0,0,0 /* dummy parameters */
			);
	if (status)
		return status;

	return ls5000_execute(s);
}

static SANE_Status ls5000_reset(ls5000_t *s)
{
	SANE_Status status;

	DBG(20, "RESET\n");
	status = ls5000_issue_cmd(s, 0,
			LS5000_CMD_SET_PARAM,
			0,		/* LUN */
			0x80,		/* Operation "Initialize" */
			0, 0, 0,	/* reserved */
			0, 0, 13,	/* parameter length */
			0,		/* ctrl */
			0,0,0,0,0,0,0,0,0,0,0,0,0 /* dummy parameters */
			);
	if (status)
		return status;

	status = ls5000_execute(s);
	if (status)
		return status;

	return ls5000_full_inquiry(s);
}

/* SANE entry points */

SANE_Status sane_ls5000_init(SANE_Int * version_code, SANE_Auth_Callback authorize)
{
	(void)authorize; /* shut up compiler */
	DBG_INIT();

	if (version_code)
		/* XXX V_MAJOR, V_MINOR */
		*version_code = SANE_VERSION_CODE(1, 0, 0);

	sanei_usb_init();

	return SANE_STATUS_GOOD;
}

void sane_ls5000_exit(void)
{
	int i;

	for (i = 0; i < n_device_list; i++) {
		free(device_list[i]->name);
		free(device_list[i]->vendor);
		free(device_list[i]->model);
		free(device_list[i]);
	}
	free(device_list);
}

/* just a tiny helper */
static SANE_Status ls5000_attach(const char *dev)
{
	return ls5000_open(dev, NULL);
}

SANE_Status sane_ls5000_get_devices(const SANE_Device ***list, SANE_Bool local_only)
{
	(void)local_only; /* shut up compiler */

	if (device_list)
		DBG(6, "sane_ls5000_get_devices: Device list already populated, not probing again.\n");
	else {
		if (open_devices) {
			DBG(4, "sane_ls5000_get_devices: Devices open, not scanning for scanners.\n");
			return SANE_STATUS_IO_ERROR;
		}

		sanei_usb_attach_matching_devices("usb 0x04b0 0x4002", ls5000_attach);

		switch (n_device_list) {
		case 0:
			DBG(6, "sane_ls5000_get_devices: No devices detected.\n");
			break;
		case 1:
			DBG(6, "sane_ls5000_get_devices: 1 device detected.\n");
			break;
		default:
			DBG(6, "sane_ls5000_get_devices: %i devices detected.\n",
			    n_device_list);
			break;
		}
	}

	*list = (const SANE_Device **)device_list;

	return SANE_STATUS_GOOD;
}

SANE_Status sane_ls5000_open(SANE_String_Const name, SANE_Handle * h)
{
	SANE_Status status;
	ls5000_t *s;
	int i_option;
	unsigned int i_list;
	SANE_Option_Descriptor o;
	SANE_Word *word_list = NULL;
	SANE_Range *range = NULL;
	SANE_String_Const *str_list;

	status = ls5000_open(name, &s);
	if (status)
		return status;

	*h = (SANE_Handle) s;

	/* get device properties */

	s->lut_r = s->lut_g = s->lut_b = s->lut_neutral = NULL;
	s->res_list = NULL;
	s->res_n_list = 0;

	status = ls5000_full_inquiry(s);
	if (status)
		return status;

	/* option descriptors */

	for (i_option = 0; i_option < LS5000_N_OPTIONS; i_option++) {
		o.name = o.title = o.desc = NULL;
		o.type = o.unit = o.cap = o.constraint_type = o.size = 0;
		o.constraint.range = NULL;	/* only one union member needs to be NULLed */
		switch (i_option) {
		case LS5000_OPTION_NUM:
			o.name = "";
			o.title = SANE_TITLE_NUM_OPTIONS;
			o.desc = SANE_DESC_NUM_OPTIONS;
			o.type = SANE_TYPE_INT;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_ADAPTER:
			o.name = SANE_NAME_SCAN_SOURCE;
			o.title = SANE_TITLE_SCAN_SOURCE;
			o.desc = SANE_DESC_SCAN_SOURCE;
			o.type = SANE_TYPE_STRING;
			o.size = sizeof(s->adapter);
			o.cap = SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_NONE;
			break;
		case LS5000_OPTION_PREVIEW:
			o.name = SANE_NAME_PREVIEW;
			o.title = SANE_TITLE_PREVIEW;
			o.desc = SANE_DESC_PREVIEW;
			o.type = SANE_TYPE_BOOL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_NONE;
			break;
		case LS5000_OPTION_NEGATIVE:
			o.name = "negative";
			o.title = "Negative";
			o.desc = "Negative film: make scanner invert colours";
			o.type = SANE_TYPE_BOOL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT | SANE_CAP_INACTIVE;
			break;
		case LS5000_OPTION_INFRARED:
			o.name = "infrared";
			o.title = "Read infrared channel";
			o.desc = "Read infrared channel as second image";
			o.type = SANE_TYPE_BOOL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_SCAN_MODE:
			o.name = SANE_NAME_SCAN_MODE;
			o.title = SANE_TITLE_SCAN_MODE;
			o.desc = SANE_DESC_SCAN_MODE;
			o.type = SANE_TYPE_STRING;
			o.size = 100;
			o.constraint_type = SANE_CONSTRAINT_STRING_LIST;
			str_list = malloc(3*sizeof(char *));
			if (!str_list)
				goto error;
			str_list[0] = SANE_VALUE_SCAN_MODE_COLOR;
			str_list[1] = SANE_VALUE_SCAN_MODE_GRAY;
			str_list[2] = NULL;
			o.constraint.string_list = str_list;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_EXPOSURE:
			o.name = "exposure";
			o.title = "Exposure multiplier";
			o.desc = "Exposure multiplier for all channels";
			o.type = SANE_TYPE_FIXED;
			o.unit = SANE_UNIT_NONE;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = SANE_FIX(0.);
			range->max = SANE_FIX(10.);
			range->quant = SANE_FIX(0.1);
			o.constraint.range = range;
			break;
		case LS5000_OPTION_EXPOSURE_R:
			o.name = SANE_NAME_SCAN_EXPOS_TIME_R;
			o.title = SANE_TITLE_SCAN_EXPOS_TIME_R;
			o.desc = SANE_DESC_SCAN_EXPOS_TIME_R;
			o.type = SANE_TYPE_FIXED;
			o.unit = SANE_UNIT_MICROSECOND;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = SANE_FIX(50.);
			range->max = SANE_FIX(20000.);
			range->quant = SANE_FIX(10.);
			o.constraint.range = range;
			break;
		case LS5000_OPTION_EXPOSURE_G:
			o.name = SANE_NAME_SCAN_EXPOS_TIME_G;
			o.title = SANE_TITLE_SCAN_EXPOS_TIME_G;
			o.desc = SANE_DESC_SCAN_EXPOS_TIME_G;
			o.type = SANE_TYPE_FIXED;
			o.unit = SANE_UNIT_MICROSECOND;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = SANE_FIX(50.);
			range->max = SANE_FIX(20000.);
			range->quant = SANE_FIX(10.);
			o.constraint.range = range;
			break;
		case LS5000_OPTION_EXPOSURE_B:
			o.name = SANE_NAME_SCAN_EXPOS_TIME_B;
			o.title = SANE_TITLE_SCAN_EXPOS_TIME_B;
			o.desc = SANE_DESC_SCAN_EXPOS_TIME_B;
			o.type = SANE_TYPE_FIXED;
			o.unit = SANE_UNIT_MICROSECOND;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = SANE_FIX(50.);
			range->max = SANE_FIX(20000.);
			range->quant = SANE_FIX(10.);
			o.constraint.range = range;
			break;
		case LS5000_OPTION_LUT_R:
			o.name = SANE_NAME_GAMMA_VECTOR_R;
			o.title = SANE_TITLE_GAMMA_VECTOR_R;
			o.desc = SANE_DESC_GAMMA_VECTOR_R;
			o.type = SANE_TYPE_INT;
			o.size = s->n_lut * sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->n_lut - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_LUT_G:
			o.name = SANE_NAME_GAMMA_VECTOR_G;
			o.title = SANE_TITLE_GAMMA_VECTOR_G;
			o.desc = SANE_DESC_GAMMA_VECTOR_G;
			o.type = SANE_TYPE_INT;
			o.size = s->n_lut * sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->n_lut - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_LUT_B:
			o.name = SANE_NAME_GAMMA_VECTOR_B;
			o.title = SANE_TITLE_GAMMA_VECTOR_B;
			o.desc = SANE_DESC_GAMMA_VECTOR_B;
			o.type = SANE_TYPE_INT;
			o.size = s->n_lut * sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->n_lut - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_LOAD:
			o.name = "load";
			o.title = "Load";
			o.desc = "Load next slide";
			o.type = SANE_TYPE_BUTTON;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_EJECT:
			o.name = "eject";
			o.title = "Eject";
			o.desc = "Eject loaded medium";
			o.type = SANE_TYPE_BUTTON;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_RESET:
			o.name = "reset";
			o.title = "Reset scanner";
			o.desc = "Initialize scanner";
			o.type = SANE_TYPE_BUTTON;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_INQUIRE:
			o.name = "inquire";
			o.title = "Inquire scanner";
			o.desc = "Inquire scanner status (attached adapter etc.)";
			o.type = SANE_TYPE_BUTTON;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_RES:
			o.name = SANE_NAME_SCAN_RESOLUTION;
			o.title = SANE_TITLE_SCAN_RESOLUTION;
			o.desc = SANE_DESC_SCAN_RESOLUTION;
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_DPI;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_WORD_LIST;
			word_list = malloc((s->res_n_list + 1) * sizeof(SANE_Word));
			if (!word_list)
				goto error;
			for (i_list = 0; i_list < s->res_n_list; i_list++)
				word_list[i_list + 1] = s->res_list[i_list];
			word_list[0] = s->res_n_list;
			o.constraint.word_list = word_list;
			break;
		case LS5000_OPTION_FRAME:
			o.name = "frame";
			o.title = "Frame number";
			o.desc = "Number of frame to be scanned, starting with 1";
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_NONE;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			if (s->n_frames <= 1)
				o.cap |= SANE_CAP_INACTIVE;
			else
				o.cap &= ~SANE_CAP_INACTIVE;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 1;
			range->max = s->n_frames;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_SUBFRAME:
			o.name = "subframe";
			o.title = "Frame shift";
			o.desc = "Fine position within the selected frame";
			o.type = SANE_TYPE_FIXED;
			o.unit = SANE_UNIT_MM;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = SANE_FIX(0.);
			range->max =
			    SANE_FIX((s->boundaryy - 1) * s->unit_mm);
			range->quant = SANE_FIX(0.);
			o.constraint.range = range;
			break;
		case LS5000_OPTION_XMIN:
			o.name = SANE_NAME_SCAN_TL_X;
			o.title = SANE_TITLE_SCAN_TL_X;
			o.desc = SANE_DESC_SCAN_TL_X;
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_PIXEL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			if (!range)
				goto error;
			range =
			    (SANE_Range *)
			    malloc(sizeof(SANE_Range));
			range->min = 0;
			range->max = s->boundaryx - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_XMAX:
			o.name = SANE_NAME_SCAN_BR_X;
			o.title = SANE_TITLE_SCAN_BR_X;
			o.desc = SANE_DESC_SCAN_BR_X;
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_PIXEL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->boundaryx - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_YMIN:
			o.name = SANE_NAME_SCAN_TL_Y;
			o.title = SANE_TITLE_SCAN_TL_Y;
			o.desc = SANE_DESC_SCAN_TL_Y;
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_PIXEL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->boundaryy - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_YMAX:
			o.name = SANE_NAME_SCAN_BR_Y;
			o.title = SANE_TITLE_SCAN_BR_Y;
			o.desc = SANE_DESC_SCAN_BR_Y;
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_PIXEL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->boundaryy - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_FOCUS_ON_CENTRE:
			o.name = "focus-on-centre";
			o.title = "Use centre of scan area as AF point";
			o.desc =
			    "Use centre of scan area as AF point instead of manual AF point selection";
			o.type = SANE_TYPE_BOOL;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_FOCUS:
			o.name = "focus";
			o.title = "Focus position";
			o.desc = "Focus position for manual focus";
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_NONE;
			o.size = sizeof(SANE_Word);
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = s->focus_min;
			range->max = s->focus_max;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_AUTOFOCUS:
			o.name = "autofocus";
			o.title = "Autofocus now";
			o.desc = "Autofocus now";
			o.type = SANE_TYPE_BUTTON;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_FOCUSX:
			o.name = "focusx";
			o.title = "X coordinate of AF point";
			o.desc = "X coordinate of AF point";
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_PIXEL;
			o.size = sizeof(SANE_Word);
			o.cap =
			    SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT |
			    SANE_CAP_INACTIVE;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->boundaryx - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_FOCUSY:
			o.name = "focusy";
			o.title = "Y coordinate of AF point";
			o.desc = "Y coordinate of AF point";
			o.type = SANE_TYPE_INT;
			o.unit = SANE_UNIT_PIXEL;
			o.size = sizeof(SANE_Word);
			o.cap =
			    SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT |
			    SANE_CAP_INACTIVE;
			o.constraint_type = SANE_CONSTRAINT_RANGE;
			range = malloc(sizeof(SANE_Range));
			if (!range)
				goto error;
			range->min = 0;
			range->max = s->boundaryy - 1;
			range->quant = 1;
			o.constraint.range = range;
			break;
		case LS5000_OPTION_SCAN_AE:
			o.name = "ae";
			o.title = "Auto-exposure scan now";
			o.desc = "Perform auto-exposure scan";
			o.type = SANE_TYPE_BUTTON;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		case LS5000_OPTION_SCAN_AE_WB:
			o.name = "ae-wb";
			o.title = "Auto-exposure scan with white balance now";
			o.desc = "Perform auto-exposure scan with white balance";
			o.type = SANE_TYPE_BUTTON;
			o.cap = SANE_CAP_SOFT_SELECT | SANE_CAP_SOFT_DETECT;
			break;
		default:
			DBG(1, "BUG: sane_ls5000_open: Unknown option number.\n");
			return SANE_STATUS_INVAL;
			break;
		}
		s->option_list[i_option] = o;
	}

	s->scan_stage = LS5000_SCAN_STAGE_IDLE;
	s->negative = SANE_FALSE;
	s->infrared = SANE_FALSE;
	s->pinfrared = SANE_FALSE;
	s->gray_scan = SANE_FALSE;
	s->preview = SANE_FALSE;
	s->i_frame = 1;
	s->subframe = 0.;
	s->res = s->res_max;
	s->xmin = 0;
	s->xmax = s->boundaryx - 1;
	s->ymin = 0;
	s->ymax = s->boundaryy - 1;
	s->focus_on_centre = SANE_TRUE;
	s->focus = 0;
	s->focusx = 0;
	s->focusy = 0;
	s->exposure = 1.;
	s->exposure_r = 1200.;
	s->exposure_g = 1200.;
	s->exposure_b = 1000.;

	return SANE_STATUS_GOOD;
 error:
	ls5000_close(s);
	return SANE_STATUS_NO_MEM;
}

void sane_ls5000_close(SANE_Handle h)
{
	ls5000_t *s = (ls5000_t *) h;

	ls5000_close(s);
}

const SANE_Option_Descriptor *
sane_ls5000_get_option_descriptor(SANE_Handle h, SANE_Int n)
{
	ls5000_t *s = (ls5000_t *) h;

	if ((n >= 0) && (n < LS5000_N_OPTIONS))
		return &s->option_list[n];
	else
		return NULL;
}

static SANE_Status
ls5000_get_option_value(ls5000_t *s, SANE_Int option, void *v)
{
	ls5000_pixel_t pixel;

	switch (option) {
	case LS5000_OPTION_NUM:
		*(SANE_Word *) v = LS5000_N_OPTIONS;
		break;
	case LS5000_OPTION_ADAPTER:
		strcpy(v, s->adapter);
		break;
	case LS5000_OPTION_PREVIEW:
		*(SANE_Word *) v = s->preview;
		break;
	case LS5000_OPTION_NEGATIVE:
		*(SANE_Word *) v = s->negative;
		break;
	case LS5000_OPTION_INFRARED:
		*(SANE_Word *) v = s->pinfrared;
		break;
	case LS5000_OPTION_SCAN_MODE:
		if (s->gray_scan)
			strcpy(v, SANE_VALUE_SCAN_MODE_GRAY);
		else
			strcpy(v, SANE_VALUE_SCAN_MODE_COLOR);
		break;
	case LS5000_OPTION_EXPOSURE:
		*(SANE_Word *) v = SANE_FIX(s->exposure);
		break;
	case LS5000_OPTION_EXPOSURE_R:
		*(SANE_Word *) v = SANE_FIX(s->exposure_r);
		break;
	case LS5000_OPTION_EXPOSURE_G:
		*(SANE_Word *) v = SANE_FIX(s->exposure_g);
		break;
	case LS5000_OPTION_EXPOSURE_B:
		*(SANE_Word *) v = SANE_FIX(s->exposure_b);
		break;
	case LS5000_OPTION_LUT_R:
		if (!(s->lut_r))
			return SANE_STATUS_INVAL;
		for (pixel = 0; pixel < s->n_lut; pixel++)
			((SANE_Word *) v)[pixel] = s->lut_r[pixel];
		break;
	case LS5000_OPTION_LUT_G:
		if (!(s->lut_g))
			return SANE_STATUS_INVAL;
		for (pixel = 0; pixel < s->n_lut; pixel++)
			((SANE_Word *) v)[pixel] = s->lut_g[pixel];
		break;
	case LS5000_OPTION_LUT_B:
		if (!(s->lut_b))
			return SANE_STATUS_INVAL;
		for (pixel = 0; pixel < s->n_lut; pixel++)
			((SANE_Word *) v)[pixel] = s->lut_b[pixel];
		break;
	case LS5000_OPTION_EJECT:
		break;
	case LS5000_OPTION_LOAD:
		break;
	case LS5000_OPTION_RESET:
		break;
	case LS5000_OPTION_INQUIRE:
		break;
	case LS5000_OPTION_FRAME:
		*(SANE_Word *) v = s->i_frame;
		break;
	case LS5000_OPTION_SUBFRAME:
		*(SANE_Word *) v = SANE_FIX(s->subframe);
		break;
	case LS5000_OPTION_RES:
		*(SANE_Word *) v = s->res;
		break;
	case LS5000_OPTION_XMIN:
		*(SANE_Word *) v = s->xmin;
		break;
	case LS5000_OPTION_XMAX:
		*(SANE_Word *) v = s->xmax;
		break;
	case LS5000_OPTION_YMIN:
		*(SANE_Word *) v = s->ymin;
		break;
	case LS5000_OPTION_YMAX:
		*(SANE_Word *) v = s->ymax;
		break;
	case LS5000_OPTION_FOCUS_ON_CENTRE:
		*(SANE_Word *) v = s->focus_on_centre;
		break;
	case LS5000_OPTION_FOCUS:
		*(SANE_Word *) v = s->focus;
		break;
	case LS5000_OPTION_AUTOFOCUS:
		break;
	case LS5000_OPTION_FOCUSX:
		*(SANE_Word *) v = s->focusx;
		break;
	case LS5000_OPTION_FOCUSY:
		*(SANE_Word *) v = s->focusy;
		break;
	case LS5000_OPTION_SCAN_AE:
		break;
	case LS5000_OPTION_SCAN_AE_WB:
		break;
	}

	return SANE_STATUS_GOOD;
}

static SANE_Status
ls5000_set_option_value(ls5000_t *s, SANE_Int option, void *v, int *flags)
{
	ls5000_pixel_t pixel;
	SANE_Option_Descriptor o = s->option_list[option];
	SANE_Status status = SANE_STATUS_GOOD;

	if (s->scan_stage != LS5000_SCAN_STAGE_IDLE)
		return SANE_STATUS_INVAL;
	/* XXX do this for all elements of arrays */
	switch (o.type) {
	case SANE_TYPE_BOOL:
		if (!v) return SANE_STATUS_INVAL;
		if ((*(SANE_Word *) v != SANE_TRUE)
		    && (*(SANE_Word *) v != SANE_FALSE))
			return SANE_STATUS_INVAL;
		break;
	case SANE_TYPE_INT:
	case SANE_TYPE_FIXED:
		if (!v) return SANE_STATUS_INVAL;
		switch (o.constraint_type) {
		case SANE_CONSTRAINT_RANGE:
			if (*(SANE_Word *) v < o.constraint.range->min) {
				*(SANE_Word *) v = o.constraint.range->min;
				*flags |= SANE_INFO_INEXACT;
			} else if (*(SANE_Word *) v > o.constraint.range->max) {
				*(SANE_Word *) v = o.constraint.range->max;
				*flags |= SANE_INFO_INEXACT;
			}
			break;
		case SANE_CONSTRAINT_WORD_LIST:
			break;
		default:
			break;
		}
		break;
	case SANE_TYPE_STRING:
		if (!v) return SANE_STATUS_INVAL;
		break;
	case SANE_TYPE_BUTTON:
		break;
	case SANE_TYPE_GROUP:
		break;
	}

	switch (option) {
	case LS5000_OPTION_NUM:
		return SANE_STATUS_INVAL;
		break;
	case LS5000_OPTION_ADAPTER:
		return SANE_STATUS_INVAL;
		break;
	case LS5000_OPTION_PREVIEW:
		s->preview = *(SANE_Word *) v;
		if (s->preview) {
			s->infrared = SANE_FALSE;
			free(s->ir_data);
			s->ir_data = NULL;
		} else
			s->infrared = s->pinfrared;
		break;
	case LS5000_OPTION_NEGATIVE:
		s->negative = *(SANE_Word *) v;
		break;
	case LS5000_OPTION_INFRARED:
		s->infrared = *(SANE_Word *) v;
		s->pinfrared = *(SANE_Word *) v;
		break;
	case LS5000_OPTION_SCAN_MODE:
		if (strcmp(v, SANE_VALUE_SCAN_MODE_GRAY) == 0)
			s->gray_scan = SANE_TRUE;
		else if (strcmp(v, SANE_VALUE_SCAN_MODE_COLOR) == 0)
			s->gray_scan = SANE_FALSE;
		else
			return SANE_STATUS_INVAL;
		*flags |= SANE_INFO_RELOAD_PARAMS;
		break;
	case LS5000_OPTION_EXPOSURE:
		s->exposure = SANE_UNFIX(*(SANE_Word *) v);
		break;
	case LS5000_OPTION_EXPOSURE_R:
		s->exposure_r = SANE_UNFIX(*(SANE_Word *) v);
		break;
	case LS5000_OPTION_EXPOSURE_G:
		s->exposure_g = SANE_UNFIX(*(SANE_Word *) v);
		break;
	case LS5000_OPTION_EXPOSURE_B:
		s->exposure_b = SANE_UNFIX(*(SANE_Word *) v);
		break;
	case LS5000_OPTION_LUT_R:
		if (!(s->lut_r))
			return SANE_STATUS_INVAL;
		for (pixel = 0; pixel < s->n_lut; pixel++)
			s->lut_r[pixel] = ((SANE_Word *) v)[pixel];
		break;
	case LS5000_OPTION_LUT_G:
		if (!(s->lut_g))
			return SANE_STATUS_INVAL;
		for (pixel = 0; pixel < s->n_lut; pixel++)
			s->lut_g[pixel] = ((SANE_Word *) v)[pixel];
		break;
	case LS5000_OPTION_LUT_B:
		if (!(s->lut_b))
			return SANE_STATUS_INVAL;
		for (pixel = 0; pixel < s->n_lut; pixel++)
			s->lut_b[pixel] = ((SANE_Word *) v)[pixel];
		break;
	case LS5000_OPTION_LOAD:
		ls5000_load(s);
		break;
	case LS5000_OPTION_EJECT:
		ls5000_eject(s);
		break;
	case LS5000_OPTION_RESET:
		*flags |= SANE_INFO_RELOAD_OPTIONS;
		return ls5000_reset(s);
		break;
	case LS5000_OPTION_INQUIRE:
		*flags |= SANE_INFO_RELOAD_OPTIONS;
		return ls5000_full_inquiry(s);
		break;
	case LS5000_OPTION_FRAME:
		s->i_frame = *(SANE_Word *) v;
		break;
	case LS5000_OPTION_SUBFRAME:
		s->subframe = SANE_UNFIX(*(SANE_Word *) v);
		break;
	case LS5000_OPTION_RES:
		s->res = *(SANE_Word *) v;
		*flags |= SANE_INFO_RELOAD_PARAMS;
		break;
	case LS5000_OPTION_XMIN:
		s->xmin = *(SANE_Word *) v;
		*flags |= SANE_INFO_RELOAD_PARAMS;
		break;
	case LS5000_OPTION_XMAX:
		s->xmax = *(SANE_Word *) v;
		*flags |= SANE_INFO_RELOAD_PARAMS;
		break;
	case LS5000_OPTION_YMIN:
		s->ymin = *(SANE_Word *) v;
		*flags |= SANE_INFO_RELOAD_PARAMS;
		break;
	case LS5000_OPTION_YMAX:
		s->ymax = *(SANE_Word *) v;
		*flags |= SANE_INFO_RELOAD_PARAMS;
		break;
	case LS5000_OPTION_FOCUS_ON_CENTRE:
		s->focus_on_centre = *(SANE_Word *) v;
		if (s->focus_on_centre) {
			s->option_list[LS5000_OPTION_FOCUSX].cap |= SANE_CAP_INACTIVE;
			s->option_list[LS5000_OPTION_FOCUSY].cap |= SANE_CAP_INACTIVE;
		} else {
			s->option_list[LS5000_OPTION_FOCUSX].cap &= ~SANE_CAP_INACTIVE;
			s->option_list[LS5000_OPTION_FOCUSY].cap &= ~SANE_CAP_INACTIVE;
		}
		*flags |= SANE_INFO_RELOAD_OPTIONS;
		break;
	case LS5000_OPTION_FOCUS:
		s->focus = *(SANE_Word *) v;
		break;
	case LS5000_OPTION_AUTOFOCUS:
		ls5000_autofocus(s);
		*flags |= SANE_INFO_RELOAD_OPTIONS;
		break;
	case LS5000_OPTION_FOCUSX:
		s->focusx = *(SANE_Word *) v;
		break;
	case LS5000_OPTION_FOCUSY:
		s->focusy = *(SANE_Word *) v;
		break;
	case LS5000_OPTION_SCAN_AE:
		status = ls5000_scan(s, LS5000_SCAN_AE);
		if (status)
			return status;
		status = ls5000_get_exposure(s);
		if (status)
			return status;
		s->exposure = 1.;
		s->exposure_r = s->real_exposure[0] / 100.;
		s->exposure_g = s->real_exposure[1] / 100.;
		s->exposure_b = s->real_exposure[2] / 100.;
		*flags |= SANE_INFO_RELOAD_OPTIONS;
		break;
	case LS5000_OPTION_SCAN_AE_WB:
		status = ls5000_scan(s, LS5000_SCAN_AE_WB);
		if (status)
			return status;
		status = ls5000_get_exposure(s);
		if (status)
			return status;
		s->exposure = 1.;
		s->exposure_r = s->real_exposure[0] / 100.;
		s->exposure_g = s->real_exposure[1] / 100.;
		s->exposure_b = s->real_exposure[2] / 100.;
		*flags |= SANE_INFO_RELOAD_OPTIONS;
		break;
	}

	return status;
}

SANE_Status
sane_ls5000_control_option(SANE_Handle h, SANE_Int option, SANE_Action action,
			   void *v, SANE_Int *outflags)
{
	ls5000_t *s = (ls5000_t *) h;
	SANE_Int *flags, dummyflags = 0;

	if (option >= LS5000_N_OPTIONS)
		return SANE_STATUS_INVAL;

	switch (action) {
	case SANE_ACTION_GET_VALUE:
		return ls5000_get_option_value(s, option, v);
	case SANE_ACTION_SET_VALUE:
		if (outflags)
			flags = outflags;
		else
			flags = &dummyflags;
		return ls5000_set_option_value(s, option, v, flags);
	default:
		DBG(1, "BUG: sane_ls5000_control_option: Unknown action.\n");
		return SANE_STATUS_INVAL;
	}
}

SANE_Status sane_ls5000_get_parameters(SANE_Handle h, SANE_Parameters *p)
{
	ls5000_t *s = (ls5000_t *) h;

	ls5000_convert_options(s);

	p->last_frame = SANE_TRUE;

	if (s->ir_data && s->scan_stage == LS5000_SCAN_STAGE_IDLE) {
		p->format = SANE_FRAME_GRAY;
		p->bytes_per_line = 1 * s->logical_width * 2;
	} else {
		if (s->gray_scan) {
			p->format = SANE_FRAME_GRAY;
			p->bytes_per_line = 1 * s->logical_width * 2;
		} else {
			p->format = SANE_FRAME_RGB;
			p->bytes_per_line = 3 * s->logical_width * 2;
		}
	}

	p->lines = s->logical_height;
	p->depth = 16;
	p->pixels_per_line = s->logical_width;

	return SANE_STATUS_GOOD;
}

SANE_Status sane_ls5000_start(SANE_Handle h)
{
	ls5000_t *s = (ls5000_t *) h;
	SANE_Status status;

	if (s->must_read_now)
		return SANE_STATUS_INVAL;

	if (s->ir_data && s->scan_stage == LS5000_SCAN_STAGE_IDLE) {
		s->must_read_now = SANE_TRUE;
		return SANE_STATUS_GOOD;
	}

	ls5000_convert_options(s);

	status = ls5000_scan(s, LS5000_SCAN_NORMAL);
	if (status)
		return status;

	s->must_read_now = SANE_TRUE;

	if (s->scan_stage == LS5000_SCAN_STAGE_IDLE)
		s->scan_stage = LS5000_SCAN_STAGE_ACQUIRE;

	return SANE_STATUS_GOOD;
}

/*
 * This function does the bulk of the data format conversion,
 * see the comment for sane_ls5000_read.
 */
static void ls5000_shuffle_block(ls5000_t *s)
{
	int line, i;
	int line_padded = s->line_bytes + s->line_padding;
	int line_pixels = s->logical_width;

	if (s->gray_scan) {
		/*
		 * Gray scans are relatively easy, we just need to
		 * split off the infrared data if present.
		 *
		 * A gray-only scan on big-endian machines could be optimised
		 * (less data copying) but that just increases code complexity
		 * for little gain.
		 */
		for (line = 0; line < s->block_lines; line++) {
			uint16_t *src = (uint16_t*)s->block + line*line_padded/2;
			uint16_t *graydst = (uint16_t*)s->ordered_block + line*line_pixels;
			uint16_t *irsrc = src + line_pixels;
			uint16_t *irdst = (uint16_t*)s->ir_data +
					  (s->line + line)*s->logical_width;
#if __BYTE_ORDER == __BIG_ENDIAN
			if (s->infrared)
				memcpy(irdst, irsrc, 2*line_pixels);
			memcpy(graydst, src, 2*line_pixels);
#else
			if (s->infrared)
				for (i = 0; i < line_pixels; i++)
					irdst[i] = ntohs(irsrc[i]);
			for (i = 0; i < line_pixels; i++)
				graydst[i] = ntohs(src[i]);
#endif
		}
	} else {
		/*
		 * For RGB scans we not only need to split off the infrared
		 * data if it is present but also need to rearrange the color
		 * components.
		 */
		for (line = 0; line < s->block_lines; line++) {
			uint16_t *src = (uint16_t*)s->block + line*line_padded/2;
			uint16_t *rgbdst = (uint16_t*)s->ordered_block + line*3*line_pixels;
			uint16_t *irsrc = src + 3*line_pixels;
			uint16_t *irdst = (uint16_t*)s->ir_data +
					  (s->line + line)*s->logical_width;
			if (s->infrared)
#if __BYTE_ORDER == __BIG_ENDIAN
				memcpy(irdst, irsrc, 2*line_pixels);
#else
				for (i = 0; i < line_pixels; i++)
					irdst[i] = ntohs(irsrc[i]);
#endif
			for (i = 0; i < line_pixels; i++) {
				rgbdst[i*3 + 0] = ntohs(src[0*line_pixels + i]);
				rgbdst[i*3 + 1] = ntohs(src[1*line_pixels + i]);
				rgbdst[i*3 + 2] = ntohs(src[2*line_pixels + i]);
			}
		}
	}
}

/*
 * Due to the image format used by the scanner, sane_ls5000_read is
 * a bit more complex.
 *
 * The data sent by the scanner is line-interleaved as opposed to
 * pixel-interleaved which SANE requires.
 *
 * If we're reading an RGB image, we get the data as follows:
 *     RR...RR GG...GG BB...BB
 * where each R/G/B consists of two bytes in big-endian order for each
 * pixel, and the number of each one depends on the line width.
 *
 * For the other modes, we get:
 * rgb/infrared:	RR...RR GG...GG BB...BB II...II
 * gray:		GG...GG
 * gray/infrared:	GG...GG II...II
 *
 * The second complication is that SANE cannot handle infrared channels
 * at all. Hence, we cache the infrared data in memory and send it out
 * as a second image when the next image is scanned.
 */
SANE_Status
sane_ls5000_read(SANE_Handle h, SANE_Byte *buf, SANE_Int maxlen, SANE_Int *len)
{
	ls5000_t *s = (ls5000_t *) h;
	SANE_Status status;
	unsigned long xfer_len;
	size_t n_recv, remaining, offset;
	int colors;

	*len = 0;

	/* colors without infrared */
	colors = 3;
	if (s->gray_scan)
		colors = 1;

	/* sane_ls5000_scan wasn't invoked! */
	if (!s->must_read_now) {
		*len = 0;
		return SANE_STATUS_CANCELLED;
	}

	/*
	 * The scanner is idle (we're not scanning right now) and we
	 * have infrared data pending. That means we just scanned an
	 * image and now it's the second "scan" to get the IR image.
	 */
	if (s->ir_data && s->scan_stage == LS5000_SCAN_STAGE_IDLE) {
		*len = s->ir_data_len - s->block_read_pos;
		if (*len == 0) {
			s->block_read_pos = 0;
			free(s->ir_data);
			s->ir_data = NULL;
			s->scan_stage = LS5000_SCAN_STAGE_IDLE;
			s->must_read_now = SANE_FALSE;
			return SANE_STATUS_EOF;
		}
		if (*len > maxlen)
			*len = maxlen;
		memcpy(buf, s->ir_data + s->block_read_pos, *len);
		s->block_read_pos += *len;
		return SANE_STATUS_GOOD;
	}

	/*
	 * No more lines to read, block read position is 0 (i.e. the whole
	 * block was read) and we do have a block. This means we're done
	 * with the gray/rgb part of the image and possibly everything.
	 */
	if (s->line == s->logical_height && s->block_read_pos == 0 && s->block) {
		*len = 0;
		free(s->block);
		s->block = NULL;
		free(s->ordered_block);
		s->ordered_block = NULL;
		s->line = 0;
		s->must_read_now = SANE_FALSE;
		/* scanimage doesn't call sane_ls5000_cancel between pages */
		s->scan_stage = LS5000_SCAN_STAGE_IDLE;
		return SANE_STATUS_EOF;
	}

	/*
	 * Similar conditions as before, but we don't have a block. This means
	 * that sane_ls5000_scan was just invoked and now we're in sane_read
	 * for the first time after that and should calculate how much data we
	 * need to read and allocate memory as necessary.
	 */
	if (s->line == 0 && s->block_read_pos == 0) {
		/* store how many bytes for each scanline */
		s->line_bytes = (colors + !!s->infrared) * s->logical_width * 2;
		/*
		 * lines are padded to multiples of 512, we have received the
		 * required line padding when the REISSUE status comes from the
		 * scanner.
		 */
		xfer_len = s->logical_height * (s->line_bytes + s->line_padding);
		/*
		 * Allocate memory for the raw block, the reordered block
		 * for the frontend and the infrared data. We could be faster
		 * and use less memory by reordering the data directly into
		 * the frontend buffer, but since we want to read large chunks
		 * from the scanner to speed up the scanner that would increase
		 * code complexity a lot.
		 */
		s->block = malloc(10 * (s->line_bytes + s->line_padding));
		if (!s->block)
			return SANE_STATUS_NO_MEM;
		s->ordered_block = malloc(10 * (s->line_bytes + s->line_padding));
		if (!s->ordered_block)
			return SANE_STATUS_NO_MEM;
		if (s->infrared) {
			s->ir_data = malloc(s->logical_height * s->logical_width * 2);
			s->ir_data_len = s->logical_height * s->logical_width * 2;
			if (!s->ir_data)
				return SANE_STATUS_NO_MEM;
		}
	}

	/*
	 * Now this is where we get either
	 * (a) right after the block above or
	 * (b) when we have already read a few blocks but copied out all data
	 *     to the frontend already.
	 */
	if (s->block_read_pos == 0) {
		/*
		 * Read in a whole block of ten lines, ten is arbitrary,
		 * we probably should adjust this based on line length
		 * for optimal performance.
		 */
		s->block_lines = 10;
		if (s->block_lines > s->logical_height - s->line)
			s->block_lines = s->logical_height - s->line;
		/* calculate how many bytes that means */
		remaining = s->block_lines * (s->line_bytes + s->line_padding);
		/* issue the read command */
		status = ls5000_issue_cmd(s, -1,
				LS5000_CMD_READ,
				0,		/* LUN */
				0,		/* data type: image */
				0,		/* reserved */
				0, 1,		/* data type qualifier: default, 2-byte data */
				(remaining >> 16) & 0xff,
				(remaining >> 8) & 0xff,
				(remaining >> 0) & 0xff, /* transfer length */
				0x80);	/* control */
		if (status != SANE_STATUS_DEVICE_BUSY) {
			*len = 0;
			s->block_read_pos = 0;
			s->must_read_now = SANE_FALSE;
			s->line = 0;
			free(s->block);
			s->block = NULL;
			free(s->ordered_block);
			s->ordered_block = NULL;
			return status;
		}

		/*
		 * Now read data into our block buffer in
		 * small blocks of 512 bytes.
		 */
		offset = 0;
		while (remaining > 0) {
			n_recv = 512;
			status = sanei_usb_read_bulk(s->fd, s->block + offset,
						     &n_recv);
			if (status)
				return status;
			offset += n_recv;
			remaining -= n_recv;
		}

		/* each command requires a status check afterwards */
		status = ls5000_check_status(s);
		if (status) {
			*len = 0;
			s->block_read_pos = 0;
			s->must_read_now = SANE_FALSE;
			s->line = 0;
			free(s->block);
			s->block = NULL;
			free(s->ordered_block);
			s->ordered_block = NULL;
			return status;
		}
		/*
		 * This is the hard part, see the comments there
		 * and at the start of this function.
		 */
		ls5000_shuffle_block(s);

		/* we've read a few lines, keep track */
		s->line += s->block_lines;
	}

	/*
	 * Now, if any code blocks above were executed or not, then
	 * at this point we have some data pending in our block buffers
	 * that we need to push to the frontend before we start reading
	 * from the scanner again.
	 */

	/* Calculate how much data we still have pending */
	*len = s->block_lines * colors * s->logical_width * 2 - s->block_read_pos;
	/* Too much? */
	if (*len > maxlen)
		*len = maxlen;

	/* Copy it to the frontend */
	memcpy(buf, s->ordered_block + s->block_read_pos, *len);
	s->block_read_pos += *len;

	/*
	 * If the whole block was read out start from zero again,
	 * this will execute some more code above again to read data
	 * from the scanner.
	 */
	if (s->block_read_pos == s->block_lines * colors * s->logical_width * 2)
		s->block_read_pos = 0;
	return SANE_STATUS_GOOD;
}

void sane_ls5000_cancel(SANE_Handle h)
{
	ls5000_t *s = (ls5000_t *) h;

	if (s->scan_stage == LS5000_SCAN_STAGE_ACQUIRE) {
		ls5000_issue_cmd(s, 0, LS5000_CMD_ABORT, 0, 0, 0, 0, 0);
		free(s->ir_data);
		s->ir_data = NULL;
		free(s->block);
		s->block = NULL;
		free(s->ordered_block);
		s->ordered_block = NULL;
	}

	s->block_read_pos = 0;
	s->line = 0;
	s->must_read_now = SANE_FALSE;
	s->scan_stage = LS5000_SCAN_STAGE_IDLE;
}

SANE_Status sane_ls5000_set_io_mode(SANE_Handle h, SANE_Bool m)
{
	(void)h; /* shut up compiler */

	if (m == SANE_FALSE)
		return SANE_STATUS_GOOD;
	else
		return SANE_STATUS_UNSUPPORTED;
}

SANE_Status sane_ls5000_get_select_fd(SANE_Handle h, SANE_Int * fd)
{
	ls5000_t *s = (ls5000_t *) h;
	(void)fd;
	(void)s;

	return SANE_STATUS_UNSUPPORTED;
}
