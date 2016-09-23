//
//    Copyright (C) 2016 Devin Hughes, JD Squared
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//---------------------------------------------------------------------------//

#include "config.h"

#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"
#include "rtapi_hexdump.h"
#include "rtapi_compat.h"
#include "rtapi_io.h"
#include "hal.h"
#include "hal_priv.h"
#include "hal/lib/config_module.h"
#include "btint_thc.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

#define BTINT_THC_SPAN 65536

#define MAXUIOIDS  100
#define MAXNAMELEN 256

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devin Hughes");
MODULE_DESCRIPTION("Driver for JD2 torch interface board");
MODULE_SUPPORTED_DEVICE("JD2 btint");

static char *config[BTINT_THC_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, BTINT_THC_MAX_BOARDS,
		      "config string for the btint boards")

static int debug;
RTAPI_MP_INT(debug, "turn on extra debug output");

static char *name = "btint-axi0";
RTAPI_MP_STRING(name, "logical device name, default btint-axi0");

static int comp_id;
static btint_thc_t board[BTINT_THC_MAX_BOARDS];
static int num_boards;

static int btint_thc_mmap(btint_thc_t *brd);
static int btint_thc_update(void *void_btint_thc, const hal_funct_args_t *fa);
static char *strlwr(char *str);

static int locate_uio_device(btint_thc_t *brd, const char *name);

//
// These are the "low-level I/O" functions. Comms only support memory mapped
// access for now.
//
static int btint_thc_read(btint_thc_t *brd, u32 addr, void *buffer, int size) {
		memcpy(buffer, brd->base + addr, size);
    return 1;  // success
}

static int btint_thc_write(btint_thc_t *brd, u32 addr, void *buffer, int size) {
		memcpy(brd->base + addr, buffer, size);
    return 1;  // success
}

static int btint_thc_mmap(btint_thc_t *brd) {
    volatile void *virtual_base;

    // The device should already be available since another module
		// must program the fpga before this one can be loaded, but just
		// in case...
    int retries = 10;
    int ret;
    while(retries > 0) {
			ret = locate_uio_device(brd, brd->name);
			if (ret == 0)
			    break;
      retries--;
      usleep(200000);
    }

    if (ret || (brd->uio_dev == NULL)) {
			BTINT_ERR(brd->name, "failed to map to /dev/uioX\n");
			return ret;
    }

    brd->uio_fd = open(brd->uio_dev, ( O_RDWR | O_SYNC ));
    if (brd->uio_fd < 0) {
        BTINT_ERR(brd->name, "Could not open %s: %s",  brd->uio_dev, strerror(errno));
        return -errno;
    }
    virtual_base = mmap( NULL, BTINT_THC_SPAN, ( PROT_READ | PROT_WRITE ),
                         MAP_SHARED, brd->uio_fd, 0);

    if (virtual_base == MAP_FAILED) {
        BTINT_ERR(brd->name, "mmap failed: %s", strerror(errno));
            close(brd->uio_fd);
        brd->uio_fd = -1;
        return -EINVAL;
    }

    if (debug) {
	    rtapi_print_hex_dump(RTAPI_MSG_INFO, RTAPI_DUMP_PREFIX_OFFSET,
			         16, 1, (const void *)virtual_base, 4096, 1,
			         "btint_thc regs at %p:", virtual_base);
		}

    // Check for the magic number to make sure we have the right device
    u32 reg = *((u32 *)(virtual_base + BTINT_THC_ADDR_MAGIC));
    if (reg != BTINT_THC_MAGIC) {
        BTINT_ERR(brd->name, "invalid magic number, got 0x%08X, expected 0x%08X\n", reg, BTINT_THC_MAGIC);
        close(brd->uio_fd);
        brd->uio_fd = -1;
        return -EINVAL;
    }

    BTINT_DBG(brd->name, "btint_thc magic number check OK");

    brd->base = (void *)virtual_base;

    return 0;
}

static int btint_thc_register(btint_thc_t *brd, const char *name)
{
    char fname[HAL_NAME_LEN + 1];
		int r = 0;
		u32 temp = 0;

		memset(brd, 0,  sizeof(btint_thc_t));

    brd->name = name;

    // Generate the board's hal name
    rtapi_snprintf(brd->halname, sizeof(brd->halname), "btint_thc.%d", num_boards);
    strlwr(brd->halname);

	r = btint_thc_mmap(brd);
	if (r) {
			BTINT_ERR(brd->halname, "btint board failed memory mapping");
			return -EIO;
	}

    BTINT_PRINT(brd->halname, "initialized JD2 BTINT board on %s\n", brd->uio_dev);
    num_boards++;

	// Export hal pins
	brd->pins = hal_malloc(sizeof(btint_thc_pins_t));
	memset(brd->pins, 0, sizeof(btint_thc_pins_t));

	// Gains
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain10int),
        comp_id, "%s.gain10int", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain10x),
        comp_id, "%s.gain10x", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain10x2),
					comp_id, "%s.gain10x2", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain10x3),
        comp_id, "%s.gain10x3", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain20int),
					comp_id, "%s.gain20int", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain20x),
        comp_id, "%s.gain20x", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain20x2),
					comp_id, "%s.gain20x2", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain20x3),
        comp_id, "%s.gain20x3", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain300int),
					comp_id, "%s.gain300int", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain300x),
					comp_id, "%s.gain300x", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain300x2),
					comp_id, "%s.gain300x2", brd->halname);
	r = hal_pin_float_newf(HAL_OUT, &(brd->pins->gain300x3),
					comp_id, "%s.gain300x3", brd->halname);

	// IO Pins
	r += hal_pin_bit_newf(HAL_OUT, &(brd->pins->arc_ok),
					comp_id, "%s.arc-ok", brd->halname);
	r += hal_pin_bit_newf(HAL_IN, &(brd->pins->torch_on),
					comp_id, "%s.torch-on", brd->halname);

	// THC
	r += hal_pin_float_newf(HAL_OUT, &(brd->pins->arc_volt),
					comp_id, "%s.arc-volt", brd->halname);
	r += hal_pin_float_newf(HAL_IN, &(brd->pins->req_arc_volt),
					comp_id, "%s.req-arc-volt", brd->halname);
	r += hal_pin_float_newf(HAL_IN, &(brd->pins->req_arc_volt_off),
					comp_id, "%s.req-arc-volt-off", brd->halname);
	r += hal_pin_float_newf(HAL_IN, &(brd->pins->cur_vel),
					comp_id, "%s.cur-vel", brd->halname);
	r += hal_pin_float_newf(HAL_IN, &(brd->pins->req_vel),
					comp_id, "%s.req-vel", brd->halname);
	r += hal_pin_float_newf(HAL_IN, &(brd->pins->z_pos_in),
					comp_id, "%s.z-pos-in", brd->halname);
	r += hal_pin_float_newf(HAL_OUT, &(brd->pins->z_pos_out),
					comp_id, "%s.z-pos-out", brd->halname);
	r += hal_pin_float_newf(HAL_OUT, &(brd->pins->z_pos_fb),
					comp_id, "%s.z-pos-fb", brd->halname);
	r += hal_pin_float_newf(HAL_IN, &(brd->pins->z_pos_fb_in),
					comp_id, "%s.z-pos-fb-in", brd->halname);
	r += hal_pin_float_newf(HAL_OUT, &(brd->pins->z_offset),
					comp_id, "%s.z-offset", brd->halname);
	r += hal_pin_float_newf(HAL_OUT, &(brd->pins->prev_err),
					comp_id, "%s.prev-err", brd->halname);

	// Status / Control
	r += hal_pin_bit_newf(HAL_OUT, &(brd->pins->ready),
					comp_id, "%s.ready", brd->halname);
	r += hal_pin_bit_newf(HAL_OUT, &(brd->pins->active),
					comp_id, "%s.active", brd->halname);
	r += hal_pin_bit_newf(HAL_IN, &(brd->pins->enable),
					comp_id, "%s.enable", brd->halname);
	r += hal_pin_u32_newf(HAL_OUT, &(brd->pins->pkt_err_cnt),
					comp_id, "%s.pkt-err-cnt", brd->halname);
	r += hal_pin_u32_newf(HAL_OUT, &(brd->pins->pkt_overfl_cnt),
					comp_id, "%s.pkt-overfl-cnt", brd->halname);
	r += hal_pin_u32_newf(HAL_OUT, &(brd->pins->pkt_frerr_cnt),
					comp_id, "%s.pkt-frerr-cnt", brd->halname);
	r += hal_pin_u32_newf(HAL_OUT, &(brd->pins->pkt_byte_cnt),
					comp_id, "%s.pkt-byte-cnt", brd->halname);
	r += hal_pin_u32_newf(HAL_OUT, &(brd->pins->pkt_chkerr_cnt),
					comp_id, "%s.pkt-chkerr-cnt", brd->halname);
	r += hal_pin_bit_newf(HAL_IN, &(brd->pins->dry_run),
					comp_id, "%s.dry-run", brd->halname);

	r += hal_pin_float_newf(HAL_RW, &(brd->pins->vel_tol),
					comp_id, "%s.vel-tol", brd->halname);
	r += hal_pin_float_newf(HAL_RW, &(brd->pins->volt_tol),
					comp_id, "%s.volt-tol", brd->halname);
	r += hal_pin_float_newf(HAL_RW, &(brd->pins->correction_kp),
					comp_id, "%s.correction-kp", brd->halname);
	r += hal_pin_float_newf(HAL_RW, &(brd->pins->correction_kd),
					comp_id, "%s.correction-kd", brd->halname);
	r += hal_pin_u32_newf(HAL_RW, &(brd->pins->range_sel),
					comp_id, "%s.range-sel", brd->halname);
	r += hal_pin_float_newf(HAL_RW, &(brd->pins->plasma_divisor),
					comp_id, "%s.plasma-divisor", brd->halname);
	r += hal_pin_bit_newf(HAL_RW, &(brd->pins->has_arc_ok),
					comp_id, "%s.has-arc-ok", brd->halname);


	if (r < 0) {
        BTINT_ERR(brd->halname, "error adding btint_thc pins, Aborting\n");
        goto fail0;
    }

		// Export the update function
    rtapi_snprintf(fname, sizeof(fname), "%s.update", brd->halname);

    hal_export_xfunct_args_t xfunct_args = {
	    .type = FS_XTHREADFUNC,
	    .funct.x = btint_thc_update,
	    .arg = brd,
	    .uses_fp = 1,
	    .reentrant = 0,
	    .owner_id = comp_id
    };

    if ((r = hal_export_xfunctf(&xfunct_args, fname)) != 0) {
        BTINT_ERR(brd->halname, "hal_export update failed - %d\n", r);
				goto fail0;
    }

		// Soft reset the controller
		temp = 1;
		btint_thc_write(brd, BTINT_THC_ADDR_CONTROL, (void *)&temp, 4);
fail0:
	return r;
}


static int btint_thc_munmap(btint_thc_t *brd) {
    if (brd->base != NULL)
        munmap((void *) brd->base, BTINT_THC_SPAN);
    if (brd->uio_fd > -1) {
        close(brd->uio_fd);
        brd->uio_fd = -1;
    }
    return 1;
}


int rtapi_app_main(void) {
    int r = 0;

    BTINT_PRINT("btint_thc", "loading JD2 BTINT THC driver version " BTINT_THC_VERSION "\n");

    comp_id = hal_init(BTINT_THC_HAL_NAME);
    if (comp_id < 0) return comp_id;

    btint_thc_t *brd = &board[0];

    r = btint_thc_register(brd, name);
    if (r != 0) {
        BTINT_ERR(brd->halname, "error registering UIO driver\n");
        hal_exit(comp_id);
        return r;
    }

    if (num_boards == 0) {
        // no cards were detected
        BTINT_PRINT(brd->halname, "error - no supported cards detected\n");
        hal_exit(comp_id);
        r = btint_thc_munmap(brd);
        return r;
    }

    hal_ready(comp_id);
    return 0;
}


void rtapi_app_exit(void)
{
    btint_thc_t *brd = &board[0];
    btint_thc_munmap(brd);
    BTINT_PRINT(brd->halname, "btint_thc driver unloaded\n");
    hal_exit(comp_id);
}


static char *strlwr(char *str)
{
  unsigned char *p = (unsigned char *)str;
  while (*p) {
     *p = tolower(*p);
      p++;
  }
  return str;
}


// fills in brd->uio_dev based on name
static int locate_uio_device(btint_thc_t *brd, const char *name)
{
    char buf[MAXNAMELEN];
    int uio_id;

    for (uio_id = 0; uio_id < MAXUIOIDS; uio_id++) {
			if (rtapi_fs_read(buf, MAXNAMELEN, "/sys/class/uio/uio%d/name", uio_id) < 0)
			    continue;
			if (strncmp(name, buf, strlen(name)) == 0)
			    break;
    }
    if (uio_id >= MAXUIOIDS)
			return -1;

    rtapi_snprintf(buf, sizeof(buf), "/dev/uio%d", uio_id);
    brd->uio_dev = strdup(buf);
    return 0;
}


static int btint_thc_update(void *void_btint_thc, const hal_funct_args_t *fa)
{
	btint_thc_t *brd = void_btint_thc;
	u32 temp = 0;
	float tempf = 0.0f;
	hal_float_t voltd = 0.0;
	hal_float_t velc;
	hal_float_t gainint = 0.0f;
	hal_float_t gainx = 0.0f;
	hal_float_t gainx2 = 0.0f;
	hal_float_t gainx3 = 0.0f;
	hal_float_t reqv;

	// Lie to the control to avoid feedback errors
	*brd->pins->z_pos_fb = *brd->pins->z_pos_in;

	if(*brd->pins->dry_run > 0) {
		// Force the torch off, but don't change the hal pin
		temp = 0;
		btint_thc_write(brd, BTINT_THC_ADDR_OUTS, (void *)&temp, 4);

		// A dry run lies about the arc_ok
		*brd->pins->arc_ok = 1;
		*brd->pins->z_pos_out = *brd->pins->z_pos_in;
		return 0;
	}

	switch(*brd->pins->range_sel)
	{
		case 10:
			gainint = *brd->pins->gain10int;
			gainx = *brd->pins->gain10x;
			gainx2 = *brd->pins->gain10x2;
			gainx3 = *brd->pins->gain10x3;
			break;
		case 20:
			gainint = *brd->pins->gain20int;
			gainx = *brd->pins->gain20x;
			gainx2 = *brd->pins->gain20x2;
			gainx3 = *brd->pins->gain20x3;
			break;
		case 300:
			gainint = *brd->pins->gain300int;
			gainx = *brd->pins->gain300x;
			gainx2 = *brd->pins->gain300x2;
			gainx3 = *brd->pins->gain300x3;
			break;
		default:
			gainint = 0.0f;
			gainx = 0.0f;
			gainx2 = 0.0f;
			gainx3 = 0.0f;
	}

	// Update the status input
	btint_thc_read(brd, BTINT_THC_ADDR_CONTROL, (void *)&temp, 4);
	temp = ((temp & 0x10000) > 0) ? 1 : 0;

	if(temp == 0) {
		*brd->pins->ready = 0;
		*brd->pins->z_pos_out = *brd->pins->z_pos_in;
	    *brd->pins->arc_ok = 0;
		return 0;
	}
	else if(*brd->pins->ready == 0) {
		// Read the gains
		btint_thc_read(brd, BTINT_THC_ADDR_G10INT, (void *)&tempf, 4);
		*brd->pins->gain10int = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G10X, (void *)&tempf, 4);
		*brd->pins->gain10x = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G10X2, (void *)&tempf, 4);
		*brd->pins->gain10x2 = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G10X3, (void *)&tempf, 4);
		*brd->pins->gain10x3 = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G20INT, (void *)&tempf, 4);
		*brd->pins->gain20int = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G20X, (void *)&tempf, 4);
		*brd->pins->gain20x = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G20X2, (void *)&tempf, 4);
		*brd->pins->gain20x2 = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G20X3, (void *)&tempf, 4);
		*brd->pins->gain20x3 = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G300INT, (void *)&tempf, 4);
		*brd->pins->gain300int = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G300X, (void *)&tempf, 4);
		*brd->pins->gain300x = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G300X2, (void *)&tempf, 4);
		*brd->pins->gain300x2 = tempf;
		btint_thc_read(brd, BTINT_THC_ADDR_G300X3, (void *)&tempf, 4);
		*brd->pins->gain300x3 = tempf;
		*brd->pins->ready = 1;
	}

	// The input pins
	if(*brd->pins->has_arc_ok > 0) {
		btint_thc_read(brd, BTINT_THC_ADDR_INS, (void *)&temp, 4);
		*brd->pins->arc_ok = ((temp & 0x2) > 0) ? 1 : 0;
	}
	else {
		*brd->pins->arc_ok = 1;
	}

	// The arc voltage is found from a 3rd order fit based on the adc selected range,
	// and the divisor used on the plasma unit
	btint_thc_read(brd, BTINT_THC_ADDR_ADCVAL, &temp, 4);
	voltd = (hal_float_t)temp;
	voltd = (voltd*voltd*voltd*gainx3 + voltd*voltd*gainx2 + voltd*gainx + gainint) * (*brd->pins->plasma_divisor);
	*brd->pins->arc_volt = (voltd > 0.0f) ? voltd : 0.0f;

	// The status regs
	btint_thc_read(brd, BTINT_THC_ADDR_ERRCNT, (void *)brd->pins->pkt_err_cnt, 4);
	btint_thc_read(brd, BTINT_THC_ADDR_OVERFL, (void *)brd->pins->pkt_overfl_cnt, 4);
	btint_thc_read(brd, BTINT_THC_ADDR_FRER, (void *)brd->pins->pkt_frerr_cnt, 4);
	btint_thc_read(brd, BTINT_THC_ADDR_BCNT, (void *)brd->pins->pkt_byte_cnt, 4);
	btint_thc_read(brd, BTINT_THC_ADDR_CHKERR, (void *)brd->pins->pkt_chkerr_cnt, 4);

	// Write the output pins
	temp = (*brd->pins->torch_on > 0) ? 1 : 0;
	btint_thc_write(brd, BTINT_THC_ADDR_OUTS, (void *)&temp, 4);

	// Do the THC
	reqv = *brd->pins->req_arc_volt + *brd->pins->req_arc_volt_off;

	// correction_kp is treated as an upper limit that we are allowed to correct
	// per period. We scale this down based on the differential from the previous
	// cycle's error. Cap error calculation so the Z head can keep up..
	velc = *brd->pins->correction_kp * 2;

	// If we are enabled, and the torch is on, we can calculate a valid shift
	if(*brd->pins->enable > 0 && (*brd->pins->torch_on > 0)) {
		hal_float_t minv = *brd->pins->req_vel * (*brd->pins->vel_tol);
		int velok = (*brd->pins->cur_vel > 0.0f && *brd->pins->cur_vel >= minv) ? 1 : 0;
		hal_float_t pdif = *brd->pins->z_pos_out - *brd->pins->z_pos_fb_in;
		hal_float_t err = 0.0;
		hal_float_t errd = 0.0;
		pdif = (pdif < 0.0) ? -1.0 * pdif : pdif;

		if((*brd->pins->arc_ok > 0) && (velok > 0)) {
			if(pdif < velc) {
				if(((reqv + *brd->pins->volt_tol) < *brd->pins->arc_volt) ||
			     ((reqv - *brd->pins->volt_tol) > *brd->pins->arc_volt)) {
						*brd->pins->active = 1;
						 // This is a dirty PD control...
						err = (reqv - *brd->pins->arc_volt) * 0.1;
						errd = err - *brd->pins->prev_err;
						pdif = *brd->pins->correction_kp * err + *brd->pins->correction_kd * errd;

						// Clamp
						if(pdif > *brd->pins->correction_kp)
							pdif = *brd->pins->correction_kp;
						else if(pdif < -1.0 * (*brd->pins->correction_kp))
							pdif = -1.0 * (*brd->pins->correction_kp);

						*brd->pins->z_offset += pdif;
						*brd->pins->prev_err = err;
				}
			}
			else {
				*brd->pins->active = 0;
			}
		}
	}
	else { // if the torch is off, or we aren't enabled, gracefully return to the controllers z height
		*brd->pins->active = 0;
		if(*brd->pins->z_offset > 0.0f) {
			*brd->pins->z_offset -= *brd->pins->correction_kp;
			// Clamp
			if(*brd->pins->z_offset < 0.0f)
				*brd->pins->z_offset = 0.0f;
		}
		else if (*brd->pins->z_offset < 0.0f){
			*brd->pins->z_offset += *brd->pins->correction_kp;
			// Clamp
			if(*brd->pins->z_offset > 0.0f)
				*brd->pins->z_offset = 0.0f;
		}
	}

	*brd->pins->z_pos_out = *brd->pins->z_pos_in + *brd->pins->z_offset;

  return 0;
}
