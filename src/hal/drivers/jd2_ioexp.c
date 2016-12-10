//
//    Copyright (C) 2016 Devin Hughes, JD Squared
//
//	  Derived from the combined work in hm2_soc_ol	  
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
#include "jd2_ioexp.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>

#define JD2_IOEXP_SPAN 65536

#define MAXUIOIDS  100
#define MAXNAMELEN 256

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devin Hughes");
MODULE_DESCRIPTION("Driver for JD2 ioexpander board");
MODULE_SUPPORTED_DEVICE("JD2 ioexpander");

static char *config[JD2_IOEXP_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config, JD2_IOEXP_MAX_BOARDS,
		      "config string for the ioexpander boards")

static int debug;
RTAPI_MP_INT(debug, "turn on extra debug output");

static char *name = "jd2_ioexp_axi1";
RTAPI_MP_STRING(name, "logical device name, default jd2_ioexp_axi1");

static int comp_id;
static jd2_ioexp_t board[JD2_IOEXP_MAX_BOARDS];
static int num_boards;

static int jd2_ioexp_mmap(jd2_ioexp_t *brd);
static int jd2_ioexp_update(void *void_jd2_ioexp, const hal_funct_args_t *fa);
static char *strlwr(char *str);

static int locate_uio_device(jd2_ioexp_t *brd, const char *name);

//
// These are the "low-level I/O" functions. Comms only support memory mapped
// access for now.
//
static int jd2_ioexp_read(jd2_ioexp_t *brd, u32 addr, void *buffer, int size) {
		memcpy(buffer, brd->base + addr, size);
    return 1;  // success
}

static int jd2_ioexp_write(jd2_ioexp_t *brd, u32 addr, void *buffer, int size) {
		memcpy(brd->base + addr, buffer, size);
    return 1;  // success
}

static int jd2_ioexp_mmap(jd2_ioexp_t *brd) {
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
			IOEXP_ERR(brd->name, "failed to map to /dev/uioX\n");
			return ret;
    }

    brd->uio_fd = open(brd->uio_dev, ( O_RDWR | O_SYNC ));
    if (brd->uio_fd < 0) {
        IOEXP_ERR(brd->name, "Could not open %s: %s",  brd->uio_dev, strerror(errno));
        return -errno;
    }
    virtual_base = mmap( NULL, JD2_IOEXP_SPAN, ( PROT_READ | PROT_WRITE ),
                         MAP_SHARED, brd->uio_fd, 0);

    if (virtual_base == MAP_FAILED) {
        IOEXP_ERR(brd->name, "mmap failed: %s", strerror(errno));
            close(brd->uio_fd);
        brd->uio_fd = -1;
        return -EINVAL;
    }

    if (debug) {
	    rtapi_print_hex_dump(RTAPI_MSG_INFO, RTAPI_DUMP_PREFIX_OFFSET,
			         16, 1, (const void *)virtual_base, 4096, 1,
			         "jd2_ioexp regs at %p:", virtual_base);
		}

    // Check for the magic number to make sure we have the right device
    u32 reg = *((u32 *)(virtual_base + JD2_IOEXP_ADDR_MAGIC));
    if (reg != JD2_IOEXP_MAGIC) {
        IOEXP_ERR(brd->name, "invalid magic number, got 0x%08X, expected 0x%08X\n", reg, JD2_IOEXP_MAGIC);
        close(brd->uio_fd);
        brd->uio_fd = -1;
        return -EINVAL;
    }

    IOEXP_DBG(brd->name, "jd2_ioexp magic number check OK");

    brd->base = (void *)virtual_base;

    return 0;
}

static int jd2_ioexp_register(jd2_ioexp_t *brd, const char *name)
{
    char fname[HAL_NAME_LEN + 1];
		int r = 0, i = 0;
		u32 temp = 0;

		memset(brd, 0,  sizeof(jd2_ioexp_t));

    brd->name = name;

    // Generate the board's hal name
    rtapi_snprintf(brd->halname, sizeof(brd->halname), "jd2_ioexp.%d", num_boards);
    strlwr(brd->halname);

	r = jd2_ioexp_mmap(brd);
	if (r) {
			IOEXP_ERR(brd->halname, "ioexp board failed memory mapping");
			return -EIO;
	}

    IOEXP_PRINT(brd->halname, "initialized JD2 IO Expander board on %s\n", brd->uio_dev);
    num_boards++;

	// Export hal pins
	brd->pins = hal_malloc(sizeof(jd2_ioexp_pins_t));
	memset(brd->pins, 0, sizeof(jd2_ioexp_pins_t));

	// Output pins
	brd->pins->outputs = hal_malloc(sizeof(jd2_iopin_t) * (JD2_IOEXP_NUM_OUTS - 1));
	memset(brd->pins->outputs, 0, sizeof(jd2_iopin_t) * (JD2_IOEXP_NUM_OUTS - 1));
	
	// Laser counts as one output
	for(i = 0; i < JD2_IOEXP_NUM_OUTS - 1; i++) {
		r += hal_pin_bit_newf(HAL_IN, &(brd->pins->outputs[i].val),
                    	comp_id, "%s.output.%d", brd->halname, i);
	}

	// Laser
	r += hal_pin_bit_newf(HAL_IN, &(brd->pins->laser_en),
					comp_id, "%s.laser-en", brd->halname);

	// Input pins
	brd->pins->inputs = hal_malloc(sizeof(jd2_iopin_t) * JD2_IOEXP_NUM_INS);
	memset(brd->pins->inputs, 0, sizeof(jd2_iopin_t) * JD2_IOEXP_NUM_INS);
	
	for(i = 0; i < JD2_IOEXP_NUM_INS; i++) {
		r += hal_pin_bit_newf(HAL_OUT, &(brd->pins->inputs[i].val),
                    	comp_id, "%s.input.%d", brd->halname, i);
	}

	// Status / Control
	r += hal_pin_bit_newf(HAL_OUT, &(brd->pins->ready),
					comp_id, "%s.ready", brd->halname);
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

	if (r < 0) {
        IOEXP_ERR(brd->halname, "error adding jd2_ioexpander pins, Aborting\n");
        goto fail0;
    }

	// Export the update function
    rtapi_snprintf(fname, sizeof(fname), "%s.update", brd->halname);

    hal_export_xfunct_args_t xfunct_args = {
	    .type = FS_XTHREADFUNC,
	    .funct.x = jd2_ioexp_update,
	    .arg = brd,
	    .uses_fp = 1,
	    .reentrant = 0,
	    .owner_id = comp_id
    };

    if ((r = hal_export_xfunctf(&xfunct_args, fname)) != 0) {
        IOEXP_ERR(brd->halname, "hal_export update failed - %d\n", r);
				goto fail0;
    }

	// Soft reset the controller
	temp = 1;
	jd2_ioexp_write(brd, JD2_IOEXP_ADDR_CONTROL, (void *)&temp, 4);

fail0:
	return r;
}


static int jd2_ioexp_munmap(jd2_ioexp_t *brd) {
    if (brd->base != NULL)
        munmap((void *) brd->base, JD2_IOEXP_SPAN);
    if (brd->uio_fd > -1) {
        close(brd->uio_fd);
        brd->uio_fd = -1;
    }
    return 1;
}


int rtapi_app_main(void) {
    int r = 0;

    IOEXP_PRINT("jd2_ioexp", "loading JD2 IO Expander driver version " JD2_IOEXP_VERSION "\n");

    comp_id = hal_init(JD2_IOEXP_HAL_NAME);
    if (comp_id < 0) return comp_id;

    jd2_ioexp_t *brd = &board[0];

    r = jd2_ioexp_register(brd, name);
    if (r != 0) {
        IOEXP_ERR(brd->halname, "error registering UIO driver\n");
        hal_exit(comp_id);
        return r;
    }

    if (num_boards == 0) {
        // no cards were detected
        IOEXP_PRINT(brd->halname, "error - no supported cards detected\n");
        hal_exit(comp_id);
        r = jd2_ioexp_munmap(brd);
        return r;
    }

	IOEXP_DBG(brd->name, "jd2_ioexp ready");

    hal_ready(comp_id);
    return 0;
}


void rtapi_app_exit(void)
{
    jd2_ioexp_t *brd = &board[0];
    jd2_ioexp_munmap(brd);
    IOEXP_PRINT(brd->halname, "jd2_ioexp driver unloaded\n");
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
static int locate_uio_device(jd2_ioexp_t *brd, const char *name)
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


static int jd2_ioexp_update(void *void_jd2_ioexp, const hal_funct_args_t *fa)
{
	jd2_ioexp_t *brd = void_jd2_ioexp;
	u32 temp = 0;
	int i = 0;

	// Update the status input - status tells us if we are connected
	jd2_ioexp_read(brd, JD2_IOEXP_ADDR_CONTROL, (void *)&temp, 4);
	temp = ((temp & 0x10000) > 0) ? 1 : 0;

	if(temp == 0) {
		*brd->pins->ready = 0;
		return 0;
	}
	else if(*brd->pins->ready == 0) {
		*brd->pins->ready = 1;
	}

	// The status regs
	jd2_ioexp_read(brd, JD2_IOEXP_ADDR_ERRCNT, (void *)brd->pins->pkt_err_cnt, 4);
	jd2_ioexp_read(brd, JD2_IOEXP_ADDR_OVERFL, (void *)brd->pins->pkt_overfl_cnt, 4);
	jd2_ioexp_read(brd, JD2_IOEXP_ADDR_FRER, (void *)brd->pins->pkt_frerr_cnt, 4);
	jd2_ioexp_read(brd, JD2_IOEXP_ADDR_BCNT, (void *)brd->pins->pkt_byte_cnt, 4);
	jd2_ioexp_read(brd, JD2_IOEXP_ADDR_CHKERR, (void *)brd->pins->pkt_chkerr_cnt, 4);

	// Check for handshake pin signaling new/ready-for-new data

	// The input pins
	jd2_ioexp_read(brd, JD2_IOEXP_ADDR_INS, (void *)&temp, 4);
	for(i = 0; i < JD2_IOEXP_NUM_INS; ++i) {
		*brd->pins->inputs[i].val = (temp >> i) & 0x1;
	}

	// Write the output pins
	temp = 0;
	if(*brd->pins->dry_run > 0) {
		// Dry run doesn't interact with the outputs directly
		// Force the outputs off temporarily, don't touch LASER
		temp |= *brd->pins->laser_en << JD2_IOEXP_LSR_IND;
		jd2_ioexp_write(brd, JD2_IOEXP_ADDR_OUTS, (void *)&temp, 4);
	}
	else {	
		// Loop over all outputs including laser
		for(i = 0; i < JD2_IOEXP_NUM_OUTS; ++i) {
			if(i == JD2_IOEXP_LSR_IND) {
				temp |= *brd->pins->laser_en << i;
			}
			else {
				temp |= *brd->pins->outputs[i].val << i;
			}
		}

		jd2_ioexp_write(brd, JD2_IOEXP_ADDR_OUTS, (void *)&temp, 4);	
	}

	return 0;
}
