
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
//

#define JD2_IOEXP_VERSION       "0.1"

#define JD2_IOEXP_MAX_BOARDS    1
#define JD2_IOEXP_HAL_NAME      "jd2_ioexp"
#define JD2_IOEXP_MAGIC         ((u32)0x12345678)

#define IOEXP_PRINT(mname, fmt, args...)  rtapi_print("/%s: " fmt, mname, ## args)

#define IOEXP_ERR(mname, fmt, args...)    rtapi_print_msg(RTAPI_MSG_ERR, "/%s: " fmt, mname, ## args)
#define IOEXP_WARN(mname, fmt, args...)   rtapi_print_msg(RTAPI_MSG_WARN, "/%s: " fmt, mname, ## args)
#define IOEXP_INFO(mname, fmt, args...)   rtapi_print_msg(RTAPI_MSG_INFO, "/%s: " fmt, mname, ## args)
#define IOEXP_DBG(mname, fmt, args...)    rtapi_print_msg(RTAPI_MSG_DBG,  "/%s: " fmt, mname, ## args)

#define JD2_IOEXP_ADDR_MAGIC    0x0000
#define JD2_IOEXP_ADDR_CONTROL  0x0010
#define JD2_IOEXP_ADDR_OUTS     0x0400
#define JD2_IOEXP_ADDR_INS      0x0500
#define JD2_IOEXP_ADDR_ERRCNT   0x0700
#define JD2_IOEXP_ADDR_OVERFL   0x0800
#define JD2_IOEXP_ADDR_FRER     0x0804
#define JD2_IOEXP_ADDR_BCNT     0x0808
#define JD2_IOEXP_ADDR_CHKERR   0x080C
#define JD2_IOEXP_NUM_OUTS		9		// total outputs including laser indicator
#define JD2_IOEXP_NUM_INS		2
#define JD2_IOEXP_LSR_IND		8

typedef struct {
    hal_bit_t *ready;
	hal_bit_t *outputs;
	hal_bit_t *inputs;
	hal_bit_t *laser_en;
	hal_bit_t *dry_run;
    hal_u32_t *pkt_err_cnt;
    hal_u32_t *pkt_overfl_cnt;
    hal_u32_t *pkt_frerr_cnt;
    hal_u32_t *pkt_byte_cnt;
    hal_u32_t *pkt_chkerr_cnt;
} jd2_ioexp_pins_t;

typedef struct {
    JD2_IOEXP_pins_t *pins;
    int uio_fd;
    const char *name;
    char halname[32];
    const char *uio_dev;
    void __iomem *base;
    unsigned long ctrl_base_addr;
    unsigned long data_base_addr;
} jd2_ioexp_t;
