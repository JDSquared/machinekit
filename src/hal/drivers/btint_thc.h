
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

#define BTINT_THC_VERSION       "0.1"

#define BTINT_THC_MAX_BOARDS    1
#define BTINT_THC_HAL_NAME      "btint_thc"
#define BTINT_THC_MAGIC         ((u32)0x12345678)
#define BTINT_THC_GAINCNT       6

#define BTINT_PRINT(mname, fmt, args...)  rtapi_print("/%s: " fmt, mname, ## args)

#define BTINT_ERR(mname, fmt, args...)    rtapi_print_msg(RTAPI_MSG_ERR, "/%s: " fmt, mname, ## args)
#define BTINT_WARN(mname, fmt, args...)   rtapi_print_msg(RTAPI_MSG_WARN, "/%s: " fmt, mname, ## args)
#define BTINT_INFO(mname, fmt, args...)   rtapi_print_msg(RTAPI_MSG_INFO, "/%s: " fmt, mname, ## args)
#define BTINT_DBG(mname, fmt, args...)    rtapi_print_msg(RTAPI_MSG_DBG,  "/%s: " fmt, mname, ## args)

#define BTINT_THC_ADDR_MAGIC    0x0000
#define BTINT_THC_ADDR_CONTROL  0x0010
#define BTINT_THC_ADDR_G10INT   0x0100
#define BTINT_THC_ADDR_G20INT   0x0200
#define BTINT_THC_ADDR_G300INT  0x0300
#define BTINT_THC_ADDR_OUTS     0x0400
#define BTINT_THC_ADDR_INS      0x0500
#define BTINT_THC_ADDR_ADCVAL   0x0600
#define BTINT_THC_ADDR_ERRCNT   0x0700
#define BTINT_THC_ADDR_OVERFL   0x0800
#define BTINT_THC_ADDR_FRER     0x0804
#define BTINT_THC_ADDR_BCNT     0x0808
#define BTINT_THC_ADDR_CHKERR   0x080C

typedef struct {
	hal_float_t *val;
} btint_thc_gain_t;

typedef struct {
    hal_bit_t *arc_ok;
    hal_bit_t *torch_on;
    hal_bit_t *torch_on_man;
    hal_bit_t *ready;
    hal_bit_t *enable;
    hal_bit_t *lockout;
    hal_bit_t *active;
    hal_float_t *arc_volt;
    hal_float_t *req_arc_volt;
    hal_float_t *req_arc_volt_off;
    hal_float_t *z_pos_out;
    hal_float_t *z_pos_in;
    hal_float_t *z_pos_fb;
    hal_float_t *z_pos_fb_in;
    hal_float_t *cur_vel;
    hal_float_t *req_vel;
    hal_float_t *z_offset;
    hal_float_t *prev_err;
    hal_bit_t *dry_run;
    hal_u32_t *pkt_err_cnt;
    hal_u32_t *pkt_overfl_cnt;
    hal_u32_t *pkt_frerr_cnt;
    hal_u32_t *pkt_byte_cnt;
    hal_u32_t *pkt_chkerr_cnt;
    btint_thc_gain_t *gain10;    // Arrays of gains
    btint_thc_gain_t *gain20;
    btint_thc_gain_t *gain300;
    hal_bit_t *has_arc_ok;
    hal_u32_t *range_sel;
    hal_float_t *plasma_divisor;
    hal_float_t *vel_tol;
    hal_float_t *volt_tol;
    hal_float_t *correction_kp;
    hal_float_t *correction_kd;
} btint_thc_pins_t;

typedef struct {
    btint_thc_pins_t *pins;
    int uio_fd;
    const char *name;
    char halname[32];
    const char *uio_dev;
    void __iomem *base;
    unsigned long ctrl_base_addr;
    unsigned long data_base_addr;
} btint_thc_t;
