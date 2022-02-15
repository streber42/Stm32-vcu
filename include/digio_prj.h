#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(HV_req,       GPIOD, GPIO3,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(start_in,    GPIOD, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(brake_in,    GPIOE, GPIO8,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(fwd_in,      GPIOE, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(rev_in,      GPIOA, GPIO6,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(dcsw_out,    GPIOE, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,     GPIOC, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led2_out,     GPIOC, GPIO6, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out1,     GPIOD, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out2,     GPIOD, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out3,     GPIOD, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(sw_mode0,     GPIOD, GPIO9, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(sw_mode1,     GPIOD, GPIO8, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(lin_wake,     GPIOA, GPIO5, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(lin_nslp,     GPIOA, GPIO7, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,     GPIOB, GPIO0,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(inv_out,     GPIOD, GPIO10,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL1_out,     GPIOC, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL2_out,     GPIOC, GPIO8,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SP_out,     GPIOD, GPIO12,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gear1_in,   GPIOE, GPIO3,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gear2_in,   GPIOE, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gear3_in,   GPIOE, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(req_out,     GPIOE, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pot1_cs,     GPIOB, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pot2_cs,     GPIOB, GPIO13,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(mcp_cs,     GPIOB, GPIO12,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(mcp_sby,     GPIOE, GPIO14,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(t15_digi,     GPIOD, GPIO6,  PinMode::INPUT_FLT)      \
    DIG_IO_ENTRY(gp_12Vin,     GPIOD, GPIO4,  PinMode::INPUT_FLT)      \



#endif // PinMode_PRJ_H_INCLUDED
