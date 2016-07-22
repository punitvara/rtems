/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief I2C support API.
 */

/*
 * Copyright (c) 2012 Claas Ziemke. All rights reserved.
 *
 *  Claas Ziemke
 *  Kernerstrasse 11
 *  70182 Stuttgart
 *  Germany
 *  <claas.ziemke@gmx.net>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_BEAGLE_I2C_H
#define LIBBSP_ARM_BEAGLE_I2C_H

#include <rtems.h>
#include <dev/i2c/i2c.h>
#include <bsp.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define BBB_I2C_SYSCLK 48000000
#define BBB_I2C_INTERNAL_CLK 12000000
#define BBB_I2C_SPEED_CLK 100000

#define BBB_I2C_IRQ_ERROR \
  (AM335X_I2C_IRQSTATUS_NACK \
    | AM335X_I2C_IRQSTATUS_ROVR \
    | AM335X_I2C_IRQSTATUS_AERR \
    | AM335X_I2C_IRQSTATUS_AL \
    | AM335X_I2C_IRQSTATUS_ARDY \
    | AM335X_I2C_IRQSTATUS_RRDY \
    | I2C_IRQSTATUS_XRDY)

#define BBB_I2C_IRQ_USED \
  ( BBB_I2C_IRQ_ERROR \
    | AM335X_I2C_IRQSTATUS_AAS \
    | AM335X_I2C_IRQSTATUS_BF \
    | AM335X_I2C_IRQSTATUS_STC \
    | AM335X_I2C_IRQSTATUS_GC \
    | AM335X_I2C_IRQSTATUS_XDR \
    | AM335X_I2C_IRQSTATUS_RDR)

#define BBB_I2C_0_BUS_PATH "/dev/i2c-0"
#define BBB_I2C_1_BUS_PATH "/dev/i2c-1"
#define BBB_I2C_2_BUS_PATH "/dev/i2c-2"

#define BBB_I2C0_IRQ 70
#define BBB_I2C1_IRQ 71
#define BBB_I2C2_IRQ 30

#define MODE2 2
#define MODE3 3

typedef enum bbb_i2c_id{
  I2C0, 
  I2C1,
  I2C2,
  I2C_COUNT
}bbb_i2c_id_t;
/*
typedef struct i2c_regs
{
  unsigned short BBB_I2C_REVNB_LO;
  unsigned short BBB_I2C_REVNB_HI;
  unsigned short BBB_I2C_SYSC;
  unsigned short BBB_I2C_IRQSTATUS_RAW;
  unsigned short BBB_I2C_IRQSTATUS;
  unsigned short BBB_I2C_IRQENABLE_SET;
  unsigned short BBB_I2C_IRQENABLE_CLR;
  unsigned short BBB_I2C_WE;
  unsigned short BBB_I2C_DMARXENABLE_SET;
  unsigned short BBB_I2C_DMATXENABLE_SET;
  unsigned short BBB_I2C_DMARXENABLE_CLR;
  unsigned short BBB_I2C_DMATXENABLE_CLR;
  unsigned short BBB_I2C_DMARXWAKE_EN;
  unsigned short BBB_I2C_DMATXWAKE_EN;
  unsigned short BBB_I2C_SYSS;
  unsigned short BBB_I2C_BUF;
  unsigned short BBB_I2C_CNT;
  unsigned short BBB_I2C_DATA;
  unsigned short BBB_I2C_CON;
  unsigned short BBB_I2C_OA;
  unsigned short BBB_I2C_SA;
  unsigned short BBB_I2C_PSC;
  unsigned short BBB_I2C_SCLL;
  unsigned short BBB_I2C_SCLH;
  unsigned short BBB_I2C_SYSTEST;
  unsigned short BBB_I2C_BUFSTAT;
  unsigned short BBB_I2C_OA1;
  unsigned short BBB_I2C_OA2;
  unsigned short BBB_I2C_OA3;
  unsigned short BBB_I2C_ACTOA;
  unsigned short BBB_I2C_SBLOCK;
}bbb_i2c_regs;
*/
typedef struct i2c_regs
{
  uint32_t BBB_I2C_REVNB_LO; // 0h
  uint32_t BBB_I2C_REVNB_HI; //4h
  uint32_t dummy1[2];
  uint32_t BBB_I2C_SYSC;  // 10h =16
  uint32_t dummy2[4]; 
  uint32_t BBB_I2C_IRQSTATUS_RAW;  //24h =36
  uint32_t BBB_I2C_IRQSTATUS;  //28h =40
  uint32_t BBB_I2C_IRQENABLE_SET;  //2Ch =44
  uint32_t BBB_I2C_IRQENABLE_CLR; //30h =48
  uint32_t BBB_I2C_WE;  // 34h = 52
  uint32_t BBB_I2C_DMARXENABLE_SET; //38h = 56
  uint32_t BBB_I2C_DMATXENABLE_SET;  //3Ch = 60
  uint32_t BBB_I2C_DMARXENABLE_CLR;  //40h = 64
  uint32_t BBB_I2C_DMATXENABLE_CLR;  //44h = 68
  uint32_t BBB_I2C_DMARXWAKE_EN;  //48h = 72
  uint32_t BBB_I2C_DMATXWAKE_EN;  //4Ch =76
  uint32_t dummy3[16];
  uint32_t BBB_I2C_SYSS;  // 90h =144
  uint32_t BBB_I2C_BUF;  // 94h =148
  uint32_t BBB_I2C_CNT;  // 98h =152
  uint32_t BBB_I2C_DATA; //9Ch =156
  uint32_t dummy4;
  uint32_t BBB_I2C_CON;  // A4h = 164 
  uint32_t BBB_I2C_OA;  //A8h = 168
  uint32_t BBB_I2C_SA;  //ACh = 172
  uint32_t BBB_I2C_PSC;  //B0h = 176
  uint32_t BBB_I2C_SCLL;  //B4h = 180
  uint32_t BBB_I2C_SCLH;  //B8h = 184
  uint32_t BBB_I2C_SYSTEST;  //BCh = 188
  uint32_t BBB_I2C_BUFSTAT;  //C0h 192
  uint32_t BBB_I2C_OA1;  //C4h 196
  uint32_t BBB_I2C_OA2;  //C8h 200
  uint32_t BBB_I2C_OA3;  //CCh 204
  uint32_t BBB_I2C_ACTOA;  //D0h 208
  uint32_t BBB_I2C_SBLOCK;  //D4h 212
}bbb_i2c_regs;

typedef struct bbb_i2c_bus{
  i2c_bus base;
  volatile bbb_i2c_regs *regs;
  //uint32_t i2c_base_address;
  i2c_msg *msgs;
  rtems_id task_id;
  rtems_vector_number irq;
  bbb_i2c_id_t i2c_bus_id;
  uint32_t input_clock;
}bbb_i2c_bus;
/*
bbb_i2c_bus devices[I2C_COUNT] =
{
{
.base = ,
.regs = AM335X_I2C0_BASE, 
.msgs = ,
.task_id = ,
.irq = BBB_I2C0_IRQ,
.input_clock = I2C_BUS_CLOCK_DEFAULT
},
{
.base = ,
.regs = AM335X_I2C1_BASE,
.msgs = ,
.task_id = ,
.irq = BBB_I2C1_IRQ,
.input_clock = I2C_BUS_CLOCK_DEFAULT
},
{
.base = ,
.regs = AM335X_I2C2_BASE,
.msgs = ,
.task_id = ,
.irq = BBB_I2C2_IRQ,
.input_clock = I2C_BUS_CLOCK_DEFAULT
}
} 
*/
/*
void deviceInit(bbb_i2c_id_t i2c_id)
{
bbb_i2c_bus *bus = &devices[i2c_id];
bus->regs->BBB_I2C_IRQSTATUS_RAW = value; 
}
*/
static inline int bbb_register_i2c_0(void)
{
  return am335x_i2c_bus_register(
    BBB_I2C_0_BUS_PATH,
    AM335X_I2C0_BASE,
    I2C_BUS_CLOCK_DEFAULT,
    BBB_I2C0_IRQ
  );
}

static inline int bbb_register_i2c_1(void)
{
  return am335x_i2c_bus_register(
    BBB_I2C_1_BUS_PATH,
    AM335X_I2C1_BASE,
    I2C_BUS_CLOCK_DEFAULT,
    BBB_I2C1_IRQ
  );
}

static inline int bbb_register_i2c_2(void)
{
  return am335x_i2c_bus_register(
    BBB_I2C_2_BUS_PATH,
    AM335X_I2C2_BASE,
    I2C_BUS_CLOCK_DEFAULT,
    BBB_I2C2_IRQ
  );
}

/* I2C Configuration Register (I2C_CON): */

#define I2C_CON_EN  (1 << 15)  /* I2C module enable */
#define I2C_CON_BE  (1 << 14)  /* Big endian mode */
#define I2C_CON_STB (1 << 11)  /* Start byte mode (master mode only) */
#define I2C_CON_MST (1 << 10)  /* Master/slave mode */
#define I2C_CON_TRX (1 << 9)   /* Transmitter/receiver mode */
           /* (master mode only) */
#define I2C_CON_XA  (1 << 8)   /* Expand address */
#define I2C_CON_STP (1 << 1)   /* Stop condition (master mode only) */
#define I2C_CON_STT (1 << 0)   /* Start condition (master mode only) */

/* I2C Status Register (I2C_STAT): */

#define I2C_STAT_SBD  (1 << 15) /* Single byte data */
#define I2C_STAT_BB (1 << 12) /* Bus busy */
#define I2C_STAT_ROVR (1 << 11) /* Receive overrun */
#define I2C_STAT_XUDF (1 << 10) /* Transmit underflow */
#define I2C_STAT_AAS  (1 << 9)  /* Address as slave */
#define I2C_STAT_GC (1 << 5)
#define I2C_STAT_XRDY (1 << 4)  /* Transmit data ready */
#define I2C_STAT_RRDY (1 << 3)  /* Receive data ready */
#define I2C_STAT_ARDY (1 << 2)  /* Register access ready */
#define I2C_STAT_NACK (1 << 1)  /* No acknowledgment interrupt enable */
#define I2C_STAT_AL (1 << 0)  /* Arbitration lost interrupt enable */

/* I2C Interrupt Enable Register (I2C_IE): */
#define I2C_IE_GC_IE  (1 << 5)
#define I2C_IE_XRDY_IE  (1 << 4) /* Transmit data ready interrupt enable */
#define I2C_IE_RRDY_IE  (1 << 3) /* Receive data ready interrupt enable */
#define I2C_IE_ARDY_IE  (1 << 2) /* Register access ready interrupt enable */
#define I2C_IE_NACK_IE  (1 << 1) /* No acknowledgment interrupt enable */
#define I2C_IE_AL_IE  (1 << 0) /* Arbitration lost interrupt enable */
/*
 * The equation for the low and high time is
 * tlow = scll + scll_trim = (sampling clock * tlow_duty) / speed
 * thigh = sclh + sclh_trim = (sampling clock * (1 - tlow_duty)) / speed
 *
 * If the duty cycle is 50%
 *
 * tlow = scll + scll_trim = sampling clock / (2 * speed)
 * thigh = sclh + sclh_trim = sampling clock / (2 * speed)
 *
 * In TRM
 * scll_trim = 7
 * sclh_trim = 5
 *
 * The linux 2.6.30 kernel uses
 * scll_trim = 6
 * sclh_trim = 6
 *
 * These are the trim values for standard and fast speed
 */
#ifndef I2C_FASTSPEED_SCLL_TRIM
#define I2C_FASTSPEED_SCLL_TRIM   6
#endif
#ifndef I2C_FASTSPEED_SCLH_TRIM
#define I2C_FASTSPEED_SCLH_TRIM   6
#endif

/* These are the trim values for high speed */
#ifndef I2C_HIGHSPEED_PHASE_ONE_SCLL_TRIM
#define I2C_HIGHSPEED_PHASE_ONE_SCLL_TRIM I2C_FASTSPEED_SCLL_TRIM
#endif
#ifndef I2C_HIGHSPEED_PHASE_ONE_SCLH_TRIM
#define I2C_HIGHSPEED_PHASE_ONE_SCLH_TRIM I2C_FASTSPEED_SCLH_TRIM
#endif
#ifndef I2C_HIGHSPEED_PHASE_TWO_SCLL_TRIM
#define I2C_HIGHSPEED_PHASE_TWO_SCLL_TRIM I2C_FASTSPEED_SCLL_TRIM
#endif
#ifndef I2C_HIGHSPEED_PHASE_TWO_SCLH_TRIM
#define I2C_HIGHSPEED_PHASE_TWO_SCLH_TRIM I2C_FASTSPEED_SCLH_TRIM
#endif

#define OMAP_I2C_STANDARD 100000
#define OMAP_I2C_FAST_MODE  400000
#define OMAP_I2C_HIGH_SPEED 3400000


/* Use the reference value of 96MHz if not explicitly set by the board */
#ifndef I2C_IP_CLK
#define I2C_IP_CLK    SYSTEM_CLOCK_96
#endif

/*
 * The reference minimum clock for high speed is 19.2MHz.
 * The linux 2.6.30 kernel uses this value.
 * The reference minimum clock for fast mode is 9.6MHz
 * The reference minimum clock for standard mode is 4MHz
 * In TRM, the value of 12MHz is used.
 */
#ifndef I2C_INTERNAL_SAMPLING_CLK
#define I2C_INTERNAL_SAMPLING_CLK 19200000
#endif

#define I2C_PSC_MAX   0x0f
#define I2C_PSC_MIN   0x00


#define DISP_LINE_LEN 128
#define I2C_TIMEOUT 1000

#define I2C_BUS_MAX 3

#define I2C_BASE1         (OMAP34XX_CORE_L4_IO_BASE + 0x070000)

#define I2C_DEFAULT_BASE      I2C_BASE1

#define I2C_SYSS_RDONE            (1 << 0)  /* Internel reset monitoring */

#define CONFIG_SYS_I2C_SPEED    100000
#define CONFIG_SYS_I2C_SLAVE    1


bool am335x_i2c_pinmux(bbb_i2c_bus *bus);

void am335x_i2c1_i2c2_module_clk_config(bbb_i2c_bus *bus);

void am335x_i2c_reset(bbb_i2c_bus *bus);

int am335x_i2c_set_clock(i2c_bus *base, unsigned long clock);

int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq
);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_I2C_H */
