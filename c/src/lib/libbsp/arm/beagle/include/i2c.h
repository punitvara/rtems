/**
 
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

struct i2c {
  unsigned short rev;   /* 0x00 */
  unsigned short res1;
  unsigned short ie;    /* 0x04 */
  unsigned short res2;
  unsigned short stat;  /* 0x08 */
  unsigned short res3;
  unsigned short iv;    /* 0x0C */
  unsigned short res4;
  unsigned short syss;  /* 0x10 */
  unsigned short res4a;
  unsigned short buf;   /* 0x14 */
  unsigned short res5;
  unsigned short cnt;   /* 0x18 */
  unsigned short res6;
  unsigned short data;  /* 0x1C */
  unsigned short res7;
  unsigned short sysc;  /* 0x20 */
  unsigned short res8;
  unsigned short con;   /* 0x24 */
  unsigned short res9;
  unsigned short oa;    /* 0x28 */
  unsigned short res10;
  unsigned short sa;    /* 0x2C */
  unsigned short res11;
  unsigned short psc;   /* 0x30 */
  unsigned short res12;
  unsigned short scll;  /* 0x34 */
  unsigned short res13;
  unsigned short sclh;  /* 0x38 */
  unsigned short res14;
  unsigned short systest; /* 0x3c */
  unsigned short res15;
};

void i2c_init( int speed, int slaveadd );

int i2c_write(
  unsigned char chip,
  unsigned int addr,
  int alen,
  unsigned char *buffer,
  int len
);

int i2c_read(
  unsigned char chip,
  uint addr,
  int alen,
  unsigned char *buffer,
  int len
);

/**
 * @brief Initializes the I2C module @a i2c.
 *
 * Valid @a clock_in_hz values are 100000 and 400000.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_INVALID_ID Invalid @a i2c value.
 * @retval RTEMS_INVALID_CLOCK Invalid @a clock_in_hz value.
 */
rtems_status_code beagle_i2c_init(
  volatile beagle_i2c *i2c,
  unsigned clock_in_hz
);

/**
 * @brief Resets the I2C module @a i2c.
 */
void beagle_i2c_reset(volatile beagle_i2c *i2c);

/**
 * @brief Sets the I2C module @a i2c clock.
 *
 * Valid @a clock_in_hz values are 100000 and 400000.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_INVALID_CLOCK Invalid @a clock_in_hz value.
 */
rtems_status_code beagle_i2c_clock(
  volatile beagle_i2c *i2c,
  unsigned clock_in_hz
);

/**
 * @brief Starts a write transaction on the I2C module @a i2c.
 *
 * The address parameter @a addr must not contain the read/write bit.
 *
 * The error status may be delayed to the next
 * beagle_i2c_write_with_optional_stop() due to controller flaws.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_IO_ERROR Received a NACK from the slave.
 */
rtems_status_code beagle_i2c_write_start(
  volatile beagle_i2c *i2c,
  unsigned addr
);

/**
 * @brief Writes data via the I2C module @a i2c with optional stop.
 *
 * The error status may be delayed to the next
 * beagle_i2c_write_with_optional_stop() due to controller flaws.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_IO_ERROR Received a NACK from the slave.
 */
rtems_status_code beagle_i2c_write_with_optional_stop(
  volatile beagle_i2c *i2c,
  const uint8_t *out,
  size_t n,
  bool stop
);

/**
 * @brief Starts a read transaction on the I2C module @a i2c.
 *
 * The address parameter @a addr must not contain the read/write bit.
 *
 * The error status may be delayed to the next
 * beagle_i2c_read_with_optional_stop() due to controller flaws.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_IO_ERROR Received a NACK from the slave.
 */
rtems_status_code beagle_i2c_read_start(
  volatile beagle_i2c *i2c,
  unsigned addr
);

/**
 * @brief Reads data via the I2C module @a i2c with optional stop.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_IO_ERROR Received a NACK from the slave.
 * @retval RTEMS_NOT_IMPLEMENTED Stop is @a false.
 */
rtems_status_code beagle_i2c_read_with_optional_stop(
  volatile beagle_i2c *i2c,
  uint8_t *in,
  size_t n,
  bool stop
);

/**
 * @brief Writes and reads data via the I2C module @a i2c.
 *
 * This will be one bus transaction.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_IO_ERROR Received a NACK from the slave.
 */
rtems_status_code beagle_i2c_write_and_read(
  volatile beagle_i2c *i2c,
  unsigned addr,
  const uint8_t *out,
  size_t out_size,
  uint8_t *in,
  size_t in_size
);

/**
 * @brief Writes data via the I2C module @a i2c.
 *
 * This will be one bus transaction.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_IO_ERROR Received a NACK from the slave.
 */
static inline rtems_status_code beagle_i2c_write(
  volatile beagle_i2c *i2c,
  unsigned addr,
  const uint8_t *out,
  size_t out_size
)
{
  return beagle_i2c_write_and_read(i2c, addr, out, out_size, NULL, 0);
}

/**
 * @brief Reads data via the I2C module @a i2c.
 *
 * This will be one bus transaction.
 *
 * @retval RTEMS_SUCCESSFUL Successful operation.
 * @retval RTEMS_IO_ERROR Received a NACK from the slave.
 */
static inline rtems_status_code beagle_i2c_read(
  volatile beagle_i2c *i2c,
  unsigned addr,
  uint8_t *in,
  size_t in_size
)
{
  return beagle_i2c_write_and_read(i2c, addr, NULL, 0, in, in_size);
}

#define BBB_I2C_SYSCLK 48000000
#define BBB_I2C_INTERNAL_CLK 12000000
#define BBB_I2C_SPEED_CLK 100000

#define BBB_I2C_IRQ_ERROR \
  (AM335X_I2C_IRQSTATUS_NACK \
    | AM335X_I2C_IRQSTATUS_ROVR \
    | AM335X_I2C_IRQSTATUS_AL \
    | AM335X_I2C_IRQSTATUS_ARDY \
    | AM335X_I2C_IRQSTATUS_RRDY \
    | AM335X_I2C_IRQSTATUS_XRDY \
    | AM335X_I2C_IRQSTATUS_XUDF)

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

typedef enum {
  I2C0,
  I2C1,
  I2C2,
  I2C_COUNT
}bbb_i2c_id_t;


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

/*
typedef struct i2c_regs {
  unsigned short BBB_I2C_REVNB_LO;   // 0x00  
  unsigned short dummy1;              
  unsigned short BBB_I2C_REVNB_HI;    // 0x04 
  unsigned short dummy2[5];
  unsigned short BBB_I2C_SYSC;  // 0x10 = 16
  unsigned short dummy3[9]; 
  unsigned short BBB_I2C_IRQSTATUS_RAW; //0x24 = 36
  unsigned short dummy4;
  unsigned short BBB_I2C_IRQSTATUS; //0x28 = 40
  unsigned short dummy5;
  unsigned short BBB_I2C_IRQENABLE_SET; //0x2C = 44
  unsigned short dummy6;
  unsigned short BBB_I2C_IRQENABLE_CLR; //0x30 = 48
  unsigned short dummy7;
  unsigned short BBB_I2C_WE; // 0x34 = 52
  unsigned short dummy8; 
  unsigned short BBB_I2C_DMARXENABLE_SET; //0x38 = 56
  unsigned short dummy9;
  unsigned short BBB_I2C_DMATXENABLE_SET; //0x3c = 60
  unsigned short dummy10;
  unsigned short BBB_I2C_DMARXENABLE_CLR; //0x40 = 64
  unsigned short dummy11;
  unsigned short BBB_I2C_DMATXENABLE_CLR; //0x44 = 68
  unsigned short dummy12;
  unsigned short BBB_I2C_DMARXWAKE_EN; //0x48 = 72
  unsigned short dummy13;
  unsigned short BBB_I2C_DMATXWAKE_EN; //0x4c = 76
  unsigned short dummy14[33]; 
  unsigned short BBB_I2C_SYSS; //0x90 = 144
  unsigned short dummy15;
  unsigned short BBB_I2C_BUF; //0x94 = 148
  unsigned short dummy16;
  unsigned short BBB_I2C_CNT; //0x98 = 152
  unsigned short dummy17;
  unsigned short BBB_I2C_DATA; //0x9c = 156
  unsigned short dummy18[3];
  unsigned short BBB_I2C_CON; //0xA4 = 164
  unsigned short dummy19;
  unsigned short BBB_I2C_OA; //0xA8= 168
  unsigned short dummy20;
  unsigned short BBB_I2C_SA; //0xAC = 172
  unsigned short dummy21;
  unsigned short BBB_I2C_PSC; //0xB0 = 176
  unsigned short dummy22;
  unsigned short BBB_I2C_SCLL; //0xB4 = 180
  unsigned short dummy23;
  unsigned short BBB_I2C_SCLH; //0xB8 = 184
  unsigned short dummy24;
  unsigned short BBB_I2C_SYSTEST; //0xBC = 188
  unsigned short dummy25;
  unsigned short BBB_I2C_BUFSTAT; //0xc0 = 192
  unsigned short dummy26;
  unsigned short BBB_I2C_OA1; //0xc4 = 196 
  unsigned short dummy27;
  unsigned short BBB_I2C_OA2; //0xc8 = 200
  unsigned short dummy28;
  unsigned short BBB_I2C_OA3; //0xcc = 204
  unsigned short dummy29;
  unsigned short BBB_I2C_ACTOA; //0xd0 = 208 
  unsigned short dummy30;
  unsigned short BBB_I2C_SBLOCK; //0xd4 = 212 
}bbb_i2c_regs;
*/


typedef struct bbb_i2c_bus{
  i2c_bus base;
  volatile bbb_i2c_regs *regs;
  i2c_msg *msgs;
  uint32_t msg_todo;
  uint32_t current_msg_todo;
  uint8_t *current_msg_byte;
  uint32_t current_todo;
  bool read;
  bool hold;
  rtems_id task_id;
  rtems_vector_number irq;
  bbb_i2c_id_t i2c_bus_id;
  uint32_t input_clock;
  int count;
  }bbb_i2c_bus;

int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq
);

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


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_I2C_H */
