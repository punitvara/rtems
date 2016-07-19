/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C bus initialization and API Support.
 */

/*
 * Copyright (c) 2016 Punit Vara <punitvara at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <dev/i2c/i2c.h>
#include <bsp/i2c.h>
#include <libcpu/am335x.h>
#include <bsp.h>
#include <rtems/irq-extension.h>
#include <rtems/score/assert.h>
#include <bsp/beagleboneblack.h>
#include <rtems.h>
#include <bsp/bbb-gpio.h>

const uint32_t i2c_base_addrs[] = {AM335X_I2C0_BASE, AM335X_I2C1_BASE, AM335X_I2C0_BASE};

const int i2c_irq_num[] = {BBB_I2C0_IRQ, BBB_I2C1_IRQ , BBB_I2C2_IRQ};

/*register definitions*/
static bbb_i2c_regs am335x_i2c_regs = {
  .BBB_I2C_REVNB_LO = AM335X_I2C_REVNB_LO,
  .BBB_I2C_REVNB_HI = AM335X_I2C_REVNB_HI,
  .BBB_I2C_SYSC = AM335X_I2C_SYSC,
  .BBB_I2C_IRQSTATUS_RAW = AM335X_I2C_IRQSTATUS_RAW,
  .BBB_I2C_IRQSTATUS = AM335X_I2C_IRQSTATUS,
  .BBB_I2C_IRQENABLE_SET = AM335X_I2C_IRQENABLE_SET,
  .BBB_I2C_IRQENABLE_CLR = AM335X_I2C_IRQENABLE_CLR,
  .BBB_I2C_WE = AM335X_I2C_WE,
  .BBB_I2C_DMARXENABLE_SET = AM335X_I2C_DMARXENABLE_SET,
  .BBB_I2C_DMATXENABLE_SET = AM335X_I2C_DMATXENABLE_SET,
  .BBB_I2C_DMARXENABLE_CLR = AM335X_I2C_DMARXENABLE_CLR,
  .BBB_I2C_DMATXENABLE_CLR = AM335X_I2C_DMATXENABLE_CLR,
  .BBB_I2C_DMARXWAKE_EN = AM335X_I2C_DMARXWAKE_EN,
  .BBB_I2C_DMATXWAKE_EN = AM335X_I2C_DMATXWAKE_EN,
  .BBB_I2C_SYSS = AM335X_I2C_SYSS,
  .BBB_I2C_BUF = AM335X_I2C_BUF,
  .BBB_I2C_CNT = AM335X_I2C_CNT,
  .BBB_I2C_DATA = AM335X_I2C_DATA,
  .BBB_I2C_CON = AM335X_I2C_CON,
  .BBB_I2C_OA = AM335X_I2C_OA,
  .BBB_I2C_SA = AM335X_I2C_SA,
  .BBB_I2C_PSC = AM335X_I2C_PSC,
  .BBB_I2C_SCLL = AM335X_I2C_SCLL,
  .BBB_I2C_SCLH = AM335X_I2C_SCLH,
  .BBB_I2C_SYSTEST = AM335X_I2C_SYSTEST,
  .BBB_I2C_BUFSTAT = AM335X_I2C_BUFSTAT,
  .BBB_I2C_OA1 = AM335X_I2C_OA1,
  .BBB_I2C_OA2 = AM335X_I2C_OA2,
  .BBB_I2C_OA3 = AM335X_I2C_OA3,
  .BBB_I2C_ACTOA = AM335X_I2C_ACTOA,
  .BBB_I2C_SBLOCK = AM335X_I2C_SBLOCK
};

static inline uint32_t get_reg_addr(uint32_t offset)
{
  return (bbb_i2c_bus.i2c_base_address + offset);
}

bool am335x_i2c_pinmux(bbb_i2c_bus *bus)
{
  bool status =true;

    // We will check i2c_bus_id in am335x_i2c_bus_register
    // Apart from mode and pull_up register what about SCREWCTRL & RXACTIVE ??
  if (bus->i2c_bus_id == I2C1) {
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_CS0) = (BBB_MUXMODE(MODE2) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D1) = (BBB_MUXMODE(MODE2) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_TXD) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_RXD) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
  } else if (bus->i2c_bus_id == I2C2) {
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_RTSN) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_CTSN) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
  } else {
  status = false;  
  }
  return status;   
}

/* ref. Table 21-4 I2C Clock Signals */
/* 
 For I2C1/2

 Interface clock - 100MHz - CORE_LKOUTM4 / 2 - pd_per_l4ls_gclk

 Functional clock - 48MHz - PER_CLKOUTM2 / 4 - pd_per_ic2_fclk
*/
void am335x_i2c1_i2c2_module_clk_config(bbb_i2c_bus *bus)
{
/*0x2 = SW_WKUP : SW_WKUP: Start a software forced wake-up
transition on the domain. */

  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) |=
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) &
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL) !=
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP);


/* 0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) |=
                        AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;

  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) &
      AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE) != AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);

/*0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
  if (bus->i2c_bus_id == I2C1) {
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C1_CLKCTRL) |=
                             AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE;

  while(REG((AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C1_CLKCTRL) &
     AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE) != AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE);
  } else if (bus->i2c_bus_id == I2C2) {
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C2_CLKCTRL) |=
                             AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE_ENABLE;

  while(REG((AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C2_CLKCTRL) &
     AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE) != AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE_ENABLE);

  while(!(REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) &
           (AM335X_CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            AM335X_CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_I2C_FCLK)));

  }
}
/*
void am335x_i2c_init(bbb_i2c_bus *bus, uint32_t input_clock)
{
  // am335x_i2c_pinmux()
  // am335x_i2c1_i2c2_module_clk_config
}
*/
/*Any suggestions to disable interrupt here*/
void am335x_i2c_reset(bbb_i2c_bus *bus)
{

  /* Disable I2C module at the time of initialization*/
  /*Should I use write32 ?? I guess mmio_clear is correct choice here*/
  mmio_clear(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON),AM335X_I2C_CON_I2C_EN); 

  mmio_clear(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_SYSC),AM335X_I2C_SYSC_AUTOIDLE);

/*
  can I clear all the interrupt here ?
  mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_IRQ_ENABLE_CLR), ??)
*/
}

void am335x_i2c_set_address_size(const i2c_msg *msg)
{
/*can be configured multiple modes here. Need to think about own address modes*/
  if (msg->flags) {/*10-bit mode*/
    mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON),(AM335X_I2C_CFG_10BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  } else { /*7 bit mode*/
    mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON),(AM335X_I2C_CFG_7BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  }
}

int am335x_i2c_transfer(
  i2c_bus *base,
  i2c_msg *msgs,
  uint32_t msg_count
)
{
  rtems_status_code sc;
  bbb_i2c_bus *bus = (bbb_i2c_bus *)base;
  bbb_i2c_regs *regs;
  
   rtems_task_wake_after(1);

        if (msg_count < 1){
                return 1;
        }
  // Add new function and call bbb_i2c_setup_transfer() 
}

int am335x_i2c_set_clock(i2c_bus *base, unsigned long clock)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;
  uint32_t prescaler,divider;
 
  prescaler = (BBB_I2C_SYSCLK / BBB_I2C_INTERNAL_CLK) -1;
  mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_PSC), prescaler);
  divider = BBB_I2C_INTERNAL_CLK/(2*clock);
  mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_SCLL), divider - 7);
  mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON), divider - 5);
  return 0;
}

void am335x_i2c_destroy(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  rtems_status_code sc;
 
  sc = rtems_interrupt_handler_remove(bus->irq, bbb_i2c_interrupt, bus);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void)sc;

  i2c_bus_destroy_and_free(&bus->base);
}

int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq,
  bbb_i2c_id_t i2c_bus_number
)
{
  bbb_i2c_bus *bus;
  rtems_status_code sc;
  int err;
  /*check bus number is >0 & <MAX*/

  bus = (bbb_i2c_bus *) i2c_bus_alloc_and_init(sizeof(*bus));
  if (bus == NULL) {
    return -1;
  }
  bus->i2c_bus_id = i2c_bus_number;
  bus->i2c_base_address = i2c_base_addrs[i2c_bus_number]; 
  bus->regs = &am335_i2c_regs; // beagle_i2c_regs ?? // How to use register_base here 
  bus->input_clock = input_clock; // By default 100KHz. Normally pass 100KHz as argument 
  bus->irq = irq;
  
  /*Initialize members of bbb_i2c_bus above this line because following function will use them*/ 
  am335x_i2c_reset(bus);
 
  err = am335x_i2c_set_clock(&bus->base, I2C_BUS_CLOCK_DEFAULT);
 
  if (err != 0) {
  (*bus->base.destroy)(&bus->base);

  rtems_set_errno_and_return_minus_one(-err);
  }

  sc  = rtems_interrupt_handler_install(
    irq,
    "BBB I2C",
    RTEMS_INTERRUPT_UNIQUE,
    am335x_i2c_interrupt,
    bus
  );
  if (sc != RTEMS_SUCCESSFUL) {
    (*bus->base.destroy)(&bus->base);
 
    rtems_set_errno_and_return_minux_one(EIO);
  }
 
  bus->base.transfer = am335x_i2c_transfer;
  bus->base.set_clock = am335x_i2c_set_clock;
  bus->base.destroy = am335x_i2c_destroy;
  return i2c_bus_register(&bus->base,bus_path);
}

