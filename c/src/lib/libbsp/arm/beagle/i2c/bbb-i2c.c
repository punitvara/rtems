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

#include <bsp/i2c.h>
#include <libcpu/am335x.h>
#include <rtems/irq-extension.h>
#include <bsp/bbb-gpio.h>
#include <rtems/score/assert.h>

/*
static bool am335x_i2c_pinmux(bbb_i2c_bus *bus)
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
*/
/* ref. Table 21-4 I2C Clock Signals */
/* 
 For I2C1/2

 Interface clock - 100MHz - CORE_LKOUTM4 / 2 - pd_per_l4ls_gclk

 Functional clock - 48MHz - PER_CLKOUTM2 / 4 - pd_per_ic2_fclk
*/
/*
static void am335x_i2c1_i2c2_module_clk_config(bbb_i2c_bus *bus)
{
*/
/*0x2 = SW_WKUP : SW_WKUP: Start a software forced wake-up
transition on the domain. */
/*
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) |=
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) &
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL) !=
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP);
*/

/* 0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
 /* REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) |=
                        AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;

  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) &
      AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE) != AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);
*/
/*0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
/*
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
*/
static void am335x_i2c0_pinmux(bbb_i2c_bus *bus)
{
REG(bus->i2c_base_address + AM335X_CONF_I2C0_SDA) =  
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN);

REG(bus->i2c_base_address + AM335X_CONF_I2C0_SCL) = 
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN); 
}

static void I2C0ModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
          CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));


    /* Configuring registers related to Wake-Up region. */

    /* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
          CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
          CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
          CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field of CM_WKUP_I2C0_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) |=
          CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_MODULEMODE));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));

    /*
    ** Waiting for CLKACTIVITY_I2C0_GFCLK field in CM_WKUP_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_I2C0_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_I2C0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_I2C0_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_IDLEST));
}

/*
void am335x_i2c_init(bbb_i2c_bus *bus, uint32_t input_clock)
{
  // am335x_i2c_pinmux()
  // am335x_i2c1_i2c2_module_clk_config
}
*/
/*Any suggestions to disable interrupt here*/
static void am335x_i2c_reset(bbb_i2c_bus *bus)
{
  volatile bbb_i2c_regs *regs = bus->regs;

  /* Disable I2C module at the time of initialization*/
  /*Should I use write32 ?? I guess mmio_clear is correct choice here*/
  mmio_clear((regs->BBB_I2C_CON),AM335X_I2C_CON_I2C_EN);
  mmio_clear((regs->BBB_I2C_SYSC),AM335X_I2C_SYSC_AUTOIDLE);

/*
  can I clear all the interrupt here ?
  mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_IRQ_ENABLE_CLR), ??)
*/
}

/*
Possible values for msg->flag 
  * - @ref I2C_M_TEN,
   * - @ref I2C_M_RD,
   * - @ref I2C_M_STOP,
   * - @ref I2C_M_NOSTART,
   * - @ref I2C_M_REV_DIR_ADDR,
   * - @ref I2C_M_IGNORE_NAK,
   * - @ref I2C_M_NO_RD_ACK, and
   * - @ref I2C_M_RECV_LEN.
*/
/*
static void am335x_i2c_set_address_size(const i2c_msg *msg)
{
  bbb_i2c_bus *bus;
  volatile bbb_i2c_regs *regs = bus->regs;
  if ((msg->flags & I2C_M_TEN) == 0)  {
    mmio_write(regs->BBB_I2C_CON,(AM335X_I2C_CFG_7BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  } else {
    mmio_write(regs->BBB_I2C_CON,(AM335X_I2C_CFG_10BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  }
}
*/
static void am335x_i2c_set_address_size(const i2c_msg *msgs,volatile bbb_i2c_regs *regs, uint16_t slave_addr)
{
   mmio_write(regs->BBB_I2C_CON, (slave_addr | AM335X_I2C_CON_I2C_EN));
/*can be configured multiple modes here. Need to think about own address modes*/
  if ((msgs->flags & I2C_M_TEN) == 0)  {/* 7-bit mode slave address mode*/
    mmio_write(regs->BBB_I2C_CON,(AM335X_I2C_CFG_7BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  } else { /* 10-bit slave address mode*/
    mmio_write(regs->BBB_I2C_CON,(AM335X_I2C_CFG_10BIT_SLAVE_ADDR | AM335X_I2C_CON_I2C_EN));
  }
}

/*
static void am335x_i2c_next_byte(bbb_i2c_bus *bus)
{
  --bus->current_msg_todo;
  ++bus->current_msg_byte;
  
  if (bus->current_msg_todo == 0) {
    i2c_msg *msg;
    
  ++bus->msgs;
  ++bus>msg_todo;

  msg = &bus->msgs[0];
 
  bus->current_msg_todo = msg->len;
  bus->current_msg_byte = msg->buf;
  }
}
*/

static void am335x_i2c_masterint_enable(volatile bbb_i2c_regs *regs, unsigned int flag)
{
  regs->BBB_I2C_IRQENABLE_SET |= flag;
}

static void am335x_i2c_masterint_disable(volatile bbb_i2c_regs *regs, unsigned int flag)
{
  regs->BBB_I2C_IRQENABLE_CLR = flag;	
}
static void am335x_i2c_setup_read_transfer(volatile bbb_i2c_regs *regs, bool send_stop)
{
  am335x_i2c_masterint_enable(regs, AM335X_I2C_INT_RECV_READY);
  if (send_stop) {
    regs->BBB_I2C_CON |= AM335X_I2C_CON_STOP; 
  } else {
    regs->BBB_I2C_CON |= AM335X_I2C_CON_START;
  }
}

static void am335x_i2c_disable_interrupts(volatile bbb_i2c_regs *regs)
{
  regs->BBB_I2C_IRQSTATUS = 0x00000000;
}

static void am335x_i2c_continue_read_transfer(
  bbb_i2c_bus *bus,
  volatile bbb_i2c_regs *regs
  )
{ 
  bus->count =0;
  bus->current_msg_byte[bus->count] = regs->BBB_I2C_DATA;
  bus->count++;
  if (bus->count == bus->current_msg_todo) {
    am335x_i2c_setup_read_transfer(regs,false);
    am335x_i2c_masterint_disable(regs, AM335X_I2C_INT_RECV_READY);
    
  }
}

static void am335x_i2c_setup_transfer(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs)
{
  const i2c_msg *msgs = bus->msgs;
  uint32_t msg_todo = bus->msg_todo;
  bool send_stop = false;
  uint32_t i;
  uint16_t slave_addr;

 
  bus->current_todo = msgs[0].len;
  for (i = 1; i < msg_todo && (msgs[i].flags & I2C_M_NOSTART) != 0; ++i) {
    bus->current_todo += msgs[i].len;
  }
  slave_addr = msgs->addr; 
  regs = bus->regs;
  // Yet to decide which FIFO buffer to clear 
  bus->regs->BBB_I2C_BUF |= AM335X_I2C_BUF_TXFIFO_CLR;
  bus->regs->BBB_I2C_BUF |= AM335X_I2C_BUF_RXFIFO_CLR;
  bus->hold = i < msg_todo;
   // Depends on FIFO depth howmuch data should hold yet to decide
  am335x_i2c_set_address_size(msgs,regs,slave_addr);
  bus->read = (msgs->flags & I2C_M_RD) != 0; 
  if (bus->read) {
  //am335x_i2c_set_transfer_status(); 
    if (bus->current_msg_todo == 1) {
      send_stop = true;
    }
    am335x_i2c_setup_read_transfer(regs, send_stop);
  } else {
  //am335x_i2c_set_transfer_status();
    //am335x_i2c_setup_write_transfer();
  }
  
}

static void am335x_i2c_interrupt(void *arg)
{
  bbb_i2c_bus *bus = arg;
  volatile bbb_i2c_regs *regs = bus->regs;
  uint32_t irqstatus = regs->BBB_I2C_IRQSTATUS;
  bool done = false;

  /* Make sure that all interrupt registers are cleared here */
  /* Clear interrupt */
  regs->BBB_I2C_IRQSTATUS = irqstatus;
  /* should consider timeout in following condition */
  /*Only error interrupt condition checked. Other interrupt bits yet to checked */
  if ((irqstatus & (AM335X_I2C_IRQSTATUS_AL | AM335X_I2C_IRQSTATUS_NACK)) != 0) {
    done = true;
  }
  if (irqstatus & AM335X_I2C_INT_RECV_READY) {
    am335x_i2c_continue_read_transfer(bus, regs);
  }

  if (done) {
    uint32_t err = irqstatus & BBB_I2C_IRQ_ERROR;
  
    if (bus->msg_todo == 0 || err != 0) {
    rtems_status_code sc;
  
    am335x_i2c_disable_interrupts(regs);
  
    regs->BBB_I2C_IRQSTATUS = err;
  
    sc = rtems_event_transient_send(bus->task_id);
    _Assert(sc == RTEMS_SUCCESSFUL);
    (void) sc;
    } else {
      am335x_i2c_setup_transfer(bus, regs);
    }
  }
}
static int am335x_i2c_transfer(i2c_bus *base, i2c_msg *msgs, uint32_t msg_count)
{
  rtems_status_code sc;
  bbb_i2c_bus *bus = (bbb_i2c_bus *)base;
  volatile bbb_i2c_regs *regs;
  uint32_t i;

  rtems_task_wake_after(1);

  if (msg_count < 1){
    return 1;
  }
  for (i=0; i<msg_count;++i) {
      if ((msgs[i].flags & I2C_M_RECV_LEN) != 0) {
        return -EINVAL;
      }
  }
  
  bus->msgs = &msgs[0];
  bus->msg_todo = msg_count;
  bus->current_msg_todo = msgs[0].len;
  bus->current_msg_byte = msgs[0].buf;
  bus->task_id = rtems_task_self();

  regs = bus->regs;
  am335x_i2c_setup_transfer(bus,regs);
  regs->BBB_I2C_IRQENABLE_SET = BBB_I2C_IRQ_USED;

  sc = rtems_event_transient_receive(RTEMS_WAIT, bus->base.timeout);
  // If timeout then return timeout error
  if (sc != RTEMS_SUCCESSFUL) {
    am335x_i2c_reset(bus);
    rtems_event_transient_clear();

    return -ETIMEDOUT;
  }
 // return bus->regs->BBB_I2C_IRQSTATUS == 0 ? 0 : -EIO;
  return 0;
}
static int am335x_i2c_set_clock(i2c_bus *base, unsigned long clock)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;
  uint32_t prescaler,divider;
 
  prescaler = (BBB_I2C_SYSCLK / BBB_I2C_INTERNAL_CLK) -1;
  mmio_write((regs->BBB_I2C_PSC), prescaler);
  divider = BBB_I2C_INTERNAL_CLK/(2*clock);
  mmio_write((regs->BBB_I2C_SCLL), (divider - 7));
  mmio_write((regs->BBB_I2C_CON), (divider - 5));
  return 0;
}

static void am335x_i2c_destroy(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  rtems_status_code sc;
 
  sc = rtems_interrupt_handler_remove(bus->irq, am335x_i2c_interrupt, bus);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void)sc;
  i2c_bus_destroy_and_free(&bus->base);
}

int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq
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
  bus->i2c_base_address = register_base;
  am335x_i2c0_pinmux(bus);
  I2C0ModuleClkConfig();
  bus->regs = (volatile bbb_i2c_regs *) register_base;
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
 
    rtems_set_errno_and_return_minus_one(EIO);
  }
 
  bus->base.transfer = am335x_i2c_transfer;
  bus->base.set_clock = am335x_i2c_set_clock;
  bus->base.destroy = am335x_i2c_destroy;
  return i2c_bus_register(&bus->base,bus_path);
}

