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
 *
 * Modified by Ben Gras <beng@shrike-systems.com> to add lots
 * of beagleboard/beaglebone definitions, delete lpc32xx specific
 * ones, and merge with some other header files.
 */

/* Interrupt controller memory map */
#define OMAP3_DM37XX_INTR_BASE 0x48200000 /* INTCPS physical address */

/* Interrupt controller memory map */
#define OMAP3_AM335X_INTR_BASE 0x48200000 /* INTCPS physical address */

#define AM335X_INT_EMUINT             0
    /* Emulation interrupt (EMUICINTR) */
#define AM335X_INT_COMMTX             1
    /* CortexA8 COMMTX */
#define AM335X_INT_COMMRX             2
    /* CortexA8 COMMRX */
#define AM335X_INT_BENCH              3
    /* CortexA8 NPMUIRQ */
#define AM335X_INT_ELM_IRQ            4
    /* Sinterrupt (Error location process completion) */
#define AM335X_INT_NMI                7
    /* nmi_int */
#define AM335X_INT_L3DEBUG            9
    /* l3_FlagMux_top_FlagOut1 */
#define AM335X_INT_L3APPINT           10
    /* l3_FlagMux_top_FlagOut0  */
#define AM335X_INT_PRCMINT            11
    /* irq_mpu */
#define AM335X_INT_EDMACOMPINT        12
    /* tpcc_int_pend_po0 */
#define AM335X_INT_EDMAMPERR          13
    /* tpcc_mpint_pend_po */
#define AM335X_INT_EDMAERRINT         14
    /* tpcc_errint_pend_po */
#define AM335X_INT_ADC_TSC_GENINT     16
    /* gen_intr_pend */
#define AM335X_INT_USBSSINT           17
    /* usbss_intr_pend */
#define AM335X_INT_USB0               18
    /* usb0_intr_pend */
#define AM335X_INT_USB1               19
    /* usb1_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT0     20
    /* pr1_host_intr0_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT1     21
    /* pr1_host_intr1_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT2     22
    /* pr1_host_intr2_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT3     23
    /* pr1_host_intr3_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT4     24
    /* pr1_host_intr4_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT5     25
    /* pr1_host_intr5_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT6     26
    /* pr1_host_intr6_intr_pend */
#define AM335X_INT_PRUSS1_EVTOUT7     27
    /* pr1_host_intr7_intr_pend */
#define AM335X_INT_MMCSD1INT          28
    /* MMCSD1  SINTERRUPTN */
#define AM335X_INT_MMCSD2INT          29
    /* MMCSD2  SINTERRUPT */
#define AM335X_INT_I2C2INT            30
    /* I2C2  POINTRPEND */
#define AM335X_INT_eCAP0INT           31
    /* ecap_intr_intr_pend */
#define AM335X_INT_GPIOINT2A          32
    /* GPIO 2  POINTRPEND1 */
#define AM335X_INT_GPIOINT2B          33
    /* GPIO 2  POINTRPEND2 */
#define AM335X_INT_USBWAKEUP          34
    /* USBSS  slv0p_Swakeup */
#define AM335X_INT_LCDCINT            36
    /* LCDC  lcd_irq */
#define AM335X_INT_GFXINT             37
    /* SGX530  THALIAIRQ */
#define AM335X_INT_ePWM2INT           39
    /* (PWM Subsystem)  epwm_intr_intr_pend */
#define AM335X_INT_3PGSWRXTHR0        40
    /* (Ethernet)  c0_rx_thresh_pend (RX_THRESH_PULSE) */
#define AM335X_INT_3PGSWRXINT0        41
    /* CPSW (Ethernet)  c0_rx_pend */
#define AM335X_INT_3PGSWTXINT0        42
    /* CPSW (Ethernet)  c0_tx_pend */
#define AM335X_INT_3PGSWMISC0         43
    /* CPSW (Ethernet)  c0_misc_pend */
#define AM335X_INT_UART3INT           44
    /* UART3  niq */
#define AM335X_INT_UART4INT           45
    /* UART4  niq */
#define AM335X_INT_UART5INT           46
    /* UART5  niq */
#define AM335X_INT_eCAP1INT           47
    /* (PWM Subsystem)  ecap_intr_intr_pend */
#define AM335X_INT_DCAN0_INT0         52
    /* DCAN0  dcan_intr0_intr_pend */
#define AM335X_INT_DCAN0_INT1         53
    /* DCAN0  dcan_intr1_intr_pend */
#define AM335X_INT_DCAN0_PARITY       54
    /* DCAN0  dcan_uerr_intr_pend */
#define AM335X_INT_DCAN1_INT0         55
    /* DCAN1  dcan_intr0_intr_pend */
#define AM335X_INT_DCAN1_INT1         56
    /* DCAN1  dcan_intr1_intr_pend */
#define AM335X_INT_DCAN1_PARITY       57
    /* DCAN1  dcan_uerr_intr_pend */
#define AM335X_INT_ePWM0_TZINT        58
    /* eHRPWM0 TZ interrupt (PWM  epwm_tz_intr_pend Subsystem) */
#define AM335X_INT_ePWM1_TZINT        59
    /* eHRPWM1 TZ interrupt (PWM  epwm_tz_intr_pend Subsystem) */
#define AM335X_INT_ePWM2_TZINT        60
    /* eHRPWM2 TZ interrupt (PWM  epwm_tz_intr_pend Subsystem) */
#define AM335X_INT_eCAP2INT           61
    /* eCAP2 (PWM Subsystem)  ecap_intr_intr_pend */
#define AM335X_INT_GPIOINT3A          62
    /* GPIO 3  POINTRPEND1 */
#define AM335X_INT_GPIOINT3B          63
    /* GPIO 3  POINTRPEND2 */
#define AM335X_INT_MMCSD0INT          64
    /* MMCSD0  SINTERRUPTN */
#define AM335X_INT_SPI0INT            65
    /* McSPI0  SINTERRUPTN */
#define AM335X_INT_TINT0              66
    /* Timer0  POINTR_PEND */
#define AM335X_INT_TINT1_1MS          67
    /* DMTIMER_1ms  POINTR_PEND */
#define AM335X_INT_TINT2              68
    /* DMTIMER2  POINTR_PEND */
#define AM335X_INT_TINT3              69
    /* DMTIMER3  POINTR_PEND */
#define AM335X_INT_I2C0INT            70
    /* I2C0  POINTRPEND */
#define AM335X_INT_I2C1INT            71
    /* I2C1  POINTRPEND */
#define AM335X_INT_UART0INT           72
    /* UART0  niq */
#define AM335X_INT_UART1INT           73
    /* UART1  niq */
#define AM335X_INT_UART2INT           74
    /* UART2  niq */
#define AM335X_INT_RTCINT             75
    /* RTC  timer_intr_pend */
#define AM335X_INT_RTCALARMINT        76
    /* RTC  alarm_intr_pend */
#define AM335X_INT_MBINT0             77
    /* Mailbox0 (mail_u0_irq)  initiator_sinterrupt_q_n */
#define AM335X_INT_M3_TXEV            78
    /* Wake M3 Subsystem  TXEV */
#define AM335X_INT_eQEP0INT           79
    /* eQEP0 (PWM Subsystem)  eqep_intr_intr_pend */
#define AM335X_INT_MCATXINT0          80
    /* McASP0  mcasp_x_intr_pend */
#define AM335X_INT_MCARXINT0          81
    /* McASP0  mcasp_r_intr_pend */
#define AM335X_INT_MCATXINT1          82
    /* McASP1  mcasp_x_intr_pend */
#define AM335X_INT_MCARXINT1          83
    /* McASP1  mcasp_r_intr_pend */
#define AM335X_INT_ePWM0INT           86
    /* (PWM Subsystem)  epwm_intr_intr_pend */
#define AM335X_INT_ePWM1INT           87
    /* (PWM Subsystem)  epwm_intr_intr_pend */
#define AM335X_INT_eQEP1INT           88
    /* (PWM Subsystem)  eqep_intr_intr_pend */
#define AM335X_INT_eQEP2INT           89
    /* (PWM Subsystem)  eqep_intr_intr_pend */
#define AM335X_INT_DMA_INTR_PIN2      90
    /* External DMA/Interrupt Pin2  */
#define AM335X_INT_WDT1INT            91
    /* (Public Watchdog) WDTIMER1 PO_INT_PEND */
#define AM335X_INT_TINT4              92
    /* DMTIMER4  POINTR_PEN */
#define AM335X_INT_TINT5              93
    /* DMTIMER5  POINTR_PEN */
#define AM335X_INT_TINT6              94
    /* DMTIMER6  POINTR_PEND */
#define AM335X_INT_TINT7              95
    /* DMTIMER7  POINTR_PEND */
#define AM335X_INT_GPIOINT0A          96
    /* GPIO 0  POINTRPEND1 */
#define AM335X_INT_GPIOINT0B          97
    /* GPIO 0  POINTRPEND2 */
#define AM335X_INT_GPIOINT1A          98
    /* GPIO 1  POINTRPEND1 */
#define AM335X_INT_GPIOINT1B          99
    /* GPIO 1  POINTRPEND2 */
#define AM335X_INT_GPMCINT            100
    /* GPMC  gpmc_sinterrupt */
#define AM335X_INT_DDRERR0            101
    /* EMIF  sys_err_intr_pend */
#define AM335X_INT_TCERRINT0          112
    /* TPTC0  tptc_erint_pend_po */
#define AM335X_INT_TCERRINT1          113
    /* TPTC1  tptc_erint_pend_po */
#define AM335X_INT_TCERRINT2          114
    /* TPTC2  tptc_erint_pend_po */
#define AM335X_INT_ADC_TSC_PENINT     115
    /* ADC_TSC  pen_intr_pend */
#define AM335X_INT_SMRFLX_Sabertooth  120
    /* Smart Reflex 0  intrpen */
#define AM335X_INT_SMRFLX_Core        121
    /* Smart Reflex 1  intrpend */
#define AM335X_INT_DMA_INTR_PIN0      123
    /* pi_x_dma_event_intr0 (xdma_event_intr0) */
#define AM335X_INT_DMA_INTR_PIN1      124
    /* pi_x_dma_event_intr1 (xdma_event_intr1) */
#define AM335X_INT_SPI1INT            125
    /* McSPI1  SINTERRUPTN */

#define OMAP3_AM335X_NR_IRQ_VECTORS    125

#define AM335X_DMTIMER0_BASE      0x44E05000
    /* DMTimer0 Registers */
#define AM335X_DMTIMER1_1MS_BASE  0x44E31000
    /* DMTimer1 1ms Registers (Accurate 1ms timer) */
#define AM335X_DMTIMER2_BASE      0x48040000
    /*  DMTimer2 Registers */
#define AM335X_DMTIMER3_BASE      0x48042000
    /*  DMTimer3 Registers */
#define AM335X_DMTIMER4_BASE      0x48044000
    /* DMTimer4 Registers  */
#define AM335X_DMTIMER5_BASE      0x48046000
    /* DMTimer5 Registers  */
#define AM335X_DMTIMER6_BASE      0x48048000
    /*  DMTimer6 Registers */
#define AM335X_DMTIMER7_BASE      0x4804A000
    /*  DMTimer7 Registers */

/* General-purpose timer registers
   AM335x non 1MS timers have different offsets */
#define AM335X_TIMER_TIDR             0x000
    /* IP revision code */
#define AM335X_TIMER_TIOCP_CFG        0x010
    /* Controls params for GP timer L4 interface */
#define AM335X_TIMER_IRQSTATUS_RAW    0x024
    /* Timer IRQSTATUS Raw Register */
#define AM335X_TIMER_IRQSTATUS        0x028
    /* Timer IRQSTATUS Register */
#define AM335X_TIMER_IRQENABLE_SET    0x02C
    /* Timer IRQENABLE Set Register */
#define AM335X_TIMER_IRQENABLE_CLR    0x030
    /* Timer IRQENABLE Clear Register */
#define AM335X_TIMER_IRQWAKEEN        0x034
    /* Timer IRQ Wakeup Enable Register */
#define AM335X_TIMER_TCLR             0x038
    /* Controls optional features */
#define AM335X_TIMER_TCRR             0x03C
    /* Internal counter value */
#define AM335X_TIMER_TLDR             0x040
    /* Timer load value */
#define AM335X_TIMER_TTGR             0x044
    /* Triggers counter reload */
#define AM335X_TIMER_TWPS             0x048
    /* Indicates if Write-Posted pending */
#define AM335X_TIMER_TMAR             0x04C
    /* Value to be compared with counter */
#define AM335X_TIMER_TCAR1            0x050
    /* First captured value of counter register */
#define AM335X_TIMER_TSICR            0x054
    /* Control posted mode and functional SW reset */
#define AM335X_TIMER_TCAR2            0x058
    /* Second captured value of counter register */
#define AM335X_WDT_BASE                0x44E35000
    /* Watchdog timer */
#define AM335X_WDT_WWPS                0x34
    /* Command posted status */
#define AM335X_WDT_WSPR                0x48
    /* Activate/deactivate sequence */

/* RTC registers */
#define AM335X_RTC_BASE         0x44E3E000
#define AM335X_RTC_SECS         0x0
#define AM335X_RTC_MINS         0x4
#define AM335X_RTC_HOURS        0x8
#define AM335X_RTC_DAYS         0xc
#define AM335X_RTC_MONTHS       0x10
#define AM335X_RTC_YEARS        0x14
#define AM335X_RTC_WEEKS        0x18
#define AM335X_RTC_CTRL_REG     0x40
#define AM335X_RTC_STATUS_REG   0x44
#define AM335X_RTC_REV_REG      0x74
#define AM335X_RTC_SYSCONFIG    0x78
#define AM335X_RTC_KICK0        0x6c
#define AM335X_RTC_KICK1        0x70
#define AM335X_RTC_OSC_CLOCK    0x54

#define AM335X_RTC_KICK0_KEY    0x83E70B13
#define AM335X_RTC_KICK1_KEY    0x95A4F1E0

/* GPIO memory-mapped registers */

#define AM335X_GPIO0_BASE           0x44E07000
    /* GPIO Bank 0 base Register */
#define AM335X_GPIO1_BASE           0x4804C000
    /* GPIO Bank 1 base Register */
#define AM335X_GPIO2_BASE           0x481AC000
    /* GPIO Bank 2 base Register */
#define AM335X_GPIO3_BASE           0x481AE000
    /* GPIO Bank 3 base Register */

#define AM335X_GPIO_REVISION        0x00
#define AM335X_GPIO_SYSCONFIG       0x10
#define AM335X_GPIO_EOI             0x20
#define AM335X_GPIO_IRQSTATUS_RAW_0 0x24
#define AM335X_GPIO_IRQSTATUS_RAW_1 0x28
#define AM335X_GPIO_IRQSTATUS_0     0x2C
#define AM335X_GPIO_IRQSTATUS_1     0x30
#define AM335X_GPIO_IRQSTATUS_SET_0 0x34
#define AM335X_GPIO_IRQSTATUS_SET_1 0x38
#define AM335X_GPIO_IRQSTATUS_CLR_0 0x3C
#define AM335X_GPIO_IRQSTATUS_CLR_1 0x40
#define AM335X_GPIO_IRQWAKEN_0      0x44
#define AM335X_GPIO_IRQWAKEN_1      0x48
#define AM335X_GPIO_SYSSTATUS       0x114
#define AM335X_GPIO_CTRL            0x130
#define AM335X_GPIO_OE              0x134
#define AM335X_GPIO_DATAIN          0x138
#define AM335X_GPIO_DATAOUT         0x13C
#define AM335X_GPIO_LEVELDETECT0    0x140
#define AM335X_GPIO_LEVELDETECT1    0x144
#define AM335X_GPIO_RISINGDETECT    0x148
#define AM335X_GPIO_FALLINGDETECT   0x14C
#define AM335X_GPIO_DEBOUNCENABLE   0x150
#define AM335X_GPIO_DEBOUNCINGTIME  0x154
#define AM335X_GPIO_CLEARDATAOUT    0x190
#define AM335X_GPIO_SETDATAOUT      0x194

/* AM335X Pad Configuration Register Base */
#define AM335X_PADCONF_BASE 0x44E10000

/* Memory mapped register offset for Control Module */
#define AM335X_CONF_GPMC_AD0 0x800
#define AM335X_CONF_GPMC_AD1 0x804
#define AM335X_CONF_GPMC_AD2 0x808
#define AM335X_CONF_GPMC_AD3 0x80C
#define AM335X_CONF_GPMC_AD4 0x810
#define AM335X_CONF_GPMC_AD5 0x814
#define AM335X_CONF_GPMC_AD6 0x818
#define AM335X_CONF_GPMC_AD7 0x81C
#define AM335X_CONF_GPMC_AD8 0x820
#define AM335X_CONF_GPMC_AD9 0x824
#define AM335X_CONF_GPMC_AD10 0x828
#define AM335X_CONF_GPMC_AD11 0x82C
#define AM335X_CONF_GPMC_AD12 0x830
#define AM335X_CONF_GPMC_AD13 0x834
#define AM335X_CONF_GPMC_AD14 0x838
#define AM335X_CONF_GPMC_AD15 0x83C
#define AM335X_CONF_GPMC_A0 0x840
#define AM335X_CONF_GPMC_A1 0x844
#define AM335X_CONF_GPMC_A2 0x848
#define AM335X_CONF_GPMC_A3 0x84C
#define AM335X_CONF_GPMC_A4 0x850
#define AM335X_CONF_GPMC_A5 0x854
#define AM335X_CONF_GPMC_A6 0x858
#define AM335X_CONF_GPMC_A7 0x85C
#define AM335X_CONF_GPMC_A8 0x860
#define AM335X_CONF_GPMC_A9 0x864
#define AM335X_CONF_GPMC_A10 0x868
#define AM335X_CONF_GPMC_A11 0x86C
#define AM335X_CONF_GPMC_WAIT0 0x870
#define AM335X_CONF_GPMC_WPN 0x874
#define AM335X_CONF_GPMC_BEN1 0x878
#define AM335X_CONF_GPMC_CSN0 0x87C
#define AM335X_CONF_GPMC_CSN1 0x880
#define AM335X_CONF_GPMC_CSN2 0x884
#define AM335X_CONF_GPMC_CSN3 0x888
#define AM335X_CONF_GPMC_CLK 0x88C
#define AM335X_CONF_GPMC_ADVN_ALE 0x890
#define AM335X_CONF_GPMC_OEN_REN 0x894
#define AM335X_CONF_GPMC_WEN 0x898
#define AM335X_CONF_GPMC_BEN0_CLE 0x89C
#define AM335X_CONF_LCD_DATA0 0x8A0
#define AM335X_CONF_LCD_DATA1 0x8A4
#define AM335X_CONF_LCD_DATA2 0x8A8
#define AM335X_CONF_LCD_DATA3 0x8AC
#define AM335X_CONF_LCD_DATA4 0x8B0
#define AM335X_CONF_LCD_DATA5 0x8B4
#define AM335X_CONF_LCD_DATA6 0x8B8
#define AM335X_CONF_LCD_DATA7 0x8BC
#define AM335X_CONF_LCD_DATA8 0x8C0
#define AM335X_CONF_LCD_DATA9 0x8C4
#define AM335X_CONF_LCD_DATA10 0x8C8
#define AM335X_CONF_LCD_DATA11 0x8CC
#define AM335X_CONF_LCD_DATA12 0x8D0
#define AM335X_CONF_LCD_DATA13 0x8D4
#define AM335X_CONF_LCD_DATA14 0x8D8
#define AM335X_CONF_LCD_DATA15 0x8DC
#define AM335X_CONF_LCD_VSYNC 0x8E0
#define AM335X_CONF_LCD_HSYNC 0x8E4
#define AM335X_CONF_LCD_PCLK 0x8E8
#define AM335X_CONF_LCD_AC_BIAS_EN 0x8EC
#define AM335X_CONF_MMC0_DAT3 0x8F0
#define AM335X_CONF_MMC0_DAT2 0x8F4
#define AM335X_CONF_MMC0_DAT1 0x8F8
#define AM335X_CONF_MMC0_DAT0 0x8FC
#define AM335X_CONF_MMC0_CLK 0x900
#define AM335X_CONF_MMC0_CMD 0x904
#define AM335X_CONF_MII1_COL 0x908
#define AM335X_CONF_MII1_CRS 0x90C
#define AM335X_CONF_MII1_RX_ER 0x910
#define AM335X_CONF_MII1_TX_EN 0x914
#define AM335X_CONF_MII1_RX_DV 0x918
#define AM335X_CONF_MII1_TXD3 0x91C
#define AM335X_CONF_MII1_TXD2 0x920
#define AM335X_CONF_MII1_TXD1 0x924
#define AM335X_CONF_MII1_TXD0 0x928
#define AM335X_CONF_MII1_TX_CLK 0x92C
#define AM335X_CONF_MII1_RX_CLK 0x930
#define AM335X_CONF_MII1_RXD3 0x934
#define AM335X_CONF_MII1_RXD2 0x938
#define AM335X_CONF_MII1_RXD1 0x93C
#define AM335X_CONF_MII1_RXD0 0x940
#define AM335X_CONF_RMII1_REF_CLK 0x944
#define AM335X_CONF_MDIO 0x948
#define AM335X_CONF_MDC 0x94C
#define AM335X_CONF_SPI0_SCLK 0x950
#define AM335X_CONF_SPI0_D0 0x954
#define AM335X_CONF_SPI0_D1 0x958
#define AM335X_CONF_SPI0_CS0 0x95C
#define AM335X_CONF_SPI0_CS1 0x960
#define AM335X_CONF_ECAP0_IN_PWM0_OUT 0x964
#define AM335X_CONF_UART0_CTSN 0x968
#define AM335X_CONF_UART0_RTSN 0x96C
#define AM335X_CONF_UART0_RXD 0x970
#define AM335X_CONF_UART0_TXD 0x974
#define AM335X_CONF_UART1_CTSN 0x978
#define AM335X_CONF_UART1_RTSN 0x97C
#define AM335X_CONF_UART1_RXD 0x980
#define AM335X_CONF_UART1_TXD 0x984
#define AM335X_CONF_I2C0_SDA 0x988
#define AM335X_CONF_I2C0_SCL 0x98C
#define AM335X_CONF_MCASP0_ACLKX 0x990
#define AM335X_CONF_MCASP0_FSX 0x994
#define AM335X_CONF_MCASP0_AXR0 0x998
#define AM335X_CONF_MCASP0_AHCLKR 0x99C
#define AM335X_CONF_MCASP0_ACLKR 0x9A0
#define AM335X_CONF_MCASP0_FSR 0x9A4
#define AM335X_CONF_MCASP0_AXR1 0x9A8
#define AM335X_CONF_MCASP0_AHCLKX 0x9AC
#define AM335X_CONF_XDMA_EVENT_INTR0 0x9B0
#define AM335X_CONF_XDMA_EVENT_INTR1 0x9B4
#define AM335X_CONF_WARMRSTN 0x9B8
#define AM335X_CONF_NNMI 0x9C0
#define AM335X_CONF_TMS 0x9D0
#define AM335X_CONF_TDI 0x9D4
#define AM335X_CONF_TDO 0x9D8
#define AM335X_CONF_TCK 0x9DC
#define AM335X_CONF_TRSTN 0x9E0
#define AM335X_CONF_EMU0 0x9E4
#define AM335X_CONF_EMU1 0x9E8
#define AM335X_CONF_RTC_PWRONRSTN 0x9F8
#define AM335X_CONF_PMIC_POWER_EN 0x9FC
#define AM335X_CONF_EXT_WAKEUP 0xA00
#define AM335X_CONF_RTC_KALDO_ENN 0xA04
#define AM335X_CONF_USB0_DRVVBUS 0xA1C
#define AM335X_CONF_USB1_DRVVBUS 0xA34

/* Added by punit vara 

   PWMSS register definitions */

#define PWMSS_CLOCK_CONFIG               0x08

#define PWMSS_CLOCK_STATUS               0x0C

#define PWMSS_ECAP_CLK_EN_ACK_SHIFT      0x00

#define PWMSS_ECAP_CLK_STOP_ACK_SHIFT    0x01

#define PWMSS_EHRPWM_CLK_EN_ACK_SHIFT    0x08

#define PWMSS_EHRPWM_CLK_STOP_ACK_SHIFT  0x09  

#define PWMSS_ECAP_CLK_EN_ACK            0x01

#define PWMSS_ECAP_CLK_STOP_ACK          0x02

#define PWMSS_EHRPWM_CLK_EN_ACK          0x100

#define PWMSS_EHRPWM_CLK_STOP_ACK        0x200

/** @brief Base addresses of PWMSS memory mapped registers.                   */

#define SOC_PWMSS0_REGS                     (0x48300000)
#define SOC_PWMSS1_REGS                     (0x48302000)
#define SOC_PWMSS2_REGS                     (0x48304000)

#define SOC_ECAP_REGS                       (0x00000100)
#define SOC_EQEP_REGS                       (0x00000180)
#define SOC_EPWM_REGS                       (0x00000200)
#define SOC_ECAP_0_REGS                     (SOC_PWMSS0_REGS + SOC_ECAP_REGS)
#define SOC_ECAP_1_REGS                     (SOC_PWMSS1_REGS + SOC_ECAP_REGS)
#define SOC_ECAP_2_REGS                     (SOC_PWMSS2_REGS + SOC_ECAP_REGS)

#define SOC_EQEP_0_REGS                     (SOC_PWMSS0_REGS + SOC_EQEP_REGS)
#define SOC_EQEP_1_REGS                     (SOC_PWMSS1_REGS + SOC_EQEP_REGS)
#define SOC_EQEP_2_REGS                     (SOC_PWMSS2_REGS + SOC_EQEP_REGS) 

#define SOC_EPWM_0_REGS                     (SOC_PWMSS0_REGS + SOC_EPWM_REGS)
#define SOC_EPWM_1_REGS                     (SOC_PWMSS1_REGS + SOC_EPWM_REGS)
#define SOC_EPWM_2_REGS                     (SOC_PWMSS2_REGS + SOC_EPWM_REGS)
 
#define CONTROL_PWMSS_CTRL   (0x664)
#define CONTROL_PWMSS_CTRL_PWMSS0_TBCLKEN   (0x00000001u)
#define CONTROL_PWMSS_CTRL_PWMMS1_TBCLKEN   (0x00000002u)
#define CONTROL_PWMSS_CTRL_PWMSS2_TBCLKEN   (0x00000004u)
#define SOC_PRCM_REGS                        (0x44E00000)
#define CM_PER_L3S_CLKSTCTRL   (0x4)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP   (0x2u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL   (0x00000003u)
#define CM_PER_L3_INSTR_CLKCTRL   (0xdc)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_OCPWP_L3_CLKSTCTRL   (0x12c)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP   (0x2u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL   (0x00000003u)
#define CM_PER_L4LS_CLKSTCTRL   (0x0)
#define CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP   (0x2u)
#define CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_L4LS_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_EPWMSS0_CLKCTRL   (0xd4)
#define CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_EPWMSS1_CLKCTRL   (0xcc)
#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK   (0x00000008u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK   (0x00000010u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK   (0x00000020u)
#define CM_PER_L4LS_CLKSTCTRL   (0x0)
#define CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK   (0x00000100u)

/* TBCTL */

#define EHRPWM_TBCTL_FREE_SOFT  (0xC000u)
#define EHRPWM_TBCTL_FREE_SOFT_SHIFT (0x000Eu)

#define EHRPWM_TBCTL_PHSDIR     (0x2000u)
#define EHRPWM_TBCTL_PHSDIR_SHIFT    (0x000Du)
#define EHRPWM_TBCTL_CLKDIV     (0x1C00u)
#define EHRPWM_TBCTL_CLKDIV_SHIFT    (0x000Au)
#define EHRPWM_TBCTL_CLKDIV_DIVBY1   (0x0000u)
#define EHRPWM_TBCTL_CLKDIV_DIVBY2   (0x0001u)
#define EHRPWM_TBCTL_CLKDIV_DIVBY4   (0x0002u)
#define EHRPWM_TBCTL_CLKDIV_DIVBY8   (0x0003u)
#define EHRPWM_TBCTL_CLKDIV_DIVBY16  (0x0004u)
#define EHRPWM_TBCTL_CLKDIV_DIVBY32  (0x0005u)
#define EHRPWM_TBCTL_CLKDIV_DIVBY64  (0x0006u)
#define EHRPWM_TBCTL_CLKDIV_DIVBY128 (0x0007u)
#define EHRPWM_TBCTL_HSPCLKDIV  (0x0380u)
#define EHRPWM_TBCTL_HSPCLKDIV_SHIFT (0x0007u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY1 (0x0000u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY2 (0x0001u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY4 (0x0002u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY6 (0x0003u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY8 (0x0004u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY10 (0x0005u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY12 (0x0006u)
#define EHRPWM_TBCTL_HSPCLKDIV_DIVBY14 (0x0007u)
#define EHRPWM_TBCTL_SWFSYNC    (0x0040u)
#define EHRPWM_TBCTL_SWFSYNC_SHIFT   (0x0006u)
#define EHRPWM_TBCTL_SYNCOSEL   (0x0030u)
#define EHRPWM_TBCTL_SYNCOSEL_SHIFT  (0x0004u)
#define EHRPWM_TBCTL_SYNCOSEL_EPWMXSYNCI (0x0000u)
#define EHRPWM_TBCTL_SYNCOSEL_TBCTRZERO (0x0001u)
#define EHRPWM_TBCTL_SYNCOSEL_TBCTRCMPB (0x0002u)
#define EHRPWM_TBCTL_SYNCOSEL_DISABLE   (0x0003u)
#define EHRPWM_TBCTL_PRDLD      (0x0008u)
#define EHRPWM_TBCTL_PRDLD_SHIFT     (0x0003u)
#define EHRPWM_TBCTL_PHSEN      (0x0004u)
#define EHRPWM_TBCTL_PHSEN_SHIFT     (0x0002u)
#define EHRPWM_TBCTL_CTRMODE    (0x0003u)
#define EHRPWM_TBCTL_CTRMODE_SHIFT   (0x0000u)
#define EHRPWM_TBCTL_CTRMODE_UP      (0x0000u)
#define EHRPWM_TBCTL_CTRMODE_DOWN    (0x0001u)
#define EHRPWM_TBCTL_CTRMODE_UPDOWN  (0x0002u)
#define EHRPWM_TBCTL_CTRMODE_STOPFREEZE (0x0003u)


/* TBSTS */

#define EHRPWM_TBSTS_CTRMAX     (0x0004u)
#define EHRPWM_TBSTS_CTRMAX_SHIFT    (0x0002u)
#define EHRPWM_TBSTS_SYNCI      (0x0002u)
#define EHRPWM_TBSTS_SYNCI_SHIFT     (0x0001u)
#define EHRPWM_TBSTS_CTRDIR     (0x0001u)
#define EHRPWM_TBSTS_CTRDIR_SHIFT    (0x0000u)

/* TBPHSHR */

#define EHRPWM_TBPHSHR_TBPHSHR  (0xFF00u)
#define EHRPWM_TBPHSHR_TBPHSHR_SHIFT (0x0008u)

/* TBPHS */

#define EHRPWM_TBPHS_TBPHS      (0xFFFFu)
#define EHRPWM_TBPHS_TBPHS_SHIFT     (0x0000u)


/* TBCTR */

#define EHRPWM_TBCTR_TBCTR      (0xFFFFu)
#define EHRPWM_TBCTR_TBCTR_SHIFT     (0x0000u)
/* TBPRD */

#define EHRPWM_TBPRD_TBPRD      (0xFFFFu)
#define EHRPWM_TBPRD_TBPRD_SHIFT     (0x0000u)


/* CMPCTL */

#define EHRPWM_CMPCTL_SHDWBFULL (0x0200u)
#define EHRPWM_CMPCTL_SHDWBFULL_SHIFT (0x0009u)
#define EHRPWM_CMPCTL_SHDWAFULL (0x0100u)
#define EHRPWM_CMPCTL_SHDWAFULL_SHIFT (0x0008u)
#define EHRPWM_CMPCTL_SHDWBMODE (0x0040u)
#define EHRPWM_CMPCTL_SHDWBMODE_SHIFT (0x0006u)
#define EHRPWM_CMPCTL_SHDWAMODE (0x0010u)
#define EHRPWM_CMPCTL_SHDWAMODE_SHIFT (0x0004u)
#define EHRPWM_CMPCTL_LOADBMODE (0x000Cu)
#define EHRPWM_CMPCTL_LOADBMODE_SHIFT (0x0002u)
#define EHRPWM_CMPCTL_LOADBMODE_TBCTRZERO (0x0000u)
#define EHRPWM_CMPCTL_LOADBMODE_TBCTRPRD (0x0001u)
#define EHRPWM_CMPCTL_LOADBMODE_ZEROORPRD (0x0002u)
#define EHRPWM_CMPCTL_LOADBMODE_FREEZE (0x0003u)
#define EHRPWM_CMPCTL_LOADAMODE (0x0003u)
#define EHRPWM_CMPCTL_LOADAMODE_SHIFT (0x0000u)
#define EHRPWM_CMPCTL_LOADAMODE_TBCTRZERO (0x0000u)
#define EHRPWM_CMPCTL_LOADAMODE_TBCTRPRD (0x0001u)
#define EHRPWM_CMPCTL_LOADAMODE_ZEROORPRD (0x0002u)
#define EHRPWM_CMPCTL_LOADAMODE_FREEZE (0x0003u)


/* CMPAHR */

#define EHRPWM_CMPAHR_CMPAHR    (0xFF00u)
#define EHRPWM_CMPAHR_CMPAHR_SHIFT   (0x0008u)


/* CMPA */

#define EHRPWM_CMPA_CMPA        (0xFFFFu)
#define EHRPWM_CMPA_CMPA_SHIFT       (0x0000u)


/* CMPB */

#define EHRPWM_CMPB_CMPB        (0xFFFFu)
#define EHRPWM_CMPB_CMPB_SHIFT       (0x0000u)


/* AQCTLA */

#define EHRPWM_AQCTLA_CBD       (0x0C00u)
#define EHRPWM_AQCTLA_CBD_SHIFT      (0x000Au)
#define EHRPWM_AQCTLA_CBD_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLA_CBD_EPWMXALOW  (0x0001u)
#define EHRPWM_AQCTLA_CBD_EPWMXAHIGH (0x0002u)
#define EHRPWM_AQCTLA_CBD_EPWMXATOGGLE (0x0003u)

#define EHRPWM_AQCTLA_CBU       (0x0300u)
#define EHRPWM_AQCTLA_CBU_SHIFT      (0x0008u)
#define EHRPWM_AQCTLA_CBU_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLA_CBU_EPWMXALOW  (0x0001u)
#define EHRPWM_AQCTLA_CBU_EPWMXAHIGH (0x0002u)
#define EHRPWM_AQCTLA_CBU_EPWMXATOGGLE (0x0003u)

#define EHRPWM_AQCTLA_CAD       (0x00C0u)
#define EHRPWM_AQCTLA_CAD_SHIFT      (0x0006u)
#define EHRPWM_AQCTLA_CAD_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLA_CAD_EPWMXALOW  (0x0001u)
#define EHRPWM_AQCTLA_CAD_EPWMXAHIGH (0x0002u)
#define EHRPWM_AQCTLA_CAD_EPWMXATOGGLE (0x0003u)

#define EHRPWM_AQCTLA_CAU       (0x0030u)
#define EHRPWM_AQCTLA_CAU_SHIFT      (0x0004u)
#define EHRPWM_AQCTLA_CAU_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLA_CAU_EPWMXALOW  (0x0001u)
#define EHRPWM_AQCTLA_CAU_EPWMXAHIGH (0x0002u)
#define EHRPWM_AQCTLA_CAU_EPWMXATOGGLE (0x0003u)

#define EHRPWM_AQCTLA_PRD       (0x000Cu)
#define EHRPWM_AQCTLA_PRD_SHIFT      (0x0002u)
#define EHRPWM_AQCTLA_PRD_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLA_PRD_EPWMXALOW  (0x0001u)
#define EHRPWM_AQCTLA_PRD_EPWMXAHIGH (0x0002u)
#define EHRPWM_AQCTLA_PRD_EPWMXATOGGLE (0x0003u)

#define EHRPWM_AQCTLA_ZRO       (0x0003u)
#define EHRPWM_AQCTLA_ZRO_SHIFT      (0x0000u)
#define EHRPWM_AQCTLA_ZRO_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLA_ZRO_EPWMXALOW  (0x0001u)
#define EHRPWM_AQCTLA_ZRO_EPWMXAHIGH (0x0002u)
#define EHRPWM_AQCTLA_ZRO_EPWMXATOGGLE (0x0003u)


/* AQCTLB */

#define EHRPWM_AQCTLB_CBD       (0x0C00u)
#define EHRPWM_AQCTLB_CBD_SHIFT      (0x000Au)
#define EHRPWM_AQCTLB_CBD_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLB_CBD_EPWMXBLOW  (0x0001u)
#define EHRPWM_AQCTLB_CBD_EPWMXBHIGH (0x0002u)
#define EHRPWM_AQCTLB_CBD_EPWMXBTOGGLE (0x0003u)

#define EHRPWM_AQCTLB_CBU       (0x0300u)
#define EHRPWM_AQCTLB_CBU_SHIFT      (0x0008u)
#define EHRPWM_AQCTLB_CBU_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLB_CBU_EPWMXBLOW  (0x0001u)
#define EHRPWM_AQCTLB_CBU_EPWMXBHIGH (0x0002u)
#define EHRPWM_AQCTLB_CBU_EPWMXBTOGGLE (0x0003u)

#define EHRPWM_AQCTLB_CAD       (0x00C0u)
#define EHRPWM_AQCTLB_CAD_SHIFT      (0x0006u)
#define EHRPWM_AQCTLB_CAD_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLB_CAD_EPWMXBLOW  (0x0001u)
#define EHRPWM_AQCTLB_CAD_EPWMXBHIGH (0x0002u)
#define EHRPWM_AQCTLB_CAD_EPWMXBTOGGLE (0x0003u)
#define EHRPWM_AQCTLB_CAU       (0x0030u)
#define EHRPWM_AQCTLB_CAU_SHIFT      (0x0004u)
#define EHRPWM_AQCTLB_CAU_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLB_CAU_EPWMXBLOW  (0x0001u)
#define EHRPWM_AQCTLB_CAU_EPWMXBHIGH (0x0002u)
#define EHRPWM_AQCTLB_CAU_EPWMXBTOGGLE (0x0003u)

#define EHRPWM_AQCTLB_PRD       (0x000Cu)
#define EHRPWM_AQCTLB_PRD_SHIFT      (0x0002u)
#define EHRPWM_AQCTLB_PRD_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLB_PRD_EPWMXBLOW  (0x0001u)
#define EHRPWM_AQCTLB_PRD_EPWMXBHIGH (0x0002u)
#define EHRPWM_AQCTLB_PRD_EPWMXBTOGGLE (0x0003u)

#define EHRPWM_AQCTLB_ZRO       (0x0003u)
#define EHRPWM_AQCTLB_ZRO_SHIFT      (0x0000u)
#define EHRPWM_AQCTLB_ZRO_DONOTHING  (0x0000u)
#define EHRPWM_AQCTLB_ZRO_EPWMXBLOW  (0x0001u)
#define EHRPWM_AQCTLB_ZRO_EPWMXBHIGH (0x0002u)
#define EHRPWM_AQCTLB_ZRO_EPWMXBTOGGLE (0x0003u)


/* AQSFRC */

#define EHRPWM_AQSFRC_RLDCSF    (0x00C0u)
#define EHRPWM_AQSFRC_RLDCSF_SHIFT   (0x0006u)
#define EHRPWM_AQSFRC_RLDCSF_EVTCTRZERO (0x0000u)
#define EHRPWM_AQSFRC_RLDCSF_EVTCTRPRD (0x0001u)
#define EHRPWM_AQSFRC_RLDCSF_ZEROORPRD (0x0002u)
#define EHRPWM_AQSFRC_RLDCSF_IMMEDIATE (0x0003u)

#define EHRPWM_AQSFRC_OTSFB     (0x0020u)
#define EHRPWM_AQSFRC_OTSFB_SHIFT    (0x0005u)
#define EHRPWM_AQSFRC_OTSFB_NOEFFECT (0x0000u)
#define EHRPWM_AQSFRC_OTSFB_EVENT    (0x0001u)

#define EHRPWM_AQSFRC_ACTSFB    (0x0018u)
#define EHRPWM_AQSFRC_ACTSFB_SHIFT   (0x0003u)
#define EHRPWM_AQSFRC_ACTSFB_DONOTHING (0x0000u)
#define EHRPWM_AQSFRC_ACTSFB_CLEAR   (0x0001u)
#define EHRPWM_AQSFRC_ACTSFB_SET     (0x0002u)
#define EHRPWM_AQSFRC_ACTSFB_TOGGLE  (0x0003u)

#define EHRPWM_AQSFRC_OTSFA     (0x0004u)
#define EHRPWM_AQSFRC_OTSFA_SHIFT    (0x0002u)
#define EHRPWM_AQSFRC_OTSFA_NOEFFECT (0x0000u)
#define EHRPWM_AQSFRC_OTSFA_EVENT    (0x0001u)

#define EHRPWM_AQSFRC_ACTSFA    (0x0003u)
#define EHRPWM_AQSFRC_ACTSFA_SHIFT   (0x0000u)
#define EHRPWM_AQSFRC_ACTSFA_DONOTHING (0x0000u)
#define EHRPWM_AQSFRC_ACTSFA_CLEAR   (0x0001u)
#define EHRPWM_AQSFRC_ACTSFA_SET     (0x0002u)
#define EHRPWM_AQSFRC_ACTSFA_TOGGLE  (0x0003u)


/* AQCSFRC */

#define EHRPWM_AQCSFRC_CSFB     (0x000Cu)
#define EHRPWM_AQCSFRC_CSFB_SHIFT    (0x0002u)
#define EHRPWM_AQCSFRC_CSFB_LOW      (0x0001u)
#define EHRPWM_AQCSFRC_CSFB_HIGH     (0x0002u)

#define EHRPWM_AQCSFRC_CSFA     (0x0003u)
#define EHRPWM_AQCSFRC_CSFA_SHIFT    (0x0000u)
#define EHRPWM_AQCSFRC_CSFA_LOW      (0x0001u)
#define EHRPWM_AQCSFRC_CSFA_HIGH     (0x0002u)


/* DBCTL */

#define EHRPWM_DBCTL_IN_MODE    (0x0030u)
#define EHRPWM_DBCTL_IN_MODE_SHIFT   (0x0004u)
#define EHRPWM_DBCTL_IN_MODE_AREDAFED (0x0000u)
#define EHRPWM_DBCTL_IN_MODE_BREDAFED (0x0001u)
#define EHRPWM_DBCTL_IN_MODE_AREDBFED (0x0002u)
#define EHRPWM_DBCTL_IN_MODE_BREDBFED (0x0003u)
#define EHRPWM_DBCTL_POLSEL     (0x000Cu)
#define EHRPWM_DBCTL_POLSEL_SHIFT    (0x0002u)
#define EHRPWM_DBCTL_POLSEL_ACTIVEHIGH (0x0000u)
#define EHRPWM_DBCTL_POLSEL_ALC      (0x0001u)
#define EHRPWM_DBCTL_POLSEL_AHC      (0x0002u)
#define EHRPWM_DBCTL_POLSEL_ACTIVELOW (0x0003u)

#define EHRPWM_DBCTL_OUT_MODE   (0x0003u)
#define EHRPWM_DBCTL_OUT_MODE_SHIFT  (0x0000u)
#define EHRPWM_DBCTL_OUT_MODE_BYPASS (0x0000u)
#define EHRPWM_DBCTL_OUT_MODE_NOREDBFED (0x0001u)
#define EHRPWM_DBCTL_OUT_MODE_AREDNOFED (0x0002u)
#define EHRPWM_DBCTL_OUT_MODE_AREDBFED (0x0003u)


/* DBRED */

#define EHRPWM_DBRED_DEL        (0x03FFu)
#define EHRPWM_DBRED_DEL_SHIFT       (0x0000u)


/* DBFED */

#define EHRPWM_DBFED_DEL        (0x03FFu)
#define EHRPWM_DBFED_DEL_SHIFT       (0x0000u)


/* TZSEL */

#define EHRPWM_TZSEL_OSHT1      (0x0100u)
#define EHRPWM_TZSEL_OSHT1_SHIFT     (0x0008u)


#define EHRPWM_TZSEL_CBC1       (0x0001u)
#define EHRPWM_TZSEL_CBC1_SHIFT      (0x0000u)
/* TZCTL */

#define EHRPWM_TZCTL_TZB        (0x000Cu)
#define EHRPWM_TZCTL_TZB_SHIFT       (0x0002u)
#define EHRPWM_TZCTL_TZB_TRISTATE    (0x0000u)
#define EHRPWM_TZCTL_TZB_FORCEHIGH   (0x0001u)
#define EHRPWM_TZCTL_TZB_FORCELOW    (0x0002u)
#define EHRPWM_TZCTL_TZB_DONOTHING   (0x0003u)

#define EHRPWM_TZCTL_TZA        (0x0003u)
#define EHRPWM_TZCTL_TZA_SHIFT       (0x0000u)
#define EHRPWM_TZCTL_TZA_TRISTATE    (0x0000u)
#define EHRPWM_TZCTL_TZA_FORCEHIGH   (0x0001u)
#define EHRPWM_TZCTL_TZA_FORCELOW    (0x0002u)
#define EHRPWM_TZCTL_TZA_DONOTHING   (0x0003u)


/* TZEINT */

#define EHRPWM_TZEINT_OST       (0x0004u)
#define EHRPWM_TZEINT_OST_SHIFT      (0x0002u)

#define EHRPWM_TZEINT_CBC       (0x0002u)
#define EHRPWM_TZEINT_CBC_SHIFT      (0x0001u)



/* TZFLG */

#define EHRPWM_TZFLG_OST        (0x0004u)
#define EHRPWM_TZFLG_OST_SHIFT       (0x0002u)
#define EHRPWM_TZFLG_CBC        (0x0002u)
#define EHRPWM_TZFLG_CBC_SHIFT       (0x0001u)
#define EHRPWM_TZFLG_INT        (0x0001u)
#define EHRPWM_TZFLG_INT_SHIFT       (0x0000u)
/* TZCLR */

#define EHRPWM_TZCLR_OST        (0x0004u)
#define EHRPWM_TZCLR_OST_SHIFT       (0x0002u)
#define EHRPWM_TZCLR_CBC        (0x0002u)
#define EHRPWM_TZCLR_CBC_SHIFT       (0x0001u)
#define EHRPWM_TZCLR_INT        (0x0001u)
#define EHRPWM_TZCLR_INT_SHIFT       (0x0000u)

/* TZFRC */

#define EHRPWM_TZFRC_OST        (0x0004u)
#define EHRPWM_TZFRC_OST_SHIFT       (0x0002u)
#define EHRPWM_TZFRC_CBC        (0x0002u)
#define EHRPWM_TZFRC_CBC_SHIFT       (0x0001u)


/* ETSEL */

#define EHRPWM_ETSEL_INTEN      (0x0008u)
#define EHRPWM_ETSEL_INTEN_SHIFT     (0x0003u)

#define EHRPWM_ETSEL_INTSEL     (0x0007u)
#define EHRPWM_ETSEL_INTSEL_SHIFT    (0x0000u)
#define EHRPWM_ETSEL_INTSEL_TBCTREQUZERO (0x0001u)
#define EHRPWM_ETSEL_INTSEL_TBCTREQUPRD (0x0002u)
#define EHRPWM_ETSEL_INTSEL_TBCTREQUCMPAINC (0x0004u)
#define EHRPWM_ETSEL_INTSEL_TBCTREQUCMPADEC (0x0005u)
#define EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBINC (0x0006u)
#define EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBDEC (0x0007u)
/* ETPS */

#define EHRPWM_ETPS_INTCNT      (0x000Cu)
#define EHRPWM_ETPS_INTCNT_SHIFT     (0x0002u)
#define EHRPWM_ETPS_INTCNT_NOEVENTS  (0x0000u)
#define EHRPWM_ETPS_INTCNT_ONEEVENT  (0x0001u)
#define EHRPWM_ETPS_INTCNT_TWOEVENTS (0x0002u)
#define EHRPWM_ETPS_INTCNT_THREEEVENTS (0x0003u)

#define EHRPWM_ETPS_INTPRD      (0x0003u)
#define EHRPWM_ETPS_INTPRD_SHIFT     (0x0000u)
#define EHRPWM_ETPS_INTPRD_FIRSTEVENT (0x0001u)
#define EHRPWM_ETPS_INTPRD_SECONDEVENT (0x0002u)
#define EHRPWM_ETPS_INTPRD_THIRDEVENT (0x0003u)


/* ETFLG */

#define EHRPWM_ETFLG_INT        (0x0001u)
#define EHRPWM_ETFLG_INT_SHIFT       (0x0000u)

/* ETCLR */

#define EHRPWM_ETCLR_INT        (0x0001u)
#define EHRPWM_ETCLR_INT_SHIFT       (0x0000u)
#define EHRPWM_ETCLR_INT_NOEFFECT    (0x0000u)
#define EHRPWM_ETCLR_INT_CLEAR       (0x0001u)


/* ETFRC */

#define EHRPWM_ETFRC_INT        (0x0001u)
#define EHRPWM_ETFRC_INT_SHIFT       (0x0000u)

/* PCCTL */

#define EHRPWM_PCCTL_CHPDUTY    (0x0700u)
#define EHRPWM_PCCTL_CHPDUTY_SHIFT   (0x0008u)
#define EHRPWM_PCCTL_CHPDUTY_1DIV8   (0x0000u)
#define EHRPWM_PCCTL_CHPDUTY_2DIV8   (0x0001u)
#define EHRPWM_PCCTL_CHPDUTY_3DIV8   (0x0002u)
#define EHRPWM_PCCTL_CHPDUTY_4DIV8   (0x0003u)
#define EHRPWM_PCCTL_CHPDUTY_5DIV8   (0x0004u)
#define EHRPWM_PCCTL_CHPDUTY_6DIV8   (0x0005u)
#define EHRPWM_PCCTL_CHPDUTY_7DIV8   (0x0006u)
#define EHRPWM_PCCTL_CHPDUTY_RESERVED (0x0007u)

#define EHRPWM_PCCTL_CHPFREQ    (0x00E0u)
#define EHRPWM_PCCTL_CHPFREQ_SHIFT   (0x0005u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY1  (0x0000u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY2  (0x0001u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY3  (0x0002u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY4  (0x0003u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY5  (0x0004u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY6  (0x0005u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY7  (0x0006u)
#define EHRPWM_PCCTL_CHPFREQ_DIVBY8  (0x0007u)

#define EHRPWM_PCCTL_OSHTWTH    (0x001Eu)
#define EHRPWM_PCCTL_OSHTWTH_SHIFT   (0x0001u)

#define EHRPWM_PCCTL_CHPEN      (0x0001u)
#define EHRPWM_PCCTL_CHPEN_SHIFT     (0x0000u)

/* HR */

#define EHRPWM_HR_HRLOAD                (0x0008u)
#define EHRPWM_HR_HRLOAD_SHIFT          (0x0003u)
#define EHRPWM_HR_HRLOAD_CTR_ZERO       (0x0000u)
#define EHRPWM_HR_HRLOAD_CTR_PRD        (0x0008u)

#define EHRPWM_HR_CTLMODE               (0x0004u)
#define EHRPWM_HR_CTLMODE_SHIFT         (0x0002u)
#define EHRPWM_HR_CTLMODE_CMPAHR        (0x0000u)
#define EHRPWM_HR_CTLMODE_TBPHSHR       (0x0004u)

#define EHRPWM_HR_EDGEMODE              (0x0003u)
#define EHRPWM_HR_EDGEMODE_SHIFT        (0x0000u)
#define EHRPWM_HR_EDGEMODE_DISABLE      (0x0000u)
#define EHRPWM_HR_EDGEMODE_RAISING      (0x0001u)
#define EHRPWM_HR_EDGEMODE_FALLING      (0x0002u)
#define EHRPWM_HR_EDGEMODE_BOTH         (0x0003u)


/* REVID */

#define EHRPWM_REVID_REV        (0xFFFFFFFFu)
#define EHRPWM_REVID_REV_SHIFT       (0x00000000u)

#define EHRPWM_TBCTL            (0x0)
#define EHRPWM_TBSTS            (0x2)
#define EHRPWM_TBPHSHR          (0x4)
#define EHRPWM_TBPHS            (0x6)
#define EHRPWM_TBCTR            (0x8)
#define EHRPWM_TBPRD            (0xA)
#define EHRPWM_CMPCTL           (0xE)
#define EHRPWM_CMPAHR           (0x10)
#define EHRPWM_CMPA             (0x12)
#define EHRPWM_CMPB             (0x14)
#define EHRPWM_AQCTLA           (0x16)
#define EHRPWM_AQCTLB           (0x18)
#define EHRPWM_AQSFRC           (0x1A)
#define EHRPWM_AQCSFRC          (0x1C)
#define EHRPWM_DBCTL            (0x1E)
#define EHRPWM_DBRED            (0x20)
#define EHRPWM_DBFED            (0x22)
#define EHRPWM_TZSEL            (0x24)
#define EHRPWM_TZCTL            (0x28)
#define EHRPWM_TZEINT           (0x2A)
#define EHRPWM_TZFLG            (0x2C)
#define EHRPWM_TZCLR            (0x2E)
#define EHRPWM_TZFRC            (0x30)
#define EHRPWM_ETSEL            (0x32)
#define EHRPWM_ETPS             (0x34)
#define EHRPWM_ETFLG            (0x36)
#define EHRPWM_ETCLR            (0x38)
#define EHRPWM_ETFRC            (0x3A)
#define EHRPWM_PCCTL            (0x3C)
#define EHRPWM_HRCNFG           (0x1040)

#define CM_PER_L3_CLKSTCTRL   (0xc)

/* TB Period load */
#define EHRPWM_PRD_LOAD_SHADOW_MASK             EHRPWM_TBCTL_PRDLD

/* Counter mode */
#define EHRPWM_COUNTER_MODE_MASK                EHRPWM_TBCTL_CTRMODE 
#define EHRPWM_COUNT_UP                         (EHRPWM_TBCTL_CTRMODE_UP << \
                                                        EHRPWM_TBCTL_CTRMODE_SHIFT)
#define EHRPWM_COUNT_DOWN                       (EHRPWM_TBCTL_CTRMODE_DOWN << \
                                                        EHRPWM_TBCTL_CTRMODE_SHIFT)
#define EHRPWM_COUNT_UP_DOWN                    (EHRPWM_TBCTL_CTRMODE_UPDOWN << \
                                                        EHRPWM_TBCTL_CTRMODE_SHIFT)
#define EHRPWM_COUNT_STOP                       (EHRPWM_TBCTL_CTRMODE_STOPFREEZE << \
                                                        EHRPWM_TBCTL_CTRMODE_SHIFT)
/* Synchronization */
#define EHRPWM_SYNC_ENABLE                      EHRPWM_TBCTL_PHSEN
//#define EHRPWM_SW_FORCED_SYNC                 0x1

#define EHRPWM_SYNCOUT_MASK                     EHRPWM_TBCTL_SYNCOSEL   
#define EHRPWM_SYNCOUT_SYNCIN                   (EHRPWM_TBCTL_SYNCOSEL_EPWMXSYNCI << \
                                                        EHRPWM_TBCTL_SYNCOSEL_SHIFT)
#define EHRPWM_SYNCOUT_COUNTER_EQUAL_ZERO       (EHRPWM_TBCTL_SYNCOSEL_TBCTRZERO << \
                                                        EHRPWM_TBCTL_SYNCOSEL_SHIFT)
#define EHRPWM_SYNCOUT_COUNTER_EQUAL_COMPAREB   (EHRPWM_TBCTL_SYNCOSEL_TBCTRCMPB << \
                                                        EHRPWM_TBCTL_SYNCOSEL_SHIFT)
#define EHRPWM_SYNCOUT_DISABLE                  (EHRPWM_TBCTL_SYNCOSEL_DISABLE << \
                                                        EHRPWM_TBCTL_SYNCOSEL_SHIFT)
/* Shadow */
#define EHRPWM_SHADOW_WRITE_ENABLE              0x0
#define EHRPWM_SHADOW_WRITE_DISABLE             0x1

/* Emulation mode */
#define EHRPWM_STOP_AFTER_NEXT_TB_INCREMENT     (0x0 << EHRPWM_TBCTL_FREE_SOFT_SHIFT)
#define EHRPWM_STOP_AFTER_A_COMPLETE_CYCLE      (0x1 << EHRPWM_TBCTL_FREE_SOFT_SHIFT)
#define EHRPWM_FREE_RUN                         (0x2 << EHRPWM_TBCTL_FREE_SOFT_SHIFT)

/* Time base clock */
#define EHRPWM_TBCTL_CLKDIV_1                   (0x0001u)
#define EHRPWM_TBCTL_CLKDIV_2                   (0x0002u)
#define EHRPWM_TBCTL_CLKDIV_4                   (0x0004u)
#define EHRPWM_TBCTL_CLKDIV_8                   (0x0008u)
#define EHRPWM_TBCTL_CLKDIV_16                  (0x0010u)
#define EHRPWM_TBCTL_CLKDIV_32                  (0x0020u)
#define EHRPWM_TBCTL_CLKDIV_64                  (0x0040u)
#define EHRPWM_TBCTL_CLKDIV_128                 (0x0080u)

#define EHRPWM_TBCTL_HSPCLKDIV_1                (0x0001u)
#define EHRPWM_TBCTL_HSPCLKDIV_2                (0x0002u)
#define EHRPWM_TBCTL_HSPCLKDIV_4                (0x0004u)
#define EHRPWM_TBCTL_HSPCLKDIV_6                (0x0006u)
#define EHRPWM_TBCTL_HSPCLKDIV_8                (0x0008u)
#define EHRPWM_TBCTL_HSPCLKDIV_10               (0x000Au)
#define EHRPWM_TBCTL_HSPCLKDIV_12               (0x000Cu)
#define EHRPWM_TBCTL_HSPCLKDIV_14               (0x000Eu)

/* Count direction after sync */
#define EHRPWM_COUNT_DOWN_AFTER_SYNC            0x0
#define EHRPWM_COUNT_UP_AFTER_SYNC              0x1
/* Counter Compare */
#define EHRPWM_SHADOW_A_EMPTY                   (0x0 << EHRPWM_CMPCTL_SHDWAFULL_SHIFT)
#define EHRPWM_SHADOW_A_FULL                    (EHRPWM_CMPCTL_SHDWAFULL)
#define EHRPWM_SHADOW_B_EMPTY                   (0x0 << EHRPWM_CMPCTL_SHDWBFULL_SHIFT)
#define EHRPWM_SHADOW_B_FULL                    (EHRPWM_CMPCTL_SHDWBFULL)

#define EHRPWM_CMPCTL_NOT_OVERWR_SH_FL          0x0
#define EHRPWM_CMPCTL_OVERWR_SH_FL              0x1

/* Compare register load */
#define EHRPWM_COMPB_LOAD_MASK                  EHRPWM_CMPCTL_LOADBMODE
#define EHRPWM_COMPB_LOAD_COUNT_EQUAL_ZERO      (EHRPWM_CMPCTL_LOADBMODE_TBCTRZERO << \
                                                        EHRPWM_CMPCTL_LOADBMODE_SHIFT)
#define EHRPWM_COMPB_LOAD_COUNT_EQUAL_PERIOD    (EHRPWM_CMPCTL_LOADBMODE_TBCTRPRD << \
                                                        EHRPWM_CMPCTL_LOADBMODE_SHIFT)
#define EHRPWM_COMPB_LOAD_COUNT_EQUAL_ZERO_OR_PERIOD \
                                                (EHRPWM_CMPCTL_LOADBMODE_ZEROORPRD << \
                                                        EHRPWM_CMPCTL_LOADBMODE_SHIFT)
#define EHRPWM_COMPB_NO_LOAD                    (EHRPWM_CMPCTL_LOADBMODE_FREEZE << \
                                                        EHRPWM_CMPCTL_LOADBMODE_SHIFT)


#define EHRPWM_COMPA_LOAD_MASK                  EHRPWM_CMPCTL_LOADAMODE
#define EHRPWM_COMPA_LOAD_COUNT_EQUAL_ZERO      (EHRPWM_CMPCTL_LOADAMODE_TBCTRZERO << \
                                                        EHRPWM_CMPCTL_LOADAMODE_SHIFT)
#define EHRPWM_COMPA_LOAD_COUNT_EQUAL_PERIOD    (EHRPWM_CMPCTL_LOADAMODE_TBCTRPRD << \
                                                        EHRPWM_CMPCTL_LOADAMODE_SHIFT)
#define EHRPWM_COMPA_LOAD_COUNT_EQUAL_ZERO_OR_PERIOD \
                                                (EHRPWM_CMPCTL_LOADAMODE_ZEROORPRD << \
                                                        EHRPWM_CMPCTL_LOADAMODE_SHIFT)                                           
#define EHRPWM_COMPA_NO_LOAD                    (EHRPWM_CMPCTL_LOADAMODE_FREEZE << \
                                                        EHRPWM_CMPCTL_LOADAMODE_SHIFT)
/* Chopper */
#define EHRPWM_CHP_DUTY_12_5_PER                EHRPWM_PCCTL_CHPDUTY_1DIV8
#define EHRPWM_CHP_DUTY_25_PER                  EHRPWM_PCCTL_CHPDUTY_2DIV8
#define EHRPWM_CHP_DUTY_37_5_PER                EHRPWM_PCCTL_CHPDUTY_3DIV8
#define EHRPWM_CHP_DUTY_50_PER                  EHRPWM_PCCTL_CHPDUTY_4DIV8
#define EHRPWM_CHP_DUTY_62_5_PER                EHRPWM_PCCTL_CHPDUTY_5DIV8
#define EHRPWM_CHP_DUTY_75_PER                  EHRPWM_PCCTL_CHPDUTY_6DIV8
#define EHRPWM_CHP_DUTY_87_5_PER                EHRPWM_PCCTL_CHPDUTY_7DIV8

/* TZ */
#define EHRPWM_TZ_ONESHOT                       0x0
#define EHRPWM_TZ_CYCLEBYCYCLE                  0x1
#define EHRPWM_TZ_ONESHOT_CLEAR                 (EHRPWM_TZCLR_OST | EHRPWM_TZCLR_INT)
#define EHRPWM_TZ_CYCLEBYCYCLE_CLEAR            (EHRPWM_TZCLR_CBC | EHRPWM_TZCLR_INT)

#define  ECAP   0x01
#define  EPWM   0x02
#define  EQEP   0x03

#define CM_PER_L4LS_CLKSTCTRL   (0x0)
#define CM_PER_L3S_CLKSTCTRL   (0x4)
#define CM_PER_L3_CLKSTCTRL   (0xc)
#define CM_PER_L4LS_CLKCTRL   (0x60)
#define CM_PER_EPWMSS1_CLKCTRL   (0xcc)
#define CM_PER_EPWMSS0_CLKCTRL   (0xd4)
#define CM_PER_EPWMSS2_CLKCTRL   (0xd8)
#define CM_PER_L3_INSTR_CLKCTRL   (0xdc)
#define CM_PER_L3_CLKCTRL   (0xe0)
#define CM_PER_OCPWP_L3_CLKSTCTRL   (0x12c)
#define CM_PER_OCPWP_CLKCTRL   (0x130)

/**************************************************************************
 * Field Definition Macros
\**************************************************************************/

#define CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL   (0x00000003u)
#define CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SHIFT   (0x00000000u)
#define CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_HW_AUTO   (0x3u)
#define CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_NO_SLEEP   (0x0u)
#define CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_SLEEP   (0x1u)
#define CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP   (0x2u)


/* L3S_CLKSTCTRL */
#define CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK   (0x00000008u)
#define CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK_SHIFT   (0x00000003u)
#define CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK_ACT   (0x1u)
#define CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK_INACT   (0x0u)

#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL   (0x00000003u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SHIFT   (0x00000000u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_HW_AUTO   (0x3u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_NO_SLEEP   (0x0u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_SLEEP   (0x1u)
#define CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP   (0x2u)

/* L3_CLKSTCTRL */
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_CPTS_RFT_GCLK   (0x00000040u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_CPTS_RFT_GCLK_SHIFT   (0x00000006u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_CPTS_RFT_GCLK_ACT   (0x1u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_CPTS_RFT_GCLK_INACT   (0x0u)

#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK   (0x00000004u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK_SHIFT   (0x00000002u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK_ACT   (0x1u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_EMIF_GCLK_INACT   (0x0u)

#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK   (0x00000010u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK_SHIFT   (0x00000004u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK_ACT   (0x1u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK_INACT   (0x0u)

#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MCASP_GCLK   (0x00000080u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MCASP_GCLK_SHIFT   (0x00000007u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MCASP_GCLK_ACT   (0x1u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MCASP_GCLK_INACT   (0x0u)

#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MMC_FCLK   (0x00000008u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MMC_FCLK_SHIFT   (0x00000003u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MMC_FCLK_ACT   (0x1u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_MMC_FCLK_INACT   (0x0u)

#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL   (0x00000003u)
#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SHIFT   (0x00000000u)
#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL_HW_AUTO   (0x3u)
#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL_NO_SLEEP   (0x0u)
#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_SLEEP   (0x1u)
#define CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP   (0x2u)

/* L4LS_CLKCTRL */
#define CM_PER_L4LS_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_L4LS_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_L4LS_CLKCTRL_IDLEST_DISABLE   (0x3u)
#define CM_PER_L4LS_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_L4LS_CLKCTRL_IDLEST_IDLE   (0x2u)
#define CM_PER_L4LS_CLKCTRL_IDLEST_TRANS   (0x1u)

#define CM_PER_L4LS_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_L4LS_CLKCTRL_MODULEMODE_SHIFT   (0x00000000u)
#define CM_PER_L4LS_CLKCTRL_MODULEMODE_DISABLED   (0x0u)
#define CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_L4LS_CLKCTRL_MODULEMODE_RESERVED   (0x3u)
#define CM_PER_L4LS_CLKCTRL_MODULEMODE_RESERVED_1   (0x1u)

/* EPWMSS1_CLKCTRL */
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST_DISABLE   (0x3u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST_IDLE   (0x2u)
#define CM_PER_EPWMSS1_CLKCTRL_IDLEST_TRANS   (0x1u)

#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_SHIFT   (0x00000000u)
#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_DISABLED   (0x0u)
#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_RESERVED   (0x3u)
#define CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_RESERVED_1   (0x1u)

/* EPWMSS0_CLKCTRL */
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST_DISABLED   (0x3u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST_IDLE   (0x2u)
#define CM_PER_EPWMSS0_CLKCTRL_IDLEST_TRANS   (0x1u)

#define CM_PER_EPWMSS0_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_SHIFT   (0x00000000u)
#define CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_DISABLE   (0x0u)
#define CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_RESERVED   (0x3u)
#define CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_RESERVED_1   (0x1u)

/* EPWMSS2_CLKCTRL */
#define CM_PER_EPWMSS2_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_EPWMSS2_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_EPWMSS2_CLKCTRL_IDLEST_DISABLE   (0x3u)
#define CM_PER_EPWMSS2_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_EPWMSS2_CLKCTRL_IDLEST_IDLE   (0x2u)
#define CM_PER_EPWMSS2_CLKCTRL_IDLEST_TRANS   (0x1u)

#define CM_PER_EPWMSS2_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_SHIFT   (0x00000000u)
#define CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_DISABLED   (0x0u)
#define CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_RESERVED   (0x3u)
#define CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_RESERVED_1   (0x1u)

/* L3_INSTR_CLKCTRL */
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST_DISABLE   (0x3u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST_IDLE   (0x2u)
#define CM_PER_L3_INSTR_CLKCTRL_IDLEST_TRANS   (0x1u)

#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_SHIFT   (0x00000000u)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_DISABLED   (0x0u)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_RESERVED   (0x3u)
#define CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_RESERVED_1   (0x1u)

/* L3_CLKCTRL */
#define CM_PER_L3_CLKCTRL_IDLEST   (0x00030000u)
#define CM_PER_L3_CLKCTRL_IDLEST_SHIFT   (0x00000010u)
#define CM_PER_L3_CLKCTRL_IDLEST_DISABLE   (0x3u)
#define CM_PER_L3_CLKCTRL_IDLEST_FUNC   (0x0u)
#define CM_PER_L3_CLKCTRL_IDLEST_IDLE   (0x2u)
#define CM_PER_L3_CLKCTRL_IDLEST_TRANS   (0x1u)

#define CM_PER_L3_CLKCTRL_MODULEMODE   (0x00000003u)
#define CM_PER_L3_CLKCTRL_MODULEMODE_SHIFT   (0x00000000u)
#define CM_PER_L3_CLKCTRL_MODULEMODE_DISABLED   (0x0u)
#define CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE   (0x2u)
#define CM_PER_L3_CLKCTRL_MODULEMODE_RESERVED   (0x3u)
#define CM_PER_L3_CLKCTRL_MODULEMODE_RESERVED_1   (0x1u)

/* OCPWP_L3_CLKSTCTRL */
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK   (0x00000010u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK_SHIFT   (0x00000004u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK_ACT   (0x1u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK_INACT   (0x0u)

#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK   (0x00000020u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK_SHIFT   (0x00000005u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK_ACT   (0x1u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK_INACT   (0x0u)

#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL   (0x00000003u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SHIFT   (0x00000000u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_HW_AUTO   (0x3u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_NO_SLEEP   (0x0u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_SLEEP   (0x1u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP   (0x2u)

#define CONTROL_CONF_GPMC_AD(n)   (0x800 + (n * 4))
#define CONTROL_CONF_LCD_DATA(n)   (0x8a0 + (n * 4))
#define EHRPWM_TBCNT	0x8

#define BBBIO_CONTROL_MODULE 0x44e10000
#define BBBIO_PWMSS_CTRL     0x664
#define BBBIO_CM_PER_ADDR    0x44e00000
#define BBBIO_CM_PER_EPWMSS1_CLKCTRL 0xcc
#define BBBIO_CM_PER_EPWMSS0_CLKCTRL 0xd4
#define BBBIO_CM_PER_EPWMSS2_CLKCTRL 0xd8

#define PWMSS0_MMAP_ADDR	0x48300000
#define PWMSS1_MMAP_ADDR	0x48302000
#define PWMSS2_MMAP_ADDR	0x48304000

#define BBBIO_PWMSS_CTRL_PWMSS0_TBCLKEN   (0x00000001u)
#define BBBIO_PWMSS_CTRL_PWMSS1_TBCLKEN   (0x00000002u)
#define BBBIO_PWMSS_CTRL_PWMSS2_TBCLKEN   (0x00000004u)

#define PWMSS_CLKCONFIG	0x8
#define PWMSS_CLK_EN_ACK          0x100

