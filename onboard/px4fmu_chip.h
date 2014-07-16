/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    Ported from nuttx\include\arch\chip\chip.h
                     
    Copyright (C) 2014  BenZeng
*/
#ifndef __PX4FMU_STM32F247_CHIP_H
#define __PX4FMU_STM32F247_CHIP_H




//#define CONFIG_ARCH_CHIP_STM32F427V
//#define CONFIG_STM32_STM32F40XX
//#define CONFIG_STM32_STM32F427


#  undef  CONFIG_STM32_STM32L15XX            /* STM32L151xx and STM32L152xx family */
#  undef  CONFIG_STM32_ENERGYLITE            /* STM32L EnergyLite vamily */
#  undef  CONFIG_STM32_STM32F10XX            /* STM32F10xxx family */
#  undef  CONFIG_STM32_LOWDENSITY            /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 16/32 Kbytes */
#  undef  CONFIG_STM32_MEDIUMDENSITY         /* STM32F100x, STM32F101x, STM32F102x and STM32F103x w/ 64/128 Kbytes */
#  undef  CONFIG_STM32_HIGHDENSITY           /* STM32F100x, STM32F101x, and STM32F103x w/ 256/512 Kbytes */
#  undef  CONFIG_STM32_VALUELINE             /* STM32F100x */
#  undef  CONFIG_STM32_CONNECTIVITYLINE      /* STM32F105x and STM32F107x */
#  undef  CONFIG_STM32_STM32F20XX            /* STM32F205x and STM32F207x */
#  undef  CONFIG_STM32_STM32F30XX            /* STM32F30xxx family */
#  define CONFIG_STM32_STM32F40XX        1   /* STM32F405xx, STM32407xx and STM32F427/437 */
#  define STM32_NFSMC                    1   /* FSMC */
#  define STM32_NATIM                    2   /* Two advanced timers TIM1 and 8 */
#  define STM32_NGTIM                    4   /* 16-bit general timers TIM3 and 4 with DMA
                                              * 32-bit general timers TIM2 and 5 with DMA */
#  define STM32_NGTIMNDMA                6   /* 16-bit general timers TIM9-14 without DMA */
#  define STM32_NBTIM                    2   /* Two basic timers, TIM6-7 */
#  define STM32_NDMA                     2   /* DMA1-2 */
#  define STM32_NSPI                     4   /* SPI1-4 */
#  define STM32_NI2S                     2   /* I2S1-2 (multiplexed with SPI2-3) */
#  define STM32_NUSART                   8   /* USART1-3 and 6, UART 4-5 and 7-8 */
#  define STM32_NI2C                     3   /* I2C1-3 */
#  define STM32_NCAN                     2   /* CAN1-2 */
#  define STM32_NSDIO                    1   /* SDIO */
#  define STM32_NLCD                     0   /* No LCD */
#  define STM32_NUSBOTG                  1   /* USB OTG FS/HS */
#  define STM32_NGPIO                    139 /* GPIOA-I */
#  define STM32_NADC                     3   /* 12-bit ADC1-3, 24 channels */
#  define STM32_NDAC                     2   /* 12-bit DAC1-2 */
#  define STM32_NCAPSENSE                0   /* No capacitive sensing channels */
#  define STM32_NCRC                     1   /* CRC */
#  define STM32_NETHERNET                1   /* 100/100 Ethernet MAC */
#  define STM32_NRNG                     1   /* Random number generator (RNG) */
#  define STM32_NDCMI                    1   /* Digital camera interface (DCMI) */

//
// Memory Map
//
/* STM32F40XXX Address Blocks *******************************************************/

#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32_FSMC_BASE12    0x60000000     /* 0x60000000-0x7fffffff: 512Mb FSMC bank1&2 block */
#  define STM32_FSMC_BANK1   0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/SRAM */
#  define STM32_FSMC_BANK2   0x70000000     /* 0x70000000-0x7fffffff: 256Mb NAND FLASH */
#define STM32_FSMC_BASE34    0x80000000     /* 0x80000000-0x8fffffff: 512Mb FSMC bank3&4 block */
#  define STM32_FSMC_BANK3   0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#  define STM32_FSMC_BANK4   0x90000000     /* 0x90000000-0x9fffffff: 256Mb PC CARD*/
#define STM32_FSMC_BASE      0xa0000000     /* 0xa0000000-0xbfffffff: 512Mb FSMC register block */
                                            /* 0xc0000000-0xdfffffff: 512Mb (not used) */
#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32_REGION_MASK    0xf0000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FSMC_BANK1)

/* Code Base Addresses **************************************************************/

#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
                                            /* 0x00100000-0x07ffffff: Reserved */
#define STM32_FLASH_BASE     0x08000000     /* 0x08000000-0x080fffff: FLASH memory */
                                            /* 0x08100000-0x0fffffff: Reserved */
#define STM32_CCMRAM_BASE    0x10000000     /* 0x10000000-0x1000ffff: 64Kb CCM data RAM */
                                            /* 0x10010000-0x1ffeffff: Reserved */
#define STM32_SYSMEM_BASE    0x1fff0000     /* 0x1fff0000-0x1fff7a0f: System memory */
                                            /* 0x1fff7a10-0x1fff7fff: Reserved */
#define STM32_OPTION_BASE    0x1fffc000     /* 0x1fffc000-0x1fffc007: Option bytes */
                                            /* 0x1fffc008-0x1fffffff: Reserved */

/* SRAM Base Addresses **************************************************************/

                                            /* 0x20000000-0x2001bfff: 112Kb aliased by bit-banding */
                                            /* 0x2001c000-0x2001ffff: 16Kb aliased by bit-banding */
#define STM32_SRAMBB_BASE    0x22000000     /* 0x22000000-          : SRAM bit-band region */

/* Peripheral Base Addresses ********************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x400023ff: APB1 */
                                            /* 0x40002400-0x400027ff: Reserved */
                                            /* 0x40002800-0x400077ff: APB1 */
                                            /* 0x40007800-0x4000ffff: Reserved */
#define STM32_APB2_BASE      0x40010000     /* 0x40010000-0x400023ff: APB2 */
                                            /* 0x40013400-0x400137ff: Reserved */
                                            /* 0x40013800-0x40013bff: SYSCFG */
#define STM32_EXTI_BASE      0x40013c00     /* 0x40013c00-0x40013fff: EXTI */
                                            /* 0x40014000-0x40014bff: APB2 */
                                            /* 0x40014c00-0x4001ffff: Reserved */
#define STM32_AHB1_BASE      0x40020000     /* 0x40020000-0x400223ff: APB1 */
                                            /* 0x40022400-0x40022fff: Reserved */
                                            /* 0x40023000-0x400233ff: CRC */
                                            /* 0x40023400-0x400237ff: Reserved */
                                            /* 0x40023800-0x40023bff: Reset and Clock control RCC */
                                            /* 0x40023c00-0x400293ff: AHB1 (?) */
                                            /* 0x40029400-0x4fffffff: Reserved (?) */
#define STM32_AHB2_BASE      0x50000000     /* 0x50000000-0x5003ffff: AHB2 */
                                            /* 0x50040000-0x5004ffff: Reserved */
                                            /* 0x50050000-0x500503ff: AHB2 */
                                            /* 0x50050400-0x500607ff: Reserved */
                                            /* 0x50060800-0x50060bff: AHB2 */
                                            /* 0x50060c00-0x5fffffff: Reserved */

/* FSMC Base Addresses **************************************************************/

#define STM32_AHB3_BASE      0x60000000     /* 0x60000000-0xa0000fff: AHB3 */

/* APB1 Base Addresses **************************************************************/

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff: TIM2 timer */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff: TIM3 timer */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff: TIM4 timer */
#define STM32_TIM5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TIM5 timer */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff: TIM6 timer */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff: TIM7 timer */
#define STM32_TIM12_BASE     0x40001800     /* 0x40001800-0x40001bff: TIM12 timer */
#define STM32_TIM13_BASE     0x40001c00     /* 0x40001c00-0x40001fff: TIM13 timer */
#define STM32_TIM14_BASE     0x40002000     /* 0x40002000-0x400023ff: TIM14 timer */
#define STM32_RTC_BASE       0x40002800     /* 0x40002800-0x40002bff: RTC & BKP registers */
#define STM32_BKP_BASE       0x40002850
#define STM32_WWDG_BASE      0x40002c00     /* 0x40002c00-0x40002fff: Window watchdog (WWDG) */
#define STM32_IWDG_BASE      0x40003000     /* 0x40003000-0x400033ff: Independent watchdog (IWDG) */
#define STM32_I2S2EXT_BASE   0x40003400     /* 0x40003400-0x400037ff: I2S2ext */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2/I2S2 */
#define STM32_I2S2_BASE      0x40003800
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3/I2S3 */
#define STM32_I2S3_BASE      0x40003c00
#define STM32_I2S3EXT_BASE   0x40004000     /* 0x40003400-0x400043ff: I2S3ext */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005Bff: I2C2 */
#define STM32_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
#define STM32_CAN1_BASE      0x40006400     /* 0x40006400-0x400067ff: bxCAN1 */
#define STM32_CAN2_BASE      0x40006800     /* 0x40006800-0x40006bff: bxCAN2 */
#define STM32_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff: Power control PWR */
#define STM32_DAC_BASE       0x40007400     /* 0x40007400-0x400077ff: DAC */
#define STM32_UART7_BASE     0x40007800     /* 0x40007800-0x40007bff: UART7 */
#define STM32_UART8_BASE     0x40007c00     /* 0x40007c00-0x40007fff: UART8 */

/* APB2 Base Addresses **************************************************************/

#define STM32_TIM1_BASE      0x40010000     /* 0x40010000-0x400103ff: TIM1 timer */
#define STM32_TIM8_BASE      0x40010400     /* 0x40010400-0x400107ff: TIM8 timer */
#define STM32_USART1_BASE    0x40011000     /* 0x40011000-0x400113ff: USART1 */
#define STM32_USART6_BASE    0x40011400     /* 0x40011400-0x400117ff: USART6 */
#define STM32_ADC_BASE       0x40012000     /* 0x40012000-0x400123ff: ADC1-3 */
#  define STM32_ADC1_BASE    0x40012000     /*                        ADC1 */
#  define STM32_ADC2_BASE    0x40012100     /*                        ADC2 */
#  define STM32_ADC3_BASE    0x40012200     /*                        ADC3 */
#  define STM32_ADCCMN_BASE  0x40012300     /*                        Common */
#define STM32_SDIO_BASE      0x40012c00     /* 0x40012c00-0x40012fff: SDIO  */
#define STM32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff: SPI1 */
#define STM32_SPI4_BASE      0x40013400     /* 0x40013000-0x400137ff: SPI4 */
#define STM32_SYSCFG_BASE    0x40013800     /* 0x40013800-0x40013bff: SYSCFG */
#define STM32_EXTI_BASE      0x40013c00     /* 0x40013c00-0x40013fff: EXTI */
#define STM32_TIM9_BASE      0x40014000     /* 0x40014000-0x400143ff: TIM9 timer */
#define STM32_TIM10_BASE     0x40014400     /* 0x40014400-0x400147ff: TIM10 timer */
#define STM32_TIM11_BASE     0x40014800     /* 0x40014800-0x40014bff: TIM11 timer */
#define STM32_SPI5_BASE      0x40015000     /* 0x40015000-0x400153ff: SPI5 */
#define STM32_SPI6_BASE      0x40015400     /* 0x40015400-0x400157ff: SPI6 */

/* AHB1 Base Addresses **************************************************************/

#define STM32_GPIOA_BASE     0x40020000     /* 0x40020000-0x400203ff: GPIO Port A */
#define STM32_GPIOB_BASE     0x40020400     /* 0x40020400-0x400207ff: GPIO Port B */
#define STM32_GPIOC_BASE     0x40020800     /* 0x40020800-0x40020bff: GPIO Port C */
#define STM32_GPIOD_BASE     0X40020C00     /* 0x40020c00-0x40020fff: GPIO Port D */
#define STM32_GPIOE_BASE     0x40021000     /* 0x40021000-0x400213ff: GPIO Port E */
#define STM32_GPIOF_BASE     0x40021400     /* 0x40021400-0x400217ff: GPIO Port F */
#define STM32_GPIOG_BASE     0x40021800     /* 0x40021800-0x40021bff: GPIO Port G */
#define STM32_GPIOH_BASE     0x40021C00     /* 0x40021C00-0x40021fff: GPIO Port H */
#define STM32_GPIOI_BASE     0x40022000     /* 0x40022000-0x400223ff: GPIO Port I */
#define STM32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */
#define STM32_RCC_BASE       0x40023800     /* 0x40023800-0x40023bff: Reset and Clock control RCC */
#define STM32_FLASHIF_BASE   0x40023c00     /* 0x40023c00-0x40023fff: Flash memory interface */
#define STM32_BKPSRAM_BASE   0x40024000     /* 0x40024000-0x40024fff: Backup SRAM (BKPSRAM) */
#define STM32_DMA1_BASE      0x40026000     /* 0x40026000-0x400263ff: DMA1  */
#define STM32_DMA2_BASE      0x40026400     /* 0x40026400-0x400267ff: DMA2  */
#define STM32_ETHERNET_BASE  0x40028000     /* 0x40028000-0x400283ff: Ethernet MAC */
                                            /* 0x40028400-0x400287ff: Ethernet MAC */
                                            /* 0x40028800-0x40028bff: Ethernet MAC */
                                            /* 0x40028c00-0x40028fff: Ethernet MAC */
                                            /* 0x40029000-0x400293ff: Ethernet MAC */
#define STM32_OTGHS_BASE     0x40040000     /* 0x40040000-0x4007ffff: USB OTG HS */
#define STM32_PERIPHBB_BASE  0x42000000     /* Peripheral bit-band region */

/* AHB2 Base Addresses **************************************************************/

#define STM32_OTGFS_BASE     0x50000000     /* 0x50000000-0x5003ffff: USB OTG FS */
#define STM32_DCMI_BASE      0x50050000     /* 0x50050000-0x500503ff: DCMI */
#define STM32_CRYP_BASE      0x50060000     /* 0x50060000-0x500603ff: CRYP */
#define STM32_HASH_BASE      0x50060400     /* 0x50060400-0x500607ff: HASH */
#define STM32_RNG_BASE       0x50060800     /* 0x50060800-0x50060bff: RNG */

/* Cortex-M4 Base Addresses *********************************************************/
/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in this
 * address range
 */

#define STM32_SCS_BASE      0xe000e000
#define STM32_DEBUGMCU_BASE 0xe0042000

//
// System config
//
/* Register Offsets *********************************************************************************/

#define STM32_SYSCFG_MEMRMP_OFFSET    0x0000 /* SYSCFG memory remap register */
#define STM32_SYSCFG_PMC_OFFSET       0x0004 /* SYSCFG peripheral mode configuration register */

#define STM32_SYSCFG_EXTICR_OFFSET(p) (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32_SYSCFG_EXTICR1_OFFSET   0x0008 /* SYSCFG external interrupt configuration register 1 */
#define STM32_SYSCFG_EXTICR2_OFFSET   0x000c /* SYSCFG external interrupt configuration register 2 */
#define STM32_SYSCFG_EXTICR3_OFFSET   0x0010 /* SYSCFG external interrupt configuration register 3 */
#define STM32_SYSCFG_EXTICR4_OFFSET   0x0014 /* SYSCFG external interrupt configuration register 4 */

#define STM32_SYSCFG_CMPCR_OFFSET     0x0020 /* Compensation cell control register */

/* Register Addresses *******************************************************************************/

#define STM32_SYSCFG_MEMRMP           (STM32_SYSCFG_BASE+STM32_SYSCFG_MEMRMP_OFFSET)
#define STM32_SYSCFG_PMC              (STM32_SYSCFG_BASE+STM32_SYSCFG_PMC_OFFSET)

#define STM32_SYSCFG_EXTICR(p)        (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR_OFFSET(p))
#define STM32_SYSCFG_EXTICR1          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR1_OFFSET)
#define STM32_SYSCFG_EXTICR2          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR2_OFFSET)
#define STM32_SYSCFG_EXTICR3          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR3_OFFSET)
#define STM32_SYSCFG_EXTICR4          (STM32_SYSCFG_BASE+STM32_SYSCFG_EXTICR4_OFFSET)

#define STM32_SYSCFG_CMPCR            (STM32_SYSCFG_BASE+STM32_SYSCFG_CMPCR_OFFSET)

/* Register Bitfield Definitions ********************************************************************/

/* SYSCFG memory remap register */

#define SYSCFG_MEMRMP_SHIFT           (0)       /* Bits 1:0 MEM_MODE: Memory mapping selection */
#define SYSCFG_MEMRMP_MASK            (3 << SYSCFG_MEMRMP_SHIFT)
#  define SYSCFG_MEMRMP_FLASH         (0 << SYSCFG_MEMRMP_SHIFT) /* 00: Main Flash memory mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_SYSTEM        (1 << SYSCFG_MEMRMP_SHIFT) /* 01: System Flash memory mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_FSMC          (2 << SYSCFG_MEMRMP_SHIFT) /* 10: FSMC Bank1 (NOR/PSRAM 1 and 2) mapped at 0x0000 0000 */
#  define SYSCFG_MEMRMP_SRAM          (3 << SYSCFG_MEMRMP_SHIFT) /* 11: Embedded SRAM (112kB) mapped at 0x0000 0000 */

/* SYSCFG peripheral mode configuration register */
#undef SYSCFG_PMC_MII_RMII_SEL
#define SYSCFG_PMC_MII_RMII_SEL       (1 << 23) /* Bit 23: Ethernet PHY interface selection */
#ifdef CONFIG_STM32_STM32F427
#  define SYSCFG_PMC_ADC3DC2          (1 << 18) /* Bit 18: See AN4073 */
#  define SYSCFG_PMC_ADC2DC2          (1 << 17) /* Bit 17: See AN4073 */
#  define SYSCFG_PMC_ADC1DC2          (1 << 16) /* Bit 16: See AN4073 */
#endif

/* SYSCFG external interrupt configuration register 1-4 */

#define SYSCFG_EXTICR_PORTA           (0)       /* 0000: PA[x] pin */
#define SYSCFG_EXTICR_PORTB           (1)       /* 0001: PB[x] pin */
#define SYSCFG_EXTICR_PORTC           (2)       /* 0010: PC[x] pin */
#define SYSCFG_EXTICR_PORTD           (3)       /* 0011: PD[x] pin */
#define SYSCFG_EXTICR_PORTE           (4)       /* 0100: PE[x] pin */
#define SYSCFG_EXTICR_PORTF           (5)       /* 0101: PF[C] pin */
#define SYSCFG_EXTICR_PORTG           (6)       /* 0110: PG[x] pin */
#define SYSCFG_EXTICR_PORTH           (7)       /* 0111: PH[x] pin */
#define SYSCFG_EXTICR_PORTI           (8)       /* 1000: PI[x] pin */

#define SYSCFG_EXTICR_PORT_MASK       (15)
#define SYSCFG_EXTICR_EXTI_SHIFT(g)   (((g) & 3) << 2)
#define SYSCFG_EXTICR_EXTI_MASK(g)    (SYSCFG_EXTICR_PORT_MASK << (SYSCFG_EXTICR_EXTI_SHIFT(g)))

#define SYSCFG_EXTICR1_EXTI0_SHIFT    (0)       /* Bits 0-3: EXTI 0 coinfiguration */
#define SYSCFG_EXTICR1_EXTI0_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI0_SHIFT)
#define SYSCFG_EXTICR1_EXTI1_SHIFT    (4)       /* Bits 4-7: EXTI 1 coinfiguration */
#define SYSCFG_EXTICR1_EXTI1_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI1_SHIFT)
#define SYSCFG_EXTICR1_EXTI2_SHIFT    (8)       /* Bits 8-11: EXTI 2 coinfiguration */
#define SYSCFG_EXTICR1_EXTI2_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI2_SHIFT)
#define SYSCFG_EXTICR1_EXTI3_SHIFT    (12)      /* Bits 12-15: EXTI 3 coinfiguration */
#define SYSCFG_EXTICR1_EXTI3_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR1_EXTI3_SHIFT)

#define SYSCFG_EXTICR2_EXTI4_SHIFT    (0)       /* Bits 0-3: EXTI 4 coinfiguration */
#define SYSCFG_EXTICR2_EXTI4_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI5_SHIFT    (4)       /* Bits 4-7: EXTI 5 coinfiguration */
#define SYSCFG_EXTICR2_EXTI5_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI6_SHIFT    (8)       /* Bits 8-11: EXTI 6 coinfiguration */
#define SYSCFG_EXTICR2_EXTI6_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI7_SHIFT    (12)      /* Bits 12-15: EXTI 7 coinfiguration */
#define SYSCFG_EXTICR2_EXTI7_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR2_EXTI7_SHIFT)

#define SYSCFG_EXTICR3_EXTI8_SHIFT    (0)       /* Bits 0-3: EXTI 8 coinfiguration */
#define SYSCFG_EXTICR3_EXTI8_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI9_SHIFT    (4)       /* Bits 4-7: EXTI 9 coinfiguration */
#define SYSCFG_EXTICR3_EXTI9_MASK     (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI10_SHIFT   (8)       /* Bits 8-11: EXTI 10 coinfiguration */
#define SYSCFG_EXTICR3_EXTI10_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI11_SHIFT   (12)      /* Bits 12-15: EXTI 11 coinfiguration */
#define SYSCFG_EXTICR3_EXTI11_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR3_EXTI11_SHIFT)

#define SYSCFG_EXTICR4_EXTI12_SHIFT   (0)       /* Bits 0-3: EXTI 12 coinfiguration */
#define SYSCFG_EXTICR4_EXTI12_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI13_SHIFT   (4)       /* Bits 4-7: EXTI 13 coinfiguration */
#define SYSCFG_EXTICR4_EXTI13_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI14_SHIFT   (8)       /* Bits 8-11: EXTI 14 coinfiguration */
#define SYSCFG_EXTICR4_EXTI14_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI15_SHIFT   (12)      /* Bits 12-15: EXTI 15 coinfiguration */
#define SYSCFG_EXTICR4_EXTI15_MASK    (SYSCFG_EXTICR_PORT_MASK << SYSCFG_EXTICR4_EXTI15_SHIFT)

/* Compensation cell control register */

#define SYSCFG_CMPCR_CMPPD            (1 << 0)  /* Bit 0: Compensation cell power-down */
#undef  SYSCFG_CMPCR_READY
#define SYSCFG_CMPCR_READY            (1 << 8)  /* Bit 8: Compensation cell ready flag */

//
// GPIO
//
// Copy from px4nuttx\nuttx\arch\arm\src\stm32\chip\stm32f40xxx_gpio.h
/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define STM32_NGPIO_PORTS          ((STM32_NGPIO + 15) >> 4)

/* Register Offsets *****************************************************************/

#define STM32_GPIO_MODER_OFFSET    0x0000 /* GPIO port mode register */
#define STM32_GPIO_OTYPER_OFFSET   0x0004 /* GPIO port output type register */
#define STM32_GPIO_OSPEED_OFFSET   0x0008 /* GPIO port output speed register */
#define STM32_GPIO_PUPDR_OFFSET    0x000c /* GPIO port pull-up/pull-down register */
#define STM32_GPIO_IDR_OFFSET      0x0010 /* GPIO port input data register */
#define STM32_GPIO_ODR_OFFSET      0x0014 /* GPIO port output data register */
#define STM32_GPIO_BSRR_OFFSET     0x0018 /* GPIO port bit set/reset register */
#define STM32_GPIO_LCKR_OFFSET     0x001c /* GPIO port configuration lock register */
#define STM32_GPIO_AFRL_OFFSET     0x0020 /* GPIO alternate function low register */
#define STM32_GPIO_AFRH_OFFSET     0x0024 /* GPIO alternate function high register */

/* Register Addresses ***************************************************************/

#if STM32_NGPIO_PORTS > 0
#  define STM32_GPIOA_MODER        (STM32_GPIOA_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOA_OTYPER       (STM32_GPIOA_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOA_OSPEED       (STM32_GPIOA_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOA_PUPDR        (STM32_GPIOA_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOA_IDR          (STM32_GPIOA_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOA_ODR          (STM32_GPIOA_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOA_BSRR         (STM32_GPIOA_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOA_LCKR         (STM32_GPIOA_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOA_AFRL         (STM32_GPIOA_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOA_AFRH         (STM32_GPIOA_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 1
#  define STM32_GPIOB_MODER        (STM32_GPIOB_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOB_OTYPER       (STM32_GPIOB_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOB_OSPEED       (STM32_GPIOB_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOB_PUPDR        (STM32_GPIOB_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOB_IDR          (STM32_GPIOB_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOB_ODR          (STM32_GPIOB_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOB_BSRR         (STM32_GPIOB_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOB_LCKR         (STM32_GPIOB_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOB_AFRL         (STM32_GPIOB_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOB_AFRH         (STM32_GPIOB_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 2
#  define STM32_GPIOC_MODER        (STM32_GPIOC_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOC_OTYPER       (STM32_GPIOC_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOC_OSPEED       (STM32_GPIOC_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOC_PUPDR        (STM32_GPIOC_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOC_IDR          (STM32_GPIOC_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOC_ODR          (STM32_GPIOC_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOC_BSRR         (STM32_GPIOC_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOC_LCKR         (STM32_GPIOC_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOC_AFRL         (STM32_GPIOC_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOC_AFRH         (STM32_GPIOC_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 3
#  define STM32_GPIOD_MODER        (STM32_GPIOD_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOD_OTYPER       (STM32_GPIOD_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOD_OSPEED       (STM32_GPIOD_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOD_PUPDR        (STM32_GPIOD_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOD_IDR          (STM32_GPIOD_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOD_ODR          (STM32_GPIOD_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOD_BSRR         (STM32_GPIOD_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOD_LCKR         (STM32_GPIOD_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOD_AFRL         (STM32_GPIOD_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOD_AFRH         (STM32_GPIOD_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 4
#  define STM32_GPIOE_MODER        (STM32_GPIOE_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOE_OTYPER       (STM32_GPIOE_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOE_OSPEED       (STM32_GPIOE_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOE_PUPDR        (STM32_GPIOE_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOE_IDR          (STM32_GPIOE_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOE_ODR          (STM32_GPIOE_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOE_BSRR         (STM32_GPIOE_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOE_LCKR         (STM32_GPIOE_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOE_AFRL         (STM32_GPIOE_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOE_AFRH         (STM32_GPIOE_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 5
#  define STM32_GPIOF_MODER        (STM32_GPIOF_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOF_OTYPER       (STM32_GPIOF_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOF_OSPEED       (STM32_GPIOF_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOF_PUPDR        (STM32_GPIOF_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOF_IDR          (STM32_GPIOF_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOF_ODR          (STM32_GPIOF_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOF_BSRR         (STM32_GPIOF_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOF_LCKR         (STM32_GPIOF_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOF_AFRL         (STM32_GPIOF_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOF_AFRH         (STM32_GPIOF_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 6
#  define STM32_GPIOG_MODER        (STM32_GPIOG_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOG_OTYPER       (STM32_GPIOG_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOG_OSPEED       (STM32_GPIOG_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOG_PUPDR        (STM32_GPIOG_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOG_IDR          (STM32_GPIOG_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOG_ODR          (STM32_GPIOG_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOG_BSRR         (STM32_GPIOG_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOG_LCKR         (STM32_GPIOG_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOG_AFRL         (STM32_GPIOG_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOG_AFRH         (STM32_GPIOG_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 7
#  define STM32_GPIOH_MODER        (STM32_GPIOH_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOH_OTYPER       (STM32_GPIOH_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOH_OSPEED       (STM32_GPIOH_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOH_PUPDR        (STM32_GPIOH_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOH_IDR          (STM32_GPIOH_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOH_ODR          (STM32_GPIOH_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOH_BSRR         (STM32_GPIOH_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOH_LCKR         (STM32_GPIOH_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOH_AFRL         (STM32_GPIOH_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOH_AFRH         (STM32_GPIOH_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 8
#  define STM32_GPIOI_MODER        (STM32_GPIOI_BASE+STM32_GPIO_MODER_OFFSET)
#  define STM32_GPIOI_OTYPER       (STM32_GPIOI_BASE+STM32_GPIO_OTYPER_OFFSET)
#  define STM32_GPIOI_OSPEED       (STM32_GPIOI_BASE+STM32_GPIO_OSPEED_OFFSET)
#  define STM32_GPIOI_PUPDR        (STM32_GPIOI_BASE+STM32_GPIO_PUPDR_OFFSET)
#  define STM32_GPIOI_IDR          (STM32_GPIOI_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOI_ODR          (STM32_GPIOI_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOI_BSRR         (STM32_GPIOI_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOI_LCKR         (STM32_GPIOI_BASE+STM32_GPIO_LCKR_OFFSET)
#  define STM32_GPIOI_AFRL         (STM32_GPIOI_BASE+STM32_GPIO_AFRL_OFFSET)
#  define STM32_GPIOI_AFRH         (STM32_GPIOI_BASE+STM32_GPIO_AFRH_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

/* GPIO port mode register */

#define GPIO_MODER_INPUT           (0) /* Input */
#define GPIO_MODER_OUTPUT          (1) /* General purpose output mode */
#define GPIO_MODER_ALT             (2) /* Alternate mode */
#define GPIO_MODER_ANALOG          (3) /* Analog mode */

#define GPIO_MODER_SHIFT(n)        ((n) << 1)
#define GPIO_MODER_MASK(n)         (3 << GPIO_MODER_SHIFT(n))

#define GPIO_MODER0_SHIFT          (0)
#define GPIO_MODER0_MASK           (3 << GPIO_MODER0_SHIFT)
#define GPIO_MODER1_SHIFT          (2)
#define GPIO_MODER1_MASK           (3 << GPIO_MODER1_SHIFT)
#define GPIO_MODER2_SHIFT          (4)
#define GPIO_MODER2_MASK           (3 << GPIO_MODER2_SHIFT)
#define GPIO_MODER3_SHIFT          (6)
#define GPIO_MODER3_MASK           (3 << GPIO_MODER3_SHIFT)
#define GPIO_MODER4_SHIFT          (8)
#define GPIO_MODER4_MASK           (3 << GPIO_MODER4_SHIFT)
#define GPIO_MODER5_SHIFT          (10)
#define GPIO_MODER5_MASK           (3 << GPIO_MODER5_SHIFT)
#define GPIO_MODER6_SHIFT          (12)
#define GPIO_MODER6_MASK           (3 << GPIO_MODER6_SHIFT)
#define GPIO_MODER7_SHIFT          (14)
#define GPIO_MODER7_MASK           (3 << GPIO_MODER7_SHIFT)
#define GPIO_MODER8_SHIFT          (16)
#define GPIO_MODER8_MASK           (3 << GPIO_MODER8_SHIFT)
#define GPIO_MODER9_SHIFT          (18)
#define GPIO_MODER9_MASK           (3 << GPIO_MODER9_SHIFT)
#define GPIO_MODER10_SHIFT         (20)
#define GPIO_MODER10_MASK          (3 << GPIO_MODER10_SHIFT)
#define GPIO_MODER11_SHIFT         (22)
#define GPIO_MODER11_MASK          (3 << GPIO_MODER11_SHIFT)
#define GPIO_MODER12_SHIFT         (24)
#define GPIO_MODER12_MASK          (3 << GPIO_MODER12_SHIFT)
#define GPIO_MODER13_SHIFT         (26)
#define GPIO_MODER13_MASK          (3 << GPIO_MODER13_SHIFT)
#define GPIO_MODER14_SHIFT         (28)
#define GPIO_MODER14_MASK          (3 << GPIO_MODER14_SHIFT)
#define GPIO_MODER15_SHIFT         (30)
#define GPIO_MODER15_MASK          (3 << GPIO_MODER15_SHIFT)

/* GPIO port output type register */

#define GPIO_OTYPER_OD(n)          (1 << (n)) /* 1=Output open-drain */
#define GPIO_OTYPER_PP(n)          (0)        /* 0=Ouput push-pull */

/* GPIO port output speed register */

#define GPIO_OSPEED_2MHz           (0) /* 2 MHz Low speed */
#define GPIO_OSPEED_25MHz          (1) /* 25 MHz Medium speed */
#define GPIO_OSPEED_50MHz          (2) /* 50 MHz Fast speed */
#define GPIO_OSPEED_100MHz         (3) /* 100 MHz High speed on 30 pF (80 MHz Output max speed on 15 pF) */

#define GPIO_OSPEED_SHIFT(n)       ((n) << 1)
#define GPIO_OSPEED_MASK(n)        (3 << GPIO_OSPEED_SHIFT(n))

#define GPIO_OSPEED0_SHIFT         (0)
#define GPIO_OSPEED0_MASK          (3 << GPIO_OSPEED0_SHIFT)
#define GPIO_OSPEED1_SHIFT         (2)
#define GPIO_OSPEED1_MASK          (3 << GPIO_OSPEED1_SHIFT)
#define GPIO_OSPEED2_SHIFT         (4)
#define GPIO_OSPEED2_MASK          (3 << GPIO_OSPEED2_SHIFT)
#define GPIO_OSPEED3_SHIFT         (6)
#define GPIO_OSPEED3_MASK          (3 << GPIO_OSPEED3_SHIFT)
#define GPIO_OSPEED4_SHIFT         (8)
#define GPIO_OSPEED4_MASK          (3 << GPIO_OSPEED4_SHIFT)
#define GPIO_OSPEED5_SHIFT         (10)
#define GPIO_OSPEED5_MASK          (3 << GPIO_OSPEED5_SHIFT)
#define GPIO_OSPEED6_SHIFT         (12)
#define GPIO_OSPEED6_MASK          (3 << GPIO_OSPEED6_SHIFT)
#define GPIO_OSPEED7_SHIFT         (14)
#define GPIO_OSPEED7_MASK          (3 << GPIO_OSPEED7_SHIFT)
#define GPIO_OSPEED8_SHIFT         (16)
#define GPIO_OSPEED8_MASK          (3 << GPIO_OSPEED8_SHIFT)
#define GPIO_OSPEED9_SHIFT         (18)
#define GPIO_OSPEED9_MASK          (3 << GPIO_OSPEED9_SHIFT)
#define GPIO_OSPEED10_SHIFT        (20)
#define GPIO_OSPEED10_MASK         (3 << GPIO_OSPEED10_SHIFT)
#define GPIO_OSPEED11_SHIFT        (22)
#define GPIO_OSPEED11_MASK         (3 << GPIO_OSPEED11_SHIFT)
#define GPIO_OSPEED12_SHIFT        (24)
#define GPIO_OSPEED12_MASK         (3 << GPIO_OSPEED12_SHIFT)
#define GPIO_OSPEED13_SHIFT        (26)
#define GPIO_OSPEED13_MASK         (3 << GPIO_OSPEED13_SHIFT)
#define GPIO_OSPEED14_SHIFT        (28)
#define GPIO_OSPEED14_MASK         (3 << GPIO_OSPEED14_SHIFT)
#define GPIO_OSPEED15_SHIFT        (30)
#define GPIO_OSPEED15_MASK         (3 << GPIO_OSPEED15_SHIFT)

/* GPIO port pull-up/pull-down register */

#define GPIO_PUPDR_NONE            (0) /* No pull-up, pull-down */
#define GPIO_PUPDR_PULLUP          (1) /* Pull-up */
#define GPIO_PUPDR_PULLDOWN        (2) /* Pull-down */

#define GPIO_PUPDR_SHIFT(n)        ((n) << 1)
#define GPIO_PUPDR_MASK(n)         (3 << GPIO_PUPDR_SHIFT(n))

#define GPIO_PUPDR0_SHIFT          (0)
#define GPIO_PUPDR0_MASK           (3 << GPIO_PUPDR0_SHIFT)
#define GPIO_PUPDR1_SHIFT          (2)
#define GPIO_PUPDR1_MASK           (3 << GPIO_PUPDR1_SHIFT)
#define GPIO_PUPDR2_SHIFT          (4)
#define GPIO_PUPDR2_MASK           (3 << GPIO_PUPDR2_SHIFT)
#define GPIO_PUPDR3_SHIFT          (6)
#define GPIO_PUPDR3_MASK           (3 << GPIO_PUPDR3_SHIFT)
#define GPIO_PUPDR4_SHIFT          (8)
#define GPIO_PUPDR4_MASK           (3 << GPIO_PUPDR4_SHIFT)
#define GPIO_PUPDR5_SHIFT          (10)
#define GPIO_PUPDR5_MASK           (3 << GPIO_PUPDR5_SHIFT)
#define GPIO_PUPDR6_SHIFT          (12)
#define GPIO_PUPDR6_MASK           (3 << GPIO_PUPDR6_SHIFT)
#define GPIO_PUPDR7_SHIFT          (14)
#define GPIO_PUPDR7_MASK           (3 << GPIO_PUPDR7_SHIFT)
#define GPIO_PUPDR8_SHIFT          (16)
#define GPIO_PUPDR8_MASK           (3 << GPIO_PUPDR8_SHIFT)
#define GPIO_PUPDR9_SHIFT          (18)
#define GPIO_PUPDR9_MASK           (3 << GPIO_PUPDR9_SHIFT)
#define GPIO_PUPDR10_SHIFT         (20)
#define GPIO_PUPDR10_MASK          (3 << GPIO_PUPDR10_SHIFT)
#define GPIO_PUPDR11_SHIFT         (22)
#define GPIO_PUPDR11_MASK          (3 << GPIO_PUPDR11_SHIFT)
#define GPIO_PUPDR12_SHIFT         (24)
#define GPIO_PUPDR12_MASK          (3 << GPIO_PUPDR12_SHIFT)
#define GPIO_PUPDR13_SHIFT         (26)
#define GPIO_PUPDR13_MASK          (3 << GPIO_PUPDR13_SHIFT)
#define GPIO_PUPDR14_SHIFT         (28)
#define GPIO_PUPDR14_MASK          (3 << GPIO_PUPDR14_SHIFT)
#define GPIO_PUPDR15_SHIFT         (30)
#define GPIO_PUPDR15_MASK          (3 << GPIO_PUPDR15_SHIFT)

/* GPIO port input data register */

#define GPIO_IDR(n)                (1 << (n))

/* GPIO port output data register */

#define GPIO_ODR(n)                (1 << (n))

/* GPIO port bit set/reset register */

#define GPIO_BSRR_SET(n)           (1 << (n))
#define GPIO_BSRR_RESET(n)         (1 << ((n)+16))

/* GPIO port configuration lock register */

#define GPIO_LCKR(n)               (1 << (n))
#define GPIO_LCKK                  (1 << 16)   /* Lock key */

/* GPIO alternate function low/high register */

#define GPIO_AFR_SHIFT(n)          ((n) << 2)
#define GPIO_AFR_MASK(n)           (15 << GPIO_AFR_SHIFT(n))

#define GPIO_AFRL0_SHIFT           (0)
#define GPIO_AFRL0_MASK            (15 << GPIO_AFRL0_SHIFT)
#define GPIO_AFRL1_SHIFT           (4)
#define GPIO_AFRL1_MASK            (15 << GPIO_AFRL1_SHIFT)
#define GPIO_AFRL2_SHIFT           (8)
#define GPIO_AFRL2_MASK            (15 << GPIO_AFRL2_SHIFT)
#define GPIO_AFRL3_SHIFT           (12)
#define GPIO_AFRL3_MASK            (15 << GPIO_AFRL3_SHIFT)
#define GPIO_AFRL4_SHIFT           (16)
#define GPIO_AFRL4_MASK            (15 << GPIO_AFRL4_SHIFT)
#define GPIO_AFRL5_SHIFT           (20)
#define GPIO_AFRL5_MASK            (15 << GPIO_AFRL5_SHIFT)
#define GPIO_AFRL6_SHIFT           (24)
#define GPIO_AFRL6_MASK            (15 << GPIO_AFRL6_SHIFT)
#define GPIO_AFRL7_SHIFT           (28)
#define GPIO_AFRL7_MASK            (15 << GPIO_AFRL7_SHIFT)

#define GPIO_AFRH8_SHIFT           (0)
#define GPIO_AFRH8_MASK            (15 << GPIO_AFRH8_SHIFT)
#define GPIO_AFRH9_SHIFT           (4)
#define GPIO_AFRH9_MASK            (15 << GPIO_AFRH9_SHIFT)
#define GPIO_AFRH10_SHIFT          (8)
#define GPIO_AFRH10_MASK           (15 << GPIO_AFRH10_SHIFT)
#define GPIO_AFRH11_SHIFT          (12)
#define GPIO_AFRH11_MASK           (15 << GPIO_AFRH11_SHIFT)
#define GPIO_AFRH12_SHIFT          (16)
#define GPIO_AFRH12_MASK           (15 << GPIO_AFRH12_SHIFT)
#define GPIO_AFRH13_SHIFT          (20)
#define GPIO_AFRH13_MASK           (15 << GPIO_AFRH13_SHIFT)
#define GPIO_AFRH14_SHIFT          (24)
#define GPIO_AFRH14_MASK           (15 << GPIO_AFRH14_SHIFT)
#define GPIO_AFRH15_SHIFT          (28)
#define GPIO_AFRH15_MASK           (15 << GPIO_AFRH15_SHIFT)

/* SPI */

#define GPIO_SPI1_MISO_1      (GPIO_ALT|GPIO_AF5|GPIO_PORTA|GPIO_PIN6)
#define GPIO_SPI1_MISO_2      (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SPI1_MOSI_1      (GPIO_ALT|GPIO_AF5|GPIO_PORTA|GPIO_PIN7)
#define GPIO_SPI1_MOSI_2      (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN5)
#define GPIO_SPI1_NSS_1       (GPIO_ALT|GPIO_AF5|GPIO_PORTA|GPIO_PIN15)
#define GPIO_SPI1_NSS_2       (GPIO_ALT|GPIO_AF5|GPIO_PORTA|GPIO_PIN4)
#define GPIO_SPI1_SCK_1       (GPIO_ALT|GPIO_AF5|GPIO_PORTA|GPIO_PIN5)
#define GPIO_SPI1_SCK_2       (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN3)

#define GPIO_SPI2_MISO_1      (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI2_MISO_2      (GPIO_ALT|GPIO_AF5|GPIO_PORTC|GPIO_PIN2)
#define GPIO_SPI2_MISO_3      (GPIO_ALT|GPIO_AF5|GPIO_PORTI|GPIO_PIN2)
#define GPIO_SPI2_MOSI_1      (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN15)
#define GPIO_SPI2_MOSI_2      (GPIO_ALT|GPIO_AF5|GPIO_PORTC|GPIO_PIN3)
#define GPIO_SPI2_MOSI_3      (GPIO_ALT|GPIO_AF5|GPIO_PORTI|GPIO_PIN3)
#define GPIO_SPI2_NSS_1       (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN12)
#define GPIO_SPI2_NSS_2       (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN9)
#define GPIO_SPI2_NSS_3       (GPIO_ALT|GPIO_AF5|GPIO_PORTI|GPIO_PIN0)
#define GPIO_SPI2_SCK_1       (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN10)
#define GPIO_SPI2_SCK_2       (GPIO_ALT|GPIO_AF5|GPIO_PORTB|GPIO_PIN13)
#define GPIO_SPI2_SCK_3       (GPIO_ALT|GPIO_AF5|GPIO_PORTI|GPIO_PIN1)
#ifdef CONFIG_STM32_STM32F427
#  define GPIO_SPI2_SCK_4     (GPIO_ALT|GPIO_AF5|GPIO_PORTD|GPIO_PIN3)
#endif


# define getreg8(a)           (*(volatile uint8_t *)(a))
# define putreg8(v,a)         (*(volatile uint8_t *)(a) = (v))
# define getreg32(a)          (*(volatile uint32_t *)(a))
# define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))

/* Some compiler options will convert short loads and stores into byte loads
 * and stores.  We don't want this to happen for IO reads and writes!
 */

# define getreg16(a)       (*(volatile uint16_t *)(a)) 
/*
static inline uint16_t getreg16(unsigned int addr)
{
    uint16_t retval;
    __asm__ __volatile__("\tldrh %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
    return retval;
}
*/

# define putreg16(v,a)       (*(volatile uint16_t *)(a) = (v))
/*
static inline void putreg16(uint16_t val, unsigned int addr)
{
    __asm__ __volatile__("\tstrh %0, [%1]\n\t": : "r"(val), "r"(addr));
}
*/

/* Save the current interrupt enable state & disable IRQs */
static inline irqstate_t irqsave(void)
{
  unsigned int flags;
  unsigned int temp;
/*
  __asm__ __volatile__
    (
     "\tmrs    %0, cpsr\n"
     "\torr    %1, %0, #128\n"
     "\tmsr    cpsr_c, %1"
     : "=r" (flags), "=r" (temp)
     :
     : "memory");
*/
  return flags;
}

/* Restore saved IRQ & FIQ state */
static inline void irqrestore(irqstate_t flags)
{
/*
  __asm__ __volatile__
    (
     "msr    cpsr_c, %0"
     :
     : "r" (flags)
     : "memory");
*/
}

#endif