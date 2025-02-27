/*
 * arch/arm64/mach-tegra/include/mach/irqs-t21x.h
 *
 * Copyright (C) 2013-2015 NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

// Imported from L4T with adjustments.

#ifndef __MACH_TEGRA_IRQS_T21X_H
#define __MACH_TEGRA_IRQS_T21X_H

#define INT_CPU_IRQS_NR         32

/* Primary Interrupt Controller */
#define INT_PRI_BASE			(INT_GIC_BASE + 0)
#define INT_TMR1			(INT_PRI_BASE + 0)
#define INT_TMR2			(INT_PRI_BASE + 1)
#define INT_RTC				(INT_PRI_BASE + 2)
#define INT_CEC				(INT_PRI_BASE + 3)
#define INT_SHR_SEM_INBOX_IBF		(INT_PRI_BASE + 4)
#define INT_SHR_SEM_INBOX_IBE		(INT_PRI_BASE + 5)
#define INT_SHR_SEM_OUTBOX_IBF		(INT_PRI_BASE + 6)
#define INT_SHR_SEM_OUTBOX_IBE		(INT_PRI_BASE + 7)
#define INT_NVJPG			(INT_PRI_BASE + 8)
#define INT_NVDEC			(INT_PRI_BASE + 9)
#define INT_QUAD_SPI			(INT_PRI_BASE + 10)
#define INT_DPAUX_INT1			(INT_PRI_BASE + 11)
/* unused				(INT_PRI_BASE + 12) */
#define INT_SATA_RX_STAT		(INT_PRI_BASE + 13)
#define INT_SDMMC1			(INT_PRI_BASE + 14)
#define INT_SDMMC2			(INT_PRI_BASE + 15)
#define INT_VGPIO			(INT_PRI_BASE + 16)
#define INT_VII2C			(INT_PRI_BASE + 17)
#define INT_AVP_UCQ			(INT_PRI_BASE + 18) // Unassigned?
#define INT_SDMMC3			(INT_PRI_BASE + 19)
#define INT_USB				(INT_PRI_BASE + 20)
#define INT_USB2			(INT_PRI_BASE + 21)
#define INT_MIPI_BIF			(INT_PRI_BASE + 22) // Unassigned?
#define INT_SATA_CTL			(INT_PRI_BASE + 23)
#define INT_PMC				(INT_PRI_BASE + 24)
#define INT_FC				(INT_PRI_BASE + 25)
#define INT_APB_DMA			(INT_PRI_BASE + 26)
#define INT_AHB_DMA			(INT_PRI_BASE + 27) // Unassigned?
#define INT_ARB_SEM_GNT_COP		(INT_PRI_BASE + 28)
#define INT_ARB_SEM_GNT_CPU		(INT_PRI_BASE + 29)
#define INT_OWR				(INT_PRI_BASE + 30) // Unassigned?
#define INT_SDMMC4			(INT_PRI_BASE + 31)

/* Secondary Interrupt Controller */
#define INT_SEC_BASE			(INT_PRI_BASE + 32)
#define INT_GPIO1			(INT_SEC_BASE + 0)
#define INT_GPIO2			(INT_SEC_BASE + 1)
#define INT_GPIO3			(INT_SEC_BASE + 2)
#define INT_GPIO4			(INT_SEC_BASE + 3)
#define INT_UARTA			(INT_SEC_BASE + 4)
#define INT_UARTB			(INT_SEC_BASE + 5)
#define INT_I2C				(INT_SEC_BASE + 6)
#define INT_USB3_HOST_INT		(INT_SEC_BASE + 7)
#define INT_USB3_HOST_SMI		(INT_SEC_BASE + 8)
#define INT_TMR3			(INT_SEC_BASE + 9)
#define INT_TMR4			(INT_SEC_BASE + 10)
#define INT_USB3_HOST_PME		(INT_SEC_BASE + 11)
#define INT_USB3_DEV_HOST		(INT_SEC_BASE + 12)
#define INT_ACTMON			(INT_SEC_BASE + 13)
#define INT_UARTC			(INT_SEC_BASE + 14)
#define INT_HOST1x_TZ_SYNCPT		(INT_SEC_BASE + 15) // Unassigned?
#define INT_THERMAL			(INT_SEC_BASE + 16)
#define INT_XUSB_PADCTL			(INT_SEC_BASE + 17)
#define INT_TSEC			(INT_SEC_BASE + 18)
#define INT_EDP				(INT_SEC_BASE + 19)
#define INT_HOST1x_TZ_GEN		(INT_SEC_BASE + 20) // Unassigned?
#define INT_I2C5			(INT_SEC_BASE + 21)
#define INT_SYS_STATS_MON		(INT_SEC_BASE + 22) // Unassigned?
#define INT_GPIO5			(INT_SEC_BASE + 23)
#define INT_USB3_DEV_SMI		(INT_SEC_BASE + 24)
#define INT_USB3_DEV_PME		(INT_SEC_BASE + 25)
#define INT_SE				(INT_SEC_BASE + 26)
#define INT_SPI_1			(INT_SEC_BASE + 27)
#define INT_APB_DMA_COP			(INT_SEC_BASE + 28)
#define INT_AHB_DMA_COP			(INT_SEC_BASE + 29) // Unassigned?
#define INT_CLDVFS			(INT_SEC_BASE + 30)
#define INT_I2C6			(INT_SEC_BASE + 31)

/* Tertiary Interrupt Controller */
#define INT_TRI_BASE			(INT_SEC_BASE + 32)
#define INT_HOST1X_SYNCPT_COP		(INT_TRI_BASE + 0)
#define INT_HOST1X_SYNCPT_CPU		(INT_TRI_BASE + 1)
#define INT_HOST1X_GEN_COP		(INT_TRI_BASE + 2)
#define INT_HOST1X_GEN_CPU		(INT_TRI_BASE + 3)
#define INT_NVENC			(INT_TRI_BASE + 4)
#define INT_VI_GENERAL			(INT_TRI_BASE + 5)
#define INT_ISPB_GENERAL		(INT_TRI_BASE + 6)
#define INT_ISP_GENERAL			(INT_TRI_BASE + 7)
#define INT_VIC_GENERAL			(INT_TRI_BASE + 8)
#define INT_DISPLAY_GENERAL		(INT_TRI_BASE + 9)
#define INT_DISPLAY_B_GENERAL		(INT_TRI_BASE + 10)
#define INT_SOR_1			(INT_TRI_BASE + 11)
#define INT_SOR				(INT_TRI_BASE + 12)
#define INT_MC_GENERAL			(INT_TRI_BASE + 13)
#define INT_EMC_GENERAL			(INT_TRI_BASE + 14)
#define INT_SPI_6			(INT_TRI_BASE + 15) // Unassigned?
#define INT_TSECB			(INT_TRI_BASE + 16)
#define INT_HDA				(INT_TRI_BASE + 17)
#define INT_SPI_2			(INT_TRI_BASE + 18)
#define INT_SPI_3			(INT_TRI_BASE + 19)
#define INT_I2C2			(INT_TRI_BASE + 20)
/* unused				(INT_TRI_BASE + 21) */
#define INT_PMU_EXT			(INT_TRI_BASE + 22)
#define INT_GPIO6			(INT_TRI_BASE + 23)
#define INT_BB2AP_INT0			(INT_TRI_BASE + 24) // Unassigned?
#define INT_GPIO7			(INT_TRI_BASE + 25)
#define INT_UARTD			(INT_TRI_BASE + 26)
#define INT_BB2AP_MEM_REQ_SOON_INT	(INT_TRI_BASE + 27) // Unassigned?
#define INT_I2C3			(INT_TRI_BASE + 28)
#define INT_SPI_4			(INT_TRI_BASE + 29)
#define INT_SPI_5			(INT_TRI_BASE + 30) // Unassigned?
#define INT_SW_RESERVED			(INT_TRI_BASE + 31) // Unassigned?

/* Quaternary Interrupt Controller */
#define INT_QUAD_BASE			(INT_TRI_BASE + 32)
#define INT_DTV				(INT_QUAD_BASE + 0)
/* unused				(INT_QUAD_BASE + 1) */
#define INT_PCIE_INTR			(INT_QUAD_BASE + 2)
#define INT_PCIE_MSI			(INT_QUAD_BASE + 3)
#define INT_PCIE			(INT_QUAD_BASE + 4)
#define INT_AVP_CACHE			(INT_QUAD_BASE + 5)
#define INT_APE_1			(INT_QUAD_BASE + 6)
#define INT_APE_0			(INT_QUAD_BASE + 7)
#define INT_APB_DMA_CH0			(INT_QUAD_BASE + 8)
#define INT_APB_DMA_CH1			(INT_QUAD_BASE + 9)
#define INT_APB_DMA_CH2			(INT_QUAD_BASE + 10)
#define INT_APB_DMA_CH3			(INT_QUAD_BASE + 11)
#define INT_APB_DMA_CH4			(INT_QUAD_BASE + 12)
#define INT_APB_DMA_CH5			(INT_QUAD_BASE + 13)
#define INT_APB_DMA_CH6			(INT_QUAD_BASE + 14)
#define INT_APB_DMA_CH7			(INT_QUAD_BASE + 15)
#define INT_APB_DMA_CH8			(INT_QUAD_BASE + 16)
#define INT_APB_DMA_CH9			(INT_QUAD_BASE + 17)
#define INT_APB_DMA_CH10		(INT_QUAD_BASE + 18)
#define INT_APB_DMA_CH11		(INT_QUAD_BASE + 19)
#define INT_APB_DMA_CH12		(INT_QUAD_BASE + 20)
#define INT_APB_DMA_CH13		(INT_QUAD_BASE + 21)
#define INT_APB_DMA_CH14		(INT_QUAD_BASE + 22)
#define INT_APB_DMA_CH15		(INT_QUAD_BASE + 23)
#define INT_I2C4			(INT_QUAD_BASE + 24)
#define INT_TMR5			(INT_QUAD_BASE + 25)
/* unused				(INT_QUAD_BASE + 26) */
#define INT_WDT_CPU			(INT_QUAD_BASE + 27)
#define INT_WDT_AVP			(INT_QUAD_BASE + 28)
#define INT_GPIO8			(INT_QUAD_BASE + 29)
#define INT_CAR				(INT_QUAD_BASE + 30)
#define INT_HIER_GROUP1_CPU		(INT_QUAD_BASE + 31) // Unassigned?

/* Quintary Interrupt Controller */
#define INT_QUINT_BASE			(INT_QUAD_BASE + 32)
#define INT_APB_DMA_CH16		(INT_QUINT_BASE + 0)
#define INT_APB_DMA_CH17		(INT_QUINT_BASE + 1)
#define INT_APB_DMA_CH18		(INT_QUINT_BASE + 2)
#define INT_APB_DMA_CH19		(INT_QUINT_BASE + 3)
#define INT_APB_DMA_CH20		(INT_QUINT_BASE + 4)
#define INT_APB_DMA_CH21		(INT_QUINT_BASE + 5)
#define INT_APB_DMA_CH22		(INT_QUINT_BASE + 6)
#define INT_APB_DMA_CH23		(INT_QUINT_BASE + 7)
#define INT_APB_DMA_CH24		(INT_QUINT_BASE + 8)
#define INT_APB_DMA_CH25		(INT_QUINT_BASE + 9)
#define INT_APB_DMA_CH26		(INT_QUINT_BASE + 10)
#define INT_APB_DMA_CH27		(INT_QUINT_BASE + 11)
#define INT_APB_DMA_CH28		(INT_QUINT_BASE + 12)
#define INT_APB_DMA_CH29		(INT_QUINT_BASE + 13)
#define INT_APB_DMA_CH30		(INT_QUINT_BASE + 14)
#define INT_APB_DMA_CH31		(INT_QUINT_BASE + 15)
#define INT_CPU0_PMU_INTR		(INT_QUINT_BASE + 16)
#define INT_CPU1_PMU_INTR		(INT_QUINT_BASE + 17)
#define INT_CPU2_PMU_INTR		(INT_QUINT_BASE + 18)
#define INT_CPU3_PMU_INTR		(INT_QUINT_BASE + 19)
#define INT_SDMMC1_SYS			(INT_QUINT_BASE + 20)
#define INT_SDMMC2_SYS			(INT_QUINT_BASE + 21)
#define INT_SDMMC3_SYS			(INT_QUINT_BASE + 22)
#define INT_SDMMC4_SYS			(INT_QUINT_BASE + 23)
#define INT_TMR6			(INT_QUINT_BASE + 24)
#define INT_TMR7			(INT_QUINT_BASE + 25)
#define INT_TMR8			(INT_QUINT_BASE + 26)
#define INT_TMR9			(INT_QUINT_BASE + 27)
#define INT_TMR0			(INT_QUINT_BASE + 28)
#define INT_GPU_STALL			(INT_QUINT_BASE + 29)
#define INT_GPU_NONSTALL		(INT_QUINT_BASE + 30)
#define INT_DPAUX			(INT_QUINT_BASE + 31)

/* Hexa Interrupt Controller */
#define INT_HEXA_BASE			(INT_QUINT_BASE + 32)
#define INT_MPCORE_AXIERRIRQ		(INT_HEXA_BASE + 0)
#define INT_MPCORE_INTERRIRQ		(INT_HEXA_BASE + 1)
#define INT_EVENT_GPIO_A		(INT_HEXA_BASE + 2)
#define INT_EVENT_GPIO_B		(INT_HEXA_BASE + 3)
#define INT_EVENT_GPIO_C		(INT_HEXA_BASE + 4)
/* unused				(INT_HEXA_BASE + 5) */
/* unused				(INT_HEXA_BASE + 6) */
/* unused				(INT_HEXA_BASE + 7) */
#define INT_FLOW_RSM_CPU		(INT_HEXA_BASE + 8)
#define INT_FLOW_RSM_COP		(INT_HEXA_BASE + 9)
#define INT_TMR_SHARED			(INT_HEXA_BASE + 10)
#define INT_MPCORE_CTIIRQ0		(INT_HEXA_BASE + 11)
#define INT_MPCORE_CTIIRQ1		(INT_HEXA_BASE + 12)
#define INT_MPCORE_CTIIRQ2		(INT_HEXA_BASE + 13)
#define INT_MPCORE_CTIIRQ3		(INT_HEXA_BASE + 14)
#define INT_MSELECT_ERROR		(INT_HEXA_BASE + 15)
#define INT_TMR10			(INT_HEXA_BASE + 16)
#define INT_TMR11			(INT_HEXA_BASE + 17)
#define INT_TMR12			(INT_HEXA_BASE + 18)
#define INT_TMR13			(INT_HEXA_BASE + 19)
/* unused				(INT_HEXA_BASE + 20) */
/* unused				(INT_HEXA_BASE + 21) */
/* unused				(INT_HEXA_BASE + 22) */
/* unused				(INT_HEXA_BASE + 23) */
/* unused				(INT_HEXA_BASE + 24) */
/* unused				(INT_HEXA_BASE + 25) */
/* unused				(INT_HEXA_BASE + 26) */
/* unused				(INT_HEXA_BASE + 27) */
/* unused				(INT_HEXA_BASE + 28) */
/* unused				(INT_HEXA_BASE + 29) */
/* unused				(INT_HEXA_BASE + 30) */
/* unused				(INT_HEXA_BASE + 31) */

#define INT_GIC_NR			(INT_HEXA_BASE + 32 + INT_CPU_IRQS_NR)

#define INT_MAIN_NR			(INT_GIC_NR - INT_CPU_IRQS_NR)

#define INT_SYNCPT_THRESH_BASE		(INT_QUINT_BASE + 32)
#define INT_SYNCPT_THRESH_NR		(32 * 6)

#define INT_GPIO_BASE			(INT_SYNCPT_THRESH_BASE + \
					 INT_SYNCPT_THRESH_NR)
#define INT_GPIO_NR			(32 * 8)

#endif
