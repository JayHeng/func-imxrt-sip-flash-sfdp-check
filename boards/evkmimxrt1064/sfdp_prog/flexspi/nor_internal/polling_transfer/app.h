/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI                 FLEXSPI2
#define FLASH_SIZE                      0x1000 /* 32Mb/KByte */
#define EXAMPLE_FLEXSPI_AMBA_BASE       FlexSPI2_AMBA_BASE
#define FLASH_PAGE_SIZE                 256
#define EXAMPLE_SECTOR                  10
#define SECTOR_SIZE                     0x1000 /* 4K */
#define EXAMPLE_FLEXSPI_CLOCK           kCLOCK_FlexSpi2
#define FLASH_PORT                      kFLEXSPI_PortA1
#define EXAMPLE_FLEXSPI_RX_SAMPLE_CLOCK kFLEXSPI_ReadSampleClkLoopbackFromDqsPad

#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL        0
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST          1
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD     2
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS         3
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE        4
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR        5
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD   7
#define NOR_CMD_LUT_SEQ_IDX_READID             8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG     9
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG      12

#define NOR_CMD_LUT_SEQ_IDX_ERASESECSFDP    10
#define NOR_CMD_LUT_SEQ_IDX_WRITESECSFDP    11
#define NOR_CMD_LUT_SEQ_IDX_READSFDP        13

#define CUSTOM_LUT_LENGTH        60
#define FLASH_QUAD_ENABLE        0x02
#define FLASH_BUSY_STATUS_POL    1
#define FLASH_BUSY_STATUS_OFFSET 0
#define FLASH_ERROR_STATUS_MASK  0x0e

/*
 * If cache is enabled, this example should maintain the cache to make sure
 * CPU core accesses the memory, not cache only.
 */
#define CACHE_MAINTAIN 1

/*${macro:end}*/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*${variable:start}*/
#if (defined CACHE_MAINTAIN) && (CACHE_MAINTAIN == 1)
typedef struct _flexspi_cache_status
{
    volatile bool DCacheEnableFlag;
    volatile bool ICacheEnableFlag;
} flexspi_cache_status_t;
#endif
/*${variable:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
static inline void flexspi_clock_init(void)
{
#if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
    CLOCK_SetMux(kCLOCK_Flexspi2Mux, 0x0); /* Choose PLL2 PFD2 clock as flexspi2 source clock. 396M */
    CLOCK_SetDiv(kCLOCK_Flexspi2Div, 2);   /* flexspi clock 133M. */
#else
    const clock_usb_pll_config_t g_ccmConfigUsbPll = {.loopDivider = 0U};
    CLOCK_InitUsb1Pll(&g_ccmConfigUsbPll);
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 24);    /* Set PLL3 PFD0 clock 360MHZ. */
    CLOCK_SetMux(kCLOCK_Flexspi2Mux, 0x1); /* Choose PLL3 PFD0 clock as flexspi2 source clock. */
    CLOCK_SetDiv(kCLOCK_Flexspi2Div, 2);   /* flexspi clock 120M. */
#endif
}
/*${prototype:end}*/

#endif /* _APP_H_ */
