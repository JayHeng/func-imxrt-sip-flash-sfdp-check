/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flexspi.h"
#include "app.h"
#include "fsl_debug_console.h"
#include "fsl_cache.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Program data buffer should be 4-bytes alignment, which can avoid busfault due to this memory region is configured as
   Device Memory by MPU. */
#if 0
SDK_ALIGN(static uint8_t s_nor_program_buffer[256], 4);
static uint8_t s_nor_read_buffer[256];
#endif

extern status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);
extern status_t flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src);
extern status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *vendorId);
extern status_t flexspi_nor_enable_quad_mode(FLEXSPI_Type *base);
extern void flexspi_nor_flash_init(FLEXSPI_Type *base);
extern void bsp_validate_jedec(void);
/*******************************************************************************
 * Code
 ******************************************************************************/
flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 120000000,
    .flashSize            = FLASH_SIZE,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 0,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = 0,
    .AWRSeqNumber         = 0,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};

const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
    /* Normal read mode -SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Fast read mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Fast read quad mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06, kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04),

    /* Read extend parameters */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x20, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

    /* Page Program - single mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Page Program - quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enable Quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x31, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Read SFDP */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSFDP] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, 0x5A, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READSFDP + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR,  kFLEXSPI_1PAD, 0xFF),

    /* Program Security registers or SFDP */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESECSFDP] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, 0x42, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESECSFDP + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0xFF, kFLEXSPI_Command_STOP,      kFLEXSPI_1PAD, 0x00),

    /* Erase Security registers or SFDP */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECSFDP] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, 0x44, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
};

/* Flexspi2 connects internal flash by default and this group of pins doesn't support the use of CFG tool configuration,
 * and don't want users to configure it as other attributes.  */
void BOARD_InitFlexspi2Pins(void)
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B1_00_FLEXSPI2_A_DQS, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B1_06_FLEXSPI2_A_SS0_B, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B0_08_FLEXSPI2_A_SCLK, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B0_02_FLEXSPI2_A_DATA00, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B1_03_FLEXSPI2_A_DATA01, 1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B1_02_FLEXSPI2_A_DATA02, 1U);

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B1_00_FLEXSPI2_A_DQS, 0x10F1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B1_06_FLEXSPI2_A_SS0_B, 0x10F1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B0_08_FLEXSPI2_A_SCLK, 0x10F1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B0_02_FLEXSPI2_A_DATA00, 0x10F1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B1_03_FLEXSPI2_A_DATA01, 0x10F1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B1_02_FLEXSPI2_A_DATA02, 0x10F1U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B0_10_FLEXSPI2_A_DATA03, 0x10F1U);
}

volatile uint32_t g_systickCounter;
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}

void BOARD_InitFlashResetPins(bool isEnabled)
{
    /* GPIO configuration of RESET */
    gpio_pin_config_t RESET_config = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode
    };

#if 0
    uint32_t swPadGpioSpiB0_00 = *((uint32_t *)0x401F86B4);
    PRINTF("Default IOMUXC_SW_PAD_CTL_PAD_GPIO_SPI_B0_00 = 0x%x\r\n", swPadGpioSpiB0_00);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B0_00, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B0_00, 0x10B0U);
    GPIO_PinInit(GPIO10, 0U, &RESET_config);
    GPIO_PinWrite(GPIO10, 0, 1U);
#endif

    uint32_t swPadGpioSpiB0_13 = *((uint32_t *)0x401F86E8);
    PRINTF("\r\nDefault IOMUXC_SW_PAD_CTL_PAD_GPIO_SPI_B0_13 = 0x%x\r\n", swPadGpioSpiB0_13);
    IOMUXC_SetPinMux(IOMUXC_GPIO_SPI_B0_13, 0U);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SPI_B0_13, 0x10B0U);
    GPIO_PinInit(GPIO10, 13U, &RESET_config);
    if (isEnabled)
    {
        PRINTF("GPIO_SPI_B0_13 reset pin is set to low, enabled\r\n");
        GPIO_PinWrite(GPIO10, 13, 0U);
    }
    else
    {
        PRINTF("GPIO_SPI_B0_13 reset pin is set to high, disabled\r\n");
        GPIO_PinWrite(GPIO10, 13, 1U);
    }
    
    SysTick_DelayTicks(1U);
}

int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    /* Update the core clock */
    SystemCoreClockUpdate();

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }

    BOARD_InitFlexspi2Pins();

    flexspi_nor_flash_init(EXAMPLE_FLEXSPI);

    PRINTF("\r\nFLEXSPI example started!\r\n");
    
    BOARD_InitFlashResetPins(false);
    bsp_validate_jedec();

    BOARD_InitFlashResetPins(true);
    bsp_validate_jedec();
#if 0
    uint32_t i = 0;
    status_t status;
    uint8_t vendorID = 0;

    /* Get vendor ID. */
    status = flexspi_nor_get_vendor_id(EXAMPLE_FLEXSPI, &vendorID);
    if (status != kStatus_Success)
    {
        return status;
    }
    PRINTF("Vendor ID: 0x%x\r\n", vendorID);

    /* Enter quad mode. */
    status = flexspi_nor_enable_quad_mode(EXAMPLE_FLEXSPI);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Erase sectors. */
    PRINTF("Erasing Serial NOR over FlexSPI...\r\n");
    status = flexspi_nor_flash_erase_sector(EXAMPLE_FLEXSPI, EXAMPLE_SECTOR * SECTOR_SIZE);
    if (status != kStatus_Success)
    {
        PRINTF("Erase sector failure !\r\n");
        return -1;
    }

    memset(s_nor_program_buffer, 0xFFU, sizeof(s_nor_program_buffer));

    DCACHE_InvalidateByRange(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE, FLASH_PAGE_SIZE);

    memcpy(s_nor_read_buffer, (void *)(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE),
           sizeof(s_nor_read_buffer));

    if (memcmp(s_nor_program_buffer, s_nor_read_buffer, sizeof(s_nor_program_buffer)))
    {
        PRINTF("Erase data -  read out data value incorrect !\r\n ");
        return -1;
    }
    else
    {
        PRINTF("Erase data - successfully. \r\n");
    }

    for (i = 0; i < 0xFFU; i++)
    {
        s_nor_program_buffer[i] = i;
    }

    status =
        flexspi_nor_flash_page_program(EXAMPLE_FLEXSPI, EXAMPLE_SECTOR * SECTOR_SIZE, (void *)s_nor_program_buffer);
    if (status != kStatus_Success)
    {
        PRINTF("Page program failure !\r\n");
        return -1;
    }

    DCACHE_InvalidateByRange(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE, FLASH_PAGE_SIZE);

    memcpy(s_nor_read_buffer, (void *)(EXAMPLE_FLEXSPI_AMBA_BASE + EXAMPLE_SECTOR * SECTOR_SIZE),
           sizeof(s_nor_read_buffer));

    if (memcmp(s_nor_read_buffer, s_nor_program_buffer, sizeof(s_nor_program_buffer)) != 0)
    {
        PRINTF("Program data -  read out data value incorrect !\r\n ");
        return -1;
    }
    else
    {
        PRINTF("Program data - successfully. \r\n");
    }
#endif

    while (1)
    {
    }
}
