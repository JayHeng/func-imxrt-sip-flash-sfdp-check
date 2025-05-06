/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flexspi.h"
#include "app.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief JEDEC id structure. */
typedef struct _jedec_id
{
    uint8_t manufacturerID;
    uint8_t memoryTypeID;
    uint8_t capacityID;
    uint8_t reserved;
} jedec_id_t;

//!@brief SFDP related definitions
#define SFDP_SIGNATURE 0x50444653 /* ASCII: SFDP */
enum
{
    kSfdp_Version_Major_1_0 = 1,
    kSfdp_Version_Minor_0 = 0, // JESD216
    kSfdp_Version_Minor_A = 5, // JESD216A
    kSfdp_Version_Minor_B = 6, // JESD216B
    kSfdp_Version_Minor_C = 7, // JESD216C

    kSfdp_BasicProtocolTableSize_Rev0 = 36,
    kSfdp_BasicProtocolTableSize_RevA = 64,
    kSfdp_BasicProtocolTableSize_RevB = kSfdp_BasicProtocolTableSize_RevA,
    kSfdp_BasicProtocolTableSize_RevC = 80,
};

typedef struct _sfdp_header
{
    uint32_t signature;
    uint8_t minor_rev;
    uint8_t major_rev;
    uint8_t param_hdr_num;
    uint8_t sfdp_access_protocol; // Defined in JESD216C, reserved for older version
} sfdp_header_t;

typedef struct _sfdp_table
{
    sfdp_header_t header;
    uint32_t tbd[62];
} sfdp_table_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern status_t flexspi_nor_write_enable(FLEXSPI_Type *base, uint32_t baseAddr);
extern status_t flexspi_nor_wait_bus_busy(FLEXSPI_Type *base);

/*******************************************************************************
 * Variables
 *****************************************************************************/

static sfdp_table_t s_sfdp_table;

/*******************************************************************************
 * Code
 ******************************************************************************/
#if defined(__ICCARM__)
#pragma optimize = none
#endif
status_t flexspi_nor_get_jedec_id(FLEXSPI_Type *base, uint32_t *jedecId)
{
    uint32_t temp = 0;
    flexspi_transfer_t flashXfer;
    flashXfer.deviceAddress = 0;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READID;
    flashXfer.data          = &temp;
    flashXfer.dataSize      = 3;

    status_t status = FLEXSPI_TransferBlocking(base, &flashXfer);

    *jedecId = temp;

    return status;
}

#if defined(__ICCARM__)
#pragma optimize = none
#endif
status_t flexspi_nor_get_jedec_sfdp(FLEXSPI_Type *base, uint32_t addr, uint32_t *jedecSfdp, uint32_t sfdpSize)
{
    flexspi_transfer_t flashXfer;
    flashXfer.deviceAddress = addr;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Read;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READSFDP;
    flashXfer.data          = jedecSfdp;
    flashXfer.dataSize      = sfdpSize;

    status_t status = FLEXSPI_TransferBlocking(base, &flashXfer);

    return status;
}

status_t flexspi_nor_sfdp_sec_erase(FLEXSPI_Type *base, uint32_t addr)
{
    status_t status;
    flexspi_transfer_t flashXfer;

    /* Write enable */
    status = flexspi_nor_write_enable(base, addr);

    if (status != kStatus_Success)
    {
        return status;
    }

    flashXfer.deviceAddress = addr;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Command;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASESECSFDP;

    status = FLEXSPI_TransferBlocking(base, &flashXfer);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset. */
    FLEXSPI_SoftwareReset(base);

    return status;
}

status_t flexspi_nor_sfdp_sec_program(FLEXSPI_Type *base, uint32_t addr, const uint32_t *src, uint32_t length)
{
    status_t status;
    flexspi_transfer_t flashXfer;

    /* Write enable */
    status = flexspi_nor_write_enable(base, addr);

    if (status != kStatus_Success)
    {
        return status;
    }

    /* Prepare program sfdp command */
    flashXfer.deviceAddress = addr;
    flashXfer.port          = FLASH_PORT;
    flashXfer.cmdType       = kFLEXSPI_Write;
    flashXfer.SeqNumber     = 1;
    flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITESECSFDP;
    flashXfer.data          = (uint32_t *)src;
    flashXfer.dataSize      = length;
    status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_wait_bus_busy(base);

    /* Do software reset or clear AHB buffer directly. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT) && defined(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK) && \
    defined(FLEXSPI_AHBCR_CLRAHBTXBUF_MASK)
    base->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK;
    base->AHBCR &= ~(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
#else
    FLEXSPI_SoftwareReset(base);
#endif

    return status;
}

void bsp_validate_jedec(void)
{
    status_t status = kStatus_Success;
    jedec_id_t jedecID;
    /* Read JEDEC id from flash */
    status = flexspi_nor_get_jedec_id(EXAMPLE_FLEXSPI, (uint32_t *)&jedecID);
    if (status != kStatus_Success)
    {
        PRINTF("Get Flash Vendor ID failed");
    }
    else
    {
        if (jedecID.manufacturerID == 0xEF)
        {
            PRINTF("Get Valid Flash Vendor ID.\r\n");
            sfdp_header_t sfdp_header;
            status = flexspi_nor_get_jedec_sfdp(EXAMPLE_FLEXSPI, 0, (uint32_t *)&s_sfdp_table, sizeof(sfdp_table_t));
            memcpy((uint8_t *)&sfdp_header, (uint8_t *)&s_sfdp_table, sizeof(sfdp_header));
            if (status == kStatus_Success)
            {
                if (sfdp_header.signature == SFDP_SIGNATURE)
                {
                    PRINTF("Get Valid Flash SFDP.\r\n");
                    if (sfdp_header.major_rev == kSfdp_Version_Major_1_0)
                    {
                        PRINTF("Flash SFDP Version is JESD216");
                        switch (sfdp_header.minor_rev)
                        {
                            case kSfdp_Version_Minor_C:
                                PRINTF("C");
                                break;
                            case kSfdp_Version_Minor_B:
                                PRINTF("B");
                                break;
                            case kSfdp_Version_Minor_A:
                                PRINTF("A");
                                break;
                            case kSfdp_Version_Minor_0:
                                break;
                            default:
                                PRINTF("x");
                                break;
                        }
                        PRINTF(" - minor_rev = %d.\r\n", sfdp_header.minor_rev);
                    }
                }
                else
                {
                    PRINTF("Get Invalid Flash SFDP, Signature = 0x%x.\r\n", sfdp_header.signature);
                    status = flexspi_nor_sfdp_sec_erase(EXAMPLE_FLEXSPI, 0x000000);
                    PRINTF("Erased Flash SFDP Region - ");
                    if (status == kStatus_Success)
                    {
                        PRINTF("Done.\r\n");
                        {
                            #pragma section = "__sfdp_table"
                            uint32_t *sfdpStart = __section_begin("__sfdp_table");
                            status = flexspi_nor_sfdp_sec_program(EXAMPLE_FLEXSPI, 0x000000, sfdpStart, sizeof(sfdp_table_t));
                        }
                        PRINTF("Programmed Flash SFDP Region - ");
                        if (status == kStatus_Success)
                        {
                            PRINTF("Done.\r\n");
                        }
                        else
                        {
                            PRINTF("Failed.\r\n");
                        }
                    }
                    else
                    {
                        PRINTF("Failed.\r\n");
                    }
                }
            }
            else
            {
                PRINTF("Get Flash SFDP failed\r\n");
            }
        }
        else
        {
            PRINTF("Get Invalid Flash Vendor ID 0x%x", jedecID.manufacturerID);
        }
    }
}


