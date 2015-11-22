/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ====================== Board.c =============================================
 *  This file is responsible for setting up the board specific items for the
 *  SRF06EB with the CC2650EM_4XS board.
 */


/*
 *  ====================== Includes ============================================
 */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/ioc.h>
#include <driverlib/udma.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/drivers/PIN.h>
#include "Board.h"

/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table.
*/
PIN_Config BoardGpioInitTable[] = {

    Board_KEY_SELECT | PIN_INPUT_EN  | PIN_PULLUP | PIN_HYSTERESIS,                             /* Button is active low          */
    Board_UART_TX    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH   | PIN_PUSHPULL,                     /* UART TX pin at inactive level */
    PIN_TERMINATE                                                                               /* Terminate list                */
};
/*============================================================================*/

/*
 *  ============================= UART begin ===================================
*/
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartCC26XXHWAttrs, ".const:uartCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

/* UART objects */
UARTCC26XX_Object uartCC26XXObjects[CC2650_UARTCOUNT];

/* UART hardware parameter structure, also used to assign UART pins */
const UARTCC26XX_HWAttrs uartCC26XXHWAttrs[CC2650_UARTCOUNT] = {
    {    /* CC2650_UART0 */
        .baseAddr = UART0_BASE,
        .intNum = INT_UART0,
        .powerMngrId = PERIPH_UART0,
        .txPin = Board_UART_TX,
        .rxPin = Board_UART_RX,
        .ctsPin = PIN_UNASSIGNED,
        .rtsPin = PIN_UNASSIGNED
    },
};

/* UART configuration structure */
const UART_Config UART_config[] = {
    { &UARTCC26XX_fxnTable, &uartCC26XXObjects[0], &uartCC26XXHWAttrs[0] },
    { NULL, NULL, NULL }
};
/*
 *  ============================= UART end =====================================
*/

/*
 *  ============================= UDMA begin ===================================
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UDMACC26XX_config, ".const:UDMACC26XX_config")
#pragma DATA_SECTION(udmaHWAttrs, ".const:udmaHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/dma/UDMACC26XX.h>

/* UDMA objects */
UDMACC26XX_Object UdmaObjects[CC2650_UDMACOUNT];

/* UDMA configuration structure */
const UDMACC26XX_HWAttrs udmaHWAttrs[CC2650_UDMACOUNT] = {
    { UDMA0_BASE, INT_UDMAERR, PERIPH_UDMA },
};

/* UDMA configuration structure */
const UDMACC26XX_Config UDMACC26XX_config[] = {
    {&UdmaObjects[0], &udmaHWAttrs[0]},
    {NULL, NULL},
};
/*
 *  ============================= UDMA end =====================================
*/

/*
 *  ========================== SPI DMA begin ===================================
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiCC26XXDMAHWAttrs, ".const:spiCC26XXDMAHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/spi/SPICC26XXDMA.h>

/* SPI objects */
SPICC26XX_Object spiCC26XXDMAObjects[CC2650_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPICC26XX_HWAttrs spiCC26XXDMAHWAttrs[CC2650_SPICOUNT] = {
    {   /* SRF06EB_CC2650_SPI0 */
        .baseAddr = SSI0_BASE,
        .intNum = INT_SSI0,
        .defaultTxBufValue = 0,
        .powerMngrId = PERIPH_SSI0,
        .rxChannelBitMask = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin = Board_SPI0_MOSI,
        .misoPin = Board_SPI0_MISO,
        .clkPin = Board_SPI0_CLK,
        .csnPin = Board_SPI0_CSN
    },
    {   /* SRF06EB_CC2650_SPI1 */
        .baseAddr = SSI1_BASE,
        .intNum = INT_SSI1,
        .defaultTxBufValue = 0,
        .powerMngrId = PERIPH_SSI1,
        .rxChannelBitMask = 1<<UDMA_CHAN_SSI1_RX,
        .txChannelBitMask = 1<<UDMA_CHAN_SSI1_TX,
        .mosiPin = Board_SPI1_MOSI,
        .misoPin = Board_SPI1_MISO,
        .clkPin = Board_SPI1_CLK,
        .csnPin = Board_SPI1_CSN
    }
};

/* SPI configuration structure */
const SPI_Config SPI_config[] = {
    /* SRF06EB_CC2650_SPI0 */
    {&SPICC26XXDMA_fxnTable, &spiCC26XXDMAObjects[0], &spiCC26XXDMAHWAttrs[0]},
    /* SRF06EB_CC2650_SPI1 */
    {&SPICC26XXDMA_fxnTable, &spiCC26XXDMAObjects[1], &spiCC26XXDMAHWAttrs[1]},
    {NULL, NULL, NULL},
};
/*
 *  ========================== SPI DMA end =====================================
*/

/*
 *  ========================== Crypto begin =======================================
 *  NOTE: The Crypto implementaion should be considered experimental and not validated!
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(CryptoCC26XX_config, ".const:CryptoCC26XX_config")
#pragma DATA_SECTION(cryptoCC26XXHWAttrs, ".const:cryptoCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/crypto/CryptoCC26XX.h>

/* Crypto objects */
CryptoCC26XX_Object cryptoCC26XXObjects[CC2650_CRYPTOCOUNT];

/* Crypto configuration structure, describing which pins are to be used */
const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC2650_CRYPTOCOUNT] = {
    {CRYPTO_BASE, INT_CRYPTO, PERIPH_CRYPTO}
};

/* Crypto configuration structure */
const CryptoCC26XX_Config CryptoCC26XX_config[] = {
    {&cryptoCC26XXObjects[0], &cryptoCC26XXHWAttrs[0]},
    {NULL, NULL}
};

/*
 *  ========================== Crypto end =========================================
*/
