/*    Copyright (C) 2015 DIAMS s.r.o. Bratislava
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#ifndef _PIDICNC_H_
#define _PIDICNC_H_

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

enum pin_input_names {
    HOME_X,
    HOME_Y,
    HOME_Z,
    HOME_A,
    STOP
};

enum pin_output_names {
    SPINDLE,
    MIST,
    FLOOD,
    OUTPUT
};

typedef enum {
    UNSUPPORTED = 0,
    RPI,        /* Raspberry Pi 1 */
    RPI_ZERO,   /* Raspberry Pi Zero and Zero W */
    RPI_2,      /* Raspberry Pi 2 */
    RPI_3,      /* Raspberry Pi 3 and Compute Module 3 */
    RPI_4,      /* Raspberry Pi 4 and Compute Module 4 */
    RPI_5       /* Raspberry Pi 5 */
} platform_t;


//  Info:
//  Bit2: Config
//  Bit1: Read
//  Bit0: Write

typedef enum {
    //  BOARDS_INFO_NOTHING = 0,
    //  BOARDS_INFO_CMD     = 0xC5,
    //  BOARDS_INFO_CYCLIC  = 0xAD

    BOARDS_INFO_NOTHING = 0xA0,         //Check ID Boards
    BOARDS_INFO_ONLY_READ = 0xA2,       //Only Read data
    BOARDS_INFO_CYCLIC = 0xA3,          //Write + Read
    BOARDS_INFO_CMD = 0xA4,             //Set Config
    BOARDS_INFO_FIRST = 0xA5            //First board
} BoardsInfo;


#define NUMAXES         4           /* X Y Z A*/
#define PWMCHANS        3
#define NUMOUTPUTS      4
#define NUMINPUTS       5

#define REQ_TIMEOUT     10000ul

#define BUFSIZE         2000


#define PAGE_SIZE       (4*1024)
#define BLOCK_SIZE      (4*1024)

#define CMD_CFG         0x4746433E  // >CFG
#define CMD_TST         0x5453543E  // >TST
#define CMD_CM1         0x314D433E  // >CM1
#define CMD_CM2         0x324D433E  // >CM2
#define CMD_RST         0x5453523E  // >RST

/* Broadcom defines */

#define BCM2835_SPICLKDIV   32      /* ~8 Mhz */

#define BCM2835_PERI_BASE   0x20000000
#define BCM2836_PERI_BASE   0x3F000000
#define BCM2837_PERI_BASE   0x3F000000
#define BCM2711_PERI_BASE   0xFE000000
#define BCM2712_PERI_BASE   0xFE000000

#define GPIO_OFFSET         0x200000 /* GPIO controller */
#define SPI_OFFSET          0x204000 /* SPI controller */

#define BCM2835_GPFSEL0     *(mem1)
#define BCM2835_GPFSEL1     *(mem1 + 1)
#define BCM2835_GPFSEL2     *(mem1 + 2)
#define BCM2835_GPFSEL3     *(mem1 + 3)
#define BCM2835_GPFSEL4     *(mem1 + 4)
#define BCM2835_GPFSEL5     *(mem1 + 5)
#define BCM2835_GPSET0      *(mem1 + 7)
#define BCM2835_GPSET1      *(mem1 + 8)
#define BCM2835_GPCLR0      *(mem1 + 10)
#define BCM2835_GPCLR1      *(mem1 + 11)
#define BCM2835_GPLEV0      *(mem1 + 13)
#define BCM2835_GPLEV1      *(mem1 + 14)

#define BCM2835_SPICS       *(mem2 + 0)
#define BCM2835_SPIFIFO     *(mem2 + 1)
#define BCM2835_SPICLK      *(mem2 + 2)

#define BCM_SPI_CS_DONE     0x00010000
#define BCM_SPI_CS_TA       0x00000080
#define BCM_SPI_CS_CLEAR_RX 0x00000020
#define BCM_SPI_CS_CLEAR_TX 0x00000010
#define BCM_SPI_CS_CPHA     0x00000004

#endif // _PIDICNC_H_
