/*    Copyright (C) 2015 DIAMS s.r.o. Bratislava
 *
 *    Portions of this code is based on stepgen.c by John Kasunich
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

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"


#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h> 


 //-----------------------------------------------------------------


 //-----------------------------------------------------------------

#include "pidi.h"

/* Build for LinuxCNC
#if !defined(BUILD_SYS_USER_DSO)
#error "This driver is for usermode threads only"
#endif
*/

MODULE_AUTHOR("DIAMS");
MODULE_DESCRIPTION("Driver for PiDiCNC boards");
MODULE_LICENSE("GPL v2");

//-----------------------------------------------------------------------------


#pragma pack(1)



#define         DEF_MAX_BOARDS              16                          // Pozor!!! minimum = 16

#define         DEF_MODNAME                 "pidi"                      //"PiDiCNC"

#define         DEF_PREFIX                  "pidi"
#define         DEF_PREFIX_3805             "pidi-3805"
#define         DEF_PREFIX_3809             "pidi-3809"
#define         DEF_PREFIX_3811             "pidi-3811"

#define         DEF_STATUS_DNA_OK           0x20
#define         DEF_STATUS_FIRSTBOARD       0x40
#define         DEF_STATUS_WANT_CONF        0x80

#define         DEF_BOARD_TYPE              0
#define         DEF_BOARD_PARAM_1           1
#define         DEF_BOARD_PARAM_2           2
#define         DEF_BOARD_PARAM_3           3


//-----------------------------------------------------------------------------


#define         DEF_POSITION_CONSTANT_3805  22.89175        //22.888882107

#define         DEF_TIME_1MS                16000           //RLC Constant for 1,0ms

#define         PI                          3.1415926535897932
#define         PI_POL                      1.5707963267948966


#define         DEF_MODE_3805_0             0               //Classic IRC,  counter 32bit, max. 200KHz
#define         DEF_MODE_3805_1             1               //Cyclic IRC, [0.0, enc_set_position], for example rotate [0, 360]
#define         DEF_MODE_3805_2             2               //Bounds IRC, [0.0, enc_set_position], for example speed [0, 120]

#define         DEF_MODE_3811_0             0               //Classic IRC,  counter 32bit, max. 3MHz
#define         DEF_MODE_3811_1             1               //Input A=IRC, Input B=DIR (1=Up,0=Down),   counter 32bit, max. 3MHz
#define         DEF_MODE_3811_2             2               //Input A=Up, Input B=Down, counter 32bit, max. 3MHz
#define         DEF_MODE_3811_3             3               //Input A=Up, Input B=Up,   counter 2x16bit, B=Hight, A=Low, max. 127KHz


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


typedef struct _S_SPI_IN_3805       //
{
    //  int     Time;               //Iba na 1.doske. Urobene posunutim pointra
    short int       Id;
    short int       Position[7];
    short int       IRCposition[2];
    unsigned char   BinIn;
    unsigned char   Status;
} S_SPI_IN_3805;

typedef struct _S_SPI_OUT_3805          //
{
    unsigned char   Info;               //0x00-nic, 0xC5-set, 0xAD-cyklus
    short int       Speed[4];
    short int       BinOut[7];
    unsigned char   BinOut2;            //0-2=out, 3=rele, 4-7=step_enable
    short int       BinOutLed;          //0-8=led
    short int       AnalogOut;
} S_SPI_OUT_3805;

typedef struct //_S_HAL_3805
{
    hal_float_t     *step_speed[7];             //step motor
    hal_float_t     *step_position[7];
    hal_s32_t       *step_stepping[4];          //Pocet impulzov na otacku
    hal_float_t     *step_scale[7];             //Pocet mm / otacku
    hal_bit_t       *step_status[7];            //Status motorov
    hal_bit_t       *step_enable[7];            //Enable motorov - 0-3 + 4-6

    hal_bit_t       *bout_out[11];              //bin out
    hal_bit_t       *bout_invert[11];
    hal_float_t     *bout_pwm_duty_cycle[7];    //Plniaci pomer 0,0=0%, 1,0=100%
    hal_float_t     *bout_angle[7];             //RLC -PI/2 +PI/2
    hal_float_t     *bout_rc_center;            //RLC - 1,5 alebo 1,6
    hal_float_t     *bout_rc_range;             //RLC - 0,5 alebo 0,8

    hal_bit_t       *bin_in[5];                 //bin in
    hal_bit_t       *bin_in_not[5];

    hal_float_t     *dac_value;                 //analog out
    hal_bit_t       *dac_enable;
    hal_float_t     *dac_offset;
    hal_float_t     *dac_scale;
    hal_float_t     *dac_high_limit;
    hal_float_t     *dac_low_limit;
    hal_float_t     *dac_bit_weight;
    hal_float_t     *dac_hw_offset;
    hal_float_t     *dac_hw_scale;

    hal_s32_t       *enc_counts[2];             //encoder in
    hal_float_t     *enc_position[2];
    hal_float_t     *enc_velocity[2];
    hal_float_t     *enc_scale[2];
    hal_s32_t       *enc_mode[2];
    hal_float_t     *enc_set_position[2];
    hal_bit_t       *enc_set[2];

    hal_bit_t       *bout_led[9];               //led


    char                boutinvert[7];
    char                UsedMotors[7];
    unsigned short int  FirstPos[7];
    unsigned short int  LastPos[7];
    //  float           ffposition[7];
    int	                HightPos[7];

    unsigned short int  enc_FirstPos[2];
    unsigned short int  enc_LastPos[2];
    int                 enc_HightPos[2];

    int                 enc_lastposition[2];
} S_HAL_3805;



typedef struct _S_SPI_IN_3809
{
    short int       Id;
    short int       BinIn;
    unsigned char   Free;
} S_SPI_IN_3809;

typedef struct _S_SPI_OUT_3809          //
{
    unsigned char   Info;               //0x00-nic, 0xE5-set, 0xAD-cyklus
    short int       BinOut;
    unsigned char   Led;
} S_SPI_OUT_3809;

typedef struct //_S_HAL_3809
{
    hal_bit_t       *bout_out[16];      //bin out
    hal_bit_t       *bout_invert[16];
    hal_bit_t       *bout_led[4];       //led

    hal_bit_t       *bin_in[16];        //bin in
    hal_bit_t       *bin_in_not[16];
} S_HAL_3809;



typedef struct _S_SPI_IN_3811
{
    short int       Id;
    short int       IRCposition[4];
    short int       ADCvalue;           //16bit
    unsigned char   Status;
} S_SPI_IN_3811;

typedef struct _S_SPI_OUT_3811          //
{
    unsigned char   Info;               //0x00-nic, 0xE5-set, 0xAD-cyklus
    short int       DACvalue[5];        //12bit
    short int       Led;                //0-8=led
} S_SPI_OUT_3811;

typedef struct //_S_HAL_3811
{
    hal_float_t     *dac_value[5];      //analog out
    hal_bit_t       *dac_enable[5];
    hal_float_t     *dac_offset[5];
    hal_float_t     *dac_scale[5];
    hal_float_t     *dac_high_limit[5];
    hal_float_t     *dac_low_limit[5];
    hal_float_t     *dac_bit_weight[5];
    hal_float_t     *dac_hw_offset[5];
    hal_float_t     *dac_hw_scale[5];

    hal_float_t     *adc_value;         //analog in
    hal_float_t     *adc_offset;
    hal_float_t     *adc_scale;
    hal_float_t     *adc_bit_weight;
    hal_float_t     *adc_hw_offset;
    hal_float_t     *adc_hw_scale;

    hal_s32_t       *enc_counts[4];     //encoder in
    hal_float_t     *enc_position[4];
    hal_float_t     *enc_velocity[4];
    hal_float_t     *enc_scale[4];
    hal_s32_t       *enc_mode[4];
    hal_float_t     *enc_set_position[4];
    hal_bit_t       *enc_set[4];

    hal_bit_t       *bout_led[9];       //led


    unsigned short int  FirstPos[4];
    unsigned short int  LastPos[4];
    //  float           ffposition[4];
    int                 HightPos[7];

    unsigned char       FirstPosB[4];
    unsigned char       LastPosB[4];
    int                 HightPosB[7];

    //  float           enc_lastposition[4];
    int                 enc_lastposition[4];
} S_HAL_3811;



typedef struct //_S_HAL_BOARDS
{
    hal_s32_t       *Type;
    //iba vnutorne premenne
    void            *HalPointer;    //pointer na HAL
    void            *SpiWrPointer;  //pointer na SPI data write
    void            *SpiRdPointer;  //pointer na SPI data read
    int             SpiWrLen;       //length SPI write data
} S_HAL_BOARDS;

S_HAL_BOARDS        *Boards;


typedef struct //_S_HAL_GLOBALS
{
    hal_bit_t       *estop;         //TotalStop od programu
    hal_u32_t       *time;          //Time


    hal_s32_t       *x_debug[20];   //for debug
    hal_s32_t       *x_rd[100];     //for debug
    hal_s32_t       *x_wr[100];     //for debug
} S_HAL_GLOBALS;

S_HAL_GLOBALS   *Globals;



#pragma pack()


//-----------------------------------------------------------------------------


static char         *data;

static int          comp_id = -1;
static const char   *modname = DEF_MODNAME;
static const char   *prefix = DEF_PREFIX;

static bool         b_debug1 = false;           //cyklus
static bool         b_debug2 = false;           //config
static bool         b_debug3 = false;           //checkid
static bool         b_rlmode = false;


//-----------------------------------------------------------------------------



static platform_t   platform;
volatile unsigned   *mem1, *mem2;

char                txBuf[BUFSIZE], rxBuf[BUFSIZE];

static char         MakeConfigure = 1;
static char         LastEstop = 0;
static unsigned int ActTime = 0;

//static char       MotorError = 0;

static bool         b_FirstPos = true;



static void HwComm(void *arg, long period);
static void HwConfigure();

static int map_gpio();
static void(*setup_gpio)();
static void(*restore_gpio)();
static void rpi_setup_gpio();
static void rpi_restore_gpio();
static void write_read_buf(char* tbuf, char* rbuf, int len);
static platform_t check_platform(void);
static int CheckIdBoars();
static void SetFirstBoard();

static void Set3805EncPosition(S_HAL_3805 *pHal_3805, S_SPI_IN_3805 *p_SPI_IN_3805, int encid, float value);


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------


#define LOW                                 0

/* Defines for SPI
   GPIO register offsets from BCM2835_SPI0_BASE.
   Offsets into the SPI Peripheral block in bytes per 10.5 SPI Register Map
*/
/* Register masks for SPI0_CS */
#define BCM2835_SPI0_CS_LEN_LONG             0x02000000 /*!< Enable Long data word in Lossi mode if DMA_LEN is set */
#define BCM2835_SPI0_CS_DMA_LEN              0x01000000 /*!< Enable DMA mode in Lossi mode */
#define BCM2835_SPI0_CS_CSPOL2               0x00800000 /*!< Chip Select 2 Polarity */
#define BCM2835_SPI0_CS_CSPOL1               0x00400000 /*!< Chip Select 1 Polarity */
#define BCM2835_SPI0_CS_CSPOL0               0x00200000 /*!< Chip Select 0 Polarity */
#define BCM2835_SPI0_CS_RXF                  0x00100000 /*!< RXF - RX FIFO Full */
#define BCM2835_SPI0_CS_RXR                  0x00080000 /*!< RXR RX FIFO needs Reading (full) */
#define BCM2835_SPI0_CS_TXD                  0x00040000 /*!< TXD TX FIFO can accept Data */
#define BCM2835_SPI0_CS_RXD                  0x00020000 /*!< RXD RX FIFO contains Data */
#define BCM2835_SPI0_CS_DONE                 0x00010000 /*!< Done transfer Done */
#define BCM2835_SPI0_CS_TE_EN                0x00008000 /*!< Unused */
#define BCM2835_SPI0_CS_LMONO                0x00004000 /*!< Unused */
#define BCM2835_SPI0_CS_LEN                  0x00002000 /*!< LEN LoSSI enable */
#define BCM2835_SPI0_CS_REN                  0x00001000 /*!< REN Read Enable */
#define BCM2835_SPI0_CS_ADCS                 0x00000800 /*!< ADCS Automatically Deassert Chip Select */
#define BCM2835_SPI0_CS_INTR                 0x00000400 /*!< INTR Interrupt on RXR */
#define BCM2835_SPI0_CS_INTD                 0x00000200 /*!< INTD Interrupt on Done */
#define BCM2835_SPI0_CS_DMAEN                0x00000100 /*!< DMAEN DMA Enable */
#define BCM2835_SPI0_CS_TA                   0x00000080 /*!< Transfer Active */
#define BCM2835_SPI0_CS_CSPOL                0x00000040 /*!< Chip Select Polarity */
#define BCM2835_SPI0_CS_CLEAR                0x00000030 /*!< Clear FIFO Clear RX and TX */
#define BCM2835_SPI0_CS_CLEAR_RX             0x00000020 /*!< Clear FIFO Clear RX  */
#define BCM2835_SPI0_CS_CLEAR_TX             0x00000010 /*!< Clear FIFO Clear TX  */
#define BCM2835_SPI0_CS_CPOL                 0x00000008 /*!< Clock Polarity */
#define BCM2835_SPI0_CS_CPHA                 0x00000004 /*!< Clock Phase */
#define BCM2835_SPI0_CS_CS                   0x00000003 /*!< Chip Select */

/*! \brief bcm2835SPIBitOrder SPI Bit order
  Specifies the SPI data bit ordering for bcm2835_spi_setBitOrder()
*/
typedef enum
{
    BCM2835_SPI_BIT_ORDER_LSBFIRST = 0,  /*!< LSB First */
    BCM2835_SPI_BIT_ORDER_MSBFIRST = 1   /*!< MSB First */
} bcm2835SPIBitOrder;

/*! \brief SPI Data mode
  Specify the SPI data mode to be passed to bcm2835_spi_setDataMode()
*/
typedef enum
{
    BCM2835_SPI_MODE0 = 0,  /*!< CPOL = 0, CPHA = 0 */
    BCM2835_SPI_MODE1 = 1,  /*!< CPOL = 0, CPHA = 1 */
    BCM2835_SPI_MODE2 = 2,  /*!< CPOL = 1, CPHA = 0 */
    BCM2835_SPI_MODE3 = 3   /*!< CPOL = 1, CPHA = 1 */
} bcm2835SPIMode;

/*! \brief bcm2835SPIChipSelect
  Specify the SPI chip select pin(s)
*/
typedef enum
{
    BCM2835_SPI_CS0 = 0,     /*!< Chip Select 0 */
    BCM2835_SPI_CS1 = 1,     /*!< Chip Select 1 */
    BCM2835_SPI_CS2 = 2,     /*!< Chip Select 2 (ie pins CS1 and CS2 are asserted) */
    BCM2835_SPI_CS_NONE = 3  /*!< No CS, control it yourself */
} bcm2835SPIChipSelect;

/*! \brief bcm2835SPIClockDivider
  Specifies the divider used to generate the SPI clock from the system clock.
  Figures below give the divider, clock period and clock frequency.
  Clock divided is based on nominal base clock rate of 250MHz
  It is reported that (contrary to the documentation) any even divider may used.
  The frequencies shown for each divider have been confirmed by measurement
*/
typedef enum
{
    BCM2835_SPI_CLOCK_FREQUENCY_3_81kHz =   0,       /*!< 3.814697260kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_7_62kHz =   32768,   /*!< 7.629394531kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_15_25kHz =  16384,   /*!< 15.25878906kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_30_51kHz =  8192,    /*!< 30.51757813kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_61_03kHz =  4096,    /*!< 61.03515625kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_122_07kHz = 2048,    /*!< 122.0703125kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_244_14kHz = 1024,    /*!< 244.140625kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_488_28kHz = 512,     /*!< 488.28125kHz */
    BCM2835_SPI_CLOCK_FREQUENCY_976_56kHz = 256,     /*!< 976.5625MHz */
    BCM2835_SPI_CLOCK_FREQUENCY_1_95MHz =   128,     /*!< 1.953125MHz */
    BCM2835_SPI_CLOCK_FREQUENCY_3_90MHz =   64,      /*!< 3.90625MHz */
    BCM2835_SPI_CLOCK_FREQUENCY_7_81MHz =   32,      /*!< 7.8125MHz */
    BCM2835_SPI_CLOCK_FREQUENCY_15_62MHz =  16,      /*!< 15.625MHz */
    BCM2835_SPI_CLOCK_FREQUENCY_31_25MHz =  8,       /*!< 31.25MHz */
    BCM2835_SPI_CLOCK_FREQUENCY_62_5MHz =   4,       /*!< 62.5MHz */
    BCM2835_SPI_CLOCK_FREQUENCY_125MHz =    2,       /*!< 125MHz, fastest you can get */
//  BCM2835_SPI_CLOCK_DIVIDER_1 =           1        /*!< 1 = 262.144us = 3.814697260kHz, same as 0/65536 */
} bcm2835SPIClockFrequency;


//static u8     debug               = 0;
//static u8     debug               = RTAPI_MSG_ALL;


/*
#if !( defined(__ARM_ARCH_6__) && !defined( __ARM_ARCH_7__ ) )
#define BCM2835_HAVE_DMB
#endif
*/

u32 bcm2835_peri_read(volatile u32* paddr);
void bcm2835_peri_write(volatile u32* paddr, u32 value);
void bcm2835_peri_set_bits(volatile u32* paddr, u32 value, u32 mask);
void bcm2835_spi_setDataMode(u8 mode);
void bcm2835_spi_setClockDivider(u16 divider);
void bcm2835_spi_chipSelect(u8 cs);
void bcm2835_spi_setChipSelectPolarity(u8 cs, u8 active);


u32             tmpdata = 123;
int             itmpdata = 0;

int             BoardTypeDetect[DEF_MAX_BOARDS];



//----------------------------------------------------------------------------------------


#define         DEF_MAX_BOARDS_PARAMETERS       (1+2+1)

int             *BoardParam[DEF_MAX_BOARDS];

int             Board0[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board0, 10, "Board0 Parameter")
int             Board1[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board1, 10, "Board1 Parameter")
int             Board2[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board2, 10, "Board2 Parameter")
int             Board3[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board3, 10, "Board3 Parameter")

int             Board4[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board4, 10, "Board4 Parameter")
int             Board5[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board5, 10, "Board5 Parameter")
int             Board6[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board6, 10, "Board6 Parameter")
int             Board7[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board7, 10, "Board7 Parameter")

int             Board8[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board8, 10, "Board8 Parameter")
int             Board9[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board9, 10, "Board9 Parameter")
int             Board10[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board10, 10, "Board10 Parameter")
int             Board11[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board11, 10, "Board11 Parameter")

int             Board12[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board12, 10, "Board12 Parameter")
int             Board13[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board13, 10, "Board13 Parameter")
int             Board14[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board14, 10, "Board14 Parameter")
int             Board15[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Board15, 10, "Board15 Parameter")


int             Debug[] = { [0 ... DEF_MAX_BOARDS_PARAMETERS - 1] = 0 };
RTAPI_MP_ARRAY_INT(Debug, 10, "Debug Parameter")


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------


//int rtapi_app_main(void)
int rtapi_app_main(char *argv)
{
    char        name[HAL_NAME_LEN + 1];
    int         n, retval;
    int         ii, jj, kk;
    int         off, offwr, offrd, len;
    int	        allmode, mode, motor;


    rtapi_set_msg_level(RTAPI_MSG_ALL);
    rtapi_set_msg_level(7);

    /*
    ii = rtapi_get_msg_level();
    rtapi_print_msg( RTAPI_MSG_ERR, "\nDIAMS START_5 %d\n", ii );
    rtapi_print_msg( RTAPI_MSG_ERR, "\nDIAMS START_6 %d\n", RTAPI_MSG_ERR );

    rtapi_print_msg( RTAPI_MSG_ERR, "\nargv=%X\n", argv );
    if( argv != NULL )
        {
    //  rtapi_print_msg( RTAPI_MSG_ERR, "\n*argv=%X\n\n\n", *(int *)argv );
    //  rtapi_print_msg( RTAPI_MSG_ERR, "\n**argv=%X\n\n\n", *((int *)argv+1) );
        }

    rtapi_print_msg( RTAPI_MSG_ERR, "\n info=%d, err=%d, all=%d, act=%d\n", RTAPI_MSG_INFO, RTAPI_MSG_ERR, RTAPI_MSG_ALL, ii );
    */
    //rtapi_print_msg( RTAPI_MSG_ERR, "Boards=%d,%d,%d,%d", BoardParam[0][0], BoardParam[1][0], BoardParam[2][0], BoardParam[3][0]);

    /*
    for( ii=0; ii<DEF_MAX_BOARDS; ii++ )
        BoardParam[ii][0] = 0;
    */


    BoardParam[0] = Board0;
    BoardParam[1] = Board1;
    BoardParam[2] = Board2;
    BoardParam[3] = Board3;
    BoardParam[4] = Board4;
    BoardParam[5] = Board5;
    BoardParam[6] = Board6;
    BoardParam[7] = Board7;
    BoardParam[8] = Board8;
    BoardParam[9] = Board9;
    BoardParam[10] = Board10;
    BoardParam[11] = Board11;
    BoardParam[12] = Board12;
    BoardParam[13] = Board13;
    BoardParam[14] = Board14;
    BoardParam[15] = Board15;

    rtapi_print_msg(RTAPI_MSG_ERR, "Boards=%d,%d,%d,%d", BoardParam[0][DEF_BOARD_TYPE], BoardParam[1][DEF_BOARD_TYPE], BoardParam[2][DEF_BOARD_TYPE], BoardParam[3][DEF_BOARD_TYPE]);

    MakeConfigure = 1;

    ii = Debug[0];
    if ((ii & 0x01) == 0x01)        b_debug1 = true;    // cyklus
    if ((ii & 0x02) == 0x02)        b_debug2 = true;    // config
    if ((ii & 0x04) == 0x04)        b_debug3 = true;    // checkid

    platform = check_platform();

    switch (platform)
    {
    case RPI:
    case RPI_2:
    case RPI_3:
        setup_gpio = rpi_setup_gpio;
        restore_gpio = rpi_restore_gpio;
        break;
    case RPI_4:
        setup_gpio = rpi_setup_gpio;
        restore_gpio = rpi_restore_gpio;
        break;
    case RPI_5:
        setup_gpio = rpi_setup_gpio;
        restore_gpio = rpi_restore_gpio;
        break;
    case UNSUPPORTED:
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: Unsupported platform detected.\n",
            modname);
        return(-1);
    default:
        rtapi_print_msg(RTAPI_MSG_ERR,
            "%s: ERROR: This driver is not for this platform.\n",
            modname);
        return(-1);
    }

    /* initialise driver */
    comp_id = hal_init(modname);
    if (comp_id < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_init() failed\n", modname);
        return(-10 + comp_id);
    }




    Boards = (S_HAL_BOARDS *)hal_malloc(sizeof(S_HAL_BOARDS) * DEF_MAX_BOARDS);
    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] != 0)
        {
            prefix = DEF_PREFIX;
            retval = hal_pin_s32_newf(HAL_IN, &Boards[ii].Type, comp_id,
                "%s.%d.type", prefix, ii);
            if (retval < 0)
            {
                retval = -20 + retval;
                goto error;
            }
            *(Boards[ii].Type) = BoardParam[ii][DEF_BOARD_TYPE];
        }
        Boards[ii].HalPointer = NULL;
        Boards[ii].SpiWrPointer = NULL;
        Boards[ii].SpiRdPointer = NULL;
        Boards[ii].SpiWrLen = 0;
    }


    prefix = DEF_PREFIX;

    Globals = (S_HAL_GLOBALS *)hal_malloc(sizeof(S_HAL_GLOBALS));
    retval = hal_pin_bit_newf(HAL_IN, &Globals->estop, comp_id, "%s.estop", prefix);
    if (retval < 0)
    {
        retval = -32;   goto error;
    }
    *Globals->estop = 0;
    LastEstop = 0;

    retval = hal_pin_u32_newf(HAL_OUT, &Globals->time, comp_id, "%s.time", prefix);
    if (retval < 0)
    {
        retval = -32;   goto error;
    }
    *Globals->time = 0;

    if (b_debug1 || b_debug2 || b_debug3)
    {
        for (jj = 0; jj < 20; jj++)
        {
            retval = hal_pin_s32_newf(HAL_OUT, &Globals->x_debug[jj], comp_id,
                "x_debug%02d", jj);
            if (retval < 0)	goto error;
            *Globals->x_debug[jj] = 0;
        }
        for (jj = 0; jj < 100; jj++)
        {
            retval = hal_pin_s32_newf(HAL_OUT, &Globals->x_rd[jj], comp_id,
                "x_rd%02d", jj);
            if (retval < 0)	goto error;
            *Globals->x_rd[jj] = 0;

            retval = hal_pin_s32_newf(HAL_OUT, &Globals->x_wr[jj], comp_id,
                "x_wr%02d", jj);
            if (retval < 0)	goto error;
            *Globals->x_wr[jj] = 0;
        }
    }



    /* allocate shared memory */
    off = 0;
    offwr = 0;
    if (BoardParam[0][DEF_BOARD_TYPE] == 3809)  offrd = 0;
    else							offrd = 4;
    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)    break;
        switch (BoardParam[ii][DEF_BOARD_TYPE])
        {
        case 3805:	off += sizeof(S_HAL_3805);
            Boards[ii].SpiWrPointer = txBuf + offwr;
            Boards[ii].SpiRdPointer = rxBuf + offrd;

            len = sizeof(S_SPI_OUT_3805);
            if (len < sizeof(S_SPI_IN_3805))    len = sizeof(S_SPI_IN_3805);
            offwr += len;
            offrd = offwr + ii + 1;
            Boards[ii].SpiWrLen = offrd;
            break;

        case 3809:	off += sizeof(S_HAL_3809);
            Boards[ii].SpiWrPointer = txBuf + offwr;
            Boards[ii].SpiRdPointer = rxBuf + offrd;

            len = sizeof(S_SPI_OUT_3809);
            if (len < sizeof(S_SPI_IN_3809))    len = sizeof(S_SPI_IN_3809);
            offwr += len;
            offrd = offwr + ii + 1;
            Boards[ii].SpiWrLen = offrd;
            break;

        case 3811:	off += sizeof(S_HAL_3811);
            Boards[ii].SpiWrPointer = txBuf + offwr;
            Boards[ii].SpiRdPointer = rxBuf + offrd;

            len = sizeof(S_SPI_OUT_3811);
            if (len < sizeof(S_SPI_IN_3811))    len = sizeof(S_SPI_IN_3811);
            offwr += len;
            offrd = offwr + ii + 1;
            Boards[ii].SpiWrLen = offrd;
            break;

        default:	break;
        }
        /*
        Premenne:   Boards[ii].SpiWrPointer
                    Boards[ii].SpiRdPointer
                    Boards[ii].SpiWrLen
        sa nastavuju aj vo funkcii: CheckIdBoars();
        */
    }
    if (off == 0)
    {
        return(-30);
    }

    off += 1000;
    data = (char *)hal_malloc(off);
    if (data == 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
        hal_exit(comp_id);
        return(-40);
    }
    memset(data, 0, off);

    /* configure board */
    retval = map_gpio();
    if (retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: cannot map GPIO memory\n", modname);
        return(-50 + retval);
    }

    setup_gpio();




    //  bcm2835_spi_setDataMode( BCM2835_SPI_MODE3 );
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_FREQUENCY_15_62MHz);
    /*
    bcm2835_spi_setDataMode( BCM2835_SPI_MODE0 );
    bcm2835_spi_setClockDivider( BCM2835_SPI_CLOCK_FREQUENCY_15_62MHz );
    bcm2835_spi_chipSelect( BCM2835_SPI_CS0 );
    bcm2835_spi_setChipSelectPolarity( BCM2835_SPI_CS0, LOW );
    */


    /* export pins and parameters */
    //------------------------

    off = 0;
    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)    break;

        switch (BoardParam[ii][DEF_BOARD_TYPE])
        {
        case 3805:
        {
            S_HAL_3805  *pHal_3805 = (S_HAL_3805 *)(&data[off]);
            Boards[ii].HalPointer = (void *)pHal_3805;
            off += sizeof(S_HAL_3805);

            for (jj = 0; jj < 7; jj++)
            {
                pHal_3805->boutinvert[jj] = -1;
                pHal_3805->UsedMotors[jj] = 0;
                pHal_3805->FirstPos[jj] = 0;
                pHal_3805->LastPos[jj] = 0;
                // pHal_3805->ffposition[jj] = 0.0;
                pHal_3805->HightPos[jj] = 0;
            }

            for (jj = 0; jj < 2; jj++)
            {
                pHal_3805->enc_FirstPos[jj] = 0;
                pHal_3805->enc_LastPos[jj] = 0;
                // pHal_3805->ffposition[jj] = 0.0;
                pHal_3805->enc_HightPos[jj] = 0;

                // pHal_3805->enc_lastposition[jj] = 0.0;
                pHal_3805->enc_lastposition[jj] = 0;
            }

            prefix = DEF_PREFIX_3805;

            for (jj = 0; jj < 4; jj++)
            {
                pHal_3805->UsedMotors[jj] = 1;


                retval = hal_pin_float_newf(HAL_IN, &pHal_3805->step_speed[jj], comp_id,
                    "%s.%d.step.%d.speed", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3805->step_speed[jj]) = 0.0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3805->step_scale[jj], comp_id,
                    "%s.%d.step.%d.scale", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3805->step_scale[jj]) = 6400.0;

                retval = hal_pin_s32_newf(HAL_IN, &pHal_3805->step_stepping[jj], comp_id,
                    "%s.%d.step.%d.stepping", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3805->step_stepping[jj]) = 32;

                retval = hal_pin_float_newf(HAL_OUT, &pHal_3805->step_position[jj], comp_id,
                    "%s.%d.step.%d.position", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3805->step_position[jj]) = 0.0;

                retval = hal_pin_bit_newf(HAL_OUT, &pHal_3805->step_status[jj], comp_id,
                    "%s.%d.step.%d.status", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3805->step_status[jj]) = 1;

                retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->step_enable[jj], comp_id,
                    "%s.%d.step.%d.enable", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3805->step_enable[jj]) = 0;
            }


            retval = hal_pin_float_newf(HAL_IN, &pHal_3805->dac_value, comp_id,
                "%s.%d.dac.0.value", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3805->dac_value) = 0.0;

            retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->dac_enable, comp_id,
                "%s.%d.dac.0.enable", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3805->dac_enable) = 1;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3805->dac_offset, comp_id,
                "%s.%d.dac.0.offset", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3805->dac_offset) = 0.0;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3805->dac_scale, comp_id,
                "%s.%d.dac.0.scale", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3805->dac_scale) = 1.0;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3805->dac_high_limit, comp_id,
                "%s.%d.dac.0.high_limit", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3805->dac_high_limit) = 10.0;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3805->dac_low_limit, comp_id,
                "%s.%d.dac.0.low_limit", prefix, ii);
            if (retval < 0)	goto error;
            *(pHal_3805->dac_low_limit) = 0.0;

            retval = hal_pin_float_newf(HAL_OUT, &pHal_3805->dac_bit_weight, comp_id,
                "%s.%d.dac.0.bit_weight", prefix, ii);
            if (retval < 0)	goto error;
            *(pHal_3805->dac_bit_weight) = 10.0 / 4096.0;       //(double)( 2 ^ 12 );

            retval = hal_pin_float_newf(HAL_IN, &pHal_3805->dac_hw_offset, comp_id,
                "%s.%d.dac.0.hw_offset", prefix, ii);
            if (retval < 0)	goto error;
            *(pHal_3805->dac_hw_offset) = 0.0;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3805->dac_hw_scale, comp_id,
                "%s.%d.dac.0.hw_scale", prefix, ii);
            if (retval < 0)	goto error;
            *(pHal_3805->dac_hw_scale) = 1.0;


            allmode = BoardParam[ii][DEF_BOARD_PARAM_2];
            for (jj = 0; jj < 7; jj++)
            {
                mode = allmode % 10;
                allmode /= 10;
                if (mode == 3)
                {
                    b_rlmode = true;
                    break;
                }
            }
            if (b_rlmode)
            {
                retval = hal_pin_float_newf(HAL_IN, &pHal_3805->bout_rc_center, comp_id,
                    "%s.%d.bout.rc-center", prefix, ii);
                if (retval < 0)	goto error;
                *(pHal_3805->bout_rc_center) = 1.5;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3805->bout_rc_range, comp_id,
                    "%s.%d.bout.rc-range", prefix, ii);
                if (retval < 0)	goto error;
                *(pHal_3805->bout_rc_range) = 0.5;
            }

            allmode = BoardParam[ii][DEF_BOARD_PARAM_2];
            for (jj = 0; jj < 7; jj++)
            {
                mode = allmode % 10;
                allmode /= 10;
                switch (mode)
                {
                case 1: retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->bout_out[jj], comp_id,
                    "%s.%d.bout.%d.out", prefix, ii, jj);
                    if (retval < 0)	goto error;
                    *(pHal_3805->bout_out[jj]) = 0;

                    retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->bout_invert[jj], comp_id,
                        "%s.%d.bout.%d.invert", prefix, ii, jj);
                    if (retval < 0)	goto error;
                    *(pHal_3805->bout_invert[jj]) = 0;
                    break;

                case 2: retval = hal_pin_float_newf(HAL_IN, &pHal_3805->bout_pwm_duty_cycle[jj], comp_id,
                    "%s.%d.bout.%d.pwm-duty-cycle", prefix, ii, jj);
                    if (retval < 0)	goto error;
                    *(pHal_3805->bout_pwm_duty_cycle[jj]) = 0.5;

                    retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->bout_invert[jj], comp_id,
                        "%s.%d.bout.%d.invert", prefix, ii, jj);
                    if (retval < 0)	goto error;
                    *(pHal_3805->bout_invert[jj]) = 0;
                    break;

                case 3: retval = hal_pin_float_newf(HAL_IN, &pHal_3805->bout_angle[jj], comp_id,
                    "%s.%d.bout.%d.angle", prefix, ii, jj);
                    if (retval < 0)	goto error;
                    *(pHal_3805->bout_angle[jj]) = 0.0;

                    retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->bout_invert[jj], comp_id,
                        "%s.%d.bout.%d.invert", prefix, ii, jj);
                    if (retval < 0)	goto error;
                    *(pHal_3805->bout_invert[jj]) = 0;
                    break;

                case 4: if (jj < 6)
                {
                    if ((jj == 1) || (jj == 3) || (jj == 5))	return(-80);
                    motor = 4 + jj / 2;

                    pHal_3805->UsedMotors[motor] = 1;

                    retval = hal_pin_float_newf(HAL_IN, &pHal_3805->step_speed[motor], comp_id,
                        "%s.%d.step.%d.speed", prefix, ii, motor);
                    if (retval < 0)	goto error;
                    *(pHal_3805->step_speed[motor]) = 0.0;

                    retval = hal_pin_float_newf(HAL_IN, &pHal_3805->step_scale[motor], comp_id,
                        "%s.%d.step.%d.scale", prefix, ii, motor);
                    if (retval < 0)	goto error;
                    *(pHal_3805->step_scale[motor]) = 200.0;

                    retval = hal_pin_float_newf(HAL_OUT, &pHal_3805->step_position[motor], comp_id,
                        "%s.%d.step.%d.position", prefix, ii, motor);
                    if (retval < 0)	goto error;
                    *(pHal_3805->step_position[motor]) = 0.0;

                    retval = hal_pin_bit_newf(HAL_OUT, &pHal_3805->step_status[motor], comp_id,
                        "%s.%d.step.%d.status", prefix, ii, motor);
                    if (retval < 0)	goto error;
                    *(pHal_3805->step_status[motor]) = 1;

                    retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->step_enable[motor], comp_id,
                        "%s.%d.step.%d.enable", prefix, ii, motor);
                    if (retval < 0)	goto error;
                    *(pHal_3805->step_enable[motor]) = 0;
                }
                        allmode /= 10;
                        jj++;
                        break;

                default:
                    break;
                }
            }


            for (jj = 7; jj <= 10; jj++)
            {
                retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->bout_out[jj], comp_id,
                    "%s.%d.bout.%d.out", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3805->bout_out[jj]) = 0;

                retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->bout_invert[jj], comp_id,
                    "%s.%d.bout.%d.invert", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3805->bout_invert[jj]) = 0;
            }



            allmode = BoardParam[ii][DEF_BOARD_PARAM_1];
            for (jj = 0; jj < 5; jj++)
            {
                mode = allmode % 10;
                allmode /= 10;
                switch (mode)
                {
                case 1:	retval = hal_pin_bit_newf(HAL_OUT, &pHal_3805->bin_in[jj], comp_id,
                    "%s.%d.bin.%d.in", prefix, ii, jj);
                    if (retval < 0) goto error;
                    *(pHal_3805->bin_in[jj]) = 0;

                    retval = hal_pin_bit_newf(HAL_OUT, &pHal_3805->bin_in_not[jj], comp_id,
                        "%s.%d.bin.%d.in-not", prefix, ii, jj);
                    if (retval < 0) goto error;
                    *(pHal_3805->bin_in_not[jj]) = 1;
                    break;

                case 2:	if (jj < 4)
                {
                    if ((jj == 1) || (jj == 3))	return(-80);
                    kk = jj / 2;

                    retval = hal_pin_s32_newf(HAL_OUT, &pHal_3805->enc_counts[kk], comp_id,
                        "%s.%d.enc.%d.counts", prefix, ii, kk);
                    if (retval < 0) goto error;
                    *(pHal_3805->enc_counts[kk]) = 0;

                    retval = hal_pin_float_newf(HAL_OUT, &pHal_3805->enc_position[kk], comp_id,
                        "%s.%d.enc.%d.position", prefix, ii, kk);
                    if (retval < 0) goto error;
                    *(pHal_3805->enc_position[kk]) = 0.0;

                    retval = hal_pin_float_newf(HAL_OUT, &pHal_3805->enc_velocity[kk], comp_id,
                        "%s.%d.enc.%d.velocity", prefix, ii, kk);
                    if (retval < 0) goto error;
                    *(pHal_3805->enc_velocity[kk]) = 0.0;

                    retval = hal_pin_float_newf(HAL_IN, &pHal_3805->enc_scale[kk], comp_id,
                        "%s.%d.enc.%d.scale", prefix, ii, kk);
                    if (retval < 0) goto error;
                    *(pHal_3805->enc_scale[kk]) = 1.0;

                    retval = hal_pin_s32_newf(HAL_IN, &pHal_3805->enc_mode[kk], comp_id,
                        "%s.%d.enc.%d.mode", prefix, ii, kk);
                    if (retval < 0) goto error;
                    *(pHal_3805->enc_mode[kk]) = DEF_MODE_3805_0;

                    retval = hal_pin_float_newf(HAL_IN, &pHal_3805->enc_set_position[kk], comp_id,
                        "%s.%d.enc.%d.set-position", prefix, ii, kk);
                    if (retval < 0) goto error;
                    *(pHal_3805->enc_set_position[kk]) = 0.0;

                    retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->enc_set[kk], comp_id,
                        "%s.%d.enc.%d.set", prefix, ii, kk);
                    if (retval < 0) goto error;
                    *(pHal_3805->enc_set[kk]) = 0;
                    /*
                                                        retval = hal_pin_bit_newf( HAL_IN, &pHal_3805->enc_clr[kk], comp_id,
                                                                            "%s.%d.enc.%d.clr", prefix, ii, kk );
                                                        if( retval < 0 )	goto error;
                                                        *(pHal_3805->enc_clr[kk]) = 0;
                    */
                }

                        allmode /= 10;
                        jj++;
                        break;

                default:
                    break;
                }
            }


            for (jj = 0; jj < 9; jj++)
            {
                retval = hal_pin_bit_newf(HAL_IN, &pHal_3805->bout_led[jj], comp_id,
                    "%s.%d.bled.%d.out", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3805->bout_led[jj]) = 0;
            }
        }
        break;

        case 3809:
        {
            S_HAL_3809  *pHal_3809 = (S_HAL_3809 *)(&data[off]);
            Boards[ii].HalPointer = (void *)pHal_3809;
            off += sizeof(S_HAL_3809);

            prefix = DEF_PREFIX_3809;

            for (jj = 0; jj < 16; jj++)
            {
                retval = hal_pin_bit_newf(HAL_IN, &pHal_3809->bout_out[jj], comp_id,
                    "%s.%d.bout.%d.out", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3809->bout_out[jj]) = 0;

                retval = hal_pin_bit_newf(HAL_IN, &pHal_3809->bout_invert[jj], comp_id,
                    "%s.%d.bout.%d.invert", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3809->bout_invert[jj]) = 0;

                retval = hal_pin_bit_newf(HAL_OUT, &pHal_3809->bin_in[jj], comp_id,
                    "%s.%d.bin.%d.in", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3809->bin_in[jj]) = 0;

                retval = hal_pin_bit_newf(HAL_OUT, &pHal_3809->bin_in_not[jj], comp_id,
                    "%s.%d.bin.%d.in-not", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3809->bin_in_not[jj]) = 1;
            }

            for (jj = 0; jj < 4; jj++)
            {
                retval = hal_pin_bit_newf(HAL_IN, &pHal_3809->bout_led[jj], comp_id,
                    "%s.%d.bled.%d.out", prefix, ii, jj);
                if (retval < 0)	goto error;
                *(pHal_3809->bout_led[jj]) = 0;
            }
        }
        break;

        case 3811:
        {
            S_HAL_3811  *pHal_3811 = (S_HAL_3811 *)(&data[off]);
            Boards[ii].HalPointer = (void *)pHal_3811;
            off += sizeof(S_HAL_3811);

            prefix = DEF_PREFIX_3811;

            for (jj = 0; jj < 4; jj++)
            {
                pHal_3811->FirstPos[jj] = 0;
                pHal_3811->LastPos[jj] = 0;
                // pHal_3811->ffposition[jj] = 0.0;
                pHal_3811->HightPos[jj] = 0;

                pHal_3811->FirstPosB[jj] = 0;
                pHal_3811->LastPosB[jj] = 0;
                pHal_3811->HightPosB[jj] = 0;

                // pHal_3811->enc_lastposition[jj] = 0.0;
                pHal_3811->enc_lastposition[jj] = 0;
            }

            for (jj = 0; jj < 5; jj++)
            {
                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->dac_value[jj], comp_id,
                    "%s.%d.dac.%d.value", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_value[jj]) = 0.0;

                retval = hal_pin_bit_newf(HAL_IN, &pHal_3811->dac_enable[jj], comp_id,
                    "%s.%d.dac.%d.enable", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_enable[jj]) = 0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->dac_offset[jj], comp_id,
                    "%s.%d.dac.%d.offset", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_offset[jj]) = 0.0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->dac_scale[jj], comp_id,
                    "%s.%d.dac.%d.scale", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_scale[jj]) = 1.0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->dac_high_limit[jj], comp_id,
                    "%s.%d.dac.%d.high_limit", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_high_limit[jj]) = 10.0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->dac_low_limit[jj], comp_id,
                    "%s.%d.dac.%d.low_limit", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_low_limit[jj]) = -10.0;

                retval = hal_pin_float_newf(HAL_OUT, &pHal_3811->dac_bit_weight[jj], comp_id,
                    "%s.%d.dac.%d.bit_weight", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_bit_weight[jj]) = 10.0 / 2048.0;       //(double)( 2 ^ 11 );

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->dac_hw_offset[jj], comp_id,
                    "%s.%d.dac.%d.hw_offset", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_hw_offset[jj]) = 0.0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->dac_hw_scale[jj], comp_id,
                    "%s.%d.dac.%d.hw_scale", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->dac_hw_scale[jj]) = 1.0;
            }


            retval = hal_pin_float_newf(HAL_IN, &pHal_3811->adc_value, comp_id,
                "%s.%d.adc.0.value", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3811->adc_value) = 0.0;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3811->adc_offset, comp_id,
                "%s.%d.adc.0.offset", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3811->adc_offset) = 0.0;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3811->adc_scale, comp_id,
                "%s.%d.adc.0.scale", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3811->adc_scale) = 1.0;

            retval = hal_pin_float_newf(HAL_OUT, &pHal_3811->adc_bit_weight, comp_id,
                "%s.%d.adc.0.bit_weight", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3811->adc_bit_weight) = 10.0 / 32768.0;		//(double)( 2 ^ 15 );

            retval = hal_pin_float_newf(HAL_IN, &pHal_3811->adc_hw_offset, comp_id,
                "%s.%d.adc.0.hw_offset", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3811->adc_hw_offset) = 0.0;

            retval = hal_pin_float_newf(HAL_IN, &pHal_3811->adc_hw_scale, comp_id,
                "%s.%d.adc.0.hw_scale", prefix, ii);
            if (retval < 0) goto error;
            *(pHal_3811->adc_hw_scale) = 1.0;


            for (jj = 0; jj < 4; jj++)
            {
                retval = hal_pin_s32_newf(HAL_OUT, &pHal_3811->enc_counts[jj], comp_id,
                    "%s.%d.enc.%d.counts", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->enc_counts[jj]) = 0;

                retval = hal_pin_float_newf(HAL_OUT, &pHal_3811->enc_position[jj], comp_id,
                    "%s.%d.enc.%d.position", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->enc_position[jj]) = 0.0;

                retval = hal_pin_float_newf(HAL_OUT, &pHal_3811->enc_velocity[jj], comp_id,
                    "%s.%d.enc.%d.velocity", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->enc_velocity[jj]) = 0.0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->enc_scale[jj], comp_id,
                    "%s.%d.enc.%d.scale", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->enc_scale[jj]) = 1.0;

                retval = hal_pin_s32_newf(HAL_IN, &pHal_3811->enc_mode[jj], comp_id,
                    "%s.%d.enc.%d.mode", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->enc_mode[jj]) = DEF_MODE_3811_0;

                retval = hal_pin_float_newf(HAL_IN, &pHal_3811->enc_set_position[jj], comp_id,
                    "%s.%d.enc.%d.set-position", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->enc_set_position[jj]) = 0.0;

                retval = hal_pin_bit_newf(HAL_IN, &pHal_3811->enc_set[jj], comp_id,
                    "%s.%d.enc.%d.set", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->enc_set[jj]) = 0;
            }


            for (jj = 0; jj < 9; jj++)
            {
                retval = hal_pin_bit_newf(HAL_IN, &pHal_3811->bout_led[jj], comp_id,
                    "%s.%d.bled.%d.out", prefix, ii, jj);
                if (retval < 0) goto error;
                *(pHal_3811->bout_led[jj]) = 0;
            }
        }
        break;

        default:break;
        }
    }



    SetFirstBoard();

    jj = CheckIdBoars();    //1=OK, 0,-1,-2,-3,...=Chyba na 0.,1.,2.,3.-tej doske
//  jj = 1;
    if (jj != 1)
    {
        ii = -jj;
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: Board%d=%d, BoardTypeDetect[%d]=%d\n",
            modname, ii, BoardParam[ii][DEF_BOARD_TYPE], ii, BoardTypeDetect[ii]);

        retval = -60 + jj;
        goto error;
    }



error:
    if (retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: program failed with err=%i\n",
            modname, retval);
        hal_exit(comp_id);
        return(retval);
    }

    /* export functions */

    prefix = DEF_PREFIX;

    rtapi_snprintf(name, sizeof(name), "%s.HwComm", prefix);
    retval = hal_export_funct(name, HwComm, data, 1, 0, comp_id);
    if (retval < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: read function export failed\n", modname);
        hal_exit(comp_id);
        return(-100);
    }

    /*
        rtapi_snprintf( name, sizeof(name), "%s.write", prefix );
        retval = hal_export_funct( name, write_spi, data, 0, 0, comp_id );
        if( retval < 0 )
            {
            rtapi_print_msg( RTAPI_MSG_ERR, "%s: ERROR: write function export failed\n", modname );
            hal_exit( comp_id );
            return( -110 );
        }
        rtapi_snprintf( name, sizeof(name), "%s.update", prefix );
        retval = hal_export_funct( name, update, data, 1, 0, comp_id );
        if( retval < 0 )
            {
            rtapi_print_msg( RTAPI_MSG_ERR, "%s: ERROR: update function export failed\n", modname );
            hal_exit( comp_id );
            return( -120 );
        }
    */

    memset(txBuf, 0, BUFSIZE);

    rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
    hal_ready(comp_id);
    return(0);
}


void rtapi_app_exit(void)
{
    restore_gpio();
    munmap((void *)mem1, BLOCK_SIZE);
    munmap((void *)mem2, BLOCK_SIZE);
    hal_exit(comp_id);
}


void HwComm(void *arg, long period)
{
    int             ii, jj, kk, nn, bin;
    int             len, pos;
    int             Id;
    float           ff, cc, rr;

    int             off, bit, led;
    int             pwm, min, rozdiel;
    int             outbit;
    int             allmode, mode, motor;
    unsigned short int  uspos;



    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)	break;

        if (BoardParam[ii][DEF_BOARD_TYPE] == 3805)
        {
            S_HAL_3805	*pHal_3805 = (S_HAL_3805 *)Boards[ii].HalPointer;

            allmode = BoardParam[ii][DEF_BOARD_PARAM_2];
            for (jj = 0; jj < 7; jj++)
            {
                mode = allmode % 10;
                allmode /= 10;
                if (mode != 4)
                {
                    if (pHal_3805->boutinvert[jj] != *(pHal_3805->bout_invert[jj]))
                    {
                        pHal_3805->boutinvert[jj] = *(pHal_3805->bout_invert[jj]);
                        MakeConfigure = true;
                    }
                }
                else
                {
                    jj++;
                    continue;
                }
            }
        }
    }



    if (((*Globals->estop != 0) || (MakeConfigure)) ||
        ((*Globals->estop == 0) && (LastEstop != 0))
        )
    {
        HwConfigure();
        MakeConfigure = 0;
    }
    LastEstop = *Globals->estop;


    len = 0;
    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)	break;

        switch (BoardParam[ii][DEF_BOARD_TYPE])
        {
        case 3805: {
            S_SPI_OUT_3805	*p_SPI_OUT_3805 = (S_SPI_OUT_3805 *)Boards[ii].SpiWrPointer;
            S_HAL_3805	*pHal_3805 = (S_HAL_3805 *)Boards[ii].HalPointer;

            p_SPI_OUT_3805->Info = BOARDS_INFO_CYCLIC;

            for (jj = 0; jj < 4; jj++)
            {
                ff = *(pHal_3805->step_speed[jj]) * *(pHal_3805->step_scale[jj]);
                ff /= DEF_POSITION_CONSTANT_3805;
                if (ff >= 0)
                {
                    ff += 0.5;
                    if (ff > 32767.0)   ff = 32767.0;
                }
                else
                {
                    ff -= 0.5;
                    if (ff < -32767.0)  ff = -32767.0;
                }
                tmpdata = (int)ff;
                if (*Globals->estop == 0)
                {
                    if (*(pHal_3805->step_enable[jj]))      p_SPI_OUT_3805->Speed[jj] = (short int)tmpdata;
                    else                                    p_SPI_OUT_3805->Speed[jj] = 0;
                }
                else            p_SPI_OUT_3805->Speed[jj] = 0;
            }

            if (*pHal_3805->dac_enable)
            {
                ff = *pHal_3805->dac_value + *pHal_3805->dac_offset;
                ff *= *pHal_3805->dac_scale;
                ff *= *pHal_3805->dac_hw_scale;
                ff += *pHal_3805->dac_hw_offset;
                if (ff < *pHal_3805->dac_low_limit) ff = *pHal_3805->dac_low_limit;
                if (ff > *pHal_3805->dac_high_limit)    ff = *pHal_3805->dac_high_limit;
            }
            else	ff = 0.0;
            tmpdata = (int)(ff * 409.6);        //4096 -> 409.6

            if (*Globals->estop == 0)   p_SPI_OUT_3805->AnalogOut = (short int)tmpdata;
            else                        p_SPI_OUT_3805->AnalogOut = 0;



            allmode = BoardParam[ii][DEF_BOARD_PARAM_2];
            for (jj = 0; jj < 7; jj++)
            {
                mode = allmode % 10;
                allmode /= 10;
                switch (mode)
                {
                case 1: //if( (*(pHal_3805->bout_out[jj])) ^ (*(pHal_3805->bout_invert[jj])) )
                    if (*(pHal_3805->bout_out[jj]))
                        p_SPI_OUT_3805->BinOut[jj] = 0xFFFF;
                    else    p_SPI_OUT_3805->BinOut[jj] = 0;
                    break;

                case 2: if (*pHal_3805->bout_pwm_duty_cycle[jj] < 0.0)      kk = 0;
                        else
                        {
                            if (*pHal_3805->bout_pwm_duty_cycle[jj] > 1.0)      kk = 4000;
                            else        kk = (int)(4000 * *pHal_3805->bout_pwm_duty_cycle[jj]);
                        }
                        p_SPI_OUT_3805->BinOut[jj] = kk;
                        break;

                case 3: cc = *pHal_3805->bout_rc_center * DEF_TIME_1MS;
                    rr = *pHal_3805->bout_rc_range  * DEF_TIME_1MS;

                    ff = *pHal_3805->bout_angle[jj] / PI_POL;
                    if (ff < -1.0)      ff = -1.0;
                    if (ff > 1.0)       ff = 1.0;
                    ff *= rr;
                    tmpdata = (int)(cc + ff);
                    if (tmpdata < 0)                    tmpdata = 0;
                    if (tmpdata > DEF_TIME_1MS * 4)     tmpdata = DEF_TIME_1MS * 4;
                    p_SPI_OUT_3805->BinOut[jj] = (short int)tmpdata;
                    break;

                case 4:	if (jj < 6)
                {
                    motor = 4 + jj / 2;

                    ff = *(pHal_3805->step_speed[motor]) * *(pHal_3805->step_scale[motor]);
                    ff /= DEF_POSITION_CONSTANT_3805;
                    if (ff >= 0)
                    {
                        ff += 0.5;
                        if (ff > 32767.0)   ff = 32767.0;
                    }
                    else
                    {
                        ff -= 0.5;
                        if (ff < -32767.0)  ff = -32767.0;
                    }
                    tmpdata = (int)ff;

                    if (*Globals->estop == 0)
                    {
                        if (*(pHal_3805->step_enable[motor]))   p_SPI_OUT_3805->BinOut[jj] = (short int)tmpdata;
                        else                                    p_SPI_OUT_3805->BinOut[jj] = 0;
                    }
                    else            p_SPI_OUT_3805->BinOut[jj] = 0;
                }
                        allmode /= 10;
                        jj++;
                        break;

                default:
                    break;
                }
            }



            outbit = 0;
            for (jj = 3; jj >= 0; jj--)
            {
                outbit <<= 1;
                if (*(pHal_3805->step_enable[jj]))
                    outbit |= 1;
            }
            for (jj = 10; jj >= 7; jj--)
            {
                outbit <<= 1;
                if ((*(pHal_3805->bout_out[jj])) ^ (*(pHal_3805->bout_invert[jj])))
                    outbit |= 1;
            }
            // if( *Globals->estop != 0 )
            p_SPI_OUT_3805->BinOut2 = outbit;



            outbit = 0;
            for (jj = 8; jj >= 0; jj--)
            {
                outbit <<= 1;
                if (*(pHal_3805->bout_led[jj]))
                    outbit |= 1;
            }
            // if( *Globals->estop != 0 )
            p_SPI_OUT_3805->BinOutLed = outbit;

        }
                   break;

        case 3809: {
            S_SPI_OUT_3809	*p_SPI_OUT_3809 = (S_SPI_OUT_3809 *)Boards[ii].SpiWrPointer;
            S_HAL_3809	*pHal_3809 = (S_HAL_3809 *)Boards[ii].HalPointer;

            p_SPI_OUT_3809->Info = BOARDS_INFO_CYCLIC;

            // if( *Globals->estop != 0 )
            //      p_SPI_OUT_3809->BinOut = 0;
            // else
            {
                bin = 0;
                for (jj = 15; jj >= 0; jj--)
                {
                    bin <<= 1;
                    if ((*(pHal_3809->bout_out[jj])) ^ (*(pHal_3809->bout_invert[jj])))
                        bin |= 1;
                }
                p_SPI_OUT_3809->BinOut = bin;

                led = 0;
                for (jj = 3; jj >= 0; jj--)
                {
                    led <<= 1;
                    if (*(pHal_3809->bout_led[jj]))
                        led |= 1;
                }
                p_SPI_OUT_3809->Led = led;
            }
        }
                   break;

        case 3811: {
            S_SPI_OUT_3811  *p_SPI_OUT_3811 = (S_SPI_OUT_3811 *)Boards[ii].SpiWrPointer;
            S_HAL_3811  *pHal_3811 = (S_HAL_3811 *)Boards[ii].HalPointer;

            p_SPI_OUT_3811->Info = BOARDS_INFO_CYCLIC;

            for (jj = 0; jj < 5; jj++)
            {
                if (*pHal_3811->dac_enable[jj])
                {
                    ff = *pHal_3811->dac_value[jj] + *pHal_3811->dac_offset[jj];
                    ff *= *pHal_3811->dac_scale[jj];
                    ff *= *pHal_3811->dac_hw_scale[jj];
                    ff += *pHal_3811->dac_hw_offset[jj];
                    if (ff < *pHal_3811->dac_low_limit[jj]) ff = *pHal_3811->dac_low_limit[jj];
                    if (ff > *pHal_3811->dac_high_limit[jj])    ff = *pHal_3811->dac_high_limit[jj];
                }
                else    ff = 0.0;
                if (ff > 10.0)  ff = 10.0;
                if (ff < -10.0) ff = -10.0;
                tmpdata = (int)(ff * 3276.7);       //32768 -> 3276.8

                if (*Globals->estop == 0)   p_SPI_OUT_3811->DACvalue[jj] = (short int)tmpdata;
                else                        p_SPI_OUT_3811->DACvalue[jj] = 0;
            }


            led = 0;
            for (jj = 8; jj >= 0; jj--)
            {
                led <<= 1;
                if (*(pHal_3811->bout_led[jj]))
                    led |= 1;
            }
            p_SPI_OUT_3811->Led = led;
        }
                   break;

        default:    break;
        }

        len = Boards[ii].SpiWrLen;
    }


    len += 1;
    write_read_buf(txBuf, rxBuf, len);


    if (b_debug1)
    {
        for (jj = 0; jj < 100; jj++)
        {
            *Globals->x_wr[jj] = txBuf[jj] & 0xFF;
            *Globals->x_rd[jj] = rxBuf[jj] & 0xFF;
        }
        for (jj = 0; jj < 5; jj++)
        {
            *Globals->x_debug[jj * 2 + 0] = (intptr_t)Boards[jj].SpiWrPointer - (intptr_t)txBuf;
            *Globals->x_debug[jj * 2 + 1] = (intptr_t)Boards[jj].SpiRdPointer - (intptr_t)rxBuf;
        }
    }



    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)	break;

        switch (BoardParam[ii][DEF_BOARD_TYPE])
        {
        case 3805: {
            S_SPI_IN_3805	*p_SPI_IN_3805 = (S_SPI_IN_3805 *)Boards[ii].SpiRdPointer;
            S_HAL_3805	*pHal_3805 = (S_HAL_3805 *)Boards[ii].HalPointer;

            if (ii == 0)
            {
                ActTime = *(int *)&rxBuf[0];
                *Globals->time = ActTime / 48;
            }

            Id = p_SPI_IN_3805->Id >> 8;
            Id += 3800;

            for (jj = 0; jj < 7; jj++)
            {
                if (pHal_3805->UsedMotors[jj])
                {
                    uspos = p_SPI_IN_3805->Position[jj];
                    if (b_FirstPos)		pHal_3805->FirstPos[jj] = uspos;
                    uspos -= pHal_3805->FirstPos[jj];

                    if (((uspos & 0xC000) == 0) && ((pHal_3805->LastPos[jj] & 0xC000) == 0xC000))
                        pHal_3805->HightPos[jj]++;
                    if (((uspos & 0xC000) == 0xC000) && ((pHal_3805->LastPos[jj] & 0xC000) == 0))
                        pHal_3805->HightPos[jj]--;

                    pHal_3805->LastPos[jj] = uspos;

                    ff = (float)pHal_3805->HightPos[jj];
                    ff *= (float)0x10000;
                    ff += (float)uspos;

                    if (fabs(*(pHal_3805->step_scale[jj])) < 0.000001)	*(pHal_3805->step_scale[jj]) = 0.000001;
                    ff = ff / *(pHal_3805->step_scale[jj]);
                    *(pHal_3805->step_position[jj]) = ff;
                }
            }



            bin = p_SPI_IN_3805->BinIn;
            allmode = BoardParam[ii][DEF_BOARD_PARAM_1];
            for (jj = 0; jj < 5; jj++)
            {
                mode = allmode % 10;
                allmode /= 10;
                switch (mode)
                {
                case 1:	if (bin & 0x01) { *(pHal_3805->bin_in[jj]) = 0; *(pHal_3805->bin_in_not[jj]) = 1; }
                        else { *(pHal_3805->bin_in[jj]) = 1;	*(pHal_3805->bin_in_not[jj]) = 0; }
                        break;

                case 2:	if (jj < 4)
                {
                    kk = jj / 2;

                    if (fabs(*(pHal_3805->enc_scale[kk])) < 0.000001)	*(pHal_3805->enc_scale[kk]) = 0.000001;

                    if (*(pHal_3805->enc_set[kk]) == 1)
                    {
                        Set3805EncPosition(pHal_3805, p_SPI_IN_3805, kk, *(pHal_3805->enc_set_position[kk]));
                    }
                    else
                    {
                        uspos = p_SPI_IN_3805->IRCposition[kk];
                        if (b_FirstPos)		pHal_3805->enc_FirstPos[kk] = uspos;
                        uspos -= pHal_3805->enc_FirstPos[kk];

                        if (((uspos & 0xC000) == 0) && ((pHal_3805->enc_LastPos[kk] & 0xC000) == 0xC000))
                            pHal_3805->enc_HightPos[kk]++;
                        if (((uspos & 0xC000) == 0xC000) && ((pHal_3805->enc_LastPos[kk] & 0xC000) == 0))
                            pHal_3805->enc_HightPos[kk]--;

                        pHal_3805->enc_LastPos[kk] = uspos;

                        ff = (float)pHal_3805->enc_HightPos[kk];
                        ff *= (float)0x10000;
                        ff += (float)uspos;

                        ff = ff / *(pHal_3805->enc_scale[kk]);
                        *(pHal_3805->enc_position[kk]) = ff;

                        nn = pHal_3805->enc_HightPos[kk];
                        nn <<= 16;
                        nn += uspos;
                        *(pHal_3805->enc_counts[kk]) = nn;

                        switch (*(pHal_3805->enc_mode[kk]))
                        {
                        case DEF_MODE_3805_0:
                            break;

                        case DEF_MODE_3805_1:
                            if (*(pHal_3805->enc_position[kk]) < 0.0)
                                Set3805EncPosition(pHal_3805, p_SPI_IN_3805, kk, *(pHal_3805->enc_position[kk]) + *(pHal_3805->enc_set_position[kk]));
                            else
                            {
                                if (*(pHal_3805->enc_position[kk]) > *(pHal_3805->enc_set_position[kk]))
                                    Set3805EncPosition(pHal_3805, p_SPI_IN_3805, kk, *(pHal_3805->enc_position[kk]) - *(pHal_3805->enc_set_position[kk]));
                            }
                            break;

                        case DEF_MODE_3805_2:
                            if (*(pHal_3805->enc_position[kk]) < 0.0)
                                Set3805EncPosition(pHal_3805, p_SPI_IN_3805, kk, 0.0);
                            else
                            {
                                if (*(pHal_3805->enc_position[kk]) > *(pHal_3805->enc_set_position[kk]))
                                    Set3805EncPosition(pHal_3805, p_SPI_IN_3805, kk, *(pHal_3805->enc_set_position[kk]));
                            }
                            break;

                        default:
                            break;
                        }

                        *(pHal_3805->enc_velocity[kk]) = (*(pHal_3805->enc_counts[kk]) - pHal_3805->enc_lastposition[kk]) / *(pHal_3805->enc_scale[kk]) * 1000;
                        pHal_3805->enc_lastposition[kk] = *(pHal_3805->enc_counts[kk]);
                    }
                }
                        allmode /= 10;
                        jj++;

                        break;

                default:
                    break;
                }

                bin >>= 1;
            }


            // pHal_3805->MotorError = 0;
            for (jj = 0; jj < 4; jj++)
            {
                *(pHal_3805->step_status[jj]) = (p_SPI_IN_3805->Status >> jj) & 1;
                // pHal_3805->MotorError += (char)( *pHal_3805->step_status[jj] ^ 1 );
            }
            for (jj = 4; jj < 7; jj++)
            {
                if (pHal_3805->UsedMotors[jj])
                    *(pHal_3805->step_status[jj]) = 1;
            }

            if (p_SPI_IN_3805->Status & DEF_STATUS_WANT_CONF)
                MakeConfigure = 1;

            // if( pHal_3805->MotorError != 0 )
            // MotorError = 1;
        }
                   break;

        case 3809: {
            S_SPI_IN_3809   *p_SPI_IN_3809 = (S_SPI_IN_3809 *)Boards[ii].SpiRdPointer;
            S_HAL_3809  *pHal_3809 = (S_HAL_3809 *)Boards[ii].HalPointer;

            Id = p_SPI_IN_3809->Id >> 8;
            Id += 3800;

            bin = p_SPI_IN_3809->BinIn;
            for (jj = 0; jj < 16; jj++)
            {
                if (bin & 0x01) { *(pHal_3809->bin_in[jj]) = 1;	*(pHal_3809->bin_in_not[jj]) = 0; }
                else { *(pHal_3809->bin_in[jj]) = 0;	*(pHal_3809->bin_in_not[jj]) = 1; }
                bin >>= 1;
            }
        }
                   break;

        case 3811: {
            S_SPI_IN_3811   *p_SPI_IN_3811 = (S_SPI_IN_3811 *)Boards[ii].SpiRdPointer;
            S_HAL_3811  *pHal_3811 = (S_HAL_3811 *)Boards[ii].HalPointer;

            if (ii == 0)
            {
                ActTime = *(int *)&rxBuf[0];
                *Globals->time = ActTime / 48;
            }

            Id = p_SPI_IN_3811->Id >> 8;
            Id += 3800;

            for (jj = 0; jj < 4; jj++)
            {
                if (fabs(*(pHal_3811->enc_scale[jj])) < 0.000001)	*(pHal_3811->enc_scale[jj]) = 0.000001;

                if (*(pHal_3811->enc_set[jj]) == 1)
                {
                    if (*(pHal_3811->enc_mode[jj]) == DEF_MODE_3811_3)
                    {
                        ff = *(pHal_3811->enc_set_position[jj]) * *(pHal_3811->enc_scale[jj]);
                        kk = (int)ff;
                        *(pHal_3811->enc_counts[jj]) = kk & 0x0000FFFF;
                        *(pHal_3811->enc_position[jj]) = *(pHal_3811->enc_set_position[jj]);
                        // pHal_3811->enc_lastposition[jj]  = *(pHal_3811->enc_position[jj]);
                        pHal_3811->enc_lastposition[jj] = *(pHal_3811->enc_counts[jj]);
                        *(pHal_3811->enc_velocity[jj]) = 0.0;

                        uspos = kk & 0x000000FF;
                        ff /= (float)0x100;
                        pHal_3811->HightPos[jj] = (int)ff;
                        pHal_3811->LastPos[jj] = 0;
                        pHal_3811->FirstPos[jj] = (p_SPI_IN_3811->IRCposition[jj] & 0xFF) - uspos;

                        pHal_3811->HightPosB[jj] = 0;
                        pHal_3811->FirstPosB[jj] = (p_SPI_IN_3811->IRCposition[jj] >> 8) & 0xFF;
                        pHal_3811->LastPosB[jj] = 0;
                    }
                    else
                    {
                        ff = *(pHal_3811->enc_set_position[jj]) * *(pHal_3811->enc_scale[jj]);
                        kk = (int)ff;
                        *(pHal_3811->enc_counts[jj]) = kk;
                        *(pHal_3811->enc_position[jj]) = *(pHal_3811->enc_set_position[jj]);
                        //  pHal_3811->enc_lastposition[jj] = *(pHal_3811->enc_position[jj]);
                        pHal_3811->enc_lastposition[jj] = *(pHal_3811->enc_counts[jj]);
                        *(pHal_3811->enc_velocity[jj]) = 0.0;

                        uspos = kk & 0x0000FFFF;
                        ff /= (float)0x10000;
                        pHal_3811->HightPos[jj] = (int)ff;
                        pHal_3811->LastPos[jj] = 0;
                        pHal_3811->FirstPos[jj] = p_SPI_IN_3811->IRCposition[jj] - uspos;
                    }
                }
                else
                {
                    if (*(pHal_3811->enc_mode[jj]) == DEF_MODE_3811_3)
                    {
                        //Input A
                        uspos = p_SPI_IN_3811->IRCposition[jj] & 0xFF;
                        if (b_FirstPos)     pHal_3811->FirstPos[jj] = uspos;
                        uspos -= pHal_3811->FirstPos[jj];
                        uspos &= 0xFF;

                        if (((uspos & 0x80) == 0) && ((pHal_3811->LastPos[jj] & 0x80) == 0x80))
                            pHal_3811->HightPos[jj]++;
                        if (((uspos & 0x80) == 0x80) && ((pHal_3811->LastPos[jj] & 0x80) == 0))
                            pHal_3811->HightPos[jj]--;

                        pHal_3811->LastPos[jj] = uspos;

                        ff = (float)pHal_3811->HightPos[jj];
                        ff *= (float)0x100;
                        ff += (float)uspos;

                        ff = ff / *(pHal_3811->enc_scale[jj]);
                        *(pHal_3811->enc_position[jj]) = ff;
                        // *(pHal_3811->enc_velocity[jj])  = ( ff - pHal_3811->enc_lastposition[jj] ) * 1000.0;
                        // pHal_3811->enc_lastposition[jj] = *(pHal_3811->enc_position[jj]);

                        kk = pHal_3811->HightPos[jj];
                        kk <<= 8;
                        kk += uspos;
                        *(pHal_3811->enc_counts[jj]) = kk & 0xFFFF;

                        *(pHal_3811->enc_velocity[jj]) = ((short int)(kk & 0xFFFF) - pHal_3811->enc_lastposition[jj]) / *(pHal_3811->enc_scale[jj]) * 1000;
                        pHal_3811->enc_lastposition[jj] = *(pHal_3811->enc_counts[jj]);

                        //Input B
                        uspos = (p_SPI_IN_3811->IRCposition[jj] >> 8) & 0xFF;
                        if (b_FirstPos)     pHal_3811->FirstPosB[jj] = uspos;
                        uspos -= pHal_3811->FirstPosB[jj];
                        uspos &= 0xFF;

                        if (((uspos & 0x80) == 0) && ((pHal_3811->LastPosB[jj] & 0x80) == 0x80))
                            pHal_3811->HightPosB[jj]++;
                        if (((uspos & 0x80) == 0x80) && ((pHal_3811->LastPosB[jj] & 0x80) == 0))
                            pHal_3811->HightPosB[jj]--;

                        pHal_3811->LastPosB[jj] = uspos;

                        kk = pHal_3811->HightPosB[jj];
                        kk <<= 8;
                        kk += uspos;
                        *(pHal_3811->enc_counts[jj]) += ((kk & 0xFFFF) << 16);
                    }
                    else
                    {
                        uspos = p_SPI_IN_3811->IRCposition[jj];
                        if (b_FirstPos)		pHal_3811->FirstPos[jj] = uspos;
                        uspos -= pHal_3811->FirstPos[jj];

                        if (((uspos & 0xC000) == 0) && ((pHal_3811->LastPos[jj] & 0xC000) == 0xC000))
                            pHal_3811->HightPos[jj]++;
                        if (((uspos & 0xC000) == 0xC000) && ((pHal_3811->LastPos[jj] & 0xC000) == 0))
                            pHal_3811->HightPos[jj]--;

                        pHal_3811->LastPos[jj] = uspos;

                        ff = (float)pHal_3811->HightPos[jj];
                        ff *= (float)0x10000;
                        ff += (float)uspos;

                        ff = ff / *(pHal_3811->enc_scale[jj]);
                        *(pHal_3811->enc_position[jj]) = ff;
                        //  *(pHal_3811->enc_velocity[jj])  = ( ff - pHal_3811->enc_lastposition[jj] ) * 1000.0;
                        //  pHal_3811->enc_lastposition[jj] = *(pHal_3811->enc_position[jj]);

                        kk = pHal_3811->HightPos[jj];
                        kk <<= 16;
                        kk += uspos;
                        *(pHal_3811->enc_counts[jj]) = kk;

                        *(pHal_3811->enc_velocity[jj]) = (kk - pHal_3811->enc_lastposition[jj]) / *(pHal_3811->enc_scale[jj]) * 1000;
                        pHal_3811->enc_lastposition[jj] = *(pHal_3811->enc_counts[jj]);
                    }

                    /*
                                                    uspos = p_SPI_IN_3811->IRCposition[jj];
                                                    if( b_FirstPos )        pHal_3811->FirstPos[jj] = uspos;
                                                    uspos -= pHal_3811->FirstPos[jj];

                                                    if( ( ( uspos & 0xC000 ) == 0 ) && ( ( pHal_3811->LastPos[jj] & 0xC000 ) == 0xC000 ) )
                                                        pHal_3811->ffposition[jj] += (float)0x10000;
                                                    if( ( ( uspos & 0xC000 ) == 0xC000 ) && ( ( pHal_3811->LastPos[jj] & 0xC000 ) == 0 ) )
                                                        pHal_3811->ffposition[jj] -= (float)0x10000;

                                                    pHal_3811->LastPos[jj] = uspos;
                                                    ff = (float)uspos;
                                                    ff += pHal_3811->ffposition[jj];

                                                    if( fabs( *(pHal_3811->enc_scale[jj]) ) < 0.000001 )        *(pHal_3811->enc_scale[jj]) = 0.000001;
                                                    ff = ff / *(pHal_3811->enc_scale[jj]);
                                                    *(pHal_3811->enc_position[jj]) = ff;
                    */
                }
            }


            //                      if( fabs( *(pHal_3811->adc_scale) ) < 0.000001 )    *(pHal_3811->adc_scale) = 0.000001;
            //                      if( fabs( *(pHal_3811->adc_hw_scale) ) < 0.000001 ) *(pHal_3811->adc_hw_scale) = 0.000001;

            tmpdata = p_SPI_IN_3811->ADCvalue;
            ff = (float)((short int)tmpdata) / 3276.7;      //32768 -> 3276.8
            ff *= 1.787;                                    //scale hw converter
            ff -= *pHal_3811->adc_hw_offset;
            ff *= *pHal_3811->adc_hw_scale;
            ff *= *pHal_3811->adc_scale;
            ff -= *pHal_3811->adc_offset;
            *(pHal_3811->adc_value) = ff;


            if (p_SPI_IN_3811->Status & DEF_STATUS_WANT_CONF)
                MakeConfigure = 1;
        }
                   break;


        default:    break;
        }
    }

    b_FirstPos = false;
}


void HwConfigure()
{
    int             ii, jj, kk, ll, bin;
    int             len, pos;
    int             Id;
    int             allmode, mode, motor;

    len = 0;
    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)    break;
        len = Boards[ii].SpiWrLen;
    }
    if (len > 0)
        memset(Boards[0].SpiWrPointer, 0, len);

    len = 0;
    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)    break;

        switch (BoardParam[ii][DEF_BOARD_TYPE])
        {
        case 3805: {
            S_SPI_OUT_3805  *p_SPI_OUT_3805 = (S_SPI_OUT_3805 *)Boards[ii].SpiWrPointer;
            S_HAL_3805      *pHal_3805 = (S_HAL_3805 *)Boards[ii].HalPointer;

            p_SPI_OUT_3805->Info = BOARDS_INFO_CMD;

            if (*Globals->estop == 0)   ll = 0x00;
            else                        ll = 0x80;

            //                      if( pHal_3805->MotorError != 0 )    ll = 0x80;

            for (jj = 0; jj < 4; jj++)
            {
                kk = 0;
                if (*pHal_3805->step_stepping[jj] >= 1)     kk = 0;
                if (*pHal_3805->step_stepping[jj] >= 2)     kk = 1;
                if (*pHal_3805->step_stepping[jj] >= 4)     kk = 2;
                if (*pHal_3805->step_stepping[jj] >= 8)     kk = 3;
                if (*pHal_3805->step_stepping[jj] >= 16)    kk = 4;
                if (*pHal_3805->step_stepping[jj] >= 32)    kk = 5;

                tmpdata = ll + kk;
                p_SPI_OUT_3805->Speed[jj] = (short int)tmpdata;
            }

            allmode = BoardParam[ii][DEF_BOARD_PARAM_2];
            for (jj = 0; jj < 7; jj++)
            {
                mode = allmode % 10;
                allmode /= 10;
                switch (mode)
                {
                case 1:	p_SPI_OUT_3805->BinOut[jj] = (short int)(0 + (*(pHal_3805->bout_invert[jj]) << 2));
                    break;
                case 2:	p_SPI_OUT_3805->BinOut[jj] = (short int)(0 + (*(pHal_3805->bout_invert[jj]) << 2));
                    break;
                case 3:	p_SPI_OUT_3805->BinOut[jj] = (short int)(2 + (*(pHal_3805->bout_invert[jj]) << 2));
                    break;
                case 4:	p_SPI_OUT_3805->BinOut[jj] = (short int)1;
                    jj++;
                    p_SPI_OUT_3805->BinOut[jj] = (short int)1;
                    allmode /= 10;
                    break;
                default:
                    break;
                }
            }
        }
                   break;

        case 3809: {
            S_SPI_OUT_3809	*p_SPI_OUT_3809 = (S_SPI_OUT_3809 *)Boards[ii].SpiWrPointer;
            S_HAL_3809	     *pHal_3809 = (S_HAL_3809 *)Boards[ii].HalPointer;

            p_SPI_OUT_3809->Info = BOARDS_INFO_CMD;

            p_SPI_OUT_3809->BinOut = 0;
        }
                   break;

        case 3811: {
            S_SPI_OUT_3811  *p_SPI_OUT_3811 = (S_SPI_OUT_3811 *)Boards[ii].SpiWrPointer;
            S_HAL_3811      *pHal_3811 = (S_HAL_3811 *)Boards[ii].HalPointer;

            p_SPI_OUT_3811->Info = BOARDS_INFO_CMD;

            for (jj = 0; jj < 4; jj++)
            {
                p_SPI_OUT_3811->DACvalue[jj] = (short int)*(pHal_3811->enc_mode[jj]);
            }
        }
                   break;

        default:    break;
        }

        len = Boards[ii].SpiWrLen;
    }

    //	len += 4;
    write_read_buf(txBuf, rxBuf, len);

    if (b_debug2)
    {
        for (jj = 0; jj < 100; jj++)
        {
            *Globals->x_wr[jj] = txBuf[jj] & 0xFF;
            *Globals->x_rd[jj] = rxBuf[jj] & 0xFF;
        }
        for (jj = 0; jj < 5; jj++)
        {
            *Globals->x_debug[jj * 2 + 0] = (intptr_t)Boards[jj].SpiWrPointer - (intptr_t)txBuf;
            *Globals->x_debug[jj * 2 + 1] = (intptr_t)Boards[jj].SpiRdPointer - (intptr_t)rxBuf;
        }
    }
}


void write_read_buf(char* tbuf, char* rbuf, int len)
{
    int     txcnt = 0;
    int     rxcnt = 0;

    /* Clear TX and RX fifos */
    BCM2835_SPICS |= BCM2835_SPI0_CS_CLEAR;
    //	BCM2835_SPICS &= ~BCM2835_SPI0_CS_CLEAR;

        /* Set TA = 1 */
    BCM2835_SPICS |= BCM2835_SPI0_CS_TA;

    /* Use the FIFO's to reduce the interbyte times */
    while (rxcnt < len)
    {
        /* TX fifo not full, so add one byte */
        if ((txcnt < len) && (BCM2835_SPICS & BCM2835_SPI0_CS_TXD))
            BCM2835_SPIFIFO = tbuf[txcnt++];

        /* Rx fifo not empty, so get the next received byte */
        if (BCM2835_SPICS & BCM2835_SPI0_CS_RXD)
            rbuf[rxcnt++] = BCM2835_SPIFIFO;
    }

    /* Wait for DONE to be set */
    while (!(BCM2835_SPICS & BCM2835_SPI0_CS_DONE));

    /* Set TA = 0, and also set the barrier */
    BCM2835_SPICS &= ~BCM2835_SPI0_CS_TA;
}



void Set3805EncPosition(S_HAL_3805	*pHal_3805, S_SPI_IN_3805 *p_SPI_IN_3805, int encid, float value)
{
    int                 nn;
    float               ff;
    unsigned short int	uspos;

    ff = value * *(pHal_3805->enc_scale[encid]);
    nn = (int)ff;
    *(pHal_3805->enc_counts[encid]) = nn;
    *(pHal_3805->enc_position[encid]) = value;
    //  pHal_3805->enc_lastposition[encid]  = *(pHal_3805->enc_position[encid]);
    pHal_3805->enc_lastposition[encid] = *(pHal_3805->enc_counts[encid]);
    *(pHal_3805->enc_velocity[encid]) = 0.0;

    uspos = nn & 0x0000FFFF;
    ff /= (float)0x10000;
    pHal_3805->enc_HightPos[encid] = (int)ff;
    pHal_3805->enc_LastPos[encid] = 0;
    pHal_3805->enc_FirstPos[encid] = p_SPI_IN_3805->IRCposition[encid] - uspos;
}

platform_t check_platform(void)
{
    FILE        *fp;
    char        buf[2048];
    size_t      fsize;

    fp = fopen("/proc/cpuinfo", "r");
    fsize = fread(buf, 1, sizeof(buf), fp);
    fclose(fp);

    if (fsize == 0 || fsize == sizeof(buf))
        return(UNSUPPORTED);

    /* NUL terminate the buffer */
    buf[fsize] = '\0';

    if (NULL != strstr(buf, "BCM2708"))
        return (RPI);
    else if (NULL != strstr(buf, "BCM2709"))
        return (RPI_2);
    else if (NULL != strstr(buf, "BCM2835") || NULL != strstr(buf, "BCM2836") || NULL != strstr(buf, "BCM2837"))
        return (RPI_3);
    else if (NULL != strstr(buf, "BCM2711") || NULL != strstr(buf, "Raspberry Pi 4"))
        return (RPI_4);
    else if (NULL != strstr(buf, "BCM2712") || NULL != strstr(buf, "Raspberry Pi 5"))
        return (RPI_5);
    else
        return(UNSUPPORTED);
}

int map_gpio()
{
    int         fd;
    static u32  mem1_base, mem2_base;

    switch (platform)
    {
    case RPI:
        mem1_base = BCM2835_PERI_BASE_PI1 + BCM2835_GPIO_OFFSET;
        mem2_base = BCM2835_PERI_BASE_PI1 + BCM2835_SPI_OFFSET;
        break;
    case RPI_2:
    case RPI_3:
        mem1_base = BCM2835_PERI_BASE_PI23 + BCM2835_GPIO_OFFSET;
        mem2_base = BCM2835_PERI_BASE_PI23 + BCM2835_SPI_OFFSET;
        break;
    case RPI_4:
        mem1_base = BCM2835_PERI_BASE_PI4 + BCM2835_GPIO_OFFSET;
        mem2_base = BCM2835_PERI_BASE_PI4 + BCM2835_SPI_OFFSET;
        break;
    case RPI_5:
        mem1_base = BCM2835_PERI_BASE_PI5 + BCM2835_GPIO_OFFSET;
        mem2_base = BCM2835_PERI_BASE_PI5 + BCM2835_SPI_OFFSET;
        break;
    default:
        return(-1);
    }

    fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: can't open /dev/gpiomem \n", modname);
        return(-1);
    }

    mem1 = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mem1_base);
    if (mem1 == MAP_FAILED)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: can't map mem1\n", modname);
        close(fd);
        return(-1);
    }

    mem2 = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mem2_base);
    if (mem2 == MAP_FAILED)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "%s: can't map mem2\n", modname);
        close(fd);
        return(-1);
    }

    close(fd);

    return(0);
}


/*    GPIO USAGE
 *
 *    RPI:
 *
 *  GPIO    Dir Signal
 *
 *  8   OUT CE0
 *  9   IN  MISO
 *  10  OUT MOSI
 *  11  OUT SCLK
 *
 */

void rpi_setup_gpio()
{
    u32     x;

    /* change SPI pins */
    x = BCM2835_GPFSEL0;
    x &= ~(0x3F000000);
    x |= 0x24000000;
    BCM2835_GPFSEL0 = x;

    x = BCM2835_GPFSEL1;
    x &= ~(0x0000003F);
    x |= 0x00000024;
    BCM2835_GPFSEL1 = x;

    /* set up SPI */
    BCM2835_SPICLK = BCM2835_SPICLKDIV;

    BCM2835_SPICS = 0;

    /* clear FIFOs */
    BCM2835_SPICS |= BCM_SPI_CS_CLEAR_RX | BCM_SPI_CS_CLEAR_TX;
}


void rpi_restore_gpio()
{
    u32		x;

    /* change SPI pins to inputs*/
    x = BCM2835_GPFSEL0;
    x &= ~(0x3F000000);
    BCM2835_GPFSEL0 = x;

    x = BCM2835_GPFSEL1;
    x &= ~(0x0000003F);
    BCM2835_GPFSEL1 = x;
}


int CheckIdBoars()	//1=OK, 0,-1,-2,-3,...=Chyba na 0.,1.,2.,3.-tej doske
{
    int         ii, jj, len;
    int         Id;
    int         retval = 1;
    short int   *pId;

    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
        BoardTypeDetect[ii] = 0;

    memset(txBuf, BOARDS_INFO_NOTHING, BUFSIZE);

    len = BUFSIZE;
    write_read_buf(txBuf, rxBuf, len);


    if (b_debug3)
    {
        for (jj = 0; jj < 100; jj++)
        {
            *Globals->x_wr[jj] = txBuf[jj] & 0xFF;
            *Globals->x_rd[jj] = rxBuf[jj] & 0xFF;
        }
        for (jj = 0; jj < 5; jj++)
        {
            *Globals->x_debug[jj * 2 + 0] = (intptr_t)Boards[jj].SpiWrPointer - (intptr_t)txBuf;
            *Globals->x_debug[jj * 2 + 1] = (intptr_t)Boards[jj].SpiRdPointer - (intptr_t)rxBuf;
        }
    }


    for (ii = 1, jj = 0; ii < BUFSIZE - 1; ii++)
    {
        if (rxBuf[ii] == 0)             continue;
        if (rxBuf[ii] == 0xff)          continue;

        pId = (short int *)(&rxBuf[ii - 1]);
        Id = *pId;

        BoardTypeDetect[jj] = 3800 + ((Id >> 8) & 0xFF);

        Boards[jj].SpiRdPointer = rxBuf + ii - 1;
        if (jj == 0)    Boards[jj].SpiWrPointer = txBuf;
        else            Boards[jj].SpiWrPointer = txBuf + ii - 1 - jj;

        switch (BoardTypeDetect[jj])
        {
        case 3805:  len = sizeof(S_SPI_OUT_3805);
            if (len < sizeof(S_SPI_IN_3805))    len = sizeof(S_SPI_IN_3805);
            Boards[jj].SpiWrLen = ii + len;
            jj++;
            break;

        case 3809:  len = sizeof(S_SPI_OUT_3809);
            if (len < sizeof(S_SPI_IN_3809))    len = sizeof(S_SPI_IN_3809);
            Boards[jj].SpiWrLen = ii + len;
            jj++;
            break;

        case 3811:  len = sizeof(S_SPI_OUT_3811);
            if (len < sizeof(S_SPI_IN_3811))    len = sizeof(S_SPI_IN_3811);
            Boards[jj].SpiWrLen = ii + len;
            jj++;
            break;

        default:
            break;
        }

        if (jj >= DEF_MAX_BOARDS)		break;
    }


    if (b_debug3)
    {
        for (jj = 5; jj < 10; jj++)
        {
            *Globals->x_debug[jj * 2 + 0] = (intptr_t)Boards[jj - 5].SpiWrPointer - (intptr_t)txBuf;
            *Globals->x_debug[jj * 2 + 1] = (intptr_t)Boards[jj - 5].SpiRdPointer - (intptr_t)rxBuf;
        }
    }


    for (ii = 0; ii < DEF_MAX_BOARDS; ii++)
    {
        if (BoardParam[ii][DEF_BOARD_TYPE] == 0)    break;

        if (BoardParam[ii][DEF_BOARD_TYPE] != BoardTypeDetect[ii])
        {
            retval = 0 - ii;
            break;
        }
    }

    return(retval);
}


void SetFirstBoard()
{
    int len;

    len = 10;
    memset(txBuf, BOARDS_INFO_NOTHING, len);

    txBuf[0] = BOARDS_INFO_FIRST;

    write_read_buf(txBuf, rxBuf, len);
}


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------


u32 bcm2835_peri_read(volatile u32* paddr)
{
    return(*paddr); /* Not used on bcm2835 */
/*
    if (debug)
    {
//        printf("bcm2835_peri_read  paddr %08X\n", (unsigned) paddr);
    return 0;
    }
    else
    {
#ifdef __arm__
    // Following code provides memory barriers before and after the read
       u32 ret;
#ifdef BCM2835_HAVE_DMB
       __asm__(        "\
  dmb                    \
  ldr %[ret], [%[paddr]] \
  dmb                                   \
" : [ret] "=r" (ret) : [paddr] "r" (paddr) : "memory" );
#else
       __asm__(            "\
  mov r10,#0               \n           \
  mcr p15,0,r10, c7, c10, 5\n           \
  ldr %[ret], [%[paddr]]   \n           \
  mcr p15,0,r10, c7, c10, 5\n                   \
" : [ret] "=r" (ret) : [paddr] "r" (paddr) : "r10", "memory" );
#endif
       return ret;
#else
       return *paddr; // Not used on bcm2835
#endif

    }
*/
}


void bcm2835_peri_write(volatile u32* paddr, u32 value)
{
    *paddr = value; // Not used on bcm2835

 /*
     if (debug)
     {
 //	printf("bcm2835_peri_write paddr %08X, value %08X\n", (unsigned) paddr, value);
     }
     else
     {
 #ifdef __arm__
     // Following code provides memory barriers before and after the write
 #ifdef BCM2835_HAVE_DMB
        __asm__(        "\
   dmb                 \
   str %[value], [%[paddr]] \
   dmb	                                \
 " : : [paddr] "r" (paddr), [value] "r" (value) : "memory" );
 #else
        __asm__(            "\
   mov r10,#0               \n          \
   mcr p15,0,r10, c7, c10, 5\n          \
   str %[value], [%[paddr]] \n          \
   mcr p15,0,r10, c7, c10, 5\n                      \
 " : : [paddr] "r" (paddr), [value] "r" (value) : "r10", "memory" );
 #endif

 #endif
     }
 */
}


void bcm2835_peri_set_bits(volatile u32* paddr, u32 value, u32 mask)
{
    u32 v = bcm2835_peri_read(paddr);
    v = (v & ~mask) | (value & mask);
    bcm2835_peri_write(paddr, v);
}


void bcm2835_spi_setDataMode(u8 mode)
{
    u32 v = BCM2835_SPICS;
    u32 mask = BCM2835_SPI0_CS_CPOL | BCM2835_SPI0_CS_CPHA;
    u32 value = mode << 2;
    v = (v & ~mask) | (value & mask);
    tmpdata = v;
    BCM2835_SPICS = v;

    /*
        volatile u32* paddr = (u32 *)BCM2835_SPICS;     //bcm2835_spi0 + BCM2835_SPI0_CS/4;
        // Mask in the CPO and CPHA bits of CS
        bcm2835_peri_set_bits(paddr, mode << 2, BCM2835_SPI0_CS_CPOL | BCM2835_SPI0_CS_CPHA);
    */
}

void bcm2835_spi_setClockDivider(u16 divider)
{
    BCM2835_SPICLK = divider;

    /*
        volatile u32* paddr = (u32 *)BCM2835_SPICLK;    //bcm2835_spi0 + BCM2835_SPI0_CLK/4;
        bcm2835_peri_write( paddr, divider );
    */
}

void bcm2835_spi_chipSelect(u8 cs)
{
    volatile    u32* paddr = (u32 *)(uintptr_t)BCM2835_SPICS;      //bcm2835_spi0 + BCM2835_SPI0_CS/4;
        /* Mask in the CS bits of CS */
    bcm2835_peri_set_bits(paddr, cs, BCM2835_SPI0_CS_CS);
}

void bcm2835_spi_setChipSelectPolarity(u8 cs, u8 active)
{
    volatile    u32* paddr = (u32 *)(uintptr_t)BCM2835_SPICS;      //bcm2835_spi0 + BCM2835_SPI0_CS/4;
    u8      shift = 21 + cs;
    /* Mask in the appropriate CSPOLn bit */
    bcm2835_peri_set_bits(paddr, active << shift, 1 << shift);
}
