/*
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dcmi.h"
#include "misc.h"
#include <stdio.h>
#include "yInc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "yInc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#if(PROCESSOR == STM32F103C8)
#else
/*OV7670 is a low cost image sensor + DSP that can operate at
 * a maximum of 30 fps
 * and 640 x 480 ("VGA") resolutions, equivalent to 0.3 Megapixels.
 * The captured image can be pre-processed by the DSP before sending it out.
 * This preprocessing can be configured via the Serial Camera Control Bus (SCCB).
 *
 * formats used by the OV7670 are the {RGB565, RGB555, RGB444}. Also it supports YCbCr422 format.
 * [Ref] YCbCr is a format in which a RGB color can be encoded.
 * The Y or luminance component is the amount of white light of a color,
 * and the Cb and Cr are the chroma components,
 * which respectly encode the blue and red levels relative to the luminance component.
 *
 * [Note] YCbCr format can be changed into RGB format.
 *
 * We here use XCLK of 24MHz from MCO2.
 * After a clock signal has been applied to the XCLK pin,
 * the OV7670 will start driving its VSYNC, HREF and D0-D7 pins.
 *
 * - D0-D7 must be sampled at the rising edge of the PCLK signal.
 * - D0-D7 must be sampled only when HREF is high.
 * - rising edge of HREF signals the start of a line,
 * - falling edge of HREF signals the end of the line.
 * - All these bytes sampled when HREF was high, correspond to the pixels in one line.
 * (Note) one byte is not a pixel, it depends on the format chosen. By default, the format is YCbCr422, this means that in average two bytes correspond to a pixel.
 *
 * By default, PCLK will have the same frequency of XCLK.
 * However prescalers and PLLs can be configured using the SCCB, to produce a PCLK of different frequency.
 * - A PCLK of 24 MHz will produce 30 fps, a PCLK of 12 MHz will produce 15 fps and so on.
 *  All this is independent of the format of the image (VGA, CIF, QCIF, etc).
 *
 * [SCCB (Serial Camera Control Bus)]
 * This SCCB protocol is very similar to the I2C protocol.
 *   The 7 bit SCCB/I2C address is 0x21,
 *   this translates to 0x42 for write address and 0x43 for read address.
 * [NOTE]
 * Always read a register first, modify the desired bits and then write it back to the OV7670.
 *
 * [Changing the FPS]
 * To change the frames per second (fps), we need to change the frequency of PCLK.
 * And for that we need to modify the related registers via the SCCB.
 * (example) From XCLK of 24MHz, we try to get 12MHz of PCLK.
 * - 0x11(CLKRC) : bit[6]=0, bit[5:0] = 2. (prescaler of 2 --> divided by 2)
 * - 0x6B(DBLV)  : bit [7:6] = 00 (bypass) (no multiplier)
 *
 * [Changing the frame format/resolution]
 * The OV7670 can use various frame formats: VGA (640 x 480), QVGA (320 x 240), CIF (352 x 240), QCIF (176 x 144), Manual scaling
 * By default, the OV7670 uses the VGA format,you might want the QCIF format instead.
 * To change the format we need to modify the following registers.
 * - 0x0c(COM3) : bit[3]=1 (ScaleEn)
 * - 0x12(COM7) : bit[3]=1 (Use QCIF)
 *
 * [Programming]
 *
 * There are two interrupts : DCMI_IRQn and DMA2_Stream7_IRQn;

 * [TEST]
 * Start only grabbing a snapshot (only one frame),
 * this is from VSYNC falling edge to VSYNC rising edge.
 * Repeat this procedure multiple times, and make sure the number of bytes per snapshot, is constant.
 *
 * Cover the camera lens, and verify that the snapshot have the following information (in bytes):
 * 128 0 128 0 128 0 128 0 ... i.e. every even byte should be 128 and every odd byte should be 0.
 * This correspond to a pitch black image.
 *
 * [RUN]
 * STM32 comes with a Digital CaMera Interface (DCMI) and a Direct Memory Access (DMA) controller,
 * these two can capture the frames without the intervention of the processor.
 * We used the QCIF format, however we was receiving
 *   174 x 144 pixels instead of 176 x 144. Because Color format was the default YCbCr422.
 * One of every six frames was sent to a PC using a UART communication at 3 Mbps.
 *
 */
/* vim: set et sw=4: */
/*
 * registers from:
 * fuyuno sakura at http://mbed.org/users/mio/
 * Martin Smith at http://mbed.org/users/ms523/
 */
/*
OV7670 : 640x480
		 30fps

AL422B = 385KB FIFO
Internal CLK = 24MHz
OV7670 needs 2.8V LDO (installed on Board)
Open Sources : RGB565 (STM2F103) and YUV422(MSP430F149)

[Pinout]
1    2
VCC(3.3V) GND
SCL SDA
iVSYNC iHREF
WEN NC/RE#
RRST OE#
RCK GND
D0 D1
D2 D3
D4 D5
D6 D7

[OV7670_I2C_ADDR (7 bit addr) = 0x21 (0x42(W) or 0x43(R))
*/
#define REG_GAIN        0x00    /* Gain lower 8 bits (rest in vref) */
#define REG_BLUE        0x01    /* blue gain */
#define REG_RED         0x02    /* red gain */
#define REG_VREF        0x03    /* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1        0x04    /* Control 1 */
#define COM1_CCIR656    0x40    /* CCIR656 enable */
#define REG_BAVE        0x05    /* U/B Average level */
#define REG_GbAVE       0x06    /* Y/Gb Average level */
#define REG_AECHH       0x07    /* AEC MS 5 bits */
#define REG_RAVE        0x08    /* V/R Average level */
#define REG_COM2        0x09    /* Control 2 */
#define COM2_SSLEEP     0x10    /* Soft sleep mode */
#define REG_PID         0x0a    /* Product ID MSB */
#define REG_VER         0x0b    /* Product ID LSB */
#define REG_COM3        0x0c    /* Control 3 */
#define COM3_SWAP       0x40    /* Byte swap */
#define COM3_SCALEEN    0x08    /* Enable scaling */
#define COM3_DCWEN      0x04    /* Enable downsamp/crop/window */
#define REG_COM4        0x0d    /* Control 4 */
#define REG_COM5        0x0e    /* All "reserved" */
#define REG_COM6        0x0f    /* Control 6 */
#define REG_AECH        0x10    /* More bits of AEC value */
#define REG_CLKRC       0x11    /* Clocl control */
#define CLK_EXT         0x40    /* Use external clock directly */
#define CLK_SCALE       0x3f    /* Mask for internal clock scale */
#define REG_COM7        0x12    /* Control 7 */
#define COM7_RESET      0x80    /* Register reset */
#define COM7_FMT_MASK   0x38
#define COM7_FMT_VGA    0x00
#define COM7_FMT_CIF    0x20    /* CIF format */
#define COM7_FMT_QVGA   0x10    /* QVGA format */
#define COM7_FMT_QCIF   0x08    /* QCIF format */
#define COM7_RGB        0x04    /* bits 0 and 2 - RGB format */
#define COM7_YUV        0x00    /* YUV */
#define COM7_BAYER      0x01    /* Bayer format */
#define COM7_PBAYER     0x05    /* "Processed bayer" */
#define REG_COM8        0x13    /* Control 8 */
#define COM8_FASTAEC    0x80    /* Enable fast AGC/AEC */
#define COM8_AECSTEP    0x40    /* Unlimited AEC step size */
#define COM8_BFILT      0x20    /* Band filter enable */
#define COM8_AGC        0x04    /* Auto gain enable */
#define COM8_AWB        0x02    /* White balance enable */
#define COM8_AEC        0x01    /* Auto exposure enable */
#define REG_COM9        0x14    /* Control 9  - gain ceiling */
#define REG_COM10       0x15    /* Control 10 */
#define COM10_HSYNC     0x40    /* HSYNC instead of HREF */
#define COM10_PCLK_HB   0x20    /* Suppress PCLK on horiz blank */
#define COM10_HREF_REV  0x08    /* Reverse HREF */
#define COM10_VS_LEAD   0x04    /* VSYNC on clock leading edge */
#define COM10_VS_NEG    0x02    /* VSYNC negative */
#define COM10_HS_NEG    0x01    /* HSYNC negative */
#define REG_HSTART      0x17    /* Horiz start high bits */
#define REG_HSTOP       0x18    /* Horiz stop high bits */
#define REG_VSTART      0x19    /* Vert start high bits */
#define REG_VSTOP       0x1a    /* Vert stop high bits */
#define REG_PSHFT       0x1b    /* Pixel delay after HREF */
#define REG_MIDH        0x1c    /* Manuf. ID high */
#define REG_MIDL        0x1d    /* Manuf. ID low */
#define REG_MVFP        0x1e    /* Mirror / vflip */
#define MVFP_MIRROR     0x20    /* Mirror image */
#define MVFP_FLIP       0x10    /* Vertical flip */
#define REG_AEW         0x24    /* AGC upper limit */
#define REG_AEB         0x25    /* AGC lower limit */
#define REG_VPT         0x26    /* AGC/AEC fast mode op region */
#define REG_HSYST       0x30    /* HSYNC rising edge delay */
#define REG_HSYEN       0x31    /* HSYNC falling edge delay */
#define REG_HREF        0x32    /* HREF pieces */
#define REG_TSLB        0x3a    /* lots of stuff */
#define TSLB_YLAST      0x04    /* UYVY or VYUY - see com13 */
#define REG_COM11       0x3b    /* Control 11 */
#define COM11_NIGHT     0x80    /* NIght mode enable */
#define COM11_NMFR      0x60    /* Two bit NM frame rate */
#define COM11_HZAUTO    0x10    /* Auto detect 50/60 Hz */
#define COM11_50HZ      0x08    /* Manual 50Hz select */
#define COM11_EXP       0x02
#define REG_COM12       0x3c    /* Control 12 */
#define COM12_HREF      0x80    /* HREF always */
#define REG_COM13       0x3d    /* Control 13 */
#define COM13_GAMMA     0x80    /* Gamma enable */
#define COM13_UVSAT     0x40    /* UV saturation auto adjustment */
#define COM13_UVSWAP    0x01    /* V before U - w/TSLB */
#define REG_COM14       0x3e    /* Control 14 */
#define COM14_DCWEN     0x10    /* DCW/PCLK-scale enable */
#define REG_EDGE        0x3f    /* Edge enhancement factor */
#define REG_COM15       0x40    /* Control 15 */
#define COM15_R10F0     0x00    /* Data range 10 to F0 */
#define COM15_R01FE     0x80    /*            01 to FE */
#define COM15_R00FF     0xc0    /*            00 to FF */
#define COM15_RGB565    0x10    /* RGB565 output */
#define COM15_RGB555    0x30    /* RGB555 output */
#define REG_COM16       0x41    /* Control 16 */
#define COM16_AWBGAIN   0x08    /* AWB gain enable */
#define REG_COM17       0x42    /* Control 17 */
#define COM17_AECWIN    0xc0    /* AEC window - must match COM4 */
#define COM17_CBAR      0x08    /* DSP Color bar */
#define REG_CMATRIX_BASE 0x4f
#define CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58
#define REG_BRIGHT      0x55    /* Brightness */
#define REG_CONTRAS     0x56    /* Contrast control */
#define REG_GFIX        0x69    /* Fix gain control */
#define REG_REG76       0x76    /* OV's name */
#define R76_BLKPCOR     0x80    /* Black pixel correction enable */
#define R76_WHTPCOR     0x40    /* White pixel correction enable */
#define REG_RGB444      0x8c    /* RGB 444 control */
#define R444_ENABLE     0x02    /* Turn on RGB444, overrides 5x5 */
#define R444_RGBX       0x01    /* Empty nibble at end */
#define REG_HAECC1      0x9f    /* Hist AEC/AGC control 1 */
#define REG_HAECC2      0xa0    /* Hist AEC/AGC control 2 */
#define REG_BD50MAX     0xa5    /* 50hz banding step limit */
#define REG_HAECC3      0xa6    /* Hist AEC/AGC control 3 */
#define REG_HAECC4      0xa7    /* Hist AEC/AGC control 4 */
#define REG_HAECC5      0xa8    /* Hist AEC/AGC control 5 */
#define REG_HAECC6      0xa9    /* Hist AEC/AGC control 6 */
#define REG_HAECC7      0xaa    /* Hist AEC/AGC control 7 */
#define REG_BD60MAX     0xab    /* 60hz banding step limit */

// DMA Stream parameters definitions. You can modify these parameters to select a different DMA Stream and/or channel.
// But note that only DMA2 Streams are capable of Memory to Memory transfers. (We here use Peripheral to Memory)
#define DMA_CameraToRAM_Stream   		DMA2_Stream7
#define DMA_Camera_Channel         		DMA_Channel_1
#define DMA_Camera_STREAM_CLOCK    		RCC_AHB1Periph_DMA2
#define DMA_Camera_STREAM_IRQ      		DMA2_Stream7_IRQn
#define DMA_Camera_IT_TCIF         		DMA_IT_TCIF7
#define DMA_Camera_STREAM_IRQHANDLER    DMA2_Stream7_IRQHandler
#define DCMI_DR_ADDRESS       0x50050028
#define picture_x 176
#define picture_y 144

#define OV7670_REG_NUM 16 //121//122

//QVGA
const uint8_t g_OV7670_reg[OV7670_REG_NUM][2] = {
	{0x12, 0x80},		// Reset Register

	/********** QCIF **********/
	//{0x0c, 0x0c},			// COM3 : 0000-1100 = ScaleEN-DcwEN,
	{0x12, 0x0c},			// COM7 : 0000-1100 = QCIF/RGB (0x14 = QVGA size, RGB mode; 0xc = QCIF (RGB);0x8 = QCIF, YUV)
	//{0x12, 0x0e},			// COM7 : 0000-1110 = QCIF/YUV (Color Bar  for TEST)
	//{0x12, 0x14},			// COM7 : QVGA size, RGB Mode -- RAM Shortage

	{0x40, 0xd0}, 			////COM15 : 11-01-xxxx = OutputRange{00~FF}, RGB565, RSVD
	{0xb0, 0x84 },			//YOON....Color mode (Not documented??)

	// Hardware window
	{ 0x11, 0x01 },		//PCLK settings: 0000-0001 =DblClkDis,NoUseExtClk,Prescaler=1. 15fps
	{ 0x32, 0x80 },		//HREF : 10-000-000 =HREFoffset(2).
	{ 0x17, 0x17 },		//HSTART : 0011-1010 = HREF Column Start High 8 bit. (with HREF[2:0] LSB) -->
	{ 0x18, 0x05 },		//HSTOP :  0011-1010 = HREF Column Stop High 8 bit. (with HREF[5:3] LSB) -->
	{ 0x03, 0x0a },		//VREF  : 00-00-00-10 = AGC, Vertical Frame(row) End High 8 bit.(with VREF[3:2] LSB)
	{ 0x19, 0x02 },		//VSTART: 0000-0010 = Vertical Frame(row) Start High 8 bit.(with VREF[1:0] LSB)
	{ 0x1a, 0x7a },		//VSTOP : 0111-1011 = Vertical Frame(row) End High 8 bit.(with VREF[3:2] LSB)

	// Scalling numbers
	{0x70, 0x3a},		//X_SCALING : 0-0111010 = NoTestOut, Horizontal ScaleFactor.
	{0x71, 0x35},		//Y_SCALING : 0-0110101 = NoTestOut, Vertical ScaleFactor.
	{0x72, 0x11},		//DCW_SCALING : 0001-0001
	{0x73, 0xf0},		//PCLK_DIV_SCALING : 1111-0001 = Divided by 2 (Should be COM14[3]=1. Should change with COM14[2:0].)
	{0xa2, 0x02}		//PCLK_DELAY_SCALING:  Not Documented. ????
};

/*
#define OV7670_REG_NUM 121//122

//ADDED BY SUNG
const uint8_t g_OV7670_reg[OV7670_REG_NUM][2] = {
	{0x12, 0x80},		// Reset Register

	// QCIF *********
	//{0x0c, 0x0c},			// COM3 : 0000-1100 = ScaleEN-DcwEN,
	//{0x12, 0x0c},			// COM7 : 0000-1100 = QCIF/YUV (0x14 = QVGA size, RGB mode; 0xc = QCIF (RGB);0x8 = QCIF, YUV)
	{0x12, 0x0e},			// COM7 : 0000-1110 = QCIF/YUV (Color Bar  for TEST)
	{0x0c, 0x4c },			// COM3 : 0100-1100 = SWAP-ScaleEN-DcwEN

	//{ 0xb0, 0x84 },		//YOON....Color mode (Not documented??)

	//{0x3e, 0x11},			// COM14 : 0001-0001 (DCW/SCALING PCLK, NoManualScaling, PCLK Divided by 2
	//{0x3e, 0x12},			// COM14 : 0001-0010 (DCW/SCALING PCLK, NoManualScaling, PCLK Divided by 4
	{0x3e, 0x1a},			// COM14 : 0001-1010 (DCW/SCALING PCLK, ManualScaling, PCLK Divided by 4

	// Hardware window
	//{ 0x11, 0xc0 },		//PCLK settings: 1100-0000 =DblClkEn,UseExtClk,NoPrescaler. 15fps
	{ 0x11, 0xc0 },		//PCLK settings: 1000-0010 =DblClkEn,NoUseExtClk,Prescaler=2. 15fps


	{ 0x32, 0x80 },		//HREF : 10-000-000 =HREFoffset(2).
	{ 0x17, 0x3a },		//HSTART : 0011-1010 = HREF Column Start High 8 bit. (with HREF[2:0] LSB) -->
	{ 0x18, 0x03 },		//HSTOP :  0011-1010 = HREF Column Stop High 8 bit. (with HREF[5:3] LSB) -->
	{ 0x03, 0x06 },		//VREF  : 00-00-00-10 = AGC, Vertical Frame(row) End High 8 bit.(with VREF[3:2] LSB)
	{ 0x19, 0x02 },		//VSTART: 0000-0010 = Vertical Frame(row) Start High 8 bit.(with VREF[1:0] LSB)
	{ 0x1a, 0x7b },		//VSTOP : 0111-1011 = Vertical Frame(row) End High 8 bit.(with VREF[3:2] LSB)

	// Scalling numbers
	{0x70, 0x3a},		//X_SCALING : 0-0111010 = NoTestOut, Horizontal ScaleFactor.
	{0x71, 0x35},		//Y_SCALING : 0-0110101 = NoTestOut, Vertical ScaleFactor.
	{0x72, 0x11},		//DCW_SCALING : 0001-0001
	//{0x73, 0xf1},		//PCLK_DIV_SCALING : 1111-0001 = Divided by 2 (Should be COM14[3]=1. Should change with COM14[2:0].)
	{0x73, 0xf2},		//PCLK_DIV_SCALING : 1111-0010 = Divided by 4 (Should be COM14[3]=1. Should change with COM14[2:0].)
	{0xa2, 0x52},		//PCLK_DELAY_SCALING:  Not Documented. ????

	///???
	{0x13, 0xe0}, 		//COM8 : 1-110-0000 = EnFastAGC,UnlimitStepSize,BandingFilterOff, AgcDis,AwbDis,AecDis
	{0x00, 0x00},		//GAIN : AGC Gain = 0
	//{0x14, 0x48},		//COM9 : 0100-1000 = MaxAGC(=32), FreezeACG/AECdisabled
	{0x14, 0x40},		//COM9 : 0100-1000 = MaxAGC(=32), FreezeACG/AECdisabled
	{0x13, 0xe5},		//COM8 : 1-110-0101 = EnFastAGC,UnlimitStepSize,BandingFilterOff, AgcEn,AwbDis,AecEn

	{0xa4, 0x89},		//???? Undocumented.


	// Matrix coefficients
	{ 0x4f, 0x80 }, //
	{ 0x50, 0x80 }, //
	{ 0x51, 0x00 }, //
	{ 0x52, 0x22 }, //
	{ 0x53, 0x5e }, //
	{ 0x54, 0x80 }, //
	{ 0x58, 0x9e },

	// Gamma curve values
	{ 0x7a, 0x20 }, //
	{ 0x7b, 0x10 }, //
	{ 0x7c, 0x1e }, //
	{ 0x7d, 0x35 }, //
	{ 0x7e, 0x5a }, //
	{ 0x7f, 0x69 }, //
	{ 0x80, 0x76 }, //
	{ 0x81, 0x80 }, //
	{ 0x82, 0x88 }, //
	{ 0x83, 0x8f }, //
	{ 0x84, 0x96 }, //
	{ 0x85, 0xa3 }, //
	{ 0x86, 0xaf }, //
	{ 0x87, 0xc4 }, //
	{ 0x88, 0xd7 }, //
	{ 0x89, 0xe8 },
	{ 0x8c, 0x00 },
	{ 0x40, 0xd0 },//COM15 : 11-01-xxxx = OutputRange{00~FF}, RGB565, RSVD

	// AGC and AEC parameters
	{ 0xa5, 0x05 }, //
	{ 0xab, 0x07 }, //
	{ 0x24, 0x95 }, //
	{ 0x25, 0x33 }, //
	{ 0x26, 0xe3 }, //
	{ 0x9f, 0x78 }, //
	{ 0xa0, 0x68 }, //
	{ 0xa1, 0x03 }, //
	{ 0xa6, 0xd8 }, //
	{ 0xa7, 0xd8 }, //
	{ 0xa8, 0xf0 }, //
	{ 0xa9, 0x90 }, //
	{ 0xaa, 0x94 }, //
	{ 0x10, 0x00 },

	// AWB parameters
	{ 0x43, 0x0a }, //
	{ 0x44, 0xf0 }, //
	{ 0x45, 0x34 }, //
	{ 0x46, 0x58 }, //
	{ 0x47, 0x28 }, //
	{ 0x48, 0x3a }, //
	{ 0x59, 0x88 }, //
	{ 0x5a, 0x88 }, //
	{ 0x5b, 0x44 }, //
	{ 0x5c, 0x67 }, //
	{ 0x5d, 0x49 }, //
	{ 0x5e, 0x0e }, //
	{ 0x6c, 0x0a }, //
	{ 0x6d, 0x55 }, //
	{ 0x6e, 0x11 }, //
	{ 0x6f, 0x9f }, //
	{ 0x6a, 0x40 }, //
	{ 0x01, 0x40 }, //
	{ 0x02, 0x60 }, //
	{ 0x13, 0xe7 },

	// Additional parameters
	{ 0x34, 0x11 }, //ArrayReferenceControl
	{ 0x3f, 0x00 }, //Edge Enhance
	{ 0x75, 0x05 }, //Edge Enhance
	{ 0x76, 0xe1 }, //Pixel Correction
	{ 0x4c, 0x00 }, //Denoise Threshold
	{ 0x77, 0x01 }, //Offset/Denoise Range Control
	{ 0xb8, 0x0a }, //Rsvd
	{ 0x41, 0x18 }, //COM16 : xx-01-1x0x :EdgeEnhanceDis, AWBgainEn,OriginalMatrix,
	{ 0x3b, 0x12 }, //COM11 : 0-00-1-0-0-1-0 (NightModeDis, MinFrameRateAtNightMode(same),50/60HzAutoDetectionEn, SelectBD60STasBandingFilterValue,..)
	{ 0xa4, 0x88 }, //Rsvd
	{ 0x96, 0x00 }, //Rsvd
	{ 0x97, 0x30 }, //Rsvd
	{ 0x98, 0x20 }, //Rsvd
	{ 0x99, 0x30 }, //Rsvd
	{ 0x9a, 0x84 }, //Rsvd
	{ 0x9b, 0x29 }, //Rsvd
	{ 0x9c, 0x03 }, //Rsvd
	{ 0x9d, 0x4c }, //50HzBandingFilterValue
	{ 0x9e, 0x3f }, //60HzBandingFilterValue
	{ 0x78, 0x04 }, //Rsvd
	{ 0x0e, 0x61 }, //COM5
	{ 0x0f, 0x4b }, //COM6 = 0100-1011 (DisableHREFatOpticalBlack, UseElectricalBlackLineAsBLCsignal, DigitalBLCdis, ResetAllTimingWhenFormatChangees
	{ 0x16, 0x02 }, //Rsvd
	{ 0x1e, 0x00 }, //MVFP : Mirror/VFlipEn (Normal, Normal)
	{ 0x21, 0x02 }, //ADCCTR1 :RSVD
	{ 0x22, 0x91 }, //ADCCTR2 :RSVD
	{ 0x29, 0x07 }, //RSVD
	{ 0x33, 0x0b }, //ArrayCurrentControl
	{ 0x35, 0x0b }, //Rsvd
	{ 0x37, 0x1d }, //ADC Control
	{ 0x38, 0x71 }, // ACOM
	{ 0x39, 0x2a }, //OFON : ADC Offset Control
	//{ 0x3c, 0x78 }, //COM12 : 0-011-1100 = NoHREFwhenVSYNCisLow, RSVD.
	{ 0x3c, 0x80 }, //COM12 : 0-011-1100 = NoHREFwhenVSYNCisLow, RSVD.
	{ 0x4d, 0x40 }, //Rsvd
	{ 0x4e, 0x20 }, //Rsvd
	{ 0x69, 0x00 }, //GFIX
	{ 0x6b, 0x3a }, //DBLV : 00-x-1-1010 = BypassPLL,intRegulatorEn,ClockDivideControlForDSPscaleControl(valid only COM14[3]=1)
	{ 0x74, 0x10 }, //REG74 : 0001-0000 = DigitalGainControledByREG74[1:0],Bypass
	{ 0x8d, 0x4f }, //Rsvd
	{ 0x8e, 0x00 }, //Rsvd
	{ 0x8f, 0x00 }, //Rsvd
	{ 0x90, 0x00 }, //Rsvd
	{ 0x91, 0x00 }, //Rsvd
	{ 0x96, 0x00 }, //Rsvd
	{ 0x9a, 0x00 }, //Rsvd
	{ 0xb1, 0x0c }, //Rsvd
	{ 0xb2, 0x0e }, //Rsvd
	{ 0xb3, 0x82 }, //Digital BLC Target
	{ 0x4b, 0x01 },// UV Average Enable
};
*/
__IO uint16_t RAM_Buffer[picture_x * picture_y];

uint8_t ov7670_get(uint8_t reg);
uint8_t ov7670_set(uint8_t reg, uint8_t data);
int ov7670_init();
void DCMI_init();
void DMA_init();

volatile int gframe_flag;

#define ONE_BYTE_REG_ADDR 0x01
#define TWO_BYTE_REG_ADDR 0x02

int g_fcount=0;

void ov7670_InitSCCB(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; // this is for the GPIO pins used as I2C1SDA and I2C1SCL
	I2C_InitTypeDef I2C_InitStructure; // this is for the I2C1 initilization
#if(PROCESSOR == STM32F103C8)
#else
	/* enable APB1 peripheral clock for I2C1*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* enable the peripheral clock for the pins used by PB8 for I2C SCL and PB9 for I2C1_SDL*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* This sequence sets up the I2C1SDA and I2C1SCL pins so they work correctly with the I2C1 peripheral  */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // Pins 10(I2C1_SCL) and 11(I2C1_SDA)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// this defines the IO speed and has nothing to do with the baudrate!
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;// this defines the output type as open drain
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;// this activates the pullup resistors on the IO pins
    GPIO_Init(GPIOB, &GPIO_InitStructure);// now all the values are passed to the GPIO_Init()

    // The I2C1_SCL and I2C1_SDA pins are now connected to their AF so that the I2C1 can take over control of the pins

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
#endif
    /* Configure I2C1 */
    I2C_StructInit(&I2C_InitStructure);
    I2C_DeInit(I2C1);

    /* Enable the I2C peripheral */
    I2C_Cmd(I2C1, ENABLE);

    /* Set the I2C structure parameters */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;//I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    /* I2C Peripheral Enable */
    I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
    /* Initialize the I2C peripheral w/ selected parameters */
    I2C_Init(I2C1,&I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
 }

int ov7670_SCCB_send_data(u8 slave_addr, u16 reg_addr, u8* data, u8 addr_len)
{
	int timeout = 0x7FFFFF;
	int ret = 0;

	I2C_GenerateSTART(I2C1, ENABLE);
	while( !I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));
	{
		if ((timeout--) == 0){
			ret = 1;
			goto exit;
		}
	}
	I2C_Send8bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);
	while(!(I2C1->SR1 & (1 << 1))); // check ADDR bit
	{
		if ((timeout--) == 0){
			ret = 2;
			goto exit;
		}
	}
	while(!(I2C1->SR2 & (1 << 2)));   // check TRA bit
	{
		if ((timeout--) == 0)  {
			ret = 3;
			goto exit;
		}
	}

	/* 2 byte reg address */
	if(addr_len == TWO_BYTE_REG_ADDR) {
		// MSB
		I2C_SendData(I2C1, (0xFF & (reg_addr >> 8)) );
		while(!(I2C1->SR1 & (1 << 7)));
		{
			if ((timeout--) == 0)  {
				ret = 4;
				goto exit;
			}
		}

		// LSB
		I2C_SendData(I2C1, (0xFF & reg_addr));
		while(!(I2C1->SR1 & (1 << 7)));
		{
			if ((timeout--) == 0){
				ret = 5;
				goto exit;
			}
		}

	}
	/* 1 byte reg address */
	else  {
		I2C_SendData(I2C1, (0xFF & reg_addr));
		while(!(I2C1->SR1 & (1 << 7)));
		{
			if ((timeout--) == 0)	{
				ret = 6;
				goto exit;
			}
		}
	}
	I2C_SendData(I2C1, *data);
	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	{
//   if ((timeout--) == 0)
   {
    ret = 7;
    goto exit;
   }
  }

exit:
  I2C_GenerateSTOP(I2C1, ENABLE);
  return ret;
}

int ov7670_SCCB_receive_data(u8 slave_addr, u16 reg_addr, u8* data, u8 addr_len)
{
  int timeout = 0x7FFFFF;
  int ret = 0;
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
  {
   if ((timeout--) == 0)
   {
    ret = 1;
    goto exit;
   }
  }

  I2C_Send8bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);
  while(!(I2C1->SR1 & (1 << 1))) // check ADDR bit
  {
   if ((timeout--) == 0)
   {
    ret = 2;
    goto exit;
   }
  }

  while(!(I2C1->SR2 & (1 << 2)))   // check TRA bit
  {
   if ((timeout--) == 0)
   {
    ret = 3;
    goto exit;
   }
  }

  /* 2 byte reg address */
  if(addr_len == TWO_BYTE_REG_ADDR)
  {
	  // MSB
	  I2C_SendData(I2C1, (0xFF & (reg_addr >> 8)) );
	  while(!(I2C1->SR1 & (1 << 7)))
	  {
		  if ((timeout--) == 0)
		  {
			  ret = 4;
			  goto exit;
		  }
	  }

	  // LSB
	  I2C_SendData(I2C1, (0xFF & reg_addr));
	  while(!(I2C1->SR1 & (1 << 7)))
	  {
		  if ((timeout--) == 0)
		  {
			  ret = 5;
			  goto exit;
		  }
	  }
  }  /* 1 byte reg address */
  else
  {
   I2C_SendData(I2C1, (0xFF & reg_addr));
   while(!(I2C1->SR1 & (1 << 7)))
   {
   if ((timeout--) == 0)
   {
    ret = 6;
    goto exit;
   }
  }
  }

  I2C_GenerateSTOP(I2C1, ENABLE);
  delayx(1000); //add
  I2C_GenerateSTART(I2C1, ENABLE);
  while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
  {
   if ((timeout--) == 0)
   {
    ret = 7;
    goto exit;
   }
  }
  I2C_Send8bitAddress(I2C1, slave_addr, I2C_Direction_Receiver);
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) // check ADDR bit
  {
   if ((timeout--) == 0)
   {
    ret = 8;
    goto exit;
   }
  }

  	  I2C_AcknowledgeConfig(I2C1, DISABLE);
      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);
      while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
      {
    	  if ((timeout--) == 0) 	  {
    		  ret = 10;
    		  goto exit;
    	  }
      }

      *data = I2C_ReceiveData(I2C1);
      I2C_AcknowledgeConfig(I2C1, ENABLE);
      return ret;

exit:
  	  I2C_GenerateSTOP(I2C1, ENABLE);
  	  delayx(1000); //add
  	  return ret;
}

uint8_t ov7670_get(uint8_t reg)//, u8* data)
{
	 static u8 data;
	 delayx(10000);
	 ov7670_SCCB_receive_data(0x43, (u16) reg, &data, ONE_BYTE_REG_ADDR);

	 return data;
}
/*******************************************************************************/
uint8_t ov7670_set(uint8_t reg, uint8_t data)
{
	int ret = 0;
	delayx(10000);
	 ret = ov7670_SCCB_send_data(0x42, (u16) reg, &data, ONE_BYTE_REG_ADDR);
	 return ret;
}

//ADDED BY SUNG ====
void set_reg_ov7670(){
	uint8_t data;
	int ret =0;
	int co;

	if (ov7670_get(REG_PID) != 0x76) {
		return 1;
	}

	for (co=0; co < OV7670_REG_NUM; co++){
		delayx(10000);
		data = g_OV7670_reg[co][1];
		ov7670_SCCB_send_data(0x42, g_OV7670_reg[co][0], &data, ONE_BYTE_REG_ADDR);
	}
}


/*
int camera_config()
{
 int ret = 0;
 u8 data = 0;
 //sprintf(&tempBuff[0],"CAMERA CONFIGURING\r\n");
 //send(&tempBuff[0]);

 ret = camera_read_reg(0x12, &data); //Reset all registers
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 data = 0x80;
 ret = camera_write_reg(0x12, &data);
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 ret = camera_read_reg(0x12, &data); //output format to qcif
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 data = 0xA;               //yuv enabled
 ret = camera_write_reg(0x12, &data);
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 ret = camera_read_reg(0x11, &data); //Set PCLK
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 data = 0x8;
 ret = camera_write_reg(0x11, &data);
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 ret = camera_read_reg(0x04, &data); //Set qqcif
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 data = 0x28;
 ret = camera_write_reg(0x04, &data);
  if(ret < 0)
  return ret;
 Delay(0xFFFF);

 ret = camera_read_reg(0x3A, &data); //Set manual UV values
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 data = 0x1C;
 ret = camera_write_reg(0x3A, &data);
  if(ret < 0)
  return ret;
 Delay(0xFFFF);

 ret = camera_read_reg(0x67, &data); //Set manual U value
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 data = 0xFF;
 ret = camera_write_reg(0x67, &data);
  if(ret < 0)
  return ret;
 Delay(0xFFFF);

 ret = camera_read_reg(0x68, &data); //Set manual V value
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
 data = 0x0;
 ret = camera_write_reg(0x68, &data);
  if(ret < 0)
  return ret;
 Delay(0xFFFF);
//      ret = camera_read_reg(0x40, &data); //Set out put data range
//   if(ret < 0)
//   return ret;
//  Delay(10);
//  data = 0xC0;
//  ret = camera_write_reg(0x40, &data);
//   if(ret < 0)
//   return ret;
//  Delay(10);
//
//  ret = camera_read_reg(0x17, &data); //Set HSTART 0
//   if(ret < 0)
//   return ret;
//  Delay(10);
//  data = 0x14;
//  ret = camera_write_reg(0x17, &data);
//   if(ret < 0)
//   return ret;
//  Delay(10);
//  ret = camera_read_reg(0x18, &data); //Set HSTOP 40
//   if(ret < 0)
//   return ret;
//  Delay(10);
//  data = 0x3C;
//  ret = camera_write_reg(0x18, &data);
//   if(ret < 0)
//   return ret;
//  Delay(10);
// //
//  ret = camera_read_reg(0x19, &data); //Set VSTART 20  We cxan set number of raws
//   if(ret < 0)
//   return ret;
//  Delay(10);
//  data = 0x14;
//  ret = camera_write_reg(0x19, &data);
//   if(ret < 0)
//   return ret;
//  Delay(10);
//  ret = camera_read_reg(0x1A, &data); //Set VSTOP 40
//   if(ret < 0)
//   return ret;
//  Delay(10);
//  data = 0x28;
//  ret = camera_write_reg(0x1A, &data);
//   if(ret < 0)
//   return ret;
//  Delay(10);

sprintf(&tempBuff[0],"CAMERA CONFIGURING DONE\r\n");send(&tempBuff[0]);
return ret;
}
*/
/*
uint8_t ov7670_get(uint8_t reg) {
	uint8_t data = 0;
	SCCB_start(I2C1, 0x42, I2C_Direction_Transmitter);
	SCCB_write(I2C1, reg);
	I2C_stop(I2C1);
	delayx(1000);
	SCCB_start(I2C1, 0x42, I2C_Direction_Receiver); //was 43
	data = I2C_read_nack(I2C1);
	I2C_stop(I2C1);
	delayx(1000);
	return data;
}

uint8_t ov7670_set(uint8_t reg, uint8_t data) {
	SCCB_start(I2C1, 0x42, I2C_Direction_Transmitter);
	SCCB_write(I2C1, reg);
	SCCB_write(I2C1, data);
	I2C_stop(I2C1);
	delayx(1000);
	return 0;
}
*/
int ov7670_init() {
	int hstart = 456, hstop = 24, vstart = 14, vstop = 494;
	unsigned char v;

	if (ov7670_get(REG_PID) != 0x76) {
		return 1;
	}
	ov7670_set(REG_COM7, COM7_RESET); /* reset to default values */
	ov7670_set(REG_CLKRC, 0x01);
	ov7670_set(REG_COM7, COM7_FMT_VGA | COM7_YUV); /* output format: YUCV */

	ov7670_set(REG_HSTART, (hstart >> 3) & 0xff);
	ov7670_set(REG_HSTOP, (hstop >> 3) & 0xff);
	v = ov7670_get(REG_HREF);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	ov7670_set(REG_HREF, v);

	ov7670_set(REG_VSTART, (vstart >> 2) & 0xff);
	ov7670_set(REG_VSTOP, (vstop >> 2) & 0xff);
	v = ov7670_get(REG_VREF);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	ov7670_set(REG_VREF, v);

	ov7670_set(REG_COM3, COM3_SCALEEN | COM3_DCWEN);
	ov7670_set(REG_COM14, COM14_DCWEN | 0x01);
	ov7670_set(0x73, 0xf1);
	ov7670_set(0xa2, 0x52);
	ov7670_set(0x7b, 0x1c);
	ov7670_set(0x7c, 0x28);
	ov7670_set(0x7d, 0x3c);
	ov7670_set(0x7f, 0x69);
	ov7670_set(REG_COM9, 0x38);
	ov7670_set(0xa1, 0x0b);
	ov7670_set(0x74, 0x19);
	ov7670_set(0x9a, 0x80);
	ov7670_set(0x43, 0x14);
	ov7670_set(REG_COM13, 0xc0);
	ov7670_set(0x70, 0x3A);
	ov7670_set(0x71, 0x35);
	ov7670_set(0x72, 0x11);

	/* Gamma curve values */
	ov7670_set(0x7a, 0x20);
	ov7670_set(0x7b, 0x10);
	ov7670_set(0x7c, 0x1e);
	ov7670_set(0x7d, 0x35);
	ov7670_set(0x7e, 0x5a);
	ov7670_set(0x7f, 0x69);
	ov7670_set(0x80, 0x76);
	ov7670_set(0x81, 0x80);
	ov7670_set(0x82, 0x88);
	ov7670_set(0x83, 0x8f);
	ov7670_set(0x84, 0x96);
	ov7670_set(0x85, 0xa3);
	ov7670_set(0x86, 0xaf);
	ov7670_set(0x87, 0xc4);
	ov7670_set(0x88, 0xd7);
	ov7670_set(0x89, 0xe8);

	/* AGC and AEC parameters.  Note we start by disabling those features,
	 then turn them only after tweaking the values. */
	ov7670_set(REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT);
	ov7670_set(REG_GAIN, 0);
	ov7670_set(REG_AECH, 0);
	ov7670_set(REG_COM4, 0x40); /* magic reserved bit */
	ov7670_set(REG_COM9, 0x18); /* 4x gain + magic rsvd bit */
	ov7670_set(REG_BD50MAX, 0x05);
	ov7670_set(REG_BD60MAX, 0x07);
	ov7670_set(REG_AEW, 0x95);
	ov7670_set(REG_AEB, 0x33);
	ov7670_set(REG_VPT, 0xe3);
	ov7670_set(REG_HAECC1, 0x78);
	ov7670_set(REG_HAECC2, 0x68);
	ov7670_set(0xa1, 0x03); /* magic */
	ov7670_set(REG_HAECC3, 0xd8);
	ov7670_set(REG_HAECC4, 0xd8);
	ov7670_set(REG_HAECC5, 0xf0);
	ov7670_set(REG_HAECC6, 0x90);
	ov7670_set(REG_HAECC7, 0x94);
	ov7670_set(REG_COM8,
			COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC);

	/* Almost all of these are magic "reserved" values.  */
	ov7670_set(REG_COM5, 0x61);
	ov7670_set(REG_COM6, 0x4b);
	ov7670_set(0x16, 0x02);
	ov7670_set(REG_MVFP, 0x07);
	ov7670_set(0x21, 0x02);
	ov7670_set(0x22, 0x91);
	ov7670_set(0x29, 0x07);
	ov7670_set(0x33, 0x0b);
	ov7670_set(0x35, 0x0b);
	ov7670_set(0x37, 0x1d);
	ov7670_set(0x38, 0x71);
	ov7670_set(0x39, 0x2a);
	ov7670_set(REG_COM12, 0x78);
	ov7670_set(0x4d, 0x40);
	ov7670_set(0x4e, 0x20);
	ov7670_set(REG_GFIX, 0);
	ov7670_set(0x6b, 0x4a);
	ov7670_set(0x74, 0x10);
	ov7670_set(0x8d, 0x4f);
	ov7670_set(0x8e, 0);
	ov7670_set(0x8f, 0);
	ov7670_set(0x90, 0);
	ov7670_set(0x91, 0);
	ov7670_set(0x96, 0);
	ov7670_set(0x9a, 0);
	ov7670_set(0xb0, 0x84);
	ov7670_set(0xb1, 0x0c);
	ov7670_set(0xb2, 0x0e);
	ov7670_set(0xb3, 0x82);
	ov7670_set(0xb8, 0x0a);

	/* Matrix coefficients */
	ov7670_set(0x4f, 0x80);
	ov7670_set(0x50, 0x80);
	ov7670_set(0x51, 0);
	ov7670_set(0x52, 0x22);
	ov7670_set(0x53, 0x5e);
	ov7670_set(0x54, 0x80);
	ov7670_set(0x58, 0x9e);

	/* More reserved magic, some of which tweaks white balance */
	ov7670_set(0x43, 0x0a);
	ov7670_set(0x44, 0xf0);
	ov7670_set(0x45, 0x34);
	ov7670_set(0x46, 0x58);
	ov7670_set(0x47, 0x28);
	ov7670_set(0x48, 0x3a);
	ov7670_set(0x59, 0x88);
	ov7670_set(0x5a, 0x88);
	ov7670_set(0x5b, 0x44);
	ov7670_set(0x5c, 0x67);
	ov7670_set(0x5d, 0x49);
	ov7670_set(0x5e, 0x0e);
	ov7670_set(0x6c, 0x0a);
	ov7670_set(0x6d, 0x55);
	ov7670_set(0x6e, 0x11);
	ov7670_set(0x6f, 0x9f); /* "9e for advance AWB" */
	ov7670_set(0x6a, 0x40);
	ov7670_set(REG_BLUE, 0x40);
	ov7670_set(REG_RED, 0x60);
	ov7670_set(REG_COM8,
			COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT | COM8_AGC | COM8_AEC
					| COM8_AWB);
	return 0;
}
/*
void SendPicture(void)
{
	int i;
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, 'O');
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, 'K');

	for(i=0;i<50688;i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		//USART_SendData(USART2, RAM_Buffer[i]); //check type 2 bytes or 1 byte?
	}
	//while(1);
}
*/
void ov7670_DCMI_Config() {
	GPIO_InitTypeDef GPIO_InitStructure;
	DCMI_InitTypeDef DCMI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//DCMI_CROPInitTypeDef DCMI_CROPInitStructure;

	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
					| RCC_AHB1Periph_GPIOE, ENABLE);


	// DCMI GPIO configuration
	//PA4/PA6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//PB6/7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PC6/7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//PE0/1/4/5/6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 ;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// Connect DCMI pins to AF13 (DCMI)
	// PORTA
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); // HSYNC PA4
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); // PCLK  PA6
	// PORTB
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); // D5 --PB6
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); // VSYNC -- PB7
	// PORTC
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); // D0  -- PC6
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); // D1  --PC7
	// PORTE
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); // D2 --PE0
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); // D3 -- PE1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); // D4  -- PE4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); // D6 --PE5
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); // D7 --PE6

	// Configures the DCMI to interface with the OV7670 camera module
	// Enable DCMI clock
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	// Reinitialize
	DCMI_DeInit();

	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;//was DCMI_CaptureMode_Continuous;////DCMI_CaptureMode_SnapShot;//DMA_Mode_Circular;//DMA_Mode_Normal;//DMA_Mode_Circular;//DCMI_CaptureMode_SnapShot;//// First take one picture
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware; //Important... YOON
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;//HSYNC Active LOW
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;// --It may occur OverFlow.; //was Low. VSYNC Active High
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_Init(&DCMI_InitStructure);
	//DCMI_JPEGCmd(DISABLE); //was...

	//=== install DCMI Interrupt in Nested Vectored Interrupt Controller
	//Mask Interrupt for DCMI
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	//DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);
	//DCMI_ITConfig(DCMI_IT_LINE, ENABLE);
	DCMI_ITConfig(DCMI_IT_OVF, ENABLE); //add
	DCMI_ITConfig(DCMI_IT_ERR, ENABLE); //add

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn; //78
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); //install isr
}
/*
 * #define DMA_CameraToRAM_Stream   	DMA2_Stream7
#define DMA_Camera_Channel         		DMA_Channel_1
#define DMA_Camera_STREAM_CLOCK    		RCC_AHB1Periph_DMA2
#define DMA_Camera_STREAM_IRQ      		DMA2_Stream7_IRQn
#define DMA_Camera_IT_TCIF         		DMA_IT_TCIF7
#define DMA_Camera_STREAM_IRQHANDLER    DMA2_Stream7_IRQHandler
 */
void ov7670_DMA_Config() {
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	__IO uint32_t Timeout = HSE_STARTUP_TIMEOUT;

	// DMA2 configuration==========================
	//Enable DMA controller Clock
	RCC_AHB1PeriphClockCmd(DMA_Camera_STREAM_CLOCK, ENABLE); //RCC_AHB1Periph_DMA2
	DMA_DeInit(DMA_CameraToRAM_Stream);//(DMA2 stream 7) DMA2_Stream7 for DMCI
	while (DMA_GetCmdStatus(DMA_CameraToRAM_Stream) != DISABLE) {}

	//DMA_StructInit(&DMA_InitStructure); //was added
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(DCMI_BASE + 0x28);//(&DCMI->DR);//was DCMI_DR_ADDRESS; //(uint32_t)(&DCMI->DR);??? //(uint32_t)(DCMI_BASE + 0x28); //Source Peipheral Address
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) RAM_Buffer; //Dest Address (To Be LCD BaseAddress Directly)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = picture_x * picture_y / 2; //was picture_x * picture_y * 2 / 4; //To Be 1
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; //32bit?
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//=16bit. was Word;//was _HalfWord???; //16bit? ,-- 8?
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //Transfer Mode = {circular, normal}
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //0 is the highest
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;//was DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA_CameraToRAM_Stream, &DMA_InitStructure);

	//Mask Interrupt for DMA
	//DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE); //Transfer complete
	DMA_ITConfig(DMA_CameraToRAM_Stream, DMA_IT_TE, ENABLE); //Transfer error
	//DMA_ITConfig(DMA_CameraToRAM_Stream, DMA_IT_HT, ENABLE); //Half transfer complete interrupt mask
	DMA_ITConfig(DMA_CameraToRAM_Stream, DMA_IT_FE, ENABLE); //FIFO error interrupt mask

	//DMA2 IRQ Channel Configuration.
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//was 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//was 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); //install isr

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);


	//Enable
	// Enable DMA2 stream 1 and DCMI interface then start image capture
	DMA_Cmd(DMA2_Stream7, ENABLE);

	//Check the DMA Stream is effectively enabled.
	Timeout = HSE_STARTUP_TIMEOUT;
	while ((DMA_GetCmdStatus(DMA2_Stream7) != ENABLE)	&& (Timeout-- > 0)) {}

	// Check if a timeout condition occurred
	if (Timeout == 0) {// Manage the error: to simplify the code enter an infinite loop
			while (1) {		}
	}
	// Insert 100ms delay: wait 100ms
     delayms(100);
}

/*
 * You can use interrupts of both DCMI and DMA for get to know whether the frame has been received or not or if anything is wrong in the settings.
 * There are many interrupts in DCMI and DMA module.

In DMA we can see following interrupts
  *   DMA_IT_TCIFx:Streamx transfer complete interrupt
  *   DMA_IT_HTIFx:  Streamx half transfer complete interrupt
  *   DMA_IT_TEIFx:  Streamx transfer error interrupt
  *   DMA_IT_DMEIFx: Streamx direct mode error interrupt
  *   DMA_IT_FEIFx:  Streamx FIFO error interrupt

Most useful interrupt is " transfer complete interrupt".
Using this ,we can identify a frame has been successfully transferred by DMA to the memory.

Following is the code for interrupt.
 */
//We found no the following interrupt.
void DMA2_Stream7_IRQHandler(void)
{
	//Test on DMA2 Channel7 Transfer Complete interrupt
	//(It notifies The DMA transferring to memory buffer has been done.)
/*	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) ==  SET)  {
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TCIF7);
		gframe_flag = 1;//when gframe_flag =1,all the data will be send to UART  in main function while loop
				                // To be support DMA to LCD directly
		printf("TC\r\n");
	}
*/
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TEIF7) ==  SET)  {
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TEIF7);
		printf("TE\r\n");
	}
	if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_FEIF7) ==  SET)  {
		DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_FEIF7);
		printf("FE\r\n");
	}

}

//We should use this DCMI Interrupt.
//[TBD: We should clarify]
void DCMI_IRQHandler(void)
{

	//static volatile int line,col,i,j = 0;
/*	if (DCMI_GetITStatus(DCMI_IT_FRAME)) {
		gframe_flag = 1;
		//__disable_irq();
		//for (i = 0; i < picture_x * picture_y; i++)
		//	usartSendWord(RAM_Buffer[i + 2]);
		//__enable_irq();
		DCMI_Cmd(DISABLE);
		DCMI_CaptureCmd(DISABLE);
		//DMA_Cmd(DMA_CameraToRAM_Stream, DISABLE);
		//num_dcmi_frame++;
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
		//printf("FI\r\n");// lines: %d\n\r",num_dcmi_line);
	}
*/
	if (DCMI_GetFlagStatus(DCMI_FLAG_VSYNCRI) == SET) {
		//num_dcmi_vsync++;
		DCMI_ClearFlag(DCMI_FLAG_VSYNCRI);
		//printf("VI\r\n");
		//printf(" lines: %d\n\r",num_dcmi_line);
//		printf(" num_dcmi: %d\n\r",num_dcmi);
//		printf(" num_dcmi_frame: %d\n\r",num_dcmi_frame);
//		printf(" num_dcmi_vsync: %d\n\r",num_dcmi_vsync);
//		printf(" num_dcmi_line: %d\n\r",num_dcmi_line);
//		printf(" =========================\n\r");
		//num_dcmi = 0;
		//num_dcmi_frame = 0;
		//num_dcmi_vsync = 0;
		//num_dcmi_line = 0;
	}
	if (DCMI_GetFlagStatus(DCMI_IT_LINE) == SET) {
		//num_dcmi_line++;
		DCMI_ClearFlag(DCMI_IT_LINE);
		//printf("LI\r\n");
	}

    if(DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET){
    	printf("FRMI\r\n");
    	gframe_flag = 1; //if we enable this, gframe_flag will be on 60 times per sec.
    	//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
    	//sprintf(&tempBuff[0],"Frame got\r\n");
    	//send(&tempBuff[0]);//LCD Display..
    	//delayx(1000); //60usec
		//GPIO_ResetBits(GPIOB, GPIO_Pin_15); // LED1 OFF
    	DCMI_ClearFlag(DCMI_FLAG_FRAMERI); //After all done, we clear Frame capture complete Raw flag mask.

    	//YOON - for Snapshot application.
    	//DMA_Cmd(DMA2_Stream7, DISABLE); //FOR SNAPSHOT -- YOON
    	//DCMI_Cmd(DISABLE);
    	DCMI_CaptureCmd(DISABLE);
    }
    if(DCMI_GetFlagStatus(DCMI_FLAG_OVFRI) == SET)    {
    	printf("overflow\r\n");
    	//send(&tempBuff[0]);
    	DCMI_ClearFlag(DCMI_FLAG_OVFRI);
    }
    if(DCMI_GetFlagStatus(DCMI_FLAG_ERRRI) == SET)    {
    	printf("ERROR\r\n");
    	//send(&tempBuff[0]);
    	DCMI_ClearFlag(DCMI_FLAG_ERRRI);
    }

}
void show_image(){
	int count_buffer=0;
	uint16_t color_dot;
	int xx, yy;

	printf("[%d]",g_fcount);
	for (xx=40; xx<60; xx++){
		printf("%02x ",RAM_Buffer[xx]);
	}
	printf("\r\n");
	/*
	for (yy=40; yy<picture_y+40; yy++){
		for (xx=70; xx<picture_x+70; xx++){
			color_dot = (RAM_Buffer[count_buffer]<<8) | RAM_Buffer[count_buffer+1];
			Draw_Pixel(yy, xx, color_dot);
			count_buffer+=2;
		}
	}
	*/
}

/*
 * If you have declare the DMA buffer size well, both the DMA transfer complete interrupt and DCMI frame received flag should occur same time(small delay).
I suggest you to first read a snap shot by configuring as,"DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;"
Once you get snap shot, then move to Continuous mode.
Following is the code in main function, Once transfer complete interrupt occur in DMA module, following code will send data through serial port.
 */
static uint32_t speed = 10;
extern void stmUser_LED_GPIO_setup(void);
extern void yI2C1_Init(u32 I2Cspeed);
int ov7670Loop()
{
	int x=0;
	speed = 36;
	gframe_flag = 0;

	stmUser_LED_GPIO_setup();
	GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
	delayms(1000);
	GPIO_ResetBits(GPIOB, GPIO_Pin_15); // LED1 OFF

	//Generate 24MHz Clock from MCO2.
	MCO2_Config_24MHz();

	ov7670_InitSCCB();//100Kbps //yI2C1_Init(100000);//100Kbps
	ov7670_DCMI_Config(); //DCMI_CaptureMode_Continuous;
	ov7670_DMA_Config();
	DCMI_Cmd(ENABLE);	//Enable DCMI Interface
	//delayms(100);// Insert 100ms delay: wait 100ms
	DCMI_CaptureCmd(ENABLE);	//Start Capture.
	//delayms(100);// Insert 100ms delay: wait 100ms

	set_reg_ov7670(); //ov7670_init();

	//set_pos(50);
	//Delay(1000);
	//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
	//delayx(10000);// Insert 100ms delay: wait 100ms
	//GPIO_ResetBits(GPIOB, GPIO_Pin_15); // LED1 OFF
	//gframe_flag = 0;

	while (1)
	{
	  if(gframe_flag) {
	    show_image();
	    //printf("%d\r\n",g_fcount);
	    g_fcount++;
	    gframe_flag = 0;
	    delayms(1000);
    	//DMA_Cmd(DMA2_Stream7, ENABLE); //FOR SNAPSHOT -- YOON
    	//DCMI_Cmd(ENABLE);
    	DCMI_CaptureCmd(ENABLE);

		//GPIO_SetBits(GPIOB, GPIO_Pin_15); // LED1 ON
		//delayx(1000);
		//GPIO_ResetBits(GPIOB, GPIO_Pin_15); // LED1 OFF


	    //delayx(1000);
	  }
	  x++;
	}
}

/*
	void show_image(void)
	  {
	   for( K =0;K< WIDTH*HEIGHT*BYTES_PER_PIXEL/4;K++)
	   {
	    {
	    sprintf(&tempBuff[0],"%d \n",frame_buffer[4*K+1]);send_no_new_line(&tempBuff[0]);
	    sprintf(&tempBuff[0],"%d \n",frame_buffer[4*K+3]);send_no_new_line(&tempBuff[0]);
	    }
	    if ((K+1)%40 == 0 )
	    {
	     sprintf(&tempBuff[0],";\r\n");send(&tempBuff[0]);
	    }
	   }
	  }

}


int num_dcmi = 0;
int num_dcmi_frame = 0;
int num_dcmi_vsync = 0;
int num_dcmi_line = 0;

void DCMI_IRQHandler(void) {
	num_dcmi++;

	uint16_t i;
	if (DCMI_GetITStatus(DCMI_IT_FRAME)) {
		__disable_irq();
		for (i = 0; i < picture_x * picture_y; i++)
			usartSendWord(RAM_Buffer[i + 2]);
		__enable_irq();
		DCMI_Cmd(DISABLE);
		DCMI_CaptureCmd(DISABLE);
		DMA_Cmd(DMA_CameraToRAM_Stream, DISABLE);
		num_dcmi_frame++;
		DCMI_ClearITPendingBit(DCMI_IT_FRAME);
		//printf(" lines: %d\n\r",num_dcmi_line);
	} else if (DCMI_GetFlagStatus(DCMI_FLAG_VSYNCRI) == SET) {
		num_dcmi_vsync++;
		DCMI_ClearFlag(DCMI_FLAG_VSYNCRI);
		//printf(" lines: %d\n\r",num_dcmi_line);
//		printf(" num_dcmi: %d\n\r",num_dcmi);
//		printf(" num_dcmi_frame: %d\n\r",num_dcmi_frame);
//		printf(" num_dcmi_vsync: %d\n\r",num_dcmi_vsync);
//		printf(" num_dcmi_line: %d\n\r",num_dcmi_line);
//		printf(" =========================\n\r");
		num_dcmi = 0;
		num_dcmi_frame = 0;
		num_dcmi_vsync = 0;
		num_dcmi_line = 0;
	} else if (DCMI_GetFlagStatus(DCMI_IT_LINE) == SET) {
		num_dcmi_line++;
		DCMI_ClearFlag(DCMI_IT_LINE);
	}
}
*/
#endif
