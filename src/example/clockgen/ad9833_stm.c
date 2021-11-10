/*
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h" //Reset and Clock Control
#include "misc.h"
*/
#include <string.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "cmdline.h"
#include "yInc.h"
#include "ySpiDrv.h"
//-- STM32 --
/** @brief  Configures the SPI3 Peripheral.
 *  CLK - PC10
 *  MISO - PC11 -- NOT USED
 *  MOSI - PC12
 *  nCS0 - PA15
 *  nCS1 - PD2(Old) or PB7(Kong)
 *  nCS2 - PD1
*/
//use nCS0 of PA15
//use nCS0 of PD10
#define nCS0_9833_1      nCS0_H//{GPIO_SetBits(GPIOD, GPIO_Pin_10);}
#define nCS0_9833_0      nCS0_L//{GPIO_ResetBits(GPIOD, GPIO_Pin_10);}

//use nCS1 of PB7
#define nCS1_9833_1      {GPIO_SetBits(GPIOB, GPIO_Pin_7);}
#define nCS1_9833_0      {GPIO_ResetBits(GPIOB, GPIO_Pin_7);}

#define nCS9833_1 nCS0_9833_1
#define nCS9833_0 nCS0_9833_0

#define SINE 0x2000  //0010-0000-0000-0000 :
#define SQUARE 0x2028
#define HALF_SQUARE 0x2020 //NEW
#define TRIANGULAR 0x2002

#define REF_FREQ 25000000

//9833 REG
#define AD9833_CTRLB28 (0x2000)  //bit 13
#define AD9833_CTRLHLB (0x1000)  //bit 12
#define AD9833_CTRLFSEL (0x0800) //bit 11
#define AD9833_CTRLPSEL (0x0400) //bit 10
#define AD9833_CTRLRESET (0x0100)//bit 8
#define AD9833_CTRLSLEEP1 (0x0080)//bit 7
#define AD9833_CTRLSLEEP12 (0x0040)//bit 6
#define AD9833_CTRLOPBITEN (0x0020) //bit 5
#define AD9833_CTRLDIV2 (0x0008) //bit 3
#define AD9833_CTRLMODE (0x0002) //bit 1

#define AD9833_SLEEPMODE_DAC_POWERDOWN 1
#define AD9833_SLEEPMODE_INTERNALCLOCK_DISABLE 2
#define AD9833_SLEEPMODE_BOTH 3

#define pow2_28 268435456LL  //2^28, =0x10-000-000

unsigned short g_ad9833cmd =0x0000;


void stmAD9833_writeShort(unsigned short vals) {
	nCS9833_0;
   	stmSpi2WrByte((vals & 0xff00)>>8);//higher 8 bits
   	stmSpi2WrByte(vals & 0xff);//lower 8 bits
   	nCS9833_1;
}

void stmAD9833_writeShort16(unsigned short vals) {
	nCS9833_0;
	somedelay(10);
	stmSpi2WrShort(vals);
	somedelay(10);
   	nCS9833_1;
}

void stmAD9833_Init(int nCS) {

	//CPOL=0 (SCK Idle HIGH between write operations
	//CPHA=1 (Data is valid on SCK falling edge.
	//Mode 1.
    stmSPI2_Config(nCS,2, 16); //use nCS0, mode2, 16 bit mode
	//AD9833_Reset
    g_ad9833cmd = AD9833_CTRLRESET;
    stmAD9833_writeShort16(g_ad9833cmd); //RESET 1: Write '1' to Bit 8 of Control Register to reset internal registers.
    delayms(10);
    g_ad9833cmd = 0x0000;
    stmAD9833_writeShort16(g_ad9833cmd); //Global RESET 0: Write '0' to Bit 8 of Control Register to exit from reset.(It also exits from SLEEP1 and SLEEP12(enable DAC)
}

void stmAD9833_SetResetBit() {
    //AD9833_Reset
	g_ad9833cmd |= AD9833_CTRLRESET;// | AD9833_CTRLSLEEP12  | AD9833_CTRLSLEEP1;//	g_ad9833cmd |= AD9833_CTRLRESET;
    stmAD9833_writeShort16(g_ad9833cmd);
}
void stmAD9833_IssueUnResetBit() {
    //AD9833_Reset
	g_ad9833cmd &= ~AD9833_CTRLRESET;
    stmAD9833_writeShort16(g_ad9833cmd);
}

void AD9833_IssueSleep(unsigned char sleepMode) {

	g_ad9833cmd =  0; //g_ad9833cmd & ~(AD9833_CTRLSLEEP1 | AD9833_CTRLSLEEP12); //zeros in Bit7 and Bit6)
	switch(sleepMode){
	case AD9833_SLEEPMODE_DAC_POWERDOWN: //1
		g_ad9833cmd = g_ad9833cmd | AD9833_CTRLSLEEP12;
		break;
	case AD9833_SLEEPMODE_INTERNALCLOCK_DISABLE:// 2
		g_ad9833cmd = g_ad9833cmd | AD9833_CTRLSLEEP1;
		break;
	case AD9833_SLEEPMODE_BOTH:// 3
		g_ad9833cmd = g_ad9833cmd | AD9833_CTRLSLEEP12  | AD9833_CTRLSLEEP1;
		break;
	}
    stmAD9833_writeShort16(g_ad9833cmd);
}

void stmAD9833_GenFreq(unsigned long freq, unsigned short waveform) {
	unsigned long FreqWord = (freq*pow2_28)/REF_FREQ;
//	FreqWord = (freq)*(pow2_28/REF_FREQ);
//	FreqWord = (freq * pow(2,28))/REF_FREQ;
	//unsigned long FreqWord = (freq * (2 << 28))/REF_FREQ;//error???
	unsigned short msb14 =(short int) ((FreqWord & 0x0fffc000) >> 14);
	unsigned short lsb14 = (short int)(FreqWord & 0x3fff);
	unsigned short MSB14, LSB14;

	printf("AD9833> Set Freq=%uHz (FreqWord=0x%08x) WaveForm=%u\r\n",freq, FreqWord,waveform);

	//stmAD9833_IssueUnResetBit();
	stmAD9833_SetResetBit();

	//Write to Freq Registers 0 and 1
	//for freqReg 0 (28bits) ===
	LSB14 = lsb14 | 0x4000; //bit 15/14 = {01} for freqReg 0
	MSB14 = msb14 | 0x4000; //bit 15/14 = {01} for freqReg 0

	//Wanted Freq
	//D13(B28) =1 (2 bytes of desired freq)
	//D12(HLB) : High or Low Byte of freq.
	//D11(FSELECT) : 0(=FREQ0 reg).
	//D10(PSELECT) : 0(=PHASE0 reg)
	g_ad9833cmd = 0x2000;
	stmAD9833_writeShort16(g_ad9833cmd);//Control Reg : 001X|0000|0000|0000 (It sets freq. In addition, it makes exit from Sleep State.)
	stmAD9833_writeShort16(LSB14); //lower 14 bits
	stmAD9833_writeShort16(MSB14); //higher 14 bits

	//Write to Phase Registers. for Phase0 and 1 Reg
	stmAD9833_writeShort16(0xC000); //110X|0000|0000|0000 //Reg0
	stmAD9833_writeShort16(0xE000); //111X|0000|0000|0000 //Reg1
	//for waveform (OPBITEN, Mode, DIV2)
	g_ad9833cmd |= waveform;
	stmAD9833_writeShort16(g_ad9833cmd); //Control Reg : set to waveform (SINE 0x2000). Also, it makes exit from reset/sleep.
}


///===========


#define pow2_28 268435456 // 2^28 used in frequency word calculation
#define BITS_PER_DEG   11.3777777777778  // 4096 / 360
#define RESET_CMD    0x0100   // Reset enabled (also CMD RESET)
/*   Sleep mode
 * D7  1 = internal clock is disabled
 * D6  1 = put DAC to sleep
 */
#define SLEEP_MODE    0x00C0   // Both DAC and Internal Clock
#define DISABLE_DAC    0x0040
#define  DISABLE_INT_CLK   0x0080

#define PHASE_WRITE_CMD   0xC000   // Setup for Phase write
#define PHASE1_WRITE_REG  0x2000   // Which phase register
#define FREQ0_WRITE_REG   0x4000   //
#define FREQ1_WRITE_REG   0x8000
#define PHASE1_OUTPUT_REG  0x0400   // Output is based off REG0/REG1
#define FREQ1_OUTPUT_REG  0x0800   // ditto

typedef enum { 	SINE_WAVE = 0x2000,
				TRIANGLE_WAVE = 0x2002,
				SQUARE_WAVE = 0x2028,
				HALF_SQUARE_WAVE = 0x2020 } WaveformType_t;

typedef enum { REG0, REG1, SAME_AS_REG0 } Registers_t;

struct _ad9833_state{
	  uint32_t   refFrequency;
	  uint16_t   waveForm0, waveForm1;
	  uint8_t    outputEnabled, DacDisabled, IntClkDisabled;
	  float    frequency0, frequency1, phase0, phase1;
	  Registers_t   activeFreq, activePhase;
} ad9833_state;

  // SleepMode, DisableDAC, or DisableInternalClock remain in effect
  void stmAd9833_ApplySignal ( WaveformType_t waveType, Registers_t freqReg,  float frequencyInHz,   Registers_t phaseReg, float phaseInDeg );//  Registers_t phaseReg = SAME_AS_REG0, float phaseInDeg = 0.0 );

  // Resets internal Registers_t to 0, which corresponds to an output of
  // midscale - digital output at 0. See EnableOutput function
  void Reset ( void );

  // Update just the frequency in REG0 or REG1
  void stmAd9833_SetFrequency ( Registers_t freqReg, float frequency );

  // Increment the selected frequency register by freqIncHz
  void IncrementFrequency ( Registers_t freqReg, float freqIncHz );

  // Update just the phase in REG0 or REG1
  void stmAd9833_SetPhase ( Registers_t phaseReg, float phaseInDeg );

  // Increment the selected phase register by phaseIncDeg
  void IncrementPhase ( Registers_t phaseReg, float phaseIncDeg );

  // Set the output waveform for the selected frequency register
  // SINE_WAVE, TRIANGLE_WAVE, SQUARE_WAVE, HALF_SQUARE_WAVE,
  void SetWaveform ( Registers_t waveFormReg, WaveformType_t waveType );

  // Output based on the contents of REG0 or REG1
  void SetOutputSource ( Registers_t freqReg, Registers_t phaseReg);// = SAME_AS_REG0 );

  // Turn ON / OFF output using the RESET command.
  void EnableOutput ( bool enable );

  // Enable/disable Sleep mode.  Internal clock and DAC disabled
  void SleepMode ( bool enable );

  // Enable / Disable DAC
  void DisableDAC ( bool enable );

  // Enable / Disable Internal Clock
  void DisableInternalClock ( bool enable );

  // Return actual frequency programmed in register
  float stmAd9833_GetActualProgrammedFrequency ( Registers_t reg );

  // Return actual phase programmed in register
  float GetActualProgrammedPhase ( Registers_t reg );

  // Return frequency resolution
  float GetResolution ( void );

void stmAd9833_WriteRegister ( int16_t dat ) {
	stmAD9833_writeShort16(dat); //MSB First
}

//Write control register. Setup register based on defined states
void stmAd9833_WriteControlRegister ( void ) {
    uint16_t waveForm;
    // TODO: can speed things up by keeping a writeReg0 and writeReg1
    // that presets all bits during the various setup function calls
    // rather than setting flags. Then we could just call WriteRegister
    // directly.
    if ( ad9833_state.activeFreq == REG0 ) {
     waveForm = ad9833_state.waveForm0;
     waveForm &= ~FREQ1_OUTPUT_REG;
    } else {
     waveForm = ad9833_state.waveForm1;
     waveForm |= FREQ1_OUTPUT_REG;
    }
    if ( ad9833_state.activePhase == REG0 )
    	waveForm &= ~PHASE1_OUTPUT_REG;
    else
    	waveForm |= PHASE1_OUTPUT_REG;

    if ( ad9833_state.outputEnabled )
    	waveForm &= ~RESET_CMD;
    else
    	waveForm |= RESET_CMD;
    if ( ad9833_state.DacDisabled )
    	waveForm |= DISABLE_DAC;
    else
    	waveForm &= ~DISABLE_DAC;
    if ( ad9833_state.IntClkDisabled )
    	waveForm |= DISABLE_INT_CLK;
    else
    	waveForm &= ~DISABLE_INT_CLK;

    stmAd9833_WriteRegister ( waveForm );
}

//Reset();  // Hold in RESET until first WriteRegister command


/*
 * Setup and apply a signal. phaseInDeg defaults to 0.0 if not supplied.
 * phaseReg defaults to value of freqReg if not supplied.
 * Note that any previous calls to EnableOut,
 * SleepMode, DisableDAC, or DisableInternalClock remain in effect.
 */
void stmAd9833_ApplySignal (
		WaveformType_t waveType,
		Registers_t freqReg,
		float frequencyInHz,
		Registers_t phaseReg,
		float phaseInDeg )
{
  stmAd9833_SetFrequency ( freqReg, frequencyInHz );
  stmAd9833_SetPhase ( phaseReg, phaseInDeg );
  stmAd9833_SetWaveform ( freqReg, waveType );
  stmAd9833_SetOutputSource ( freqReg, phaseReg );
}

/***********************************************************************
       Control Register
------------------------------------------------------------------------
D15,D14(MSB)  10 = FREQ1 write, 01 = FREQ0 write,
      11 = PHASE write, 00 = control write
D13  If D15,D14 = 00, 0 = individual LSB and MSB FREQ write,
        1 = both LSB and MSB FREQ writes consecutively
  If D15,D14 = 11, 0 = PHASE0 write, 1 = PHASE1 write
D12  0 = writing LSB independently
   1 = writing MSB independently
D11  0 = output FREQ0,
  1 = output FREQ1
D10  0 = output PHASE0
  1 = output PHASE1
D9  Reserved. Must be 0.
D8  0 = RESET disabled
  1 = RESET enabled
D7  0 = internal clock is enabled
  1 = internal clock is disabled
D6  0 = onboard DAC is active for sine and triangle wave output,
   1 = put DAC to sleep for square wave output
D5  0 = output depends on D1
  1 = output is a square wave
D4  Reserved. Must be 0.
D3  0 = square wave of half frequency output
  1 = square wave output
D2  Reserved. Must be 0.
D1  If D5 = 1, D1 = 0.
  Otherwise 0 = sine output, 1 = triangle output
D0  Reserved. Must be 0.
***********************************************************************/

/*
 * Hold the AD9833 in RESET state.
 * Resets internal Registers_t to 0, which corresponds to an output of
 * midscale - digital output at 0.
 *
 * The difference between Reset() and EnableOutput(false) is that
 * EnableOutput(false) keeps the AD9833 in the RESET state until you
 * specifically remove the RESET state using EnableOutput(true).
 * With a call to Reset(), ANY subsequent call to ANY function (other
 * than Reset itself and Set/IncrementPhase) will also remove the RESET
 * state.
 */
void stmAd9833_Reset ( void ) {
	stmAd9833_WriteRegister(RESET_CMD);
  delayms(15);
}

/*
 *  Set the specified frequency register with the frequency (in Hz)
 */
void stmAd9833_SetFrequency ( Registers_t freqReg, float frequency ) {
  // TODO: calculate max frequency based on refFrequency.
  // Use the calculations for sanity checks on numbers.
  // Sanity check on frequency: Square - refFrequency / 2
  //          Sine/Triangle - refFrequency / 4
  if ( frequency > 12500000.0 ) frequency = 12500000.0;
  if ( frequency < 0.0 ) frequency = 0.0;

  // Save frequency for use by IncrementFrequency function
  if ( freqReg == REG0 ) ad9833_state.frequency0 = frequency;
  else ad9833_state.frequency1 = frequency;

  int32_t freqWord = (frequency * pow2_28) / (float)ad9833_state.refFrequency;
  int16_t upper14 = (int16_t)((freqWord & 0xFFFC000) >> 14);
  int16_t lower14 = (int16_t)(freqWord & 0x3FFF);

  // Which frequency register are we updating?
  uint16_t reg = freqReg == REG0 ? FREQ0_WRITE_REG : FREQ1_WRITE_REG;
  lower14 |= reg;
  upper14 |= reg;

  // I do not reset the Registers_t during write. It seems to remove
  // 'glitching' on the outputs.
  stmAd9833_WriteControlRegister();
  // Control register has already been setup to accept two frequency
  // writes, one for each 14 bit part of the 28 bit frequency word
  stmAd9833_WriteRegister(lower14);    // Write lower 14 bits to AD9833
  stmAd9833_WriteRegister(upper14);    // Write upper 14 bits to AD9833
}

/*
 * Increment the specified frequency register with the frequency (in Hz)
 */
void stmAd9833_IncrementFrequency ( Registers_t freqReg, float freqIncHz ) {
  // Add/subtract a value from the current frequency programmed in
  // freqReg by the amount given
  float frequency = (freqReg == REG0) ? ad9833_state.frequency0 : ad9833_state.frequency1;
  stmAd9833_SetFrequency(freqReg,frequency+freqIncHz);
}

/*
 * Set the specified phase register with the phase (in degrees)
 * The output signal will be phase shifted by 2\'a5\'f0/4096 x PHASEREG
 */
void stmAd9833_SetPhase ( Registers_t phaseReg, float phaseInDeg ) {
  // Sanity checks on input
  phaseInDeg = fmod(phaseInDeg,360);
  if ( phaseInDeg < 0 ) phaseInDeg += 360;

  // Phase is in float degrees ( 0.0 - 360.0 )
  // Convert to a number 0 to 4096 where 4096 = 0 by masking
  uint16_t phaseVal = (uint16_t)(BITS_PER_DEG * phaseInDeg) & 0x0FFF;
  phaseVal |= PHASE_WRITE_CMD;

  // Save phase for use by IncrementPhase function
  if ( phaseReg == REG0 ) {
	  ad9833_state.phase0 = phaseInDeg;
  }else {
	  ad9833_state.phase1 = phaseInDeg;
	  phaseVal |= PHASE1_WRITE_REG;
  }
  stmAd9833_WriteRegister(phaseVal);
}

/*
 * Increment the specified phase register by the phase (in degrees)
 */
void stmAd9833_IncrementPhase ( Registers_t phaseReg, float phaseIncDeg ) {
  // Add/subtract a value from the current phase programmed in
  // phaseReg by the amount given
  float phase = (phaseReg == REG0) ? ad9833_state.phase0 : ad9833_state.phase1;
  stmAd9833_SetPhase(phaseReg,phase + phaseIncDeg);
}

/*
 * Set the type of waveform that is output for a frequency register
 * SINE_WAVE, TRIANGLE_WAVE, SQUARE_WAVE, HALF_SQUARE_WAVE
 */
void stmAd9833_SetWaveform (  Registers_t waveFormReg, WaveformType_t waveType ) {
  // TODO: Add more error checking?
  if ( waveFormReg == REG0 )
	  ad9833_state.waveForm0 = waveType;
  else
	  ad9833_state.waveForm1 = waveType;
  stmAd9833_WriteControlRegister();
}

/*
 * EnableOutput(false) keeps the AD9833 is RESET state until a call to
 * EnableOutput(true). See the Reset function description.
 */
void stmAd9833_EnableOutput ( bool enable ) {
  ad9833_state.outputEnabled = enable;
  stmAd9833_WriteControlRegister();
}

/*
 * Set which frequency and phase register is being used to output the
 * waveform. If phaseReg is not supplied, it defaults to the same
 * register as freqReg.
 */
void stmAd9833_SetOutputSource ( Registers_t freqReg, Registers_t phaseReg ) {
  // TODO: Add more error checking?
	ad9833_state.activeFreq = freqReg;
  if ( phaseReg == SAME_AS_REG0 )  ad9833_state.activePhase = ad9833_state.activeFreq;
  else ad9833_state.activePhase = phaseReg;
  stmAd9833_WriteControlRegister();
}

//---------- LOWER LEVEL FUNCTIONS NOT NORMALLY NEEDED -------------

/*
 * Disable/enable both the internal clock and the DAC. Note that square
 * wave outputs are avaiable if using an external Reference.
 * TODO: ?? IS THIS TRUE ??
 */
void stmAd9833_SleepMode ( bool enable ) {
	ad9833_state.DacDisabled = enable;
	ad9833_state.IntClkDisabled = enable;
  stmAd9833_WriteControlRegister();
}

/*
 * Enables / disables the DAC. It will override any previous DAC
 * setting by Waveform type, or via the SleepMode function
 */
void stmAd9833_DisableDAC ( bool enable ) {
	ad9833_state.DacDisabled = enable;
  stmAd9833_WriteControlRegister();
}

/*
 * Enables / disables the internal clock. It will override any
 * previous clock setting by the SleepMode function
 */
void stmAd9833_DisableInternalClock ( bool enable ) {
	ad9833_state.IntClkDisabled = enable;
  stmAd9833_WriteControlRegister();
}

// ------------ STATUS / INFORMATION FUNCTIONS -------------------
/*
 * Return actual frequency programmed
 */
float stmAd9833_stmAd9833_GetActualProgrammedFrequency ( Registers_t reg ) {
  float frequency = reg == REG0 ? ad9833_state.frequency0 : ad9833_state.frequency1;
  int32_t freqWord = (uint32_t)((frequency * pow2_28) / (float)ad9833_state.refFrequency) & 0x0FFFFFFFUL;
  return (float)freqWord * (float)ad9833_state.refFrequency / (float)pow2_28;
}

/*
 * Return actual phase programmed
 */
float stmAd9833_GetActualProgrammedPhase ( Registers_t reg ) {
  float phase = reg == REG0 ? ad9833_state.phase0 : ad9833_state.phase1;
  uint16_t phaseVal = (uint16_t)(BITS_PER_DEG * phase) & 0x0FFF;
  return (float)phaseVal / BITS_PER_DEG;
}

/*
 * Return frequency resolution
 */
float stmAd9833_GetResolution ( void ) {
  return (float)ad9833_state.refFrequency / (float)pow2_28;
}

stmAD9833_Loop ( uint8_t FNCpin, uint32_t referenceFrequency ) {
  // Pin used to enable SPI communication (active LOW)
  /* TODO: The minimum resolution and max frequency are determined by
   * by referenceFrequency. We should calculate these values and use
   * them during stmAd9833_SetFrequency. The problem is if the user programs a
   * square wave at refFrequency/2, then changes the waveform to sine.
   * The sine wave will not have enough points?
   */
}


//TEST
#define RUNNING       F("\\tRUNNING")
#define NOT_RUNNING   F("")
#define ON            F("ON")
#define OFF           F("OFF")
#define LED_PIN       13      // I'm alive blinker
#define FNC_PIN       4       // Any digital pin. Used to enable SPI transfers (active LO

/*
 * We need to manually call serialEventRun since we're not returning through the loop()
 * function while inside the test functions. If a character is in the receive buffer,
 * exit the test function. We also blink the I'm Alive LED to give a visual indication
 * that the program is not hung up.
 */
/*
#define YIELD_ON_CHAR     if ( serialEventRun ) serialEventRun(); \\
                          if ( Serial.available() ) return; \\
                          BLINK_LED

#define DELAY_WITH_YIELD  for ( uint8_t i = 0; i < 10; i++ ) { \\
                              YIELD_ON_CHAR \\
                              delayms(100);   \\
                          }

#define FLUSH_SERIAL_INPUT  if ( serialEventRun ) serialEventRun(); \\
                            do { Serial.read(); delayms(100); } while ( Serial.available() > 0 );


void loop() {
    static bool outputOn = false;

    // Setup some defaults
    ad9833.refFrequency = 25000000; //referenceFrequency;
    ad9833_state.DacDisabled = false;
    ad9833_state.IntClkDisabled = false;
    ad9833_state.outputEnabled = false;
    ad9833_state.waveForm0 = waveForm1 = SINE_WAVE;
    ad9833_state.frequency0 = ad9833_state.frequency1 = 1000;   // 1 KHz sine wave to start
    ad9833_state.phase0 = ad9833_state.phase1 = 0.0;     // 0 phase
    ad9833_state.activeFreq = REG0;
    ad9833_state.activePhase = REG0;
    stmAd9833_EnableOutput(false);  // Turn ON the output

        PrintMenu(0,true);        // Display menu for the first time


    if ( Serial.available() ) {
        char ch = Serial.read();

        FLUSH_SERIAL_INPUT

        PrintMenu(ch,outputOn);

        switch ( ch ) {
            case '1':
                IncrementFrequencyTest();
                break;
            case '2':
                CycleWaveformsTest();
                break;
            case '3':
                SwitchFrequencyRegisterTest();
                break;
            case '4':
                PhaseTest();
                break;
            case '5':
                RequestedvsProgrammedValues();
                break;
            case '6':
                outputOn = ! outputOn;
                EnableOutput(outputOn);    // Turn off output
                break;
            default:
                printf(F("*** Invalid command ***"));
                break;
        }
    }
}
*/
/*
 * Setup a manual ramp from a Start frequency to a Stop frequency in some increment
 * over a ramp time.
 */
void IncrementFrequencyTest ( void ) {

	float i;
    float startHz = 1000, stopHz = 5000, incHz = 1, sweepTimeSec = 5.0;

    // Calculate the delay between each increment.
    uint16_t numMsecPerStep = (sweepTimeSec * 1000.0) / ((uint16_t)((stopHz - startHz) / incHz) + 1);
    if ( numMsecPerStep == 0 ) numMsecPerStep = 1;

    // Apply a signal to the output. If phaseReg is not supplied, then
    // a phase of 0.0 is applied to the same register as freqReg
    stmAd9833_ApplySignal(SINE_WAVE,REG1,startHz,0,0);

    while ( true ) {

        stmAd9833_SetFrequency(REG1,startHz-incHz);

        for ( i = startHz ; i <= stopHz; i += incHz ) {
           // YIELD_ON_CHAR
            stm9833_IncrementFrequency(REG1,incHz);
            delayms(numMsecPerStep);
        }
    }
}

/*
 * Cycle through all of the waveform types. Also cycle the
 * frequency Registers_t.
 */
void CycleWaveformsTest ( void )
{
    WaveformType_t waveType = SINE_WAVE;
    stmAd9833_SetFrequency(REG0,10000.0);   // Load values
    stmAd9833_SetFrequency(REG1,1000.0);
    // We don't care about phase for this test

    while ( true ) {

    	stmAd9833_SetWaveform(REG1,waveType);   // Next waveform
    	stmAd9833_SetWaveform(REG0,waveType);
    	stmAd9833_SetOutputSource(REG1,0);        // Output 1000 Hz waveform

        // Hack to allow I'm alive lamp a chance to blink and give a better
        // response to user input
        //DELAY_WITH_YIELD

    	stmAd9833_SetOutputSource(REG0,0);        // Output 10000 Hz waveform

        //DELAY_WITH_YIELD

        switch ( waveType ) {             // Cycle through all the waveform types
            case SINE_WAVE:
                waveType = TRIANGLE_WAVE;
                break;
            case TRIANGLE_WAVE:
                waveType = SQUARE_WAVE;
                break;
            case SQUARE_WAVE:
                waveType = HALF_SQUARE_WAVE;
                break;
            case HALF_SQUARE_WAVE:
                waveType = SINE_WAVE;
                break;
        }
    }
}

/*
 * Fast switching example.
 * I use the FFT display capability on my scope
 */
void SwitchFrequencyRegisterTest ( void )
{
    stmAd9833_ApplySignal(SINE_WAVE,REG0,500000,0,0);
    stmAd9833_ApplySignal(SINE_WAVE,REG1,100000,0,0);
    stmAd9833_SetPhase(REG1,180);           // Offset second freq by 180 deg
    stmAd9833_Reset();

    while ( true ) {                  // This takes time

        //YIELD_ON_CHAR                 // This takes more time

    	stmAd9833_SetOutputSource(REG0,REG0);    // This takes about 18 usec
    	stmAd9833_SetOutputSource(REG1,REG1);    // This takes about 18 usec

        // What ends up is REG0 frequency is active a shorter amount of time
        // then REG1 frequency. In the sepctrum, the duty cycle differences will
        // show up (power is lower by 10log(DC))
    }
}

/*
 * Phase shift between REG0 and REG1. Use a oscilloscope set to Normal
 * triggering, AC coupling, 500usec/div, 100 mV/div. This will display
 * about two cycles for register 0, 4 cycle for register 1, plus dead
 * time for the Reset.
 * Use Normal triggering so the display remains even when triggering is
 * lost. Can use any waveform for this test. Remember that the square
 * wave is about 5v-pp while sine and triangle are about 600 mv-pp
 */
void stmAd9833_PhaseTest ( void )
{
	int16_t i;
    stmAd9833_ApplySignal(TRIANGLE_WAVE,REG0,1000,0,0);
    stmAd9833_ApplySignal(SINE_WAVE,REG1,2000,0,0);

    bool reverse = true;

    while ( true ) {
        reverse = ! reverse;

        for ( i = 0; i <= 360; i += 1 ) {
            if ( ! reverse )
            	stmAd9833_IncrementPhase(REG1,-1);
            else
            	stmAd9833_IncrementPhase(REG1,1);

           // YIELD_ON_CHAR
            /*
             * Display ~ 2 cycles using REG0 phase. If no REG is supplied for phase,
             * defaults to REG specified for frequency. RESET is removed during this
             * function call.
             */
            stmAd9833_SetOutputSource(REG0,0);
            /*
             * This is just a wag to try to get exactly 2 cycles of the waveform.
             * It makes the phase alignments easier to verify.
             */
            //delayMicroseconds(1900);

            //YIELD_ON_CHAR

            /* This also works if you keep using REG1 for frequency
             * Now display ~ 4 cycles using REG1
             */
            stmAd9833_SetOutputSource(REG1,0);
            //delayMicroseconds(1950);
            /*
             * Turn off for remaining trace. Reset the Registers_t so triggering occurs
             * on the start of REG0 signal. Reset() includes 15 msec delay which is good
             * to ensure sweep is completed.
             * I tried using EnableOutput(true) then EnableOutput(false) in this
             * loop but could not get reliable triggering on the scope.
             *
             * The difference between Reset() and EnableOutput(false) is that EnableOutput(false)
             * keeps the AD9833 in RESET until you specifically remove the RESET using
             * EnableOutput(true). However, after a call to Reset(), calls to ANY function
             * EXCEPT Set/Increment Phase will also remove the RESET.
             *
             */
            stmAd9833_Reset();

            if ( i % 90 == 0  )
                delayms(1000);    // Stop and show phase alignment between REG0 REG1
        }
    }
}

/*
 * Show the requested versus actual programmed values for frequency and phase
 * Also show resolution, max frequency (based on refFrequency)
 */
/*
void stmAd9833_RequestedvsProgrammedValues ( void ) {

    float requestedFrequency, programmedFrequency;
    char  buffer[20];   // 14 characters actually needed for display

    stmAd9833_ApplySignal(SINE_WAVE,REG0,1000.0,0,0);

    while ( true ) {

        //FLUSH_SERIAL_INPUT

        printf(F("\\nEnter frequency ('Q' to quit) >"));
        //while ( !Serial.available() )   BLINK_LED

        if ( toupper(Serial.peek()) == 'Q' ) {
            // Need an extra <CR> ?
            //FLUSH_SERIAL_INPUT    // why isn't this flushing input?
            return;
        }
        //requestedFrequency = Serial.parseFloat();
        //stmAd9833_SetFrequency(REG0,requestedFrequency);
       // programmedFrequency = stmAd9833_GetActualProgrammedFrequency(REG0);
        //printf(F("Requested :"));
        //dtostrf(requestedFrequency,14,5,buffer);
        //printf(buffer);
        //printf(F("   Actual :"));
        //dtostrf(programmedFrequency,14,5,buffer);
        //printf(buffer);
    }
}
*/
/*
 * Display the command menu
 */
/*
void PrintMenu ( char ch, bool outputOn ) {
    printf(); printf();
    printf(F("****** AD9833 Test Menu ******\\n"));
    printf(F("'1' IncrementFrequencyTest"));
    printf(ch == '1' ? RUNNING : NOT_RUNNING);
    printf(F("'2' CycleWaveformsTest\\t"));
    printf(ch == '2' ? RUNNING : NOT_RUNNING);
    printf(F("'3' SwitchFrequencyRegisterTest"));
    printf(ch == '3' ? RUNNING : NOT_RUNNING);
    printf(F("'4' PhaseTest\\t\\t"));
    printf(ch == '4' ? RUNNING : NOT_RUNNING);
    printf(F("'5' RequestedvsProgrammedValues"));
    printf(ch == '5' ? RUNNING : NOT_RUNNING);
    printf(F("'6' Output "));
    if ( ch == '6' ) {
        if ( outputOn ) printf(OFF);
        else            printf(ON);
    }
    else {
        if ( outputOn ) printf(ON);
        else            printf(OFF);
    }
    printf(F("Enter a number 1 to 6 >"));
}
*/

unsigned char stmAd9833_FreqGenLoop(){

    unsigned freq = 60000;
    unsigned waveForm = SINE;
    int nCS=0;

    printf("AD9833 Frequency Generator with SPI2 Mode 2, 1Mbps, 16bit data.");

    stmAD9833_Init(nCS); //SPI2 with nCS0
    delayms(10);

    //TEST SPI MODE 2
	//while(1){
    	//stmAD9833_writeShort16(0x5555);
		//delayms(1);
		//stmAD9833_writeShort16(0x5555);
		//delayms(2);
    //}


   // stmAd9833_Reset();
   // delayms(10);
/*
    // Setup some defaults
    ad9833_state.refFrequency = 25000000; //referenceFrequency;
    ad9833_state.DacDisabled = true;//false;
    ad9833_state.IntClkDisabled = false;
    ad9833_state.outputEnabled = false;
    ad9833_state.waveForm0 = ad9833_state.waveForm1 = SQUARE_WAVE;//SINE_WAVE;
    ad9833_state.frequency0 = ad9833_state.frequency1 = 1000.0;   // 1 KHz sine wave to start
    ad9833_state.phase0 = ad9833_state.phase1 = 0.0;     // 0 phase
    ad9833_state.activeFreq = REG0;
    ad9833_state.activePhase = REG0;
    stmAd9833_EnableOutput(false);  // Turn OFF the output
    delayms(10);
    //ad9833_state.outputEnabled = 1;
    stmAd9833_ApplySignal(SQUARE_WAVE,REG0,1000.0,REG0,0.0);//stmAd9833_ApplySignal(SINE_WAVE,REG1,1000,0,0);
    delayms(10);
    stmAd9833_ApplySignal(SQUARE_WAVE,REG1,1000.0,REG1,0.0);//stmAd9833_ApplySignal(SINE_WAVE,REG1,1000,0,0);
    delayms(10);
	stmAd9833_SetOutputSource(REG0,REG0);    // This takes about 18 usec
	delayms(10);
    stmAd9833_EnableOutput(true);  // Turn ON the output
    delayms(10);

    //stmAd9833_Reset();
*/


	printf("Freq=%d\r\n", freq); //TBD
	//stmAD9833_writeShort16(0);
	stmAD9833_GenFreq(freq, SINE);//SQUARE);//TRIANGULAR);//SQUARE);//SINE);
	//stmAD9833_IssueUnResetBit();
	//stmAD9833_writeShort16(0x5000);//WHY???
    while(1){
    	stmAD9833_SetResetBit();//AD9833_IssueSleep(AD9833_SLEEPMODE_DAC_POWERDOWN);//AD9833_SLEEPMODE_BOTH); //stmAD9833_SetResetBit();
    	//printf("Stop\r\n"); //TBD
    	delayms(1);
    	stmAD9833_IssueUnResetBit();
    	//printf("Go\r\n"); //TBD
    	delayms(2);
    }

    return 1; //retVal;
}
