#include <string.h>
#include <stdarg.h>
#include "example/yInc.h"
#if (PROCESSOR == PROCESSOR_STM32F407VGT6)
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include "core_cm4.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_spi.h"
#elif ((PROCESSOR == PROCESSOR_STM32F103C8T6) || (PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT)|| (PROCESSOR == PROCESSOR_GD32F130FX))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "math.h"
#include "core_cm3.h"

//#include "stm32f10x_nvic.h"
#endif
#include "misc.h"

//ADC Read Methods : 3 Methods
//a) polling : repeat start ADC converion and Stop.
//b) Interrupt : We can trigger the ADC. On conversion completion, the ADC issues the IRQ. Much overhead to CPU loading.
//c) DMA : DMA can transfer 1KB buffer.

extern void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
//void TIM3_IRQHandler(void);

#define B1_Pin GPIO_Pin_13
#define B1_GPIO_Port GPIOC
#define OUT1_Pin GPIO_Pin_2
#define OUT1_GPIO_Port GPIOC
#define OUT2_Pin GPIO_Pin_3
#define OUT2_GPIO_Port GPIOC
#define LD2_Pin GPIO_Pin_5
#define LD2_GPIO_Port GPIOA

//OM = PB1
#define MB_OM_Pin GPIO_Pin_1
#define MB_OM_GPIO_Port GPIOB
//MM = PB11
#define MB_MM_Pin GPIO_Pin_11
#define MB_MM_GPIO_Port GPIOB
//IM = PA4
#define MB_IM_Pin GPIO_Pin_4
#define MB_IM_GPIO_Port GPIOA

#define DTMFBUFsz  256               // DTMF Input Buffer
#define MULTIPLIER_FOR_NOT_USING_FLOAT (256*64) //2^14
#define BITSHIFT_FOR_NOT_USING_FLOAT (14) //2^14 or >>14
#define GOERTZEL_PWR_THRESHOLD 		1000 //was 0x50

#define ADC_SCHEME_POLLING    0
#define ADC_SCHEME_INTERRUPT  1
#define ADC_SCHEME_DMA        2
#define ADC_SCHEME ADC_SCHEME_INTERRUPT //ADC_SCHEME_DMA//ADC_SCHEME_POLLING //

#define ADC_APPL_DTMF 	0
#define ADC_APPL_MARKER 1
#define ADC_APPL ADC_APPL_MARKER//ADC_APPL_DTMF //

#if (ADC_APPL == ADC_APPL_DTMF)

// ----- Parameters and Variables for Tone Detector ----- 
// cos = 2*(cos (2*PI*(Target_Freq/8000.0))) * 256*64; //2^14
#define DTMF_697Hz   27980        // DTMF Row Frequency
#define DTMF_770Hz   26956
#define DTMF_852Hz   25701
#define DTMF_941Hz   24219
#define DTMF_1209Hz  19073        // DTMF Column Frequency
#define DTMF_1336Hz  16325
#define DTMF_1477Hz  13085
#define DTMF_1633Hz   9315
#define DTMF_1394Hz  15014        // DTMF Row Frequency    2nd harm
#define DTMF_1540Hz  11583
#define DTMF_1704Hz   7549
#define DTMF_1882Hz   3032
#define DTMF_2418Hz -10565        // DTMF Column Frequency 2nd harm
#define DTMF_2672Hz -16503
#define DTMF_2954Hz -22318
#define DTMF_3266Hz -27472

#define N  114                    // Input Data deep

/*  DTMF Digit encoding */
static char DTMFchar[16] = {
  '1', '2', '3', 'A',
  '4', '5', '6', 'B',
  '7', '8', '9', 'C',
  '*', '0', '#', 'D',
};



#elif (ADC_APPL == ADC_APPL_MARKER)
// ----- Parameters and Variables for Tone Detector -----
#if 1
#define N  114//80                    // Input Data deep
// coeff = 2*(cos (2*PI*(Target_Freq/SamplingFreq))) * 256*64; //2*...*2^14
#define OM_400Hz   31164        // Outer Marker Frequency -- For 400Hz it continues Good or Bad...
#define MM_1300Hz  17121        // Middle Marker Frequency -- GOOD
#define IM_3000Hz  -23170		 // Inner Marker Frequency -- GOOD
#else
#define N  114                    // Input Data deep
// coeff = 2*(cos (2*PI*(Target_Freq/8000.0))) * 256*64; //2*...*2^14
#define OM_400Hz   31164        // Outer Marker Frequency
#define MM_1300Hz  17121        // Middle Marker Frequency
#define IM_3000Hz  -23170		 // Inner Marker Frequency
#endif
static char *MARKERstr[4] = {
		"No...",
		"Outer(400Hz)",
		"Middle(1300Hz)",
		"Inner(3000KHz)"
};

#endif

struct DTMF_MODULE  {
  GPIO_InitTypeDef gpio;
  ADC_InitTypeDef  adc;

  //dtmf_buffer points
  unsigned int   AIindex;         // Input Data Index
  unsigned int   AIcheck;         // Index Window Trigger for DTMF check

  //decoded 4 digit results
  unsigned char  digit;           // detected digit
  unsigned char  early;           // early detected digit
  unsigned char  new;             // set to 1 when new digit detected
  unsigned char  d[4];			  // last four detected digits
  unsigned int   d_i;             // index

  unsigned short ADInput[DTMFBUFsz]; // raw ADC sample Data (12 bit ADC data)
  unsigned short ADC_ConvertedValue; // holds current ADC sample data

  short GainContolledADInput[N];          		// gain controlled Data
};

static struct DTMF_MODULE dtmfModule;   // DTMF info of one input

void DTMF_Detect (struct DTMF_MODULE *t);// check for valid DTMF tone

//Init ADC @ PA1
//-ADC channel 1 is used as analog input. (12-bit resolution)
//-Use DMA channel 1 for ADC1

//We here use the 8KHz sampling. It requires 125usec period.
//We should complete sampling and conversion per sample.
//ADC will take 3 cycles for sampling and 15 cycles for conversion.
//The ADC of our MCU uses 12MHz Clock, its cycle time is 1/12MHz =  83nsec.
//It consumes only (3+15)*83nsec = 1.5usec per sample.
//Thus we can use MCU's ADC.

#define ADC1_DR_Address ((unsigned int)0x4001244C)

void dtmf_Marker_Adc_Config()
{
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef   NVIC_InitStructure;

	printf("dtmf_Marker_Adc_Config..GPIO Setup (PA1 for ADC1)\r\n");

	//PA1 : analog input (ADC12_CH1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	dtmfModule.gpio.GPIO_Pin = GPIO_Pin_1; //set to PA1
	dtmfModule.gpio.GPIO_Mode = GPIO_Mode_AIN; //set as analog input
	GPIO_Init(GPIOA, &dtmfModule.gpio); //set PA1 to AIN for ADC1_CH1

	//ADC Config
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //clock for ADC (max 14MHz, 72/6=12MHz)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //enable ADC clock

	//DeInit before ADC config.
	ADC_DeInit(ADC1);

#if (ADC_SCHEME == ADC_SCHEME_DMA)
	printf("ADC is trigger by TRGO from TIM3, and N samples will be transferred by DMA1_CH1\r\n");
	//DMA1 Clocking
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//We will use DMA1/Channel1 to link the ADC1 peripheral
	DMA_DeInit(DMA1_Channel1);
	DMA_StructInit(&DMA_InitStruct);

	//We use DMA for transferring N halfWords(16 bits).
	//After N halfWords transferring, the DMA engine will issue Transfer Complete Interrupt.
	DMA_InitStruct.DMA_BufferSize 		= N;	//We move N HalfWord from ADC to dtmfModule.ADInput Buffer.
	DMA_InitStruct.DMA_MemoryBaseAddr 	= (unsigned)&(dtmfModule.ADInput[0]);// //(unsigned)&(dtmfModule.ADC_ConvertedValue);	//user memory address to be moved by DMA.
	DMA_InitStruct.DMA_PeripheralBaseAddr = ADC1_DR_Address;//(unsigned)&(ADC1->DR);	//0x4001244C	//Set channel 1 peripheral(ADC) address
	DMA_InitStruct.DMA_PeripheralInc  	= DMA_PeripheralInc_Disable; //No address increment during DMA
	DMA_InitStruct.DMA_MemoryInc 	  	= DMA_MemoryInc_Enable; //DMA_MemoryInc_Disable; //We advance user memory position each ADC conversion.
	DMA_InitStruct.DMA_MemoryDataSize 	= DMA_MemoryDataSize_HalfWord;//16-bit //DMA_MemoryDataSize_Word;//
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_DIR 				= DMA_DIR_PeripheralSRC; //from DMA to User
	DMA_InitStruct.DMA_Mode 		  	= DMA_Mode_Circular; //Automatically return to index 0 of user buffer after completion. //DMA_Mode_Normal;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	DMA_Cmd(DMA1_Channel1,ENABLE);

	printf("dtmf_Marker_Adc_Config\..DMA1_CH1 TransferComplete IRQ Setup\r\n");
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); //Enable DMA1 Channel Transfer Complete Interrupt.

	//Link DMA1_Channel1 to ADC1
	ADC_DMACmd(ADC1,ENABLE); //enable ADC1 DMA

	//ADC Config
	printf("ADC1 Setup\r\n");
	//configure ADC parameters (Independent(no interleaved with other ADCs), Continuous, TRGO(Trigger Output)
	dtmfModule.adc.ADC_NbrOfChannel  	= 1;
	dtmfModule.adc.ADC_Mode 			= ADC_Mode_Independent;
	dtmfModule.adc.ADC_ScanConvMode 	= DISABLE; //Disable for our single channel.//ENABLE for multiple channel;
	dtmfModule.adc.ADC_ContinuousConvMode = DISABLE; //Should be DISALBE for sampling on TRGO signal from TIM3.
	dtmfModule.adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO; //ADC is triggered by TRGO signal from TIM3.
	dtmfModule.adc.ADC_DataAlign 		= ADC_DataAlign_Right;
	ADC_Init(ADC1, &(dtmfModule.adc));

	ADC_RegularChannelConfig(
			ADC1,
			ADC_Channel_1,				//PA1 as Input
			1, //rank
			ADC_SampleTime_55Cycles5);	// Sample time equal to 55.5 cycles	(We should add the conversion time of 15 cycles).
	                                    // Total time per samples takes (55.5+15)*83nsec = 5.81usec.
			//ADC_SampleTime_7Cycles5);	// Sample time equal to 7.5 cycles	(We should add the conversion time of 15 cycles).
                                // Total time per samples takes (7.5+15)*83nsec = ...usec.

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

#else  //Interrupt or polling case
	//configure ADC parameters (Independent(no interleaved with other ADCs), Continuous, SoftwareTrigger without External Trigger
	dtmfModule.adc.ADC_NbrOfChannel  		= 1;
	dtmfModule.adc.ADC_Mode 				= ADC_Mode_Independent;
	dtmfModule.adc.ADC_ScanConvMode 		= DISABLE; //Disable for our single channel.//ENABLE for multiple channel; //ADC_SCAN_DIRECTION_BACKWARD;
	dtmfModule.adc.ADC_ContinuousConvMode 	= DISABLE;
	dtmfModule.adc.ADC_ExternalTrigConv 	= ADC_ExternalTrigConv_None;
	dtmfModule.adc.ADC_DataAlign 			= ADC_DataAlign_Right;
	ADC_Init(ADC1, &dtmfModule.adc);

	ADC_RegularChannelConfig( //for TIM3
			ADC1,
			ADC_Channel_1,
			1, //rank
			ADC_SampleTime_55Cycles5);//ADC_SampleTime_71Cycles5);// Sample time equal to 55.5 cycles	//PA1 as Input
	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
#endif
	//Calibrate ADC at the first time use.
	ADC_ResetCalibration(ADC1);	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);	while(ADC_GetCalibrationStatus(ADC1));
	printf("dtmf_Marker_Adc_Config()..Caliberate ADC Done\r\n");
}


#if (ADC_SCHEME == ADC_SCHEME_DMA)

//For generating 8KHz triggering signal to ADC using TRGO
void dtmf_MarkerBeacon_8KHzSampling_TIM3_Init(void){
	GPIO_InitTypeDef init_AF;//
	TIM_TimeBaseInitTypeDef timer_init;

	printf("dtmf_MarkerBeacon_8KHzSampling_TIM3 Setup with TRGO\r\n");

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//(2b) TIM init
   	//Freq = 36MHz/(Precaler+1)/(period+1)
   	//We need 8KHz
   	TIM_DeInit(TIM3);

   	//8KHz
   	//Freq = TIM_CLK/(TIM_PSC+1)/(TIM_ARR+1)
   	//8000 = 72MHz/(71+1)/(124+1) = 72000000/72/125 = 8KHz
    TIM_TimeBaseStructInit(&timer_init);
   	timer_init.TIM_Prescaler 	= 71;
   	timer_init.TIM_ClockDivision = 0;
   	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
   	timer_init.TIM_Period 		= 124;//(this value will be loaded in Auto-Reload Register(ARR) when the current count value reaches this value.)
   	                                  //and then the current count value will be 0, and issues TRGO
   	timer_init.TIM_RepetitionCounter = 0; //This parameter is valid only for TIM1 and TIM8
   	TIM_TimeBaseInit(TIM3, &timer_init);

   	//Trigger Output Enable --> to issue TIM3 Interrupt.
   // TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable); //synchronization between the current timer and its slaves(ADC?) through TRGO.
    //TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Disable);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);//Not use TIM_TRGOSource_Reset);
                         //TIM_TRGOSource_Reset: The UG(update generation) bit in the TIM_EGR register is used as the trigger output (TRGO).
                         //TIM_TRGOSource_Update: The update event is selected as the trigger output (TRGO).

    //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //We not use TIM3 interrupt.

   	TIM_Cmd(TIM3, ENABLE);
}

//This function handles DMA1 channel 1 global interrupt after gathering all N samples by DMA.
void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC1))
	{
		//stop ADC now
    	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    	//DMA_ClearFlag(DMA1_FLAG_TC1);

    	//Notify the buffer has been filled.
    	dtmfModule.AIindex = N;
    	dtmfModule.new = 1;
	}else
		printf("DMA??");
	DMA_ClearITPendingBit(DMA1_IT_TC1 | DMA1_IT_GL1);
}


#else
//For 8KHz Sampler using TIM_TRGOSource_Update
void dtmf_MarkerBeacon_8KHzSampling_TIM3_Init(void){

	TIM_TimeBaseInitTypeDef timer_init;
	NVIC_InitTypeDef   NVIC_InitStructure;

	printf("dtmf_MarkerBeacon_8KHzSampling_TIM3_Init\r\n");

	//(2b) TIM init
   	//Freq = 36MHz/(Precaler+1)/(period+1)
   	//We need 8KHz
   	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   	//8KHz
   	//Freq = TIM_CLK/(TIM_PSC+1)/(TIM_ARR+1)
   	//8000 = 72MHz/(71+1)/(124+1) = 72000000/72/125 = 8KHz
    TIM_TimeBaseStructInit(&timer_init);
   	timer_init.TIM_Prescaler 	= 71;
   	timer_init.TIM_ClockDivision = 0;
   	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
   	timer_init.TIM_Period 		= 124;//(this value will be loaded in Auto-Reload Register(ARR) when the current count value reaches this value.)
   	                                  //and then the current count value will be 0, and issues TRGO
   	timer_init.TIM_RepetitionCounter = 0; //Infinite.
   	TIM_TimeBaseInit(TIM3, &timer_init);

   	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Disable);

   	//For triggering ADC by this TIM3.
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);//TIM_TRGOSource_Reset);//
                         //TIM_TRGOSource_Enable:
                         //TIM_TRGOSource_Reset: The UG(update generation) bit in the TIM_EGR register is used as the trigger output (TRGO).
                         //TIM_TRGOSource_Update: The update event is selected as the trigger output (TRGO).-- All
  	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//TIM_IT_Trigger,ENABLE);

   	TIM_Cmd(TIM3, ENABLE);
}

#if 1
//Called on TIM3's TIM_IT_Update per sampling time(125usec)
//Do get a sample value.
void TIM3_IRQHandler()
{
	int err;

    if (TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) { //TIM_IT_Update,TIM_IT_Trigger
    	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);//TIM_IT_Trigger); //TIM_IT_Update,

    	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //Do sampling
    	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    	//get its ADC value
    	dtmfModule.ADC_ConvertedValue = ADC_GetConversionValue(ADC1); //get data
    	//printf("%03x,", dtmfModule.ADC_ConvertedValue);

    	//save it into ADInput buffer.
    	dtmfModule.ADInput[dtmfModule.AIindex & (DTMFBUFsz-1)] = dtmfModule.ADC_ConvertedValue;
    	//increase the buffer index.
    	dtmfModule.AIindex++;

    	//stop sampling.
    	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    }
}
#endif
#endif

//Show ADInput Values
void dtmf_dumpAdcValues(){
	unsigned char pos=0;
	unsigned char row=0;

	printf("ADC[00.N]=");
	for(pos=0;pos<N; pos++){
		if(pos % 16)
			printf("%x,", dtmfModule.ADInput[pos]);
		else
			printf("\r\n[%u] ",row++);
	}
	printf("Done\r\n");
}

//  Check Input Signal and Copy it to the GainContolledADInput Buffer
static void dtmf_GainControl (struct DTMF_MODULE *m)  {
  unsigned int  v;
  unsigned int  avg;
  unsigned int  min, max;
  unsigned int  idx;
  short *pGainContolledADInput;

  //dtmf_dumpAdcValues();

#if 1 //No gain control
  for(idx = 0;idx < N; idx++){
	  m->GainContolledADInput[idx] = dtmfModule.ADInput[idx];
  }

#else


  min = 0xFFFF;
  max = 0;

  pGainContolledADInput = &(dtmfModule.GainContolledADInput[N]); //pointer of the last one
  avg  = 0x10000L / N;  	// normalize factor for average calculation
  idx = m->AIindex;//N;//

  //get avg value
  do  {
    v =  m->ADInput[idx & (DTMFBUFsz-1)];
    if (v < min)  min = v;
    if (v > max)  max = v;
    avg += (0x10000L / N) * v;
    idx--;
    *--pGainContolledADInput = v;
  } while (pGainContolledADInput != &(dtmfModule.GainContolledADInput[0]));

  avg >>= 16;                  // average value
  min = max - min;

// calculate prior value in 'v'
  for (v = 0; v < 15 && (min & 0x8000)==0; v++)  {
    min <<= 1;
  }

  printf("min = %u, max = %u, avg=%u\r\n", min, max, avg);

  if (v < 7)  {
    v = 7 - v;
    for (pGainContolledADInput = &(dtmfModule.GainContolledADInput[0]); pGainContolledADInput != &(dtmfModule.GainContolledADInput[N]); )  {
      *pGainContolledADInput++ = ((short) (*pGainContolledADInput - avg)) >> v;
    }
    return;
  }

  v -= 7;
  for (pGainContolledADInput = &(dtmfModule.GainContolledADInput[0]); pGainContolledADInput != &(dtmfModule.GainContolledADInput[N]); )  {
    *pGainContolledADInput++ = ((int) (*pGainContolledADInput - avg)) << v;
  }
#endif
}

//  Calculate Power of Signal
static unsigned int Goertzel (int coeff)
{
  short *x;
  long  q0, q1, q2;
  int  pwr;
  int p1, p2, p01;
  unsigned int  i;
  int scalingFactor;
  int real, imag;

  scalingFactor = N/2;

  q1  = 0;
  q2  = 0;
// 1. Compute for each sample:
// vk(n) = (2*cos(2*PI*f0/fs)) * vk(n-1) - vk(n-2) + x(n)
  x = dtmfModule.GainContolledADInput; //12bit data ... {0..4095}

  //do iteration of N
  for (i = 0; i < N; i++) {

    *x = *x - 2048; //Normalize for 12-bit ADC...YOON

    //q0 = ((coeff*q1)>>14)-q2+*x;
    q0 = ((coeff*q1)>>BITSHIFT_FOR_NOT_USING_FLOAT) - q2 + *x;
    q2 = q1;
    q1 = q0;
    x++;
  }

// 2. Using the final q0,q1 and q2, we computer real and imag.
#if 1
// |X(k)|^2 = vk(N)^2 + vk(N-1)^2 - (2*cos(2*PI*f0/fs)) * vk(N) * vk(N-1))
  p1  = q1*q1;
  p2  = q2*q2;
  p01 = q1*q2;
  //pwr = p1 + p2 - ((coeff*p01) >> 14);
  pwr = p1 + p2 - ((coeff*p01)>> BITSHIFT_FOR_NOT_USING_FLOAT);

  if (pwr < 0){
	  printf("pwr < 0 (%d)\r\n",pwr);
	  return (0); //  make sure that -1 is not returned
  }

  return ((pwr >> 16)); //??? divide by 65536.

#else
  real = (q1 - q2 * cos_fact) / scalingFactor;
  imag = (q2 * sin_fact) /scalingFactor;
  pwr = sqrt(real*real + imag*imag);

  if (pwr < 0){
	  //printf("pwr<0\r\n");
	  return (0); //  make sure that -1 is not returned
  }

  return ((pwr >> 16)); //sqrt??
#endif
}
//=================== applications ============================================================
#if (ADC_APPL == ADC_APPL_DTMF)

//  Check if remaining powers are outside
//  return 0 if invalid power values detected
static int chk_valid (unsigned int p[4],     // power results
                      unsigned int d,        // maximum power
                      unsigned int pref)  {  // power reference

  if (d == 0)  return 0;                     // no digit
  pref /= 8;
  if (d != 1 && p[0] > pref) return (0);
  if (d != 2 && p[1] > pref) return (0);
  if (d != 3 && p[2] > pref) return (0);
  if (d != 4 && p[3] > pref) return (0);
  return (1);
}

/*------------------------------------------------------------------------------
  DTMF Digit:  Checks for valid DTMF digit
      return  digit+0x10  or 0 for invalid digit
 *------------------------------------------------------------------------------*/
static unsigned char DTMF_digit (void)  {
  unsigned int f, rampl, campl;
  unsigned int row, col;
  unsigned int validrow, validcol;
  unsigned int p[4];

//--- Check Row Frequency -------------------------------------
  p[0] = Goertzel (DTMF_697Hz);
  p[1] = Goertzel (DTMF_770Hz);
  p[2] = Goertzel (DTMF_852Hz);
  p[3] = Goertzel (DTMF_941Hz);

  row = 0; rampl = 0x40;  // initial sensivity
  if (p[0] > rampl)  { row = 1;  rampl = p[0]; }
  if (p[1] > rampl)  { row = 2;  rampl = p[1]; }
  if (p[2] > rampl)  { row = 3;  rampl = p[2]; }
  if (p[3] > rampl)  { row = 4;  rampl = p[3]; }
  if (!chk_valid (p, row, rampl)){
	  //printf("Invalid Row\r\n");
	  validrow = 0;
	  //goto invalid;
  }
  else{
	  validrow = 1;
	  printf ("\n\r>row=%d(rowPwr=%d %d %d %d)\r\n", row, p[0], p[1],p[2],p[3]);
  }

//--- Check Col Frequency -------------------------------------
  p[0] = Goertzel (DTMF_1209Hz);
  p[1] = Goertzel (DTMF_1336Hz);
  p[2] = Goertzel (DTMF_1477Hz);
  p[3] = Goertzel (DTMF_1633Hz);

  col = 0; campl = 0x50;  // initial sensivity
  if (p[0] > campl)  { col = 1;  campl = p[0]; }
  if (p[1] > campl)  { col = 2;  campl = p[1]; }
  if (p[2] > campl)  { col = 3;  campl = p[2]; }
  if (p[3] > campl)  { col = 4;  campl = p[3]; }
  if (!chk_valid (p, col, campl)){
	  if(validrow )
		  printf("Valid Row, but Invalid Column\r\n");
	  else
		  printf("Invalid both Row and Column\r\n");
	  goto invalid;
  }else{
	  printf ("col=%d(colPwr=%d %d %d %d)\r\n", col, p[0], p[1],p[2],p[3]);
	  if(!validrow ){
		  printf("Valid Column, but Invalid Row\r\n");
		  goto invalid;
	  }else{
		  printf ("Valid both row and col = (%d,%d)\r\n", row, col);
	  }
  }

  if (col && row)  {    // valid digit detected
// Amplitute Check: col must be within -4dB..+8dB of row
    if ((rampl << 4) < campl)  goto invalid;
    if ((campl << 3) < rampl)  goto invalid;

// check 2nd harmonic
    switch (row)  {
      case 1:
        if (col == 2 || col == 3)  break;   // do not check it
        f = Goertzel (DTMF_1394Hz);
        if (f > (campl / 8))   goto invalid;
        break;

      case 2:
        if (col == 3 || col == 4)  break;   // do not check it
        f = Goertzel (DTMF_1540Hz);
        if (f > (campl / 8))   goto invalid;
        break;

      case 3:
        if (col == 4)  break;              // do not check it
        f = Goertzel (DTMF_1704Hz);
        if (f > (campl / 8))   goto invalid;
        break;

      case 4:
        f = Goertzel (DTMF_1882Hz);
        if (f > (campl / 8))   goto invalid;
        break;
    }

    switch (col)  {
      case 1:
        f = Goertzel (DTMF_2418Hz);
        if (f > (rampl / 8))   goto invalid;
        break;

      case 2:
        f = Goertzel (DTMF_2672Hz);
        if (f > (rampl / 8))   goto invalid;
        break;

      case 3:
        if (row == 4)  break;              // do not check it
        f = Goertzel (DTMF_2954Hz);
        if (f > (rampl / 8))   goto invalid;
        break;

      case 4:
        f = Goertzel (DTMF_3266Hz);
        if (f > (rampl / 8))   goto invalid;
        break;
    }

// digit is valid
    return ((row-1) << 2) | (col-1) | 0x10;
  }

invalid:
	return (0);
}
/*------------------------------------------------------------------------------
  DTMF Detect
 *------------------------------------------------------------------------------*/
void DTMF_Detect (struct DTMF_MODULE *t)  {
  unsigned char d;
  unsigned int  cnt;

  if (t->AIindex >= t->AIcheck)  {

	  dtmf_GainControl (t);                  // Copy AD Input to DTMF Buffer

	  t->AIindex &= (DTMFBUFsz-1);           // ToDo make atomic
	  t->AIcheck = t->AIindex + ((N*2)/3);   // Increment DTMF Window (Overlapping Input Buffer)

	  d = DTMF_digit();

	  t->early = d;     cnt = 0;
	  if (t->d[0] == d) cnt++;
	  if (t->d[1] == d) cnt++;
	  if (t->d[2] == d) cnt++;
	  if (t->d[3] == d) cnt++;
	  t->d[(t->d_i++ & 3)] = d;
	  if (cnt >= 2)  {
		  if (t->digit != d)  {
			  t->digit = d;
			  if (d)  t->new   = 1;
		  }
	  }
  }
}
void dtmf_loop(void)
{
	char dtmfcode;

	printf("Welcome to DTMF decoder \r\n");

	//dtmp_GPIO_Init();

	dtmf_Marker_Adc_Config(); //(with DMA)

	dtmf_MarkerBeacon_8KHzSampling_TIM3_Init();

	//enable ADC to work
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //

  while (1)
  {
		if (dtmfModule.AIindex >= dtmfModule.AIcheck)  {

			DTMF_Detect (&dtmfModule);
			//printf("Detect1:val=%x, Index=%u, Check=%u\r\n", dtmfModule.ADInput[dtmfModule.AIindex & (DTMFBUFsz-1)],dtmfModule.AIindex, dtmfModule.AIcheck);
		}


		//if (dtmfModule.early)		GPIO_SetBits(LD2_GPIO_Port, LD2_Pin);
		//else		GPIO_ResetBits(LD2_GPIO_Port, LD2_Pin);

		if (dtmfModule.new){
			dtmfcode = DTMFchar[dtmfModule.digit & 0x0F];
			printf ("%c ", dtmfcode);
			//dtmfLedOutput(dtmfcode);
			dtmfModule.new = 0;
		}
	}

}
#elif (ADC_APPL == ADC_APPL_MARKER)
static unsigned char MARKER_GetWhichMakerBeacon (void)  {
  unsigned int Ampl;
  unsigned int whichMb;
  unsigned int power[3];

//--- Check Col Frequency -------------------------------------
  power[0] = Goertzel (OM_400Hz);  if(power[0] == 0) return 0;
  power[1] = Goertzel (MM_1300Hz); if(power[1] == 0) return 0;
  power[2] = Goertzel (IM_3000Hz); if(power[2] == 0) return 0;

  whichMb = 0; Ampl = GOERTZEL_PWR_THRESHOLD;  // initial sensitivity
  if (power[0] > Ampl)  { whichMb = 1;  Ampl = power[0]; }
  if (power[1] > Ampl)  { whichMb = 2;  Ampl = power[1]; }
  if (power[2] > Ampl)  { whichMb = 3;  Ampl = power[2]; }

  if(whichMb !=0)
	  printf ("whichMb = %d (mbPwr={%d, %d, %d}\r\n", whichMb, power[0], power[1],power[2]);
  else
	  printf ("...TooWeak mbPwr={%d, %d, %d}\r\n", power[0], power[1],power[2]);

  return (whichMb);
}
/*------------------------------------------------------------------------------
  Do discrimination
 *------------------------------------------------------------------------------*/
void MarkerBeaconDiscriminate (struct DTMF_MODULE *m)  {
  unsigned char whichMb;
  unsigned int  cnt;

  dtmf_GainControl (m);                  // Copy ADInput to GainContolledADInput Buffer

  whichMb = MARKER_GetWhichMakerBeacon();

  if(whichMb > 0){
	  printf("> Detected MarkerBeacon = \"%s\"\r\n", MARKERstr[whichMb]);

  }
  mbLedOutput(whichMb);
  //wait for next discrimination
  delayms(1000);

  //reset buffer position for next
  m->new = 0;
  m->AIindex &= (DTMFBUFsz-1);
  m->AIcheck = m->AIindex + ((N*2)/3);   //Next stop sampling.

}

////////////////////////////////////////////////////////////
void mbLedOutput(unsigned char whichMb)
{
	switch(whichMb){

		case 0:
			GPIO_ResetBits(MB_OM_GPIO_Port, MB_OM_Pin);
			GPIO_ResetBits(MB_MM_GPIO_Port, MB_MM_Pin);
			GPIO_ResetBits(MB_IM_GPIO_Port, MB_IM_Pin);
			break;
		case 1 : //OM
			GPIO_SetBits(MB_OM_GPIO_Port, MB_OM_Pin);
			//delayms(100);
			//GPIO_ResetBits(MB_OM_GPIO_Port, MB_OM_Pin);
			break;

		case 2 : //MM
			GPIO_SetBits(MB_MM_GPIO_Port, MB_MM_Pin);
			//delayms(100);
			//GPIO_ResetBits(MB_MM_GPIO_Port, MB_MM_Pin);
			break;

		case 3 : //IM
			GPIO_SetBits(MB_IM_GPIO_Port, MB_IM_Pin);
			//delayms(100);
			//GPIO_ResetBits(MB_IM_GPIO_Port, MB_IM_Pin);
			break;
		default:
			break;
	}
}
void markerBeacon_loop(void)
{
	char dtmfcode;
	unsigned short val16;
	NVIC_InitTypeDef   NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;

	printf("Welcome to MarkerBeacon decoder \r\n");

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//OM = PB1/MM = PB11
	GPIO_InitStruct.GPIO_Pin = MB_OM_Pin | MB_MM_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MB_OM_GPIO_Port, &GPIO_InitStruct);
	//IM = PA4
	GPIO_InitStruct.GPIO_Pin = MB_IM_Pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MB_IM_GPIO_Port, &GPIO_InitStruct);

	//dtmp_GPIO_Init();
	dtmf_Marker_Adc_Config(); //(with DMA)

	dtmfModule.AIindex = 0;            						// Start index of sampling data buffer.
	dtmfModule.AIcheck = dtmfModule.AIindex + ((N*2)/3);   	//Next stop sampling.

#if (ADC_SCHEME == ADC_SCHEME_POLLING)
	while(1)	{

		ADC_SoftwareStartConvCmd(ADC1, ENABLE); //
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		//Get value
		val16 = ADC_GetConversionValue(ADC1);
		//printf("Val=%u\r\n", val16);
		//ADC_SoftwareStartConvCmd(ADC1, DISABLE); //

	   	dtmfModule.ADInput[dtmfModule.AIindex & (DTMFBUFsz-1)] = val16;
    	dtmfModule.AIindex++;

    	somedelay(5000); //5000 is not correct. Should use a correct value for sampling period(125usec)..

		if (dtmfModule.AIindex >= dtmfModule.AIcheck){
			printf("--Index=%u, Check=%u\r\n",dtmfModule.AIindex,  dtmfModule.AIcheck);
			MarkerBeaconDiscriminate(&dtmfModule);

			dtmfModule.AIindex = 0; //&= (DTMFBUFsz-1);           // ToDo make atomic
			dtmfModule.AIcheck = dtmfModule.AIindex + ((N*2)/3);
		}
	}
}
#elif (ADC_SCHEME == ADC_SCHEME_INTERRUPT)
	//init 8KHz sampling Timer with TIM3
	dtmf_MarkerBeacon_8KHzSampling_TIM3_Init();

	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //start sampling
	while(1){
		//if we gather plenty samples, we do discrimination.
		if (dtmfModule.AIindex >= dtmfModule.AIcheck){
			ADC_SoftwareStartConvCmd(ADC1, DISABLE); //stop sampling..
			//printf("--Index=%u, Check=%u\r\n",dtmfModule.AIindex,  dtmfModule.AIcheck);
			MarkerBeaconDiscriminate(&dtmfModule);
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		}
#if 0
		//if (dtmfModule.early)		GPIO_SetBits(LD2_GPIO_Port, LD2_Pin);
		//else		GPIO_ResetBits(LD2_GPIO_Port, LD2_Pin);

		if (dtmfModule.new){
			MarkerBeaconDiscriminate(&dtmfModule); //dtmfcode = MARKERchar[dtmfModule.digit & 0x0F];
			//printf ("%c ", dtmfcode);
			//mbLedOutput(dtmfcode);
			dtmfModule.new = 0;
		}
#endif
	}
}
#else //DMA
	//Config and enable interrupt for DMA1-Channel 1
	printf("NVIC DMA1_CH1 Setup\r\n");
	NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	dtmf_MarkerBeacon_8KHzSampling_TIM3_Init();

	//enable ADC to work now.
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //

	while (1) {

		if (dtmfModule.AIindex >= dtmfModule.AIcheck){
			printf("--Index=%u, Check=%u\r\n",dtmfModule.AIindex,  dtmfModule.AIcheck);
			MarkerBeaconDiscriminate(&dtmfModule);
			dtmfModule.AIindex = 0; //&= (DTMFBUFsz-1);           // ToDo make atomic
			dtmfModule.AIcheck = dtmfModule.AIindex + ((N*2)/3);
		}
		//if(dtmfModule.AIindex == N) //Only when the buffer is filled.
		//{
		//	MarkerBeaconDiscriminate(&dtmfModule);
		//}


	}
}
#endif
#endif

#if 0

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void dtmp_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin Output Level */
  GPIO_SetBits(GPIOC, OUT1_Pin|OUT2_Pin);// HAL_GPIO_WritePin(GPIOC, OUT1_Pin|OUT2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(LD2_GPIO_Port, LD2_Pin);//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(GPIOC, GPIO_Pin_4);//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  GPIO_ResetBits(GPIOB, GPIO_Pin_13);//GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;//B1_Pin;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IT_FALLING;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  //HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT1_Pin OUT2_Pin */
  GPIO_InitStruct.GPIO_Pin = OUT1_Pin|OUT2_Pin;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.GPIO_Pin = LD2_Pin;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC4 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  //GPIO_InitStruct.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_SPEED_FREQ_LOW;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

}

////////////////////////////////////////////////////////////
void dtmfLedOutput(char dtmfcode)
{
	switch(dtmfcode){
		case '1' :
			GPIO_ResetBits(OUT1_GPIO_Port, OUT1_Pin);
			break;

		case '2' :
			GPIO_SetBits(OUT1_GPIO_Port, OUT1_Pin);
			break;

		case '4' :
			GPIO_ResetBits(OUT2_GPIO_Port, OUT2_Pin);
			break;

		case '5' :
			GPIO_SetBits(OUT2_GPIO_Port, OUT2_Pin);
			break;

		default:
			break;
	}
}
#endif
#if 0// (ADC_SCHEME == ADC_SCHEME_INTERRUPT) //Interrupt
//Issued by ADC when End of Conversion, and Full or Half-Full.
void ADC1_2_IRQHandler(){

	if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {

		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

		//Get ADC Value
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    	dtmfModule.ADC_ConvertedValue = ADC_GetConversionValue(ADC1); //get data
    	printf(">%u,", dtmfModule.ADC_ConvertedValue);
    	dtmfModule.ADInput[dtmfModule.AIindex & (DTMFBUFsz-1)] = dtmfModule.ADC_ConvertedValue;
    	dtmfModule.AIindex++;

    	ADC_SoftwareStartConvCmd(ADC1, DISABLE); //should be... ??
    }
}
#endif
