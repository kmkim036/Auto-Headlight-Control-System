
#include "yInc.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#if ((PROCESSOR == PROCESSOR_STM32F103C8T6))
//NOT SUPPOTRED
#elif ((PROCESSOR == PROCESSOR_STM32F103RCT6) || (PROCESSOR == PROCESSOR_STM32F107VCT))
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "math.h"
#include "core_cm3.h"
#include "misc.h"

#define DAC_SCHEME_POLLING    	0
#define DAC_SCHEME_DMA        	2
#define DAC_SCHEME 				DAC_SCHEME_DMA //DAC_SCHEME_POLLING //

//=====================DAC with DMA for Two channels =========================
//PA4(CH1) and PA5(CH2)Only
// DAC channel 1 (DAC_OUT1 = PA.4) configuration
// DAC channel 2 (DAC_OUT2 = PA.5) configuration
//DAC_CH1 uses DMA2_CH2
//DAC_CH2 uses DMA2_CH4

//#define DAC_DHR12R1 (unsigned) &(DAC->DHR12R1)//Addr of Peripheral DAC Channel 1
//#define DAC_DHR12R2 (unsigned) &(DAC->DHR12R2)  //Addr of Peripheral DAC Channel 2
#define DAC_DHR12R1 0x40007408 //Addr of Peripheral DAC Channel 1 (12-bit right-justified data holding register of dac channel1)
#define DAC_DHR12R2 0x40007414 //(*)Addr of Peripheral DAC Channel 2 (12-bit right-justified data holding register of dac channel2)

//==========================================
#define PI 3.14156
#if 0
#define SAMPLING_RATE 8000
#define FREQ_GIVEN 100
#define SAMPLES (SAMPLING_RATE/FREQ_GIVEN)
#endif
//Num samples for a full period = 8000/f
#define SAMPLES 256

//TIMER ============================================
// TicksPerSec = TIM_CLK/((prescaler+1)*(Period+1)),
// It becomes (Period+1) = TIM_CLK/((prescaler+1)*TicksPerSec)
// For generating a sine wave, we need Ticks with the number of SAMPLES for a full wave.
// Thus, we get the TIM_Period for given freq of a sine wave =  TIM_CLK/((prescaler+1)*freq*SAMPLES) - 1.
#define YDAC_GET_TIM_PERIOD_FOR_GIVEN_FREQ(freq) ((72000000/(1)/freq/SAMPLES) - 1)

#define YTIM_FOR_DAC_CH1_TRIG  TIM6
#define YTIM_FOR_DAC_CH2_TRIG  TIM2
//====================================================

unsigned short g_sineWaveTable[SAMPLES];

//We prepare sine wave table for a single period with 256 samples.
void stmDacPrepareSineWaveTable(float freq)
{
	int i;
	double x;
	unsigned short value;
	float step;

#if 0
	step = freq * ((2.0*3.14159))/SAMPLING_RATE;
	for(i=0;i<SAMPLES;i++){
		g_sineWaveTable[i] = sin(i*step)*2048.0 + 2048.0;
		printf("sin(%u)=%s (%u)\r\n", i, float2str(sin(i*step)), g_sineWaveTable[i]);
		//printf("step1=%s\r\n", float2str(step));
	}

#else
	for (i=0; i<SAMPLES;i++){
		//360 degree / 256 = 1.40625
		//value = (-1 .. +1) --> +1 --> (0..2)
		//*2048 --> 0..4095
		//g_sineWaveTable[i] = (unsigned short) rint((sinf((2*PI/SAMPLES)*i)+1)*2048);     //it has Zero and 4095.
		g_sineWaveTable[i] = (unsigned short) rint((sinf((2*PI/SAMPLES)*i)+1)*1800 + 400); //1800 = Weighting Factor for 12 bit DAC; 400 = offset.
		printf("sineTable[%u]=%u\r\n",i,g_sineWaveTable[i]);
	}
#endif
}

#if (DAC_SCHEME == DAC_SCHEME_DMA)

//For generating triggering signal to DAC using TRGO
void stmDac_TIM_Config_For_Gen_SineWave(unsigned char whichChannels, float freq1, float freq2)
{
	GPIO_InitTypeDef init_AF;//
	TIM_TimeBaseInitTypeDef timer_init;
	NVIC_InitTypeDef   NVIC_InitStructure;

	//Setup TIM6 with TRGO for DAC_CH1
    if(whichChannels & 0x01){
    	printf("TIM6 Setup with TRGO)\r\n");
    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
       	//Freq = 72MHz/(Precaler+1)/(period+1)
       	//We need 8KHz
       	TIM_DeInit(YTIM_FOR_DAC_CH1_TRIG);
    	//We found max freq = 17.5795KHz with Prescaler (0..7) and Period = 1
    	//Freq = TIM_CLK/(TIM_PSC+1)/(TIM_ARR+1)
    	//8000 = 72MHz/(71+1)/(124+1) = 72000000/72/125 = 8KHz
       	//Result oFreqOfSineWave = 72MHz/(Precaler+1)/(period+1)/SAMPLES
    	TIM_TimeBaseStructInit(&timer_init);
    	//   	timer_init.TIM_Prescaler 	= 71;
    	timer_init.TIM_Prescaler 	= 0;
    	timer_init.TIM_ClockDivision = 1;
    	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
    	timer_init.TIM_Period 		= (unsigned short) YDAC_GET_TIM_PERIOD_FOR_GIVEN_FREQ(freq1);//2047;//(this value will be loaded in Auto-Reload Register(ARR) when the current count value reaches this value.)
   	                                  //and then the current count value will be 0, and issues TRGO
    	//timer_init.TIM_RepetitionCounter = 0; //This parameter is valid only for TIM1 and TIM8
    	TIM_TimeBaseInit(YTIM_FOR_DAC_CH1_TRIG, &timer_init);

    	printf("Period=%u\r\n", timer_init.TIM_Period );

    	//Trigger Output Enable for DAC_CH1
    	TIM_SelectOutputTrigger(YTIM_FOR_DAC_CH1_TRIG, TIM_TRGOSource_Update);//Not use TIM_TRGOSource_Reset);
                         //TIM_TRGOSource_Reset: The UG(update generation) bit in the TIM_EGR register is used as the trigger output (TRGO).
                         //TIM_TRGOSource_Update: The update event is selected as the trigger output (TRGO).
    }

    //Setup TIM2 with TRGO for DAC_CH2
    if(whichChannels & 0x02)
    {
    	printf("TIM2 Setup with TRGO)\r\n");
    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

       	TIM_DeInit(YTIM_FOR_DAC_CH2_TRIG); //Stop

    	TIM_TimeBaseStructInit(&timer_init);
    	timer_init.TIM_Prescaler 	= 0; //71
    	timer_init.TIM_ClockDivision = 1;
    	timer_init.TIM_CounterMode = TIM_CounterMode_Up;
    	timer_init.TIM_Period 		=(unsigned short) YDAC_GET_TIM_PERIOD_FOR_GIVEN_FREQ(freq2); //124
    	TIM_TimeBaseInit(YTIM_FOR_DAC_CH2_TRIG, &timer_init);
    	TIM_SelectOutputTrigger(YTIM_FOR_DAC_CH2_TRIG, TIM_TRGOSource_Update);
    }
}
#if 0

//void DMA1_Channel1_IRQHandler(void)//This function handles DMA1 channel 1 global interrupt after gathering all N samples by DMA.
void DMA2_Channel4_5_IRQHandler(void)//This function handles DMA2 channel 4 global interrupt after gathering all N samples by DMA.
{
	if(DMA_GetITStatus(DMA2_IT_TC2))
	{
		printf(".");
		//stop ADC now
    	//DAC_SoftwareStartConvCmd(DAC1, DISABLE);
    	//DMA_ClearFlag(DMA1_FLAG_TC1);

    	//Notify the buffer has been filled.
    	//dtmfModule.AIindex = N;
    	//dtmfModule.new = 1;
	}else
		printf("DMA??");
	DMA_ClearITPendingBit(DMA2_IT_TC2 | DMA2_IT_GL2);
}
#endif
#else

void stmDac_Output(unsigned short val){
	printf("val=%u\r\n", val);
    if (val > 4095) val = 4095;
    DAC_SetChannel2Data(DAC_Align_12b_R, val); //CH2
}
#endif
void stmDac_Config(unsigned char whichChannels) //1=CH1, 2=CH2, 3=BOTH
{
    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef DAC_InitStructure;
    DMA_InitTypeDef DMA_InitStruct;
    int val;

    // GPIOA clock enable (to be used with DAC)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // DAC channel 1 (DAC_OUT1 = PA.4) configuration
    // DAC channel 2 (DAC_OUT2 = PA.5) configuration
    if(whichChannels & 0x01){
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //set as analog input
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    	GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    if(whichChannels & 0x02){
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //set as analog input
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    	GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
	//DeInit before DAC config.
	DAC_DeInit();

#if (DAC_SCHEME == DAC_SCHEME_DMA)

	// DAC Periph clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);



    //DMA-CH1 @ PA4
    if(whichChannels & 0x01){

     	printf("DAC_Channel_1 Setup\r\n");
     	printf("DAC is trigger by TRGO of TIM6, and N samples will be transferred by DMA2_CH3\r\n");
     	//configure DAC parameters (Independent(no interleaved with other ADCs), Continuous, TRGO(Trigger Output)
     	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO; //DAC Trigger the timer trigger 6
     	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
     	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable; //DAC_OutputBuffer_Enable;//
     	//DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;

     	//We use DMA2/Channel3 to link the DAC_CH1 peripheral
     	//We use DMA for transferring N halfWords(16 bits).
     	//After N halfWords transferring, the DMA engine will issue Transfer Complete Interrupt.
     	printf("Config DMA2_CH3 for DAC_CH1\r\n");
     	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
     	DMA_DeInit(DMA2_Channel3);
     	DMA_InitStruct.DMA_BufferSize 		= SAMPLES;						//We move N HalfWord from the user memory to DAC peripheral
     	DMA_InitStruct.DMA_MemoryBaseAddr 	= (unsigned)g_sineWaveTable; 	//user memory address to be moved by DMA.
     	DMA_InitStruct.DMA_PeripheralBaseAddr = DAC_DHR12R1; 				//0x40007408 for DAC_CH1
     	DMA_InitStruct.DMA_PeripheralInc  	= DMA_PeripheralInc_Disable; 	//No address increment during DMA
     	DMA_InitStruct.DMA_MemoryInc 	  	= DMA_MemoryInc_Enable; 		//We advance user memory position each DAC conversion.
     	DMA_InitStruct.DMA_MemoryDataSize 	= DMA_MemoryDataSize_HalfWord;	//16-bit
     	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
     	DMA_InitStruct.DMA_DIR 				= DMA_DIR_PeripheralDST; 		//from User Memory to DMA Peripheral for DAC
     	DMA_InitStruct.DMA_Priority         = DMA_Priority_High;
     	DMA_InitStruct.DMA_M2M              = DMA_M2M_Disable;
     	DMA_InitStruct.DMA_Mode 		  	= DMA_Mode_Circular; 			//Automatically return to index 0 of user buffer(==g_sineWaveTable) after completion.
     	DMA_Init(DMA2_Channel3, &DMA_InitStruct);
     	DMA_Cmd(DMA2_Channel3,ENABLE);

     	//printf("DMA2_CH3 TransferComplete IRQ Setup\r\n");
     	//DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE); //Enable DMA2 Channel Transfer Complete Interrupt.

     	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

     	DAC_Cmd(DAC_Channel_1, ENABLE); //Enable DAC_CH1: When this DAC_CH1 is enabled, PA4 is connected to the DAC-CH1.

     	//Enable DMA for DAC_CH1
     	DAC_DMACmd(DAC_Channel_1,ENABLE); //enable DAC_CH1 DMA

     	//Star Timer for triggering DAC output.
     	//TIM_Cmd(YTIM_FOR_DAC_CH1_TRIG, ENABLE);
     }
    //DAC-CH2 @ PA5
    if(whichChannels & 0x02){

    	printf("DAC_Channel_2 Setup\r\n");
    	printf("DAC is trigger by TRGO from TIM2, and N samples will be transferred by DMA2_CH4\r\n");
    	//configure DAC parameters (Independent(no interleaved with other ADCs), Continuous, TRGO(Trigger Output)
    	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO; //DAC Trigger the timer trigger 2
    	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable; //DAC_OutputBuffer_Enable;//
    	//DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;

    	//We will use DMA2/Channel4 to link the DAC_CH2 peripheral
    	//We use DMA for transferring N halfWords(16 bits).
    	//After N halfWords transferring, the DMA engine will issue Transfer Complete Interrupt.
    	printf("Config DMA2_CH4 for DAC_CH2\r\n");
    	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    	DMA_DeInit(DMA2_Channel4);
    	DMA_InitStruct.DMA_BufferSize 		= SAMPLES;	                  //We move N HalfWord from the user memory to DAC peripheral
    	DMA_InitStruct.DMA_MemoryBaseAddr 	= (unsigned)g_sineWaveTable;  //user memory address to be moved by DMA.
    	DMA_InitStruct.DMA_PeripheralBaseAddr = DAC_DHR12R2; 			  //0x40007414 for DAC_CH2
    	DMA_InitStruct.DMA_PeripheralInc  	= DMA_PeripheralInc_Disable;  //No address increment during DMA
    	DMA_InitStruct.DMA_MemoryInc 	  	= DMA_MemoryInc_Enable;       //We advance user memory position each DAC conversion.
    	DMA_InitStruct.DMA_MemoryDataSize 	= DMA_MemoryDataSize_HalfWord;//16-bit
    	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    	DMA_InitStruct.DMA_DIR 				= DMA_DIR_PeripheralDST;      //from DMA to Peripheral
    	DMA_InitStruct.DMA_Priority         = DMA_Priority_High;
    	DMA_InitStruct.DMA_M2M              = DMA_M2M_Disable;
    	DMA_InitStruct.DMA_Mode 		  	= DMA_Mode_Circular;
    	DMA_Init(DMA2_Channel4, &DMA_InitStruct);
    	DMA_Cmd(DMA2_Channel4,ENABLE);

    	//printf("DMA2_CH4 TransferComplete IRQ Setup\r\n");
    	//DMA_ITConfig(DMA2_Channel4, DMA_IT_TC, ENABLE); //Enable DMA2 Channel Transfer Complete Interrupt.

    	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

    	DAC_Cmd(DAC_Channel_2, ENABLE); //When the DAC_CH2 is enabled, PA5 is connected to the DAC.

    	//Enable DMA for DAC_CH2
    	DAC_DMACmd(DAC_Channel_2,ENABLE); //enable DAC_CH2 DMA
    	//Star Timer for triggering DAC output.
    	//TIM_Cmd(YTIM_FOR_DAC_CH2_TRIG, ENABLE);
    }

#else  //polling case

    DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    //DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;
    DAC_Init(DAC_Channel_2, &DAC_InitStructure);

    DAC_Cmd(DAC_Channel_2, ENABLE); //Enable DAC_CH2: When the DAC_CH2 is eabled, PA5 is connected to the DAC.

#endif
}
void stmDac_StartAndStop(unsigned char bStart1_Stop0){
	if(bStart1_Stop0){
		//Start Timers for triggering DAC output.
		TIM_Cmd(YTIM_FOR_DAC_CH1_TRIG, ENABLE);
		TIM_Cmd(YTIM_FOR_DAC_CH2_TRIG, ENABLE);
	}else{
		//Stop Timers for triggering DAC output.
		TIM_Cmd(YTIM_FOR_DAC_CH1_TRIG, DISABLE);
		TIM_Cmd(YTIM_FOR_DAC_CH2_TRIG, DISABLE);
	}
}
//OK
void stmDacLoop(){

	unsigned short i = 0;
    printf("\r\nDual Channel DAC with DMA for generating beautiful sine waves Test\r\n");
    printf("If you want the dual tone for DTMF, ILS, and SELCAL applications, please tie PA4 and PA5 pins\r\n");

    stmDacPrepareSineWaveTable(1.0);//FREQ_GIVEN);

    stmDac_Config(3); //1=CH1; 2=CH2; 3=BOTH

	stmDac_TIM_Config_For_Gen_SineWave(3,  //1=CH1; 2=CH2; 3=BOTH
			312.6, //freq1 for channel 1
			1333.5);//freq2 for channel 2

    while(1)
    {
#if (DAC_SCHEME == DAC_SCHEME_DMA)

    	//TIM and DMA will do all without CPU intervention.

    	//We here can simply start and stop.
    	stmDac_StartAndStop(1);
    	delayms(1000);
    	stmDac_StartAndStop(0);
    	delayms(1000);


#else
		if(i == 360) //SAMPLES)
			i = 0;
        stmDac_Output(g_sineWaveTable[i]);
        //stmDac_Output(sin(i*2*3.14/360)*2048.0 + 2048.0);
        //somedelay(1000);
        i++;
#endif
    }
}
#endif

