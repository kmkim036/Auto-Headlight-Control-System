// Filename: DH11TempSensor.c
//-- It uses a specific 1-wire commun. See its datasheet. Too slow sensor.
// - Sensor Data pin is open drain(needs pullup resistor of 4.7K)
// - No under Zero degree...
// - Not support decimal point for Humidity and Temperature.

//-- We use PA8
//Not support float in STM32F103.

//GPIO PINs
//+-----------------+-----------+-----------+-----------+---------+---------+
//|                 |103C8T6
//+-----------------+-----------+-----------+-----------+---------+---------+
//| iSensorData     | PA8
//+-----------------+-----------+-----------+-----------+---------+---------+
//*H= Active High/L=Active Low
//*F= FallingEdge
/*
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#include <stdio.h>
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "yInc.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "lwipopts.h"
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
//#include "stm32f10x_flash.h"
#include "yInc.h"
#include "core_cm3.h"    //for irq
#include "core_cmFunc.h" //for irq

extern void delayus(uint32_t ms);

#define DHTTYPE 11   // DHT 11
#define MIN_INTERVAL 2000 /**< min interval value (2msec)*/
#define TIMEOUT -1        /**< timeout on */

#define LOW 0
#define HIGH 1

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic,  connect pin 1 to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

struct DHT {
  uint8_t data[5];
  uint8_t _pin, _type;
  uint32_t _lastreadtime, _maxcycles;
  boolean _lastresult;
  uint8_t pullTime; // Time (in usec) to pull up data line before reading
} dht;

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t DHT_expectPulse(unsigned char siglevel) {
  uint16_t count = 0; // To work fast enough on slower AVR boards
  while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == siglevel){ // while (digitalRead(_pin) == level) {
	  //the input signal is no changed...
    if (count++ >= dht._maxcycles) { //1000usec,1msec
      return TIMEOUT; // Exceeded timeout, fail.
    }
    delayus(1);
  }

  //if signal is changed, the count is less than maxcycles.
  return count;
}

/*!
 *  @brief  Converts Celcius to Fahrenheit
 *  @param  c
 *					value in Celcius
 *	@return float value in Fahrenheit
 */
float DHT_convertCtoF(float c) { return c * 1.8 + 32; }

/*!
 *  @brief  Converts Fahrenheit to Celcius
 *  @param  f
 *					value in Fahrenheit
 *	@return float value in Celcius
 */
float DHT_convertFtoC(float f) { return (f - 32) * 0.55555; }


/*!
 *  @brief  Setup sensor pins and set pull timings
 *  @param  usec
 *          Optionally pass pull-up time (in microseconds) before DHT reading
 *starts. Default is 55 (see function declaration in DHT.h).
 */
void DHT_config(uint8_t usec) {
	GPIO_InitTypeDef   GPIO_InitStruct;

	//// set up the pins! PA8
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;//GPIO_Mode_Out_PP;//GPIO_Mode_IPU;//GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	delayms(1);
	GPIO_SetBits(GPIOA, GPIO_Pin_8); //HIGH in idle
	delayms(1000);


	//dht._maxcycles =  microsecondsToClockCycles(1000); // 1 millisecond timeout for reading pulses from DHT sensor.
	dht._maxcycles =  1000; // 1 millisecond timeout for reading pulses from DHT sensor.
  // Using this value makes sure that millis() - lastreadtime will be >= MIN_INTERVAL right away.
	// Note that this assignment wraps around, but so will the subtraction.
	dht._lastreadtime = micros() - MIN_INTERVAL;
	printf("DHT max clock cycles = %u ", dht._maxcycles);
	dht.pullTime = usec;


}

/*!
 *  @brief  Read value from sensor or return last one from less than two
 *seconds.
 *  @param  force
 *          true if using force mode
 *	@return float value
 */
boolean DHT_read(boolean force) {
  uint32_t cycles[80];
  int i;
  GPIO_InitTypeDef   GPIO_InitStruct;

  //printf("Read\r\n");

  // Check if sensor was read less than two seconds ago and return early to use last reading.
  uint32_t currenttime = millis();

  if (!force && ((currenttime - dht._lastreadtime) < MIN_INTERVAL)) {
    return dht._lastresult; // return last correct measurement
  }
  dht._lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  dht.data[0] = dht.data[1] = dht.data[2] = dht.data[3] = dht.data[4] = 0;


  // Send start signal.  See DHT datasheet for full signal diagram:
  // Go into high impedence state to let pull-up raise data line level and  start the reading process.
  // change pin config of PA8
  //GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_IPU;//GPIO_Mode_Out_OD;//
  //GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;
  //GPIO_Init(GPIOA, &GPIO_InitStruct);
  //delayms(1);
  //                                            | <8bit RH Integ><8bit RH decimal><8bit T Integ><8bit T decimal><8bitCRC>|
  // -----Master ---------|---DH11--------------|------DH11 Data Seq (total 40bits)--- ----------------------------------|
  //                                            | --'0'---------| ------ '1'------| ...
  //----+ 18ms    +20~40us+           +-80us----+      +-26~28us+      +- 70us ---+
  //    |         |       |           |         |      |        |      |          |
  //    +---------+       +-80us -----+         +-50us-+        +-50us-+          +-- ..
  // First set data line low for a period according to sensor type
  //GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;//GPIO_Mode_Out_OD;//
  //GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;
  //GPIO_Init(GPIOA, &GPIO_InitStruct);
  //somedelay(10);
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_OD;//GPIO_Mode_Out_PP;//GPIO_Mode_IPU;//GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  somedelay(10);

  GPIO_ResetBits(GPIOA, GPIO_Pin_8); //LOW;
  delayms(20); // data sheet says at least 18ms, 20ms just to be safe

  // At the End the start signal by setting data line high (PULL UP) for 40 microseconds.
  //GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;//GPIO_Mode_Out_OD;//GPIO_Mode_Out_PP;
  //GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;
  //GPIO_Init(GPIOA, &GPIO_InitStruct);
  //somedelay(100);
  GPIO_SetBits(GPIOA, GPIO_Pin_8); //HIGH;
  delayus(40);//dht.pullTime);// Delay a moment to let sensor pull data line low.

  // Now start reading the data line to get the value from the DHT sensor.
  // Turn off interrupts temporarily because the next sections are timing critical and we don't want any interruptions.
  //__disable_irq();// InterruptLock lock;----------


  // First expect a low signal for ~80 microseconds
  //followed by a high signal for ~80 microseconds again.

    if (DHT_expectPulse(LOW) == TIMEOUT) {
      printf("Fail:DHT timeout waiting for start signal low pulse.\r\n");
      dht._lastresult = false;
      return dht._lastresult;
    }else{
    	//we got the pulse
    }
    if (DHT_expectPulse(HIGH) == TIMEOUT) {
      printf("Fail:DHT timeout waiting for start signal high pulse.\r\n");
      dht._lastresult = false;
      return dht._lastresult;
    }else{
    	//we got the pulse
    }
    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed
    // all the pulses are read into a array and then examined in a later step.

    //receving 80 pulses(40 alternate pulses).
    for (i = 0; i < 80; i += 2) {
      cycles[i] = DHT_expectPulse(LOW); //start bit
      cycles[i + 1] = DHT_expectPulse(HIGH); //length dependent.. we save its length
    }
   // Timing critical code is now complete.
  //__enable_irq();//

  //printf("We got the data\r\n");

  // Inspect pulses and determine which ones are 0 (high state cycle count < low  state cycle count),
  // or 1 (high state cycle count > low state cycle count).

  for (i = 0; i < 40; ++i) {
    uint32_t lowCycles = cycles[2 * i]; 	//even -- bit start (always 50usec)
    uint32_t highCycles = cycles[2 * i + 1];//odd -- value (short = '0', long = '1')
    if ((lowCycles == TIMEOUT) || (highCycles == TIMEOUT))
    {
      printf("DHT timeout waiting for pulse.");
      dht._lastresult = false;
      return dht._lastresult;
    }

    //Otherwise, it is OK.
    dht.data[i / 8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) { //highCycles(data value) > 50use(bit start --> 1, else 0)
      // High cycles are greater than 50us low cycle count, must be a 1.
    	dht.data[i / 8] |= 1;
    }else{
    	//0
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  printf("Rx from DHT> [%02x %02x | %02x %02x] - %02x(CRC)\r\n", dht.data[0], dht.data[1], dht.data[2], dht.data[3],dht.data[4],(dht.data[0] + dht.data[1] + dht.data[2] + dht.data[3]) & 0xFF);

  // Check we read 40 bits and that the checksum matches.
  if (dht.data[4] == ((dht.data[0] + dht.data[1] + dht.data[2] + dht.data[3]) & 0xFF)) {
    dht._lastresult = true;
    return dht._lastresult;
  } else {
    printf("DHT checksum failure!");
    dht._lastresult = false;
    return dht._lastresult;
  }
}

/*!
 *  @brief  Read temperature
 *  @param  S
 *          Scale. booleanean value:
 *					- true = Fahrenheit
 *					- false = Celcius
 *  @param  force
 *          true if in force mode
 *	@return Temperature value in selected scale
 */

//Byte2 and 3
#if 0
float DHT_readTemperature(boolean S, boolean force) {
  float f = 0.0;

  if (DHT_read(force)) {
      f = (float) dht.data[2];
      if (dht.data[3] & 0x80) {
        f = -1 - f;
      }
      f += (float) (dht.data[3] & 0x0f)/10;//f += (dht.data[3] & 0x0f) * 0.1;
      //if (S) {
      //  f = DHT_convertCtoF(f);
      //}
  }
  return f;
}
#else
int DHT_readTemperature(boolean S, boolean force) {
  int t = 0;

  if (DHT_read(force)) {
      t = dht.data[2];
      if (dht.data[3] & 0x80) {
        t = -1 - t;
      }
      //if (S) {
      //  f = DHT_convertCtoF(f);
      //}
  }
  return t;
}
#endif
/*!
 *  @brief  Read Humidity
 *  @param  force
 *					force read mode
 *	@return float value - humidity in percent
 */

#if 0
//Byte0 and 1
float DHT_readHumidity(boolean force) {
  float f = 0.0;
  if (DHT_read(force)) {
	  f = (float) (dht.data[0] + dht.data[1]/10) ;//f = dht.data[0] + dht.data[1] * 0.1;
  }
  return f;
}
#else
int DHT_readHumidity(boolean force) {
  int h = 0;
  if (DHT_read(force)) {
	  h =  dht.data[0];
  }
  return h;
}
#endif
/*!
 *  @brief  Compute Heat Index
 *          Simplified version that reads temp and humidity from sensor
 *  @param  isFahrenheit
 * 					true if fahrenheit, false if celcius (default
 *true)
 *	@return float heat index

float DHT_computeHeatIndex(boolean isFahrenheit) {
  float hi = DHT_computeHeatIndex(
		  DHT_readTemperature(isFahrenheit, true), DHT_readHumidity(true),  isFahrenheit
		  );
  return hi;
}
 */
/*!
 *  @brief  Compute Heat Index
 *  				Using both Rothfusz and Steadman's equations
 *					(http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml)
 *  @param  temperature
 *          temperature in selected scale
 *  @param  percentHumidity
 *          humidity in percent
 *  @param  isFahrenheit
 * 					true if fahrenheit, false if celcius
 *	@return float heat index
 */
float DHT_computeHeatIndex(
		float temperature,
		float percentHumidity,
        boolean isFahrenheit)
{
  float hi;

  if (!isFahrenheit)
    temperature = DHT_convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) +
              (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 + 2.04901523 * temperature + 10.14333127 * percentHumidity +
         -0.22475541 * temperature * percentHumidity +
         -0.00683783 * pow(temperature, 2) +
         -0.05481717 * pow(percentHumidity, 2) +
         0.00122874 * pow(temperature, 2) * percentHumidity +
         0.00085282 * temperature * pow(percentHumidity, 2) +
         -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if ((percentHumidity < 13) && (temperature >= 80.0) &&
        (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) *
            sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if ((percentHumidity > 85.0) && (temperature >= 80.0) &&
             (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : DHT_convertFtoC(hi);
}

void dh11_loop() {
	int h;
	int t;

  // Wait a few seconds between measurements.
  delayus(2000);

  printf("DH11 Sensor Test\r\n");

  DHT_config(55); //55 microsec

  Init_SysTick(1000); //1000000 = 1usec...//100000    10us

  while(1){

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = DHT_readHumidity(true);

  // Read temperature as Celsius (the default)
  //read temp in celcius
  delayms(2000);
  t = DHT_readTemperature(false, true);

  // Read temperature as Fahrenheit (isFahrenheit = true)
  //delayms(2000);
  //float f = DHT_readTemperature(true, true);

  // Check if any reads failed and exit early (to try again).
  //if (isnan(h) || isnan(t) || isnan(f)) {
  //  printf("Failed to read from DHT sensor!\r\n");
  //  return;
  //}

  // Compute heat index in Celsius (isFahreheit = false)
  float hic = DHT_computeHeatIndex(t, h, false);

  // Compute heat index in Fahrenheit (the default)
  //float hif = DHT_computeHeatIndex(f, h, true);
  printf("Humidity = %d percent, ", h);//%s ",float2str(h));
  printf("Temperature= %d degree/C \r\n", t);//%s C ", float2str(t));//printf("Temperature: %u°C(%u°F)", t, f);
  //printf("Heat index: = %u C\r\n",hic);//,hif);
  delayms(2000);
  }
}
#if 0
//===
// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayus;

void setup() {
  Serial.begin(9600);
  // Initialize device.
  dht.begin();
  printfln(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  printfln(F("------------------------------------"));
  printfln(F("Temperature Sensor"));
  printf  (F("Sensor Type: ")); printfln(sensor.name);
  printf  (F("Driver Ver:  ")); printfln(sensor.version);
  printf  (F("Unique ID:   ")); printfln(sensor.sensor_id);
  printf  (F("Max Value:   ")); printf(sensor.max_value); printfln(F("°C"));
  printf  (F("Min Value:   ")); printf(sensor.min_value); printfln(F("°C"));
  printf  (F("Resolution:  ")); printf(sensor.resolution); printfln(F("°C"));
  printfln(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  printfln(F("Humidity Sensor"));
  printf  (F("Sensor Type: ")); printfln(sensor.name);
  printf  (F("Driver Ver:  ")); printfln(sensor.version);
  printf  (F("Unique ID:   ")); printfln(sensor.sensor_id);
  printf  (F("Max Value:   ")); printf(sensor.max_value); printfln(F("%"));
  printf  (F("Min Value:   ")); printf(sensor.min_value); printfln(F("%"));
  printf  (F("Resolution:  ")); printf(sensor.resolution); printfln(F("%"));
  printfln(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayus = sensor.min_delay / 1000;
}

void loop() {
  // Delay between measurements.
  delayus(delayus);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    printfln(F("Error reading temperature!"));
  }
  else {
    printf(F("Temperature: "));
    printf(event.temperature);
    printfln(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    printfln(F("Error reading humidity!"));
  }
  else {
    printf(F("Humidity: "));
    printf(event.relative_humidity);
    printfln(F("%"));
  }
}



//extern boolean g_I2C_InitDone;
extern unsigned char g_bI2CModuleConfigDone;
void stmAp3216Loop (void)
{
    int byteh;
    int bytel;
    int var;
    u8 retVal;
    //float output;
    unsigned short output;
	u8 i;
	u8 rxbuf[20];
	u8 txbuf[2];
	u8 ir_ps_InValid;
	u8 ints;
	unsigned short ird, msb, lsb;
	u8 irvalid;
	u8 mid,pid, rid;

	printf("stmAp3216 Light Sensor Test with I2C1\r\n");

	//LED
	//stmUser_LED_GPIO_setup(); //Accessory LED

	//stmConfirmLEDBlink();

	delayus(100);

	stmAp3216_Init(1,1);//enable ALS, and PS

	/* LED CUrrent = 60mA
	 * * INTEGRATION TIME=300usec
	 * THUP{MSB}=0x8F
	 * THUP_LSB = 0xFF
	 * THLOMSB=0x70
	 * THLOLSB=0x99
	 * FILTER : OFF

	 * PS INTERVAL WAIT = 50ms
	 * ENABLE
	 *
	 */

	//Clear IRQ
	//stmAp3216CheckAndClrIRQ();
	//Reg10 - default
	//stmAp3216Sendbuf[0] =  Ap3216_ALSCONF_REG10H; //Reg10
	//stmAp3216Sendbuf[1] = 0x00; //default : ALS DynamicRange=20661 Lux. ALS Persist = 0000(ALS Int after 1 conversion time)
	//stm_I2C_SendBurst(stmAp3216ADDR8, &stmAp3216Sendbuf[0], 2);

	printf("...\r\n");


    while(1){

		Ap3216_Show_Lux_api();
		Ap3216_Show_PsValue_api();
		delayus(10);
/*
		 //IRQ Based
    	retVal = stmAp3216_CheckIrqThenServeIt();
    	if(retVal > 0){
    		Ap3216_Show_Lux_api();
    		Ap3216_Show_PsValue_api();
    		printf("%u\r\n",retVal);

        	printf("PS>HIGHTH=%d\r\n",Ap3216_Get_PsHighThresholdFromLocalCache());
        	printf("PS>LOWTH=%d\r\n",Ap3216_Get_PsLowThresholdFromLocalCache());
    	}


    	//Ap3216_show_ALS_LOW_thres_api();
    	//Ap3216_show_ALS_HIGH_thres_api();
*/


/*
    	ints = stmAp3216CheckAndClrIRQ();//IRQ Status Read
    	if(ints != 0){

    		//get ALS
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_ALSdataLow_Reg0CH, &rxbuf[0], 1);
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_ALSdataHigh_Reg0DH, &rxbuf[1], 1);
    		lsb = rxbuf[0];
    		msb = (unsigned short)rxbuf[1]*256;
    		printf("ALS(12/13)=%d (0x%02x,0x%02x)\r\n",msb + lsb, rxbuf[0], rxbuf[1]);
    		delayus(10);

    		//get IR
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_IRdataLow_Reg0AH, &rxbuf[0], 1); //L
    		stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_IRdataHigh_Reg0BH, &rxbuf[1], 1); //H
    		msb = rxbuf[1];
    		lsb = rxbuf[0];
    		ird = msb*4 + (lsb & 0x03);
    		ir_ps_InValid = (lsb & 0x80) >> 7;

    		//if(ir_ps_InValid){
    		//	printf("IR/PS : Invalid\r\n");
    		//}else{
    			//get ps
    			stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataLow_Reg0EH, &rxbuf[0], 1); //LOW+HIGH
    			stm_I2C_ReceiveBurstWithRestartCondition(stmAp3216ADDR8, Ap3216_PSdataLow_Reg0EH, &rxbuf[1], 1); //LOW+HIGH
    			msb = rxbuf[1];
    			lsb = rxbuf[0];
    			printf("IR(10/11)=%d (IV=%d)\r\n", ird, ir_ps_InValid );
    			printf("PS(14/15)=%d/OVF=%d){[14L]0x%02x,[15H]0x%02x}\r\n",((msb & 0x3f) * 16) + (lsb & 0x0F) , (lsb & 0x40)>>6, lsb, msb);
    			delayus(300);
    		//}
    		 *

    		printf("================\r\n");
    	}
    	*/
        delayus(10);
    }
}



#endif
