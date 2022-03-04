#include <Arduino.h>
#include <SoftwareWire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <Rtc_Pcf8563.h>
#include <PinChangeInterrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "LowPower.h"
#include "ADS1114.h"
#include "WSSFM10RAT.h"
/**************************************************Miscellaneous I/O Defines*******************************************************/
#define DBG_LED                         11          //Debug LED Pin        
#define VSENSOR_ARR_EN                  5           //Enable analogue voltage supply Pin
#define ALERT_RDY                       8           //ADS11114 ALERT/RDY Pin
#define WSSFM10RAT_RESET_Pin            9           //WSSFMR10RAT Reset Pin
#define WSSFM10RAT_WAKEUP_Pin           13          //WSSFM10RAT  Wakeup Pin
#define MAX_CT_SENSORS                  16          //Maximum number of CT sensors per unit
#define SIGFOX_MSG_SIZE                 12          //Number of bytes in sigfox message
#define SENSOR_ARRAY_MESSAGE_SIZE       24          //Two 12-bytes messages are required to transmit sensor array readings
#define HEART_BEAT_MESSAGE_SIZE         10          //Heart beat messages consist of two float variables & a uint16_t variable -> 4+4+2 = 10
#define SENSOR_TYPE_MESSAGE             4           //4-Byte message containing sensor type of each sensor in the array
#define SAMPLES_PER_ACQUISITION_CYCLE   800         //Number of time each CT sensor will be sampled in each acquisition cycle
#define ACQUISITIONS_PER_HOUR_1S_SLEEP  555         //Number of acquisitions per hour when unit sleeps for 1s every time sensor array is sampled
#define ACQUISITIONS_PER_HOUR_2S_SLEEP  480         //Number of acquisitions per hour when unit sleeps for 2s every time sensor array is sampled
#define ACQUISITIONS_PER_HOUR_4S_SLEEP  20          //Number of acquisitions per hour when unit sleeps for 4s every time sensor array is sampled
#define ACQUISITIONS_PER_HOUR_8S_SLEEP  265         //Number of acquisitions per hour when unit sleeps for 8s every time sensor array is sampled
#define SAMPLING_TIME_PER_SENSOR        392         //Sampling time taken by a sensor for a single acquisition cycle
#define HOURS_PER_DAY                   24          
#define SLEEP_TIME                      2           // 0 = 1s ,1 = 2s,2 = 4s,3 = 8s
#define CONFIG_Pin                      PB2         //I/O used to put device into config mode -> Active low.Insert Jumper between MOSI & GND  
#define WAIT_FOR_SERIAL                 30000U

#define  ADC_REF                        3.0         //Please change to 3.3 if the default 3.3v reference is used
#define  MAX_STEPCOUNT                  1023        //1023 Steps for 10-bit ADC
#define  STEPSIZE                       ADC_REF/MAX_STEPCOUNT      //Volts/Step

#define  V_SOURCE_MONITOR_ENABLE_PIN    12          //I/O Pin used to enable adapter voltage monitoring line
#define  V_SOURCE_MONITOR_ANALOG_IN     A5          //Analog input used to measure adapter voltage input
#define  V_SOURCE_MONITOR_DIVIDER_RATIO 5.60f       //Inverse resistor divider ratio used to compute adapter voltage
#define  V_SOURCE_MONITOR_MIN_VOLTAGE   3.4f       //Minimum value that adapter voltage should have

#define  V_BATT_MONITOR_ENABLE_PIN      6           //I/O Pin used to enable battery voltage monitoring line
#define  V_BATT_MONITOR_ANALOG_IN       A4          //Analog input used to measure battery voltage input
#define  V_BATT_MONITOR_DIVIDER_RATIO   2.00f       //Inverse resistor divider ration used to compute battery voltage

#define  RTC_GPIO_PWR_PIN               4           //I/O pin used to provide power rtc module
/**************************************************Delay Defines*******************************************************************/
#define PWR_STABILIZATION_DELAY         100         //Delay used when pwr rails are switched on to allow stabilization
/**************************************************ADS1114 Defines*****************************************************************/
#define I2C_SCL_Pin                     2          //Software SCL Pin
#define I2C_SDA_Pin                     3          //Software SDA Pin
#define I2C_CLK_SPEED                   400000U    //I2C Clock Speed
#define ADS1114_I2C_ADD                 0x48       //ADS1114 I2C address
#define ADS1114_LOW_THRESH_DRDY_VAL     0x0000     //ADS1114 Low Threshold value required for ALERT Pin to generate interrupt after every conversion
#define ADS1114_HIGH_THRESH_DRDY_VAL    0x8000     //ADS1114 High Threshold value required for ALERT Pin to generate interrupt after every conversion
/**************************************************E-Monitor Config Defines********************************************************/
#define E_MONITOR_CONFIG_START_ADDR     0x00
/*********************************************CD74HC4067SM 16-to-1 Mux Defines*****************************************************/
#define CD74HC4067SM_MUX0               A3
#define CD74HC4067SM_MUX1               A2
#define CD74HC4067SM_MUX2               A1
#define CD74HC4067SM_MUX3               A0

#define CD74HC4067SM_ENABLE_PIN         2
#define CD74HC4067SM_ENABLE_PORT        PORTE

#define CD74HC4067SM_MUXSEL_PORT        PORTF

#define _HWB_H() (CD74HC4067SM_ENABLE_PORT |= (1<<CD74HC4067SM_ENABLE_PIN))
#define _HWB_L() (CD74HC4067SM_ENABLE_PORT &= ~(1<<CD74HC4067SM_ENABLE_PIN))

void CD74HC4067SM_Init();
void CD74HC4067SM_Enable();
void CD74HC4067SM_Disable();
void CD74HC4067SM_SelectChannel(uint8_t ChannelNo);
/******************************************************Enums***********************************************************************/
/*Enum used to describe three types of SCT current sensors used by the device*/
typedef enum
{
  SCT_25A  = 0,
  SCT_50A  = 1,
  SCT_100A  = 2,
  SCT_160A = 3
} CT_SENSOR_TYPE;

/*Enum used to specify input voltage source to monitor*/
typedef enum
{
  ADAPTER_PWR = 0,
  BATTERY_PWR = 1,
  USB_PWR = 2
}INPUT_PWR_SOURCE;
/***************************************************Structs & Typedefs*************************************************************/
/*Struct used to describe SCT current sensors*/
struct CT_SENSOR
{
  unsigned IrmsSteps : 12;
  unsigned SensorType : 2;
  bool     Enabled;
  float    CalibrationConstant;
  float    Irms;
  float    Isigma;
};

/*Struct used to configure each CT Sensor in Array*/
struct CT_Sensor_Config
{
  unsigned SensorType:2;
  float    CalibrationConstant;
  bool     Enabled;  
};

/*24-bit variable that will be used to bit-pack Irms readings from pairs of CT sensors*/
typedef union {
  uint32_t bits : 24;
} __attribute__((packed)) uint24_t;
/*************************************************EnergyMonitor Defines************************************************************/
#define V_BIAS                         1640U                                                //DC bias applied to CT sensors
/*************************************************Linearization Constants**********************************************************/
#define  GRADIENT                     1.036979f
#define  Y_INTERCEPT                  0.3153813f
/************************************************Global Variables******************************************************************/
ADS1114 myADC(ADS1114_I2C_ADD, ALERT_RDY);                                                  //Create ADS1114 Instance
WSSFM10RAT myWisol(&Serial1, WSSFM10RAT_RESET_Pin, WSSFM10RAT_WAKEUP_Pin);                  //Create instance of WSSFM10RAT module
Rtc_Pcf8563 RTC;                                                                            //Create instance of PCF8563 RTC

const uint16_t  SleepDurationPeriod[10]  = {15, 30, 60, 120, 250, 500, 1000, 2000, 4000, 8000}; //Array containing all possible sleep periods in milliseconds               
CT_Sensor_Config  ctSensor_Config[MAX_CT_SENSORS] = {0};                                    //Array of 16 CT sensor config objects
CT_SENSOR ctSensors[MAX_CT_SENSORS] = {0};                                                  //Array of 16 CT sensor objects
float    V_Bias = 0 ;                                                                       //Bias voltage applied to all CT Sensors in millivolts,read from E2PROM
uint24_t ctSensorPairs[MAX_CT_SENSORS / 2] = {0};                                           //Array of eight 24-bit variables
uint8_t  transmitArr[SENSOR_ARRAY_MESSAGE_SIZE] = {0};                                      //Array used to hold data to be transmitted to backend
uint8_t  NumberOfEnabledSensors = 0;                                                        //Number of enabled sensors
uint16_t AcquisitionCount = 0;                                                              //Used to track number of acquisition cycles completed
uint8_t  TransmissionsCount = 0;                                                            //Used to track number of transmissions
uint8_t  HourCount = 0;                                                                     //Used to track number of hours passed
uint8_t  TransmissionsPerHour = 1;                                                          //Number of messages sent to the sigfox backend per hour (1-6)
uint8_t  SleepDurationIndex = 0;                                                            //Sleep duration per acquisition cycle
uint16_t AcquisitionsPerWindow = 0;                                                         //Number of acquisitions per window    
bool     ConfigMode = false;                                                                //Used to check if config mode is enabled
bool     Debug_LED_Enabled = true;                                                          //Used enable/disable debug LED
/*************************************************Private function prototypes******************************************************/
void    ADS1114_AlertRdy_ISR();                                                             //ISR callback function for  ADS1114
uint8_t Irms_Calculate(float *Irms, uint16_t SamplesToTake, float CalibrationConstant);     //Calculate RMS current of SCT current sensors
float   Irms_OffsetCorrection(float Irms);                                                  //Used to linearize CT sensor response

void    E_Monitor_Create_Irms_Msg(CT_SENSOR *ctSensorArr, uint8_t *transmitArr);                //Used to packetize RMS Current readings for transmission to backend
void    E_Monitor_CreateSensorTypeMsg(CT_Sensor_Config *ctSensor_Config, uint8_t *transmitArr);  //Used to packetize sensor types for transmission to backend
void    E_Monitor_CreateHeartBeatMsg(uint8_t *transmitArr);

void    E_Monitor_CT_Array_Isigma_Update(CT_SENSOR *ctSensorArr);                                                                                            //Calculate sum of Irms values
void    E_Monitor_WriteConfig(CT_Sensor_Config *ctSensor_Config, uint8_t *TransmissionsPerHour, uint8_t *SleepPeriod,float *V_Bias,bool *Dbg_LED_Enabled);  //Write config details for each CT Sensor to EEPROM
void    E_Monitor_ReadConfig(CT_Sensor_Config *ctSensor_Config, uint8_t *TransmissionsPerHour, uint8_t *SleepPeriod,float *V_Bias,bool *Dbg_LED_Enabled);   //Read config details for each CT Sensor from EEPROM

void    E_Monitor_SetTransmissionsPerHour();                                                                            //Set the number of transmissions per hour
void    E_Monitor_SetSleepPeriod(uint8_t *SleepPeriod);                                                                 //Set the sleep period for each acquisition cycle                                                                              

void    E_Monitor_PrintMainMenu();
char    E_Monitor_MainMenu_ProcessInput();

void    E_Monitor_Print_SensorConfig_Menu();
bool    E_Monitor_IsConfigModeEnabled();
void    E_Monitor_Print_SensorConfig_SubMenu();
char    E_Monitor_SensorConfigSubMenu_ProcessInput();
uint8_t E_Monitor_GetSensorNumber();
uint8_t E_Monitor_NumberOfEnabledDevices(CT_Sensor_Config  *ctSensor_Config);
void    E_Monitor_PrintDeviceDetails();


void    E_Monitor_PrintSensorType_SubMenu(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number);
char    E_Monitor_SensorTypeSubMenu_ProcessInput(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number);

void    E_Monitor_SetCalibrationConstant(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number);

void    E_Monitor_SensorEnable(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number);
void    E_Monitor_SensorDisable(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number);

float   E_Monitor_GetPwrSourceVoltage(INPUT_PWR_SOURCE InputPwr);

void    E_Monitor_UpdateBiasVoltage(float  *V_Bias);

void    E_Monitor_Dbg_LED_Menu();
void    E_Monitor_LED_Fade(uint8_t NoOfFades);
/**********************************************************************************************************************************/

void setup()
{
 
  /*Configure the debug LED I/O*/
  pinMode(DBG_LED, OUTPUT);

  /*Configure enable pin for analogue voltage supply*/
  pinMode(VSENSOR_ARR_EN, OUTPUT);

  /*Enable analogue voltage regulator*/
  digitalWrite(VSENSOR_ARR_EN, HIGH);

  /*Enable Serial1 for comms with WSSFM10RAT*/
  Serial1.begin(9600);

  /*Enable Serial out for debug*/
  Serial.begin(115200);

  /*Power stabilization delay*/
  delay(PWR_STABILIZATION_DELAY);

  /*Initialize ADS1114 & configure with desired settings*/
  if (myADC_ConfigureForAcquisition() == 0)
  {
    LED_Blink(10, 250);
  }

  /*Initialize multiplexer*/
  CD74HC4067SM_Init();

  /*Enable Multiplexer*/
  CD74HC4067SM_Enable();

  /*Initialize Wisol Module*/
  if (myWisol.begin() == 0)
  {
    LED_Blink(10,500);
  }

  /*Perform reset & wake-up*/
  myWisol.Reset();

  /*Wait for Serial terminal to open for 30 seconds on power-up*/
  uint32_t SerialTimeOut = millis() + WAIT_FOR_SERIAL;
  
  while (SerialTimeOut > millis())
  {
    if(Serial)
    {
      ConfigMode = true;
      break;
    }
  }

  /*Read config*/
  E_Monitor_ReadConfig(ctSensor_Config,&TransmissionsPerHour,&SleepDurationIndex,&V_Bias,&Debug_LED_Enabled);

  if(ConfigMode)
  {
    /*Print device details*/
    E_Monitor_PrintDeviceDetails();
         
    /*Print config menu*/
    E_Monitor_PrintMainMenu();
  }

  /*Copy config info to ctSensor object array*/
  for(uint8_t  i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
    ctSensors[i].SensorType = ctSensor_Config[i].SensorType;
    ctSensors[i].CalibrationConstant = ctSensor_Config[i].CalibrationConstant;
    ctSensors[i].Enabled = ctSensor_Config[i].Enabled;
  }

  /*Determine number of enabled devices*/
  NumberOfEnabledSensors = E_Monitor_NumberOfEnabledDevices(ctSensor_Config);

  /*Create heartbeat message*/
  E_Monitor_CreateHeartBeatMsg(transmitArr);

  /*Transmit Heart beat message*/
  if (myWisol.TransmitData(transmitArr,10) == 0){}

  /*Write new config*/
  E_Monitor_WriteConfig(ctSensor_Config,&TransmissionsPerHour,&SleepDurationIndex,&V_Bias,&Debug_LED_Enabled); 
  
  /*Determine number of acquisitions per sampling window*/
  E_Monitor_CalculateAcquisitionsPerWindow(NumberOfEnabledSensors,&AcquisitionsPerWindow,&TransmissionsPerHour,&SleepDurationIndex);
     
  /*Put Wisol to sleep*/
  if (myWisol.Sleep() == 0)
  {
    LED_Blink(10, 500);
  }

  /*Fade LED 10 times before starting acquisitions*/
  E_Monitor_LED_Fade(10);
  
  /*Switch off debug LED*/
  digitalWrite(DBG_LED, LOW);
  
  /*Disable USB*/
  E_Monitor_USB_Disable();
}

void loop()
{  
  /*Update Sigma(sum of Irms)*/
  E_Monitor_CT_Array_Isigma_Update(ctSensors);
 
  /*Increment acquisition count*/
  AcquisitionCount++;

  /*Check if required number of cycles per sampling Window has been reached*/
  if (AcquisitionCount ==  AcquisitionsPerWindow)
  {
    /*Compute average RMS current at the end of sampling window*/
    E_Monitor_Irms_Update(ctSensors);
    
    /*Create Sigfox message from measured current*/
    E_Monitor_Create_Irms_Msg(ctSensors, transmitArr);

    /*Set Reset and wake-up pins to outputs again*/
    pinMode(WSSFM10RAT_RESET_Pin,OUTPUT);
    pinMode(WSSFM10RAT_WAKEUP_Pin,OUTPUT);
    
    /*Enable Power to USART1 for comms with WSSFM10R*/
    power_usart1_enable();
 
    /*Wake wisol module up*/
    myWisol.Wakeup();
    myWisol.Reset();

    /*Clear transmitArr*/
    memset(transmitArr, 0, SENSOR_ARRAY_MESSAGE_SIZE * sizeof(uint8_t));

    /*Transmit Data to backend,switch LED ON to indicate this*/
    if(Debug_LED_Enabled){digitalWrite(DBG_LED, HIGH);}

    /*Transmit Messsage 1*/
    if (myWisol.TransmitData(transmitArr,12) == 0){}

    /*Transmit Message 2*/
    if (myWisol.TransmitData(transmitArr + 12,12) == 0){}

    /*Transmission to backend complete,switch LED off to indicate this*/
    digitalWrite(DBG_LED, LOW);

    /*Clear transmitArr*/
    memset(transmitArr, 0, SENSOR_ARRAY_MESSAGE_SIZE * sizeof(uint8_t));

    /*Increase transmissions count*/
    TransmissionsCount++;

    /*Determine if an hour has passed*/
    if(TransmissionsCount == TransmissionsPerHour)
    {
      /*Increment hour count*/
      HourCount++;

      /*Set transmission count to zero*/
      TransmissionsCount = 0;
    }

    /*If 24 hours have passed then we send a heartbeat message as well*/
    if(HourCount == HOURS_PER_DAY)
    {
      /*Create Heart beat message*/
      E_Monitor_CreateHeartBeatMsg(transmitArr);

      /*Transmit Heart beat message*/
      if (myWisol.TransmitData(transmitArr,10) == 0){}

      /*Set hour count to zero*/
      HourCount = 0;
    }

      /*Set acquisition count to zero*/
      AcquisitionCount = 0;
     
     /*Put Wisol to sleep*/
     if (myWisol.Sleep() == 0) {}
  }

  /*Check if device needs to go to sleep*/
  if(SleepDurationIndex != 0)
  {
    /*Put E-monitor to sleep*/
    E_Monitor_Sleep();
  
    /*Wake-up from sleep*/
    E_Monitor_Wake();
  }
}

/*@brief      This function sums the RMS current measured by each CT sensor during each sampling interval occuring within the hourly window
* @param[in]  ctSensor : Array of ctSensor objects that characterizes the physical CT sensor array
* @return     none
*/
void E_Monitor_CT_Array_Isigma_Update(CT_SENSOR *ctSensorArr)
{
  /*Acquire Irms measurement from each sensor in array*/
  for (uint8_t  i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
      if(ctSensorArr[i].Enabled)
      {
        /*Select multiplexer channel*/
        CD74HC4067SM_SelectChannel(i);

        /*Clear conversion buffer*/
        Irms_Calculate(&ctSensorArr[i].Irms, 1, ctSensorArr[i].CalibrationConstant);
  
        /*Set Irms to zero before performing actual measurement*/
        ctSensorArr[i].Irms= 0;
  
        /*Compute Irms*/
        Irms_Calculate(&ctSensorArr[i].Irms, 800, ctSensorArr[i].CalibrationConstant);

        /*Apply Linearization curve*/
        ctSensorArr[i].Irms = Irms_OffsetCorrection(ctSensorArr[i].Irms);
  
        /*Sum measured current*/
        ctSensorArr[i].Isigma += ctSensorArr[i].Irms;
     }
  }
}

/*@brief      This function is computes the average RMS current measured over the hourly interval
* @param[in]  ctSensor : Array of ctSensor objects that characterizes the physical CT sensor array
* @return     none
*/
void E_Monitor_Irms_Update(CT_SENSOR *ctSensorArr)
{
  /*Compute average RMS current over the hour*/
  for (uint8_t  i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
    /*Calculate average Irms current over the hour*/
    ctSensors[i].Irms = ctSensors[i].Isigma/ AcquisitionsPerWindow;
    
    /*Reset total in preparation for next window/hour*/
    ctSensors[i].Isigma = 0;
  }
}

/*@brief        This functions computes the rms current from a set of instantaneous current measurements as measured by a CT Sensor
* @param[out]   Irms : Pointer to float value used hold the computed rms current 
* @param[in]    SamplesToTake:Number of instantaneous current measurements to take to compute RMS current
* @param[in]    CalibrationConstant:Calibration constant for CT Sensor used
* @return       0 if function computes rms current successfully
*/
uint8_t Irms_Calculate(float *I_rms, uint16_t SamplesToTake, float CalibrationConstant)
{
  /*Local variable used to track number of samples collected*/
  uint16_t SampleCount = 0;

  /*Local variables used to compute RMS current*/
  float    Isquared = 0;
  float    Isample = 0;
  float    Ifiltered = 0;
  float    Isum = 0;

  /*Collect specified number of samples*/
  while (SampleCount < SamplesToTake)
  {
    if (myADC.GetConversionResult(&Isample) == 0)
    {
      Ifiltered = (Isample - V_Bias);
      Isquared = Ifiltered * Ifiltered;
      Isum += Isquared;
      SampleCount++;
    }
  }

  /*Compute RMS Current*/
  *I_rms = (sqrt(Isum / SampleCount)) * CalibrationConstant;

  return 0;
}

/*@brief This function is used to linearize behaviour of CT Sensors*/
float Irms_OffsetCorrection(float Irms)
{
  const float Epsilon = 0.01f;
  float delta = Irms - Y_INTERCEPT;

  if((signbit(delta) == 0) && (fabs(Irms) > Y_INTERCEPT))
  {
     return (GRADIENT * Irms) - Y_INTERCEPT;
  }
  
  return 0;
}
/************************************************************************************************************ADC_FUNCTIONS**********************************************************************************/
/*@brief Configures   ADS1014 with settings required to sample CT Sensors
* @param[in/out]      none
* @return             0 if ADS1014 is successfully configured for CT Sensor measurement
*                     1 if configuration fails
*/
uint8_t myADC_ConfigureForAcquisition()
{
  /*Enable I2C bus*/
  Wire.begin();

  /*Set clock speed of I2C bus as fast as possile,lowest prescaler*/
  TWBR = 1;

  /*Allow i2c bus to initialize*/
  delay(10);

  /*Initialize ADS1114*/
  if (myADC.begin() != 0)
  {
    return 1;
  }

  /*Set sample rate to 2400SPS*/
  if (myADC.SetSampleRate(DR_2400SPS) != 0)
  {
    return 1;
  }

  /*Set PGA setting to 2, allows input voltage range from 0-4.096v with a resolution of 125 μV*/
  if (myADC.SetPGA(PGA2) != 0)
  {
    return 1;
  }


  /*Set Mode to continuous conversion mode*/
  if (myADC.SetMode(ADS1114_CONTINUOUS_CONVERSION) != 0)
  {
    return 1;
  }

  /*Set comparator to generate positive going pulse after every conversion*/
  if (myADC.SetComparator(ASSERT_ONE_CONV) != 0)
  {
    return 1;
  }

  /*Set Low threshold & high threshold values*/
  if (myADC.SetHighThreshold(0x8000) != 0)
  {
    return 1;
  }

  if (myADC.SetLowThreshold(0x0000) != 0)
  {
    return 1;
  }

  /*Set callback function to handle Alert Ready callback */
  myADC.SetAlertRdy_ISR_Callback(ADS1114_AlertRdy_ISR);

  return 0;
}

void ADS1114_AlertRdy_ISR()
{
  /*Data ready flag to true*/
  myADC.AlertRdy_ISR_Handler();
}
/**************************************************************************************************MULTIPLEXER FUNCTIONS**************************************************************************************/
/* @brief This function is used to select different channels that feed into 16x1 mux
 * @param[in] ChannelNo : Number of desired channel [0 - 15]
 * @return    none
 */
void CD74HC4067SM_SelectChannel(uint8_t ChannelNo)
{
  /*Shift by 4 bits to the right to correspond to physical pin mapping*/
  uint8_t ChannelNumber = (ChannelNo << 4);

  /*Set required port pins*/
  PORTF = ChannelNumber;
}

/*@brief        This function is used to initialize the CD74HC4067 mux 
 *@param[in]    none
 *@return       none
 */
void CD74HC4067SM_Init()
{

  //Set enable pin high before initializing it
  _HWB_H();
  digitalWrite(CD74HC4067SM_MUX0, LOW);
  digitalWrite(CD74HC4067SM_MUX1, LOW);
  digitalWrite(CD74HC4067SM_MUX2, LOW);
  digitalWrite(CD74HC4067SM_MUX3, LOW);


  //Initialize enable and channel select pins
  DDRE |= (1 << CD74HC4067SM_ENABLE_PIN);
  pinMode(CD74HC4067SM_MUX0, OUTPUT);
  pinMode(CD74HC4067SM_MUX1, OUTPUT);
  pinMode(CD74HC4067SM_MUX2, OUTPUT);
  pinMode(CD74HC4067SM_MUX3, OUTPUT);

  //Set channel to zero by default
  digitalWrite(CD74HC4067SM_MUX0, LOW);
  digitalWrite(CD74HC4067SM_MUX1, LOW);
  digitalWrite(CD74HC4067SM_MUX2, LOW);
  digitalWrite(CD74HC4067SM_MUX3, LOW);
}

/*@brief This function is used to enable CD74HC4067*/ 
void CD74HC4067SM_Enable()
{
  _HWB_L();
}

/*@brief This function is used to disable CD74HC4067*/ 
void CD74HC4067SM_Disable()
{
  _HWB_H();
}

void CD74HC4067SM_Reset(uint8_t ChannelNo)
{
  /*Shift by 4 bits to the right to correspond to physical pin mapping*/
  uint8_t ChannelNumber = (ChannelNo << 4);

  /*Bring PORTF back to its initial state*/
  PORTF &= (~ChannelNumber);
}
/***************************************DEBUG_FUNCTIONS**************************************************************************/
/*@brief      This function is used to blink debug LED for a specified number of times and period
* @param[in]  NoOfBlinks : Number of times to blink the debug led
* @param[in]  BlinkTime_Ms: The period for which each blink must last
* @return     none
*/
void LED_Blink(uint8_t NoOfBlinks, uint16_t BlinkTime_Ms)
{

  /*Check if Debug LED is enabled before blinking*/
  if(Debug_LED_Enabled)
  {
    for (uint8_t i = 0 ; i < NoOfBlinks ; i++)
    {
      digitalWrite(DBG_LED, !digitalRead(DBG_LED));
      delay(BlinkTime_Ms);
    }
  }
}

void  E_Monitor_LED_Fade(uint8_t NoOfFades)
{
  for(uint8_t  i = 0; i < NoOfFades ;i++)
  {
    for(int i=0; i<255; i++)
    {
      analogWrite(DBG_LED, i);
      delay(5);
    }

    for(int i=255; i>0; i--)
    {
      analogWrite(DBG_LED, i);
      delay(5);
    }
  }
}
/*******************************************************************SLEEP_FUNCTIONS**********************************************/
/*@brief This function disables,de-initializes ext. and internal peripherals before putting the E-Monitor into deep-sleep*/
void E_Monitor_Sleep()
{
  /*Disable Multiplexer*/
  CD74HC4067SM_SelectChannel(0);
  CD74HC4067SM_Enable();
    
  /*Dettach pinChange interrupt used for Alert/Data Ready functionality of ADS1114*/
  myADC.DettachAlertRdy_Interrupt();

  /*Set ALERT RDY Pin as input to prevent ISR from interfering with device before it goes to sleep*/
  pinMode(ALERT_RDY, INPUT);
  delay(2);

  /*Turn of Analogue voltage regulator as a precautionary measure*/
  digitalWrite(VSENSOR_ARR_EN, LOW);
  delay(2);

  /*Set WSSFM10RAT RESET & WAKEUP Pins as inputs to prevent current draw through I/Os during sleep*/
  pinMode(WSSFM10RAT_RESET_Pin, INPUT);
  pinMode(WSSFM10RAT_WAKEUP_Pin, INPUT);

  /*Turn off DBG_LED as a precautionary measure*/
  digitalWrite(DBG_LED, LOW);

  /*Safety Delay*/
  delay(2);

  /*Put device to sleep for 8s*/
  LowPower.powerDown(SleepDurationIndex, ADC_OFF, BOD_OFF);

}

/*@brief Function used to enable required peripherals on wake-up from sleep*/
void E_Monitor_Wake()
{
  /*Enable Analogue Voltage regulator*/
  digitalWrite(VSENSOR_ARR_EN, HIGH);

  /*Power stabilization delay*/
  delay(PWR_STABILIZATION_DELAY);

  /*Configure ADS1114 for acquisition after sleep*/
  if (myADC_ConfigureForAcquisition() == 0)
  {
    LED_Blink(10,250);
  }
}

/*@brief This function is used to disable the USB peripheral*/
void E_Monitor_USB_Disable()
{
   USBCON &= ~(1 << USBE); 
   USBCON &= ~(1 << VBUSTE); 
   USBCON &= ~(1 << OTGPADE); 
   USBCON &= ~(1 << FRZCLK);  
   UHWCON &= ~(1 << UVREGE); 
   USBINT &= ~(1 << VBUSTI); 
   UDCON |= (1 << DETACH);
}
/*******************************************************************BIT_PACKING & MSG CREATION****************************************/

/*@brief      This function is used to pack rms current readings from CT Sensors into a byte array for transmission to backend
* @param[in]  ctSensorArr : Pointer to array that defines each CT Sensor in array 
* @param[in]  transmitArr : Pointer to array that contains packed data to be transmitted
* @return     none
*/
void E_Monitor_Create_Irms_Msg(CT_SENSOR *ctSensorArr, uint8_t *transmitArr)
{
  /*Convert rms current readings from sensor array to digital steps*/
  for (uint8_t i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
    /*Variable to store maximum current based on sensor type*/
    uint16_t MaxCurrent = 0;

    switch (ctSensorArr[i].SensorType)
    {
      case SCT_25A:
        MaxCurrent = 25;
        break;

      case SCT_50A:
        MaxCurrent = 50;
        break;

      case SCT_100A:
        MaxCurrent = 100;
        break;

      case SCT_160A:
        MaxCurrent = 160;
        break;

      default:
        break;
    }

    /*Determine Resolution based on stepcount*/
    float Resolution = MaxCurrent / (pow(2, 12) - 1);

    /*Compute digitalSteps*/
    ctSensorArr[i].IrmsSteps =  (ctSensorArr[i].Irms) / Resolution;
  }

  /*Bit pack into 24-bit variables & split into 8-byte array for transmission*/
  for (uint8_t  i = 0 ; i < MAX_CT_SENSORS / 2 ; i++)
  {
    ctSensorPairs[i].bits  =   ctSensorArr[2 * i].IrmsSteps;
    ctSensorPairs[i].bits = ctSensorPairs[i].bits << 12;
    ctSensorPairs[i].bits |= ctSensorArr[2 * i + 1].IrmsSteps;
    

    transmitArr[3 * i + 2] = ctSensorPairs[i].bits ;
    transmitArr[3 * i + 1] = ctSensorPairs[i].bits >> 8 ;
    transmitArr[3 * i] =   ctSensorPairs[i].bits >> 16;
  }

  /*Set LSB of  1st 12-byte message to 0 & LSB of 2nd 12-byte message to 1*/
  transmitArr[11] = (transmitArr[11] & 0xfe);
  transmitArr[23] = (transmitArr[23] | 0x01);

}

/*@brief      This function is used to pack the CT Sensor Type of each sensor into a 4-byte array for transmission to backend
* @param[in]  ctSensorArr : Pointer to array that defines each CT Sensor in array 
* @param[in]  transmitArr : Pointer to array that contains packed data to be transmitted
* @return     none
*/
void E_Monitor_CreateSensorTypeMsg(CT_Sensor_Config *ctSensorArr, uint8_t *transmitArr)
{

  for(uint8_t i = 0 ; i < MAX_CT_SENSORS/4 ; i++)
  {
    transmitArr[i] = ctSensorArr[4*i].SensorType;
    transmitArr[i] = transmitArr[i] << 2;

    transmitArr[i] |= ctSensorArr[4*i + 1].SensorType;
    transmitArr[i] = transmitArr[i] << 2;

    transmitArr[i] |= ctSensorArr[4*i + 2].SensorType;
    transmitArr[i] = transmitArr[i] << 2;
  
    transmitArr[i] |= ctSensorArr[4*i + 3].SensorType;
  }
}

/*@brief      This function is used to create a heartbeat message consisting of adapter voltage,battery voltage & temperature
* @param[in]  transmitArr : Pointer to array that contains packed data to be transmitted
* @return     none
*/
void E_Monitor_CreateHeartBeatMsg(uint8_t *transmitArr)
{
  /*Clear transmit Arr*/
  memset(transmitArr,0,SENSOR_ARRAY_MESSAGE_SIZE);
  
  /*Read battery voltage*/
  float BatteryVoltage = E_Monitor_GetPwrSourceVoltage(BATTERY_PWR);
  
  /*Read Adapter voltage*/
  float AdapterVoltage = E_Monitor_GetPwrSourceVoltage(ADAPTER_PWR);
  
  /*Read Wisol Temperature*/
  int16_t WisolTemperature  = myWisol.GetTemperature()/10;
  
  /*Copy variables into array for transmit*/
  memcpy(transmitArr,&BatteryVoltage,sizeof(float));
  memcpy((transmitArr + 4),&AdapterVoltage,sizeof(float));
  memcpy((transmitArr + 8),&WisolTemperature,sizeof(int16_t));

  /*Reverse transmit array so data appears correctly on backend*/
  uint8_t ReverseArr[SENSOR_ARRAY_MESSAGE_SIZE] = {0};

  /*Reverse Array so data appears correctly on backend*/
  for(uint8_t i = 0 ; i < 10 ; i++)
  {
    ReverseArr[i] = transmitArr[10 - i -1];
  }

  memcpy(transmitArr,ReverseArr,10);
  
}

/*@brief     This function is to determine the number of acquisition cycles the device must perform in each transmission Window
 *           based on sleep settings ,number of transmission per hour and number of sensors enabled
 *@param[in] No_Of_Sensors_Enabled : The number of CT Sensors enabled (1-16)
 *@param[in] TransmissionsPerHour: The number Irms measurements to send to sigfox backend in an hour
 *@param[in] SleepTime:The number of seconds to sleep after each acquisition
 *@return    The number of acquisitions in each transmission window
 */
uint8_t E_Monitor_CalculateAcquisitionsPerWindow(uint8_t No_Of_Sensors_Enabled,uint16_t *AcquisitionsPerWindow,uint8_t *TransmissionsPerHour,uint8_t *SleepDurationIndex)
{
  /*Compute time per cycle*/
  float TimePerCycle = (float)((No_Of_Sensors_Enabled * SAMPLING_TIME_PER_SENSOR) + SleepDurationPeriod[*SleepDurationIndex])/(float)1000;

  /*Compute Transmission Window period*/
  uint16_t TransmissionWindowPeriod = (3600/(*TransmissionsPerHour));

  /*Determine number of acquisitions per hour*/
  *AcquisitionsPerWindow = TransmissionWindowPeriod/TimePerCycle;

  Serial.println(*AcquisitionsPerWindow);
}
/***********************************************************E_Monitor_Config_Functions********************************************************/
void E_Monitor_WriteConfig(CT_Sensor_Config *ctSensor_Config, uint8_t *TransmissionsPerHour, uint8_t *SleepPeriod,float *V_Bias,bool *Dbg_LED_Enabled)
{
   /*Set location where config data should be stored*/
   int E_Monitor_ConfigStartAdd = E_MONITOR_CONFIG_START_ADDR;

   /*Write Updated Config to EEPROM*/
   for(uint8_t i = 0 ; i < MAX_CT_SENSORS ; i++)
   {
     EEPROM.put(E_Monitor_ConfigStartAdd, ctSensor_Config[i]);
     E_Monitor_ConfigStartAdd += sizeof(CT_Sensor_Config);
   }

  /*E2Prom address of transmissions per hour setting*/
  int TransmissionsPerHour_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config);

  /*E2Prom address of sleeptime setting*/
  int SleepTime_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config) + sizeof(uint8_t);

  /*E2Prom address of bias voltage*/
  int V_Bias_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config) + sizeof(uint8_t) + sizeof(uint8_t);

  /*E2Prom address of debug led setting*/
  int Debug_LED_State_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(float);

  /*Write transmissions per hour setting to eeprom*/
  EEPROM.write(TransmissionsPerHour_E2PROM_Addr, *TransmissionsPerHour);

  /*Write SleepPeriod setting to eeprom*/
  EEPROM.write(SleepTime_E2PROM_Addr, *SleepPeriod);

  /*Write V_Bias value to eeprom*/
  EEPROM.put(V_Bias_E2PROM_Addr,*V_Bias);

  /*Write Debug LED state to eeprom*/
  EEPROM.put(Debug_LED_State_E2PROM_Addr,*Dbg_LED_Enabled);
}

void E_Monitor_ReadConfig(CT_Sensor_Config *ctSensor_Config, uint8_t *TransmissionsPerHour, uint8_t *SleepPeriod,float *V_Bias,bool *Dbg_LED_Enabled)
{
    /*Set location where config data IS stored*/
    int E_Monitor_ConfigStartAdd = E_MONITOR_CONFIG_START_ADDR;

    /*Read config details from EEPROM*/
    for(uint8_t  i = 0 ; i < MAX_CT_SENSORS ; i++)
    {
      EEPROM.get(E_Monitor_ConfigStartAdd, ctSensor_Config[i]);
      E_Monitor_ConfigStartAdd += sizeof(CT_Sensor_Config);
    }

    
   /*E2Prom address of transmissions per hour setting*/
   int TransmissionsPerHour_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config);

   /*E2Prom address of sleeptime setting*/
   int SleepTime_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config) + sizeof(uint8_t);

   /*E2Prom address of bias voltage*/
   int V_Bias_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config) + sizeof(uint8_t) + sizeof(uint8_t);

   /*E2Prom address of debug led setting*/
   int Debug_LED_State_E2PROM_Addr = MAX_CT_SENSORS*sizeof(CT_Sensor_Config) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(float);

   /*Read number of transmissions per hour from eeprom*/
   *TransmissionsPerHour = EEPROM.read(TransmissionsPerHour_E2PROM_Addr);

   /*Read number sleep time setting from eeprom*/
   *SleepPeriod = EEPROM.read(SleepTime_E2PROM_Addr);

   /*Read V_Bias value*/
   EEPROM.get(V_Bias_E2PROM_Addr,*V_Bias);

   /*Read debug led setting*/
   EEPROM.get(Debug_LED_State_E2PROM_Addr,*Dbg_LED_Enabled);
}
/*********************************************************E_Monitor_Config_Menu**************************************************************/
void E_Monitor_PrintMainMenu()
{
  /*Variable used to store user input*/
  char UserInput = '0';

  do
  {
    Serial.println(F("/*********************E_Monitor_Config_Menu********************************/"));
    Serial.println(F("(1)Edit CT Sensor Configuration"));
    Serial.println(F("(2)Print CT Sensor Configuration"));
    Serial.println(F("(3)Set transmissions/hour"));
    Serial.println(F("(4)Set Sleep Period"));
    Serial.println(F("(5)Update Bias Voltage"));
    Serial.println(F("(6)Enable/Disable Debug LED"));
    Serial.println(F("(7)Send Sensor Type Message"));
    Serial.println(F("(e)Exit Calibration Menu"));
    
    /*Process user input*/
    UserInput = E_Monitor_MainMenu_ProcessInput();

    /*Flush serial buffer*/
    Serial.flush();
    
  }while(UserInput != 'e');
  /*Exits menu or continues to print sub-menu*/
}

void E_Monitor_PrintDeviceDetails()
{
   /*E-Monitor Device Settings/Status*/
    Serial.println(F("*************E-Monitor V1.0 Configuration Settings/Details****************"));
    
    /*Print Battery Voltage*/
    Serial.print(F("Battery Voltage :"));
    Serial.println(E_Monitor_GetPwrSourceVoltage(BATTERY_PWR));

    /*Print Adapter voltage*/
    Serial.print(F("Adapter Voltage:"));
    Serial.println(E_Monitor_GetPwrSourceVoltage(ADAPTER_PWR));

    /*Print Bias Voltage to screen*/
    Serial.print(F("Bias Voltage:"));
    Serial.print(V_Bias);
    Serial.println(F(" mV"));

    /*Print transmissions per hour setting*/
    Serial.print(F("Transmissions/Hour :"));
    Serial.println(TransmissionsPerHour);

    /*Print Sleep Duration*/
    Serial.print(F("Sleep Duration:"));
    Serial.println(SleepDurationPeriod[SleepDurationIndex]);

    /*Debug LED Status*/
    Serial.print(F("Debug LED State:"));
    Serial.println(Debug_LED_Enabled);

    /*Print WiSOL modules details to terminal*/
    Serial.print("Sigfox ID:");
    Serial.println(myWisol.ID);

    Serial.print("Sigfox PAK:");
    Serial.println(myWisol.PAK);
}

void E_Monitor_Print_CT_SensorConfigDetails(CT_Sensor_Config  *ctSensor_Config)
{
  for(uint8_t  i = 0 ; i < MAX_CT_SENSORS; i++)
  {

    /*Print Sensor Number*/
    Serial.print(F("Sensor ["));
    Serial.print(i);
    Serial.println(F("] Configuration Details"));
    
    /*Print sensor type*/
    Serial.print(F("Sensor Type :"));
    Serial.println(ctSensor_Config[i].SensorType);

    /*Print calibration constant*/
    Serial.print(F("Calibration Constant :"));
    Serial.println(ctSensor_Config[i].CalibrationConstant);

    /*Print Sensor status i.e. enabled/disabled*/
    Serial.print(F("Sensor Enabled:"));
    Serial.println(ctSensor_Config[i].Enabled);
  }
}

char E_Monitor_MainMenu_ProcessInput()
{
  /*Variable used to store user input*/
  char UserInput = '0';

  /*Wait for user input*/
  while(Serial.available() == 0);

  /*Read byte*/
  UserInput = Serial.read();

  /*Choose appropriate sub-menu based on user-input*/
  switch(UserInput)
  {
    case '1':
    E_Monitor_PrintSensorConfig_SubMenu(ctSensor_Config);
    break;

    case '2':
    E_Monitor_Print_CT_SensorConfigDetails(ctSensor_Config);
    break;

    case '3':
    E_Monitor_SetTransmissionsPerHour(&TransmissionsPerHour);
    break;

    case '4':
    E_Monitor_SetSleepPeriod(&SleepDurationIndex);
    break;

    case '5':
    E_Monitor_UpdateBiasVoltage(&V_Bias);
    break;

    case '6':
    E_Monitor_Dbg_LED_Menu();
    break;

    case '7':
    Serial.println(F("Sending Sensor Type Message to Sigfox Backend"));
    E_Monitor_CreateSensorTypeMsg(ctSensor_Config, transmitArr);
    myWisol.TransmitData(transmitArr,4);
    break;

    case 'e':
    break;

    default:
    Serial.println(F("Invalid User Input"));
    break;
  }

  return UserInput;
}

void E_Monitor_Dbg_LED_Menu()
{
  /*Array to store user input*/
  char UserInput = '0';

  do
  {
    /*Prompt user to enable/disable debug LED*/
    Serial.println(F("**************Debug LED Menu**************"));
    Serial.println(F("(1) Enable Dbg LED"));
    Serial.println(F("(2) Disable Dbg LED"));
    Serial.println(F("(e) Exit"));

    /*Wait for serial input from user*/
    while(Serial.available()== 0);

   /*Read user input*/
    UserInput= Serial.read();

    switch(UserInput)
    {
      case '1':
      Serial.println(F("Debug LED Enabled"));
      Debug_LED_Enabled = true;
      break;

      case '2':
      Serial.println(F("Debug LED Disabled"));
      Debug_LED_Enabled = false;
      break;

      default:
      break;
    }

    
  }while(UserInput != 'e');
}

void E_Monitor_SetSleepPeriod(uint8_t *SleepPeriod)
{
    /*Array to store user input*/
  char UserInput = '0' ;

  do
  { 
    /*Prompt User to enter number of transmissions per hour*/
    Serial.println(F("Please select sleep period:"));
    Serial.println(F("(1) 0s (Sleep disabled)"));
    Serial.println(F("(2) 1s"));
    Serial.println(F("(3) 2s"));
    Serial.println(F("(4) 4s"));
    Serial.println(F("(5) 8s"));
    Serial.println(F("(e) Exit"));
       
    /*Wait for serial input from user*/
    while(Serial.available()== 0);

   /*Read user input*/
    UserInput= Serial.read();
    
    switch(UserInput)
    {
      case '1':
      *SleepPeriod = 0;
      break;

      case '2':
      *SleepPeriod = SLEEP_1S;
      break;

      case '3':
      *SleepPeriod = SLEEP_2S;
      break;

      case '4':
      *SleepPeriod = SLEEP_4S;
      break;

      case '5':
      *SleepPeriod = SLEEP_8S;      
      break;

      default:
      Serial.println(F("Invalid option selected.Please select an option between (1-5)"));
      break;
    }

  }while(UserInput == 'e');

  Serial.println(F("Sleep time changed successfully"));
}

void E_Monitor_SetTransmissionsPerHour(uint8_t *TransmissionsPerHour)
{
  /*Array to store user input*/
  char UserInput[6] = {0} ;

  do
  { 
    /*Prompt User to enter number of transmissions per hour*/
    Serial.println(F("Please enter number of tranmssions per hour [1-6]"));
       
    /*Wait for serial input from user*/
    while(Serial.available()== 0);

    /*Determine number of bytes available*/
    uint8_t bytesAvailable = 0;
    bytesAvailable = Serial.available();

    /*Read input from user into array*/
    for(uint8_t  i = 0 ; i < bytesAvailable ; i++)
    {
      UserInput[i] = Serial.read();
    }

    /*Convert user input to float*/
    *TransmissionsPerHour = atoi(UserInput);

    /*Determine if Transmissions per hour entered is valid*/
    if((*TransmissionsPerHour < 1) || (*TransmissionsPerHour > 6))
    {
      Serial.println(F("Invalid Number of transmissions per hour"));
    }
    else
    {
      Serial.println(F("Number of transmissions per hour changed successful"));
    }
  }while(TransmissionsPerHour == 0);
}


void E_Monitor_UpdateBiasVoltage(float  *V_Bias)
{
  /*Local variable used to track number of samples collected*/
  uint16_t SampleCount = 0;

  /*Local variables used to compute RMS Voltage*/
  float    Vsquared = 0;
  float    Vsample = 0;
  float    Vfiltered = 0;
  float    Vsum = 0;

  /*Collect specified number of samples*/
  while (SampleCount < 800)
  {
    if (myADC.GetConversionResult(&Vsample) == 0)
    {
      Vfiltered = Vsample ;
      Vsquared = Vfiltered * Vfiltered;
      Vsum += Vsquared;
      SampleCount++;
    }
  }

  /*Compute RMS Voltage*/
  *V_Bias = (sqrt(Vsum / SampleCount));

  /*Print Bias Voltage to screen*/
  Serial.print("V_Bias :");
  Serial.println(*V_Bias);

  return 0;
}

void E_Monitor_PrintSensorConfig_SubMenu(CT_Sensor_Config  *ctSensor_Config)
{

  /*Variable to store sensor number*/
  uint8_t CT_Sensor_Number = 0;

  /*Prompt user to enter sensor number*/
  CT_Sensor_Number = E_Monitor_GetSensorNumber();

  /*Variable used to store user input*/
  char UserInput = '0';

  do
  {
    Serial.print(F("/**********Sensor ["));
    Serial.print(CT_Sensor_Number);
    Serial.println(F("] Configuration Sub-Menu***********/"));
    Serial.println(F("(1) Enter Sensor Type"));
    Serial.println(F("(2) Enter Calibration Constant"));
    Serial.println(F("(3) Enable Sensor"));
    Serial.println(F("(4) Disable Sensor"));
    Serial.println(F("(e) Exit"));

    /*Process User Input*/
    UserInput = E_Monitor_SensorConfigSubMenu_ProcessInput(ctSensor_Config,CT_Sensor_Number);
        
  }while(UserInput != 'e');
}

char E_Monitor_SensorConfigSubMenu_ProcessInput(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number)
{
   /*Variable used to store user input*/
  char UserInput = '0';
  
  /*Wait for user input*/
  while(Serial.available() == 0);

  /*Read byte*/
  UserInput = Serial.read();

  switch(UserInput)
  {
    case '1':
    E_Monitor_PrintSensorType_Menu(ctSensor_Config , CT_Sensor_Number);
    break;

    case '2':
    E_Monitor_SetCalibrationConstant(ctSensor_Config , CT_Sensor_Number);
    break;

    case '3':
    E_Monitor_SensorEnable(ctSensor_Config ,CT_Sensor_Number);
    break;

    case '4':
    E_Monitor_SensorDisable(ctSensor_Config ,CT_Sensor_Number);
    break;

    case 'e':
    break;

    default:
    Serial.println(F("Invalid User Input"));
    break;
  }

  return UserInput;
}

void E_Monitor_PrintSensorType_Menu(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number)
{
  /*Variable used to store user input*/
  char UserInput = '0';

  do
  {
    Serial.print(F("/***************Sensor["));  
    Serial.print(CT_Sensor_Number);
    Serial.println(F("] Sensor Type Menu*************/"));
    Serial.println(F("(1) SCT_25A"));
    Serial.println(F("(2) SCT_50A"));
    Serial.println(F("(3) SCT_100A"));
    Serial.println(F("(4) SCT_160A"));
    Serial.println(F("(e) Exit"));

    /*Get Sensor Type from user*/
    UserInput = E_Monitor_SensorTypeSubMenu_ProcessInput(ctSensor_Config , CT_Sensor_Number);
    
  }while(UserInput != 'e');
}

char E_Monitor_SensorTypeSubMenu_ProcessInput(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number)
{
  /*Variable used to store user input*/
  char UserInput = '0';
  
  /*Wait for user input*/
  while(Serial.available() == 0);

  /*Read byte*/
  UserInput = Serial.read();

  switch(UserInput)
  {
    case '1':
    ctSensor_Config[CT_Sensor_Number - 1].SensorType = SCT_25A;
    Serial.println(F("Sensor type set successfully"));
    break;

    case '2':
    ctSensor_Config[CT_Sensor_Number - 1].SensorType = SCT_50A;
    Serial.println(F("Sensor type set successfully"));
    break;

    case '3':
    ctSensor_Config[CT_Sensor_Number - 1].SensorType = SCT_100A;
    Serial.println(F("Sensor type set successfully"));
    break;

    case '4':
    ctSensor_Config[CT_Sensor_Number - 1].SensorType = SCT_160A;
    Serial.println(F("Sensor type set successfully"));
    break;

    case 'e':
    break;

    default:
    Serial.println(F("Invalid User Input Entered"));
    break;
  }

  return UserInput;
}

uint8_t E_Monitor_NumberOfEnabledDevices(CT_Sensor_Config  *ctSensor_Config)
{
  /*Variable to store number of enabled devices*/
  uint8_t NumberOfEnabled_Devices = 0;

  for(uint8_t i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
    /*Determine number of enabled devices*/
    if(ctSensor_Config[i].Enabled == true)
    {
      NumberOfEnabled_Devices++;
    }
  }
   return NumberOfEnabled_Devices;
}

void E_Monitor_SetCalibrationConstant(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number)
{
  /*Array to store user input*/
  char UserInput[6] = {0} ;

  /*Used to store calibration constant from user*/
  float CalibrationConstant = 0 ;

  do
  { 
    /*Prompt user to enter calibration constant*/
    Serial.println(F("Please enter calibration constant"));
       
    /*Wait for serial input from user*/
    while(Serial.available()== 0);

    /*Determine number of bytes available*/
    uint8_t bytesAvailable = 0;
    bytesAvailable = Serial.available();

    /*Read input from user into array*/
    for(uint8_t  i = 0 ; i < bytesAvailable ; i++)
    {
      UserInput[i] = Serial.read();
    }

    /*Convert user input to float*/
    CalibrationConstant = atof(UserInput);

    /*Determine if calibration constant entered is valid*/
    if(CalibrationConstant == 0.00)
    {
      Serial.println(F("Invalid Calibration Constant Entered"));
    }
    else
    {
      Serial.println(F("Calibration Constant Changed Successfully")); 
    }
  }while(CalibrationConstant == 0.00);

  /*Set calibration constant of CT Sensor*/
  ctSensor_Config[CT_Sensor_Number - 1].CalibrationConstant = CalibrationConstant;
}
void    E_Monitor_SensorEnable(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number)
{
  Serial.print("CT Sensor [");
  Serial.print(CT_Sensor_Number);
  Serial.println("] Enabled");
  ctSensor_Config[CT_Sensor_Number - 1].Enabled = true;
}

void    E_Monitor_SensorDisable(CT_Sensor_Config  *ctSensor_Config , uint8_t CT_Sensor_Number)
{
  Serial.print("CT Sensor [");
  Serial.print(CT_Sensor_Number);
  Serial.println("] Disabled");
  ctSensor_Config[CT_Sensor_Number - 1].Enabled = false;
}

uint8_t E_Monitor_GetSensorNumber()
{
    /*Variable used to store user input*/
    char UserInput[3] = {0};

    /*Variable to hold selected sensor number*/
    uint8_t CT_Sensor_Number = 0;
  
    /*Prompt user to enter sensor to be configured*/
    Serial.println(F("/**************Sensor Configuration Menu**************/"));
    Serial.println(F("Please enter #No of sensor[1-16] you wish to configure:"));
    
    /*Wait for user input*/
    while(Serial.available() == 0);

    /*Get number of bytes avaiable*/
    uint8_t bytesAvailable = Serial.available();

    /*Readout all bytes in serial buffer*/
    for(uint8_t  i = 0 ; i < bytesAvailable ; i++)
    {
      UserInput[i] = Serial.read(); 
    }

    /*Convert input to integer*/
    CT_Sensor_Number = atoi(UserInput);

    /*Check if user input is valid*/
    if((CT_Sensor_Number >= 1) && (CT_Sensor_Number <= 16))
    {
      Serial.println(F("Valid Sensor Number entered"));
      return CT_Sensor_Number;
    }

     Serial.println(F("Invalid Sensor Number entered"));
     return 0;
}
  
bool E_Monitor_IsConfigModeEnabled()
{
  return digitalRead(CONFIG_Pin);
}
/********************************************************************************************Battery Voltage*******************************************************************/
float E_Monitor_GetPwrSourceVoltage(INPUT_PWR_SOURCE InputPwr)
{                                               
  /*Variable to hold step count*/
  int   StepCount = 0;
  float InputVoltage = 0;

  switch(InputPwr)
  {
    case ADAPTER_PWR:
    /*Set  I/O used to switch on monitoring line as output*/
    pinMode(V_SOURCE_MONITOR_ENABLE_PIN,OUTPUT);

    /*Switch line on*/
    digitalWrite(V_SOURCE_MONITOR_ENABLE_PIN,HIGH);

    /*Settling delay*/
    delay(2);
       
    /*Take an analogue reading*/
    StepCount=analogRead(V_SOURCE_MONITOR_ANALOG_IN);

    /*Compute Input Voltage*/
    InputVoltage = StepCount * STEPSIZE * V_SOURCE_MONITOR_DIVIDER_RATIO ;

    /*Check if voltage is floating*/
    if(InputVoltage < V_SOURCE_MONITOR_MIN_VOLTAGE)
    {
      InputVoltage = 0;
    }

    /*Switch line off*/
    digitalWrite(V_SOURCE_MONITOR_ENABLE_PIN,LOW);

    /*Set pinMode back to input*/
     pinMode(V_SOURCE_MONITOR_ENABLE_PIN,INPUT);
    
    break;

    case BATTERY_PWR:

    /*Set  I/O used to switch on monitoring line as output*/
    pinMode(V_BATT_MONITOR_ENABLE_PIN,OUTPUT);

    /*Switch line on*/
    digitalWrite(V_BATT_MONITOR_ENABLE_PIN,HIGH);

    /*Allows capacitor C1 to discharge,it charges reverse leakage current flowing through OR'ing diodes*/
    delay(1000);

    /*Take an analogue reading*/
    StepCount=analogRead(V_BATT_MONITOR_ANALOG_IN);

    /*Compute Input Voltage*/
    InputVoltage = StepCount * STEPSIZE * V_BATT_MONITOR_DIVIDER_RATIO;

    /*Switch line off*/
    digitalWrite(V_BATT_MONITOR_ENABLE_PIN,LOW);

    /*Set pinMode back to input*/
    pinMode(V_BATT_MONITOR_ENABLE_PIN,INPUT);

    break;

    default:
    break;
  }

  return InputVoltage;
}

void E_Monitor_SetTime(uint8_t Hour,uint8_t Minute,uint8_t Second)
{
  /*Set RTC time*/
   RTC.setTime(Hour, Minute, Second);
}

void E_Monitor_SetDate(uint8_t Day,uint8_t Month,uint8_t Year)
{
  /*Set RTC Date*/
  RTC.setDate(Day, 6, Month, 1, Year);
}

void E_Monitor_GetTime(uint8_t Hour,uint8_t Minute,uint8_t Second)
{
  /*Read RTC hour*/
  Hour = RTC.getHour();

  /*Read RTC Minute*/
  Minute = RTC.getMinute();

  /*Read RTC Second*/
  Second = RTC.getSecond();
}

void E_Monitor_GetDate(uint8_t Day,uint8_t Month,uint8_t Year)
{
  /*Read RTC Day*/
  Day = RTC.getDay();

  /*Read RTC Month*/
  Month = RTC.getMonth();

  /*Read RTC Year*/
  Year = RTC.getYear();
}
