#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <Vcc.h>
#include <AnalogSmooth.h>
#include <math.h>


//*************************************DEFINITION OF FIRMWARE VERSION AND I2C DEFAULT ADDRESS************************************//
#define VERSION 0x01                                        // Firmware Version
#define EC_METER_DEFAULT_ADDRESS 0x4C                       // EC-Meter default I2C Address

uint8_t EC_METER = 0x4C;                                    // EC-Meter I2C Address
//*************************************END OF DEFINITION OF FIRMWARE VERSION AND I2C DEFAULT ADDRESS*****************************//

//*************************************DEFINITION OF COMMAND INDEX***************************************************************//
#define EC_MEASURE_EC 80                                    // Task index for measure EC
#define EC_MEASURE_TEMP 40                                  // Task index for measure Temperature
#define EC_CALIBRATE_PROBE 20                               // Task index for calibrate the probe
#define EC_CALIBRATE_LOW 10                                 // Task index for calibrate low
#define EC_CALIBRATE_HIGH 8                                 // Task index for calibrate high
#define EC_DRY 4                                            // Task index for measure EC when probe is dry
#define EC_I2C 2                                            // Task index that runs the change I2C address
//*************************************END OF DEFINITION OF COMMAND INDEX********************************************************//

//*************************************DEFINITION OF VARIABLE REGISTER INDEX*****************************************************//
#define EC_VERSION_REGISTER 0                               // Version                  register
#define EC_TEMP_COMPENSATION_REGISTER 1                     // Temperature compensation register
#define EC_CONFIG_REGISTER 2                                // Configuration            register
#define EC_TASK_REGISTER 3                                  // Task                     register
#define EC_MS_REGISTER 4                                    // Milli Siemens            register
#define EC_TEMP_REGISTER 8                                  // Temperature in C         register
#define EC_K_REGISTER 12                                    // Cell constant            register
#define EC_SOLUTION_REGISTER 16                             // Calibration solution     register
#define EC_TEMPCOEF_REGISTER 20                             // Temperature Coefficient  register
#define EC_CALIBRATE_OFFSET_REGISTER 24                     // Calibration Offset       register
#define EC_CALIBRATE_REFHIGH_REGISTER 28                    // Reference High           register
#define EC_CALIBRATE_REFLOW_REGISTER 32                     // Reference Low            register
#define EC_CALIBRATE_READHIGH_REGISTER 36                   // Reading High             register
#define EC_CALIBRATE_READLOW_REGISTER 40                    // Reading Low              register
#define EC_DRY_REGISTER 44                                  // Dry EC measurement       register 

#define EC_I2C_ADDRESS_REGISTER 200                         // I2C Address register
//*************************************END OF DEFINITION OF VARIABLE REGISTERS INDEX**********************************************//


//*************************************CONFIGURATION STRUCT**********************************************************************//
struct config {

    uint8_t useDualPoint        : 1;                        // useDualPoint Configuration this is Bit 0
    uint8_t useTempCompensation : 1;                        // useTempCompensation Configuration this is Bit 1
    uint8_t useInterruption     : 1;                        // Use interruption Configuration this is Bit 2
    uint8_t buffer              : 5;                        // buffer Configuration this is Bit 3-7 
};
//*************************************END OF CONFIGURATION STRUCT***************************************************************//

//*************************************VARIABLES DATA STRUCTURE******************************************************************//
struct variablesRegister{
    uint8_t version;                                        // Version register                 0
    uint8_t tempConstant;                                   // Temperature Constant register    1
    config CONFIG;                                          // CONFIG register                  2
    uint8_t TASK;                                           // TASK register                    3
    float mS;                                               // milli-Siemens register           4-7
    float tempC;                                            // Temperature register             8-11
    float k;                                                // Cell constant register           12-15
    float solutionEC;                                       // Calibration solution register    16-19
    float tempCoef;                                         // Temperature coefficient register 20-23
    float calibrationOffset;                                // Calibration offset register      24-27
    float referenceHigh;                                    // Reference high register          28-31
    float referenceLow;                                     // Reference low register           32-35
    float readingHigh;                                      // Reading High register            36-39
    float readingLow;                                       // Reading Low register             40-43
    float dry;                                              // Dry EC measurment                44-47
}i2cRegister;

volatile uint8_t regPosition;
const uint8_t    regSize = sizeof(i2cRegister);
//*************************************END OF VARIABLES DATA STRUCTURE***********************************************************//

//*************************************DEFINITION OF I/O PINS********************************************************************//

#define EC_PIN 0                                            // This pin will be set to read the ADC on A0
#define TEMP_PIN 1                                          // This pin will be set to read the ADC on A1
#define POWER_PIN 8                                         // This pin will be set to power on/off the EC-Meter Net
#define SINK_PIN 4                                          // This pin will be set to sink the voltage on the EC-Meter Net
#define LED_PIN 13                                          // This pin will be set to blink the built-in LED of the device

#define adc_disable() (ADCSRA &= ~(1 << ADEN))              // Disable ADC (before power-off)
#define adc_enable() (ADCSRA |=  (1 << ADEN))               // Re-enable ADC
#define ac_disable() ACSR    |= _BV(ACD);                   // Disable analog comparator
#define ac_enable() ACSR     &= _BV(ACD)                    // Enable analog comparator
#define timer1_disable() PRR |= _BV(PRTIM1)                 // Disable timer1_disable
//*************************************END OF DEFINITION OF I/O PINS*************************************************************//

//*************************************INSTANTIATION OF OBJECTS******************************************************************//
const float vccCorrection = 1.0 / 1.0;
Vcc vcc(vccCorrection);
//AnalogSmooth ecSmoothing = AnalogSmooth();
//*************************************END OF INSTANTIATION OF OBJECTS***********************************************************//

//*************************************DEFINITION OF FUNCTIONS*******************************************************************//
float measureConductivity();                                // This function is used to measure EC, it returns mS
float measureTemp();                                        // This function is used to measure Temperature, it returns temp in C
void calibrateProbe();                                      // This function is used to calibrate the offset to the probe, return calibrationOffset
void calibrateHigh();                                       // This function is used to calibrate the High point
void calibrateLow();                                        // This function is used to calibrate the Low point
void calibrateDry();                                        // This function is used to calibrate the probe when its dry
void setI2CAddress();                                       // This function is used to set the I2C address
void sleep();
//*************************************END OF DEFINITION OF FUNCTIONS************************************************************//

//*************************************DEFINITION OF ACTION FLAGS****************************************************************//
bool runEC              = false;                            // Flag used to run the measureConductivity() function
bool runTemp            = false;                            // Flag used to run the measureTemp() function
bool runCalibratProbe   = false;                            // Flag used to run calibrateProbe() function
bool runCalibrateHigh   = false;                            // Flag used to run calibrateHigh() function
bool runCalibrateLow    = false;                            // Flag used to run calibrateLow() function
bool runDry             = false;                            // Flag used to run calibrateDry() function
bool runI2CAddress      = false;                            // Flag used to run setI2CAddress() function
//*************************************END OF DEFINITION OF ACTION FLAGS*********************************************************//

//*************************************DEFINITION OF CONSTANT VARIABLES, CBI&SBI*************************************************//
static const int Resistor = 499;

#ifndef cbi
# define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif // ifndef cbi
#ifndef sbi
# define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif // ifndef sbi
//*************************************END OF DEFINITION OF CONSTANT VARIABLES, CBI&SBI******************************************//

//*************************************AUXILIAR FUNCTIONS LOWPOWER, VCC & TEMPERATURE********************************************//

void inline lowPower()
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  adc_disable();
  ac_disable();
  sleep_enable();
  sleep_mode();
  sleep_disable();
  adc_enable();
  ac_enable();
}

float getVin(){
    float v = vcc.Read_Volts();
    return v;
}

float tempMeasurement(int tempRaw) {  //Function to perform the fancy math of the Steinhart-Hart equation
 float Temp;
 Temp = log(((10240000/tempRaw) - 8800));
 Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
 Temp = Temp - 273.15;              // Convert Kelvin to Celsius
 //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit - comment out this line if you need Celsius
 return Temp;
}

//*************************************AUXILIAR FUNCTIONS VCC & TEMPERATURE******************************************************//






