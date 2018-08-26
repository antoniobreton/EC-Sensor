#include <main.h>

double readADC(int channel){
  uint32_t total       = 0UL;
  uint16_t sampleCount = 4096;

  for (uint16_t i = 0; i < sampleCount; i++) {
    total += analogRead(channel);
  }

  total = total >> 6;
  double proportional = (total * 1.0) / (0b00000001 << 6);
  return proportional;     
}

void requestEvent(){

    Wire.write(*((uint8_t *)&i2cRegister + regPosition));

    // If the last byte of the i2cRegister.mS is sent, that means the EC_MS_REGISTER was consumed from the master
    if (*((uint8_t *)&i2cRegister + regPosition) == *((uint8_t *)&i2cRegister.mS + 3)){
        digitalWrite(LED_PIN, LOW);
    }
    
    regPosition++;
    if(regPosition >= regSize){
        regPosition = 0 ;
    }
}

void receiveEvent(int howMany){
    if((howMany < 1) || (howMany > 16)){
        return;
    }
    regPosition = Wire.read();
    howMany--;
    if(!howMany){
        return;
    }

    while (howMany--){

        *((uint8_t *)&i2cRegister + regPosition) = Wire.read();

        //If the regPosition is equal to the task register, Do the command received from the master device
        if (regPosition == EC_TASK_REGISTER){
            if (i2cRegister.TASK == EC_MEASURE_EC)      runEC               = true;
            if (i2cRegister.TASK == EC_MEASURE_TEMP)    runTemp             = true;
            if (i2cRegister.TASK == EC_CALIBRATE_PROBE) runCalibratProbe    = true;
            if (i2cRegister.TASK == EC_CALIBRATE_HIGH) runCalibrateHigh     = true;
            if (i2cRegister.TASK == EC_CALIBRATE_LOW) runCalibrateLow       = true;
            if (i2cRegister.TASK == EC_DRY) runDry                          = true;
            if (i2cRegister.TASK == EC_I2C) runI2CAddress                   = true;
        }

        //Save things when all 4 bytes fo the float have been received
        if (regPosition == (EC_CONFIG_REGISTER)) EEPROM.put(EC_CONFIG_REGISTER, i2cRegister.CONFIG);
        if (regPosition == (EC_TEMP_COMPENSATION_REGISTER)) EEPROM.put(EC_TEMP_COMPENSATION_REGISTER, i2cRegister.tempConstant);
        if (regPosition == (EC_K_REGISTER + 3)) EEPROM.put(EC_K_REGISTER, i2cRegister.k);
        // if (regPosition == (EC_TEMPCOEF_REGISTER + 3)) EEPROM.put(EC_TEMPCOEF_REGISTER, i2cRegister.tempCoef);
        // if (regPosition == (EC_SOLUTION_REGISTER + 3)) EEPROM.put(EC_SOLUTION_REGISTER, i2cRegister.solutionEC);
        if (regPosition == (EC_CALIBRATE_OFFSET_REGISTER + 3)) EEPROM.put(EC_CALIBRATE_OFFSET_REGISTER, i2cRegister.calibrationOffset);
        if (regPosition == (EC_CALIBRATE_REFHIGH_REGISTER + 3)) EEPROM.put(EC_CALIBRATE_REFHIGH_REGISTER, i2cRegister.referenceHigh);
        if (regPosition == (EC_CALIBRATE_REFLOW_REGISTER + 3)) EEPROM.put(EC_CALIBRATE_REFLOW_REGISTER, i2cRegister.referenceLow);
        if (regPosition == (EC_CALIBRATE_READHIGH_REGISTER + 3)) EEPROM.put(EC_CALIBRATE_READHIGH_REGISTER, i2cRegister.readingHigh);
        if (regPosition == (EC_CALIBRATE_READLOW_REGISTER + 3)) EEPROM.put(EC_CALIBRATE_READLOW_REGISTER, i2cRegister.readingLow);
       
        
        regPosition++;
        if (regPosition >= regSize){
            regPosition = 0;
            
        }
    }
}



void setup() {
 
    timer1_disable();

    pinMode(EC_PIN,     INPUT);
    pinMode(SINK_PIN,   INPUT);
    pinMode(POWER_PIN,  INPUT);
    pinMode(TEMP_PIN,   INPUT);
    pinMode(LED_PIN,    OUTPUT);

    // set prescaler
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);

    // Retrieving data from the EEPROM if there is any

    EEPROM.get(EC_I2C_ADDRESS_REGISTER,                EC_METER);
    EEPROM.get(EC_K_REGISTER,                          i2cRegister.k);
    EEPROM.get(EC_TEMP_COMPENSATION_REGISTER,          i2cRegister.tempConstant);
    // EEPROM.get(EC_TEMPCOEF_REGISTER,                   i2cRegister.tempCoef);
    // EEPROM.get(EC_SOLUTION_REGISTER,                   i2cRegister.solutionEC);
    EEPROM.get(EC_CONFIG_REGISTER,                     i2cRegister.CONFIG);
    EEPROM.get(EC_CALIBRATE_OFFSET_REGISTER,           i2cRegister.calibrationOffset);
    EEPROM.get(EC_CALIBRATE_REFHIGH_REGISTER,          i2cRegister.referenceHigh);
    EEPROM.get(EC_CALIBRATE_REFLOW_REGISTER,           i2cRegister.referenceLow);
    EEPROM.get(EC_CALIBRATE_READHIGH_REGISTER,         i2cRegister.readingHigh);
    EEPROM.get(EC_CALIBRATE_READLOW_REGISTER,          i2cRegister.readingLow);
    EEPROM.get(EC_DRY_REGISTER,                        i2cRegister.dry);

    i2cRegister.version = VERSION;
    i2cRegister.tempC   = 25; 

    if(EC_METER == 0xff){
        EC_METER = EC_METER_DEFAULT_ADDRESS;
    }

    if (i2cRegister.CONFIG.buffer == 0b11111){
        i2cRegister.CONFIG.useDualPoint         = 0;
        i2cRegister.CONFIG.useTempCompensation  = 0;
        i2cRegister.CONFIG.useInterruption      = 0;
        i2cRegister.tempConstant                = 25;
        i2cRegister.CONFIG.buffer               = 0;
        EEPROM.put(EC_CONFIG_REGISTER, i2cRegister.CONFIG);
    }

    Wire.begin(EC_METER);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void loop() {
    
    lowPower();
    
    if(runTemp){
        i2cRegister.tempC = measureTemp();
        runTemp = false; 
    }

    if (runEC){
        measureConductivity();
        runEC = false;
    }

    if (runCalibratProbe){
        calibrateProbe();
        runCalibratProbe = false;
    }

    if (runCalibrateHigh){
        calibrateHigh();
        runCalibrateHigh = false;
    }

    if (runCalibrateLow){
        calibrateLow();
        runCalibrateLow = false;
    }  

    if (runDry){
        calibrateDry();
        runDry = false;
    }

    if (runI2CAddress){
        setI2CAddress();
        runI2CAddress = false;
    }

    if (i2cRegister.CONFIG.useInterruption){
        measureTemp();
        measureConductivity();
    }
}

// measureTemp() Function
// Runs the algorithm that reads the Temperature
// return temp in °C
float measureTemp(){

    float temp;

    int rawTemp = analogRead(TEMP_PIN);
    temp = tempMeasurement(rawTemp);
    return temp;
}

// measureConductivity() function
// Runs the algorithm that reads the conductivity of the solution
// return mS
float measureConductivity(){
    float inputV, outputV, mS, resistance;
    uint32_t analogRaw;

    pinMode(POWER_PIN,      OUTPUT);
    pinMode(SINK_PIN,       OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    digitalWrite(SINK_PIN,  LOW);

    analogRaw = readADC(EC_PIN);
    //adcSmooth = ecSmoothing.smooth(analogRaw);

    digitalWrite(POWER_PIN, LOW);
    digitalWrite(SINK_PIN,  LOW);
    digitalWrite(EC_PIN,    LOW);
    delay(1000);

    pinMode(POWER_PIN,      INPUT);
    pinMode(SINK_PIN,       INPUT);

    // Gets the input voltage
    inputV  = getVin();
    // Calculates the output voltage
    outputV = (inputV * analogRaw) / 1024.0;
    // Calculates the resistance of the solution
    resistance  = Resistor * (1 /((inputV/ outputV) - 1));
    // converts the resistance in milli-Siemens
    mS          = ((100000 * i2cRegister.k) / resistance);

    //Compensate for temperature if configured
    if (i2cRegister.CONFIG.useTempCompensation){
        mS = mS / (1.0 + i2cRegister.tempCoef * (i2cRegister.tempC - i2cRegister.tempConstant));
    }   

    //Use single point adjustment, ignoring if NAN
    if (i2cRegister.calibrationOffset == i2cRegister.calibrationOffset){
        mS = mS - (mS * i2cRegister.calibrationOffset);
    }  

    if (i2cRegister.CONFIG.useDualPoint){
        // Use dual-point calibration
        float referenceRange = i2cRegister.referenceHigh - i2cRegister.referenceLow;
        float readingRange   = i2cRegister.readingHigh - i2cRegister.readingLow;

        mS = (((mS - i2cRegister.readingLow) * referenceRange) / readingRange) + i2cRegister.referenceLow;
    }

    //If using the interruption mode
    if (i2cRegister.CONFIG.useInterruption){
        i2cRegister.mS = mS;
        digitalWrite(LED_PIN , HIGH);
    }

    // Check if the probe is dry/disconnected
    if(mS <= i2cRegister.dry) mS = -1;

    i2cRegister.mS = mS;
    return mS;

}

// calibrateProbe() Function
// Runs the algorithm to calibrate the probe and saves the Offset into the EEPROM
void calibrateProbe(){
    i2cRegister.calibrationOffset = NAN;
    float mS = measureConductivity();

    i2cRegister.calibrationOffset = (mS - i2cRegister.solutionEC) / mS;
    EEPROM.put(EC_CALIBRATE_OFFSET_REGISTER, i2cRegister.calibrationOffset);
}

// calibrateHigh() Function
// Runs the algorithm that calibrates the high point of the dual-point calibration
// Saves the High reference and the High reading to the EEPROM
void calibrateHigh(){
    i2cRegister.referenceHigh = i2cRegister.solutionEC;

    measureConductivity();
    i2cRegister.readingHigh = i2cRegister.mS;
    EEPROM.put(EC_CALIBRATE_REFHIGH_REGISTER, i2cRegister.referenceHigh);
    EEPROM.put(EC_CALIBRATE_READHIGH_REGISTER, i2cRegister.readingHigh);
}

// CalibrateLow() Function
// Runs the algorithm that calibrates the low point of the dual-point calibration
// Saves the Low reference and the low reading to the EEPROM
void calibrateLow(){
    i2cRegister.referenceLow = i2cRegister.solutionEC;

    measureConductivity();
    i2cRegister.readingLow = i2cRegister.mS;
    EEPROM.put(EC_CALIBRATE_REFLOW_REGISTER, i2cRegister.referenceLow);
    EEPROM.put(EC_CALIBRATE_READLOW_REGISTER, i2cRegister.readingLow);
}

// CalibrateDry() Function
// Runs the algorithm that calibrates the probe when its Dry
// Saves the Dry EC measurement into the EEPROM
void calibrateDry(){
    measureConductivity();
    i2cRegister.dry = i2cRegister.mS;
    EEPROM.put(EC_DRY_REGISTER, i2cRegister.dry);
}

// setI2CAddress() Function
// Runs the algorithm that sets the I2C address
// Saves the new Addres to the EEPROM if you forget the I2C a I2C bus analizer
void setI2CAddress(){
    // For convinience, the solution register is used to send the address
    EEPROM.put(EC_I2C_ADDRESS_REGISTER, i2cRegister.solutionEC);
    Wire.begin((int)i2cRegister.solutionEC);
}