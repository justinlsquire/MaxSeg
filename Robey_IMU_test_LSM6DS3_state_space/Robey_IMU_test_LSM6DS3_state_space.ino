//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// LIBRARIES INCLUDED
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#include <TimerOne.h>
#include <SPI.h>
#include <EEPROM.h>


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// DEBUGGING FLAGS
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

// Leaving these uncommented for now - I used these for printing things to 
// serial monitor during development and debugging

//#define DEBUG
//#define INTRO_DEBUG
// Debugging for BLE Communication functions
//#define BLE_DEBUG



//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// CONFIGURATION SETTINGS
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//**********************************************************************
// INITIAL MOTOR SETTINGS
//**********************************************************************
#define MOTORS_ON 1

#define GEAR_RATIO 51.45
#define CPR_INPUT_SHAFT 12
#define OUTPUT_CPR 617.4
#define R_WHEEL .05

//**********************************************************************
// SERIAL PORT CONFIGURATION
//**********************************************************************
// Adjust these to change serial ports for the different components   //
#define BLE_SERIAL Serial2                                            //
//#define BLE_SERIAL_BAUDRATE 9600                    //
#define BLE_SERIAL_BAUDRATE 115200                   //
// baud rate for Serial
#define SERIAL_BAUDRATE 115200
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//**********************************************************************
// CONTROL LOOP SETTINGS
//**********************************************************************
// control loop frequency setting for timer interrupt
#define CONTROL_LOOP_FREQUENCY 200.0 // Hz 
// exponential filter for PID derivative
#define ALPHA_D_TERM 0.4

//**********************************************************************
// CALIBRATION CONSTANTS
//**********************************************************************
// number of readings to take for calibration
#define CALIBRATION_READINGS 35

// initial calibration settings
#define CALIBRATE_GYRO 0
#define CALIBRATE_ACCEL 0

//**********************************************************************
// ANGLE SETTINGS
//**********************************************************************
// +/- angle where motors are not active around balance point
#define DEAD_BAND .003317 //4 degrees
// +/- angle where the motors are active
#define ACTIVE_ANGLE .2853292 //10 degrees


//**********************************************************************
// EEPROM MAP
//**********************************************************************
#define GYRO_OFFSET_EEPROM_LSB 4
#define GYRO_OFFSET_EEPROM_MSB 5
#define XL_X_OFFSET_EEPROM_LSB 6
#define XL_X_OFFSET_EEPROM_MSB 7
#define XL_Z_OFFSET_EEPROM_LSB 8
#define XL_Z_OFFSET_EEPROM_MSB 9


//**********************************************************************
// MOTOR SETUP
//**********************************************************************
// amount of motor power to add on top of control
#define PREAMP 18

// DRV8835 Motor Driver Specific Pins
#define DRV_MODE_PIN 6 //chip pin 15, PH3
#define A_EN_PIN 7// PWM OUTPUT PIN, chip pin 16, PH4
#define A_PHASE_PIN 4// DIRECTION PIN, chip pin 1, PG5
#define A_PHASE_PORT PORTG
#define A_PHASE_PORT_PIN PG5
#define B_EN_PIN 8// PWM OUTPUT PIN, chip pin 17, PH5
#define B_PHASE_PIN 5// DIRECTION PIN, chip pin 5, PE3
#define B_PHASE_PORT PORTE
#define B_PHASE_PORT_PIN PE3

//Encoder Related Pins
#define M1_ENC_A_PIN 2 // chip pin 6, PE4
#define M1_ENC_A_PORT PORTE
#define M1_ENC_A_PORT_PIN PE4
#define M1_ENC_A_PORT_READ PINE
#define M1_ENC_B_PIN 3 // chip pin 7, PE5
#define M1_ENC_B_PORT PORTE
#define M1_ENC_B_PORT_PIN PE5
#define M1_ENC_B_PORT_READ PINE
#define M2_ENC_A_PIN 18 // chip pin 46, PD3
#define M2_ENC_A_PORT PORTD
#define M2_ENC_A_PORT_PIN PD3
#define M2_ENC_A_PORT_READ PIND
#define M2_ENC_B_PIN 19 // chip pin 45, PD2
#define M2_ENC_B_PORT PORTD
#define M2_ENC_B_PORT_PIN PD2
#define M2_ENC_B_PORT_READ PIND



//**********************************************************************
// IMU ACCELEROMETER AND GYRO CONSTANTS / SETUP
//**********************************************************************
// exponential gyro filter constant                                   //
#define ALPHA_GYRO 0.5                                               //
// exponential accelerometer filter constant                          //
#define ALPHA_ACC 0.5                                                //
// exponential complementary filter constant                          //
#define ALPHA_COMPLEMENTARY 0.99                                      //
// IMU slave select pin                                               //
#define IMU_SS 37                                                     //
// These need to be determined from the mega 2560 schematic           //
#define IMU_SS_PORT PORTC                                             //
#define IMU_SS_DDR DDRC                                               //
#define IMU_SS_PIN PC0                                                //
// Accelerometer startup command (address of CTRL1_XL + command)      //
// CTRL1_XL (address 0x10)                                            //
// [   4   ][   2   ][   2   ]                                        //
// [ODR_XL ][FS_XL  ][BW_XL  ]                                        //
// ODR SETTINGS:                                                      //
// -> 1000 = 1.66 kHz ODR (which results in 3320 Hz LPF1)             //
// -> 0110 = 416 Hz ODR (which results in 742 HZ LPF1)                //
// FS_XL SETTINGS:                                                    //
// -> 10 = +/- 4G full scale range                                    //
// -> 00 = +/- 2G full scale range                                    //
// BW_XL SETTINGS:                                                    //
// -> 00 = 400 Hz Anti-aliasing filter                                //
// -> 01 = 200 Hz Anti-aliasing filter                                //
//#define ACC_STARTUP 0x1088                                            //
#define ACC_STARTUP 0b0001000001100001                                //
//#define ACC_STARTUP 0x1068                                            //
// Same register, but with the read/write bit flipped                 //
#define ACC_STARTUP_READ 0x9000                                       //
// Scaling factor for accelerometer                                   //
#define ACC_FS_RANGE 2 //(+/- 2g)                                     //


// Gyro startup command (address of CTRL2_G + command)                //
// CTRL2_G (address 0x11)                                             //
// [   4   ][   2   ][ 1 ][ 1 ]                                       //
// [ ODR_G ][ FS_G  ][ FS_125 ][0]                                    //
// ODR SETTINGS:                                                      //
// -> 1000 = 1.66 kHz ODR                                             //
// -> 0110 = 416 Hz ODR                                               //
// -> 0101 = 208 Hz ODR                                               //
// FS_G SETTINGS:                                                     //
// -> 00 = 250 dps full scale range                                   //
// -> 01 = 500 dps full scale range                                   //
// FS_125 SETTINGS:                                                   //
// -> 0 = disabled, 1 = enabled                                       //
// last bit is always zero                                            //
//#define GYRO_STARTUP 0x1184                                           //
// 1.66 kHz ODR, 250 dps range                                        //
#define GYRO_STARTUP 0b0001000101100000                               //
#define GYRO_STARTUP_READ 0x9100                                      //
// Scaling factor for gyro                                            //
#define GYR_FS_RANGE 250 // (+/- 500 dps)                             //

//// Auto register address increment startup command                    //
//#define CTRL3_C_STARTUP 0x1204                                        //
//#define CTRL3_C_STARTUP_READ 0x9200                   //


// TAP_CFG Register (For enabling SLOPE_FDS (for LPF2))             //
#define TAP_CFG_STARTUP 0x5810                                      //
#define TAP_CFG_STARTUP_READ 0xD800                                 //

// CTRL8_XL register (for enabling LPF2 and setting rate)           //
// Address of CTRL8_XL (0x17) + command                             //
//[    1     ][   2   ][ 2 ][       1      ][1][       1      ]     //
//[LPF2_XL_EN][HPCF_XL][ 0 ][HP_SLOPE_XL_EN][0][LOW_PASS_ON_6D]     //

//LPF2_XL_EN SETTINGS:
// 1 = enable LPF2 (as long as HP_SLOPE_XL_EN is also enabled)      //
// 0 = disable LPF2                                                 //
//HPCF_XL SETTINGS:                                                 //
// When LPF2 is enabled                                             //
// -> 00 = ODR_XL/50                                                //
// -> 01 = ODR_XL/100                                               //
// -> 10 = ODR_XL/9                                                 //
// -> 11 = ODR_XL/400                                               //
//HP_SLOPE_XL_EN SETTINGS:                                          //
// 1 = enabled, 0 = disabled                                        //

// 17C4 Sets up LPF2 cutoff at ODR / 9                                   //
// 1784 Sets up LPF2 cutoff at ODR / 50                                  //
// 17E4 Sets up LPF2 cutoff at ODR / 400                                 //
//#define CTRL8_XL_STARTUP 0x17C4                                        //
#define CTRL8_XL_STARTUP 0b0001011110100100                           //
#define CTRL8_XL_STARTUP_READ 0x9700                                  //

// Registers for accelerometer / gyro data                            //
// All include 1 at bit0 for read, plus address (e.g. 0x29)           //
#define OUTX_H_XL 0xA900                                              //
#define OUTX_L_XL 0xA800                                              //
#define OUTY_H_XL 0xAB00                                              //
#define OUTY_L_XL 0xAA00                                              //
#define OUTZ_H_XL 0xAD00                                              //
#define OUTZ_L_XL 0xAC00                                              //
//
#define OUTX_H_G 0xA300                                               //
#define OUTX_L_G 0xA200                                               //
#define OUTY_H_G 0xA500                                               //
#define OUTY_L_G 0xA400                                               //
#define OUTZ_H_G 0xA700                                               //
#define OUTZ_L_G 0xA600                                               //


//**********************************************************************
// BLE COMMUNICATION SETTINGS
//**********************************************************************
// Max number of times to request a message re-send before setting    //
// a communication error and stopping communication                   //
#define MAX_RESEND_ATTEMPTS 4                                         //
// max number of milliseconds to wait for echo response after         //
// sending a BLE message to app                                       //
#define WAIT_FOR_ECHO_TIMEOUT 100                                     //
// Maximum number of retries for getting an echo back for a           //
// diagnostic message                                                 //
#define MAX_DIAG_ECHO_WAIT_CNT 15                                      //


// number of milliseconds to wait between live data streaming attempts
#define DEBUG_REQUEST_BLE_INTERVAL 50
//----------------------------------------------------------------------
//----------------------------------------------------------------------



//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// PROGRAM VARIABLES
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//=================================================================
// === INITIAL CONTROL SCHEME SELECTION
//=================================================================
boolean stateSpace = false; //true for SS control, false for PID control

//**********************************************************************
// ACCELEROMETER AND GYRO RELATED
//**********************************************************************
// The buffer to hold the 6 values from acc and gyro                  //
// Order for simple balancing is gyroY,accX,accZ                      //
int dataIMU[6] = {0, 0, 0, 0, 0 , 0};                                 //
uint16_t IMU_Low;                                                     //
uint16_t IMU_High;                                                    //
// variables for exponential filtered gyro                            //
float lastFilteredGyro = 0.0;                                         //
float filteredGyro = 0.0;                                             //
// angle from accelerometer                                           //
float angleAcc = 0.0;                                                 //
// variable for previous value                                        //
float lastAngleAcc = 0.0;                                             //
// variable for the calculated and filtered angle                     //
volatile float angleCalc  = 0.0;                                      //
// variable for the previous calculated angle                         //
volatile float lastAngleCalc = 0.0;                                   //
// calibration values                                                 //
float gyroOffset = -400;                                              //
float accelXOffset = 0;                                               //
float accelZOffset = 0;                                               //

float x_dot = 0.0;

//**********************************************************************
// ENCODER AND POSITION RELATED
//**********************************************************************
volatile long encCountM1 = 0;                                         //
volatile long encCountM2 = 0;                                         //
long tempEncCountM1 = 0;                                              //
long tempEncCountM2 = 0;                                              //
float metersEncM1 = 0.0;                                              //
float metersEncM2 = 0.0;                                              //
float lastMetersEncM1 = 0.0;                                          //
float lastMetersEncM2 = 0.0;                                          //
float velocity = 0.0;                                                 //
float distance = 0.0;                                                 //

// variables for filtered derivative
float Xw0 = 0.0;
float Xw1 = 0.0;
float Xw2 = 0.0;
float Xw3 = 0.0;
float Xw4 = 0.0;

//**********************************************************************
// PID, STATE SPACE, AND CONTROL LOOP RELATED
//**********************************************************************
// output of control loop                                             //
volatile float output = 0.0;                                          //
// error from control loop                                            //
volatile float error = 0.0;                                           //
// previous error                                                     //
volatile float lastError = 0.0;                                       //
// integral term of PID calculation                                   //
volatile float integralTerm = 0.0;                                    //
// derivative term                                                    //
volatile float dAng = 0.0; //derivative of angle                      //
// max integral term for anti-windup                                  //
volatile float integralMax = 10.0;                                    //
//
// PID Values                                                         //
float Kp = 11.5;                                                      //
float Ki = 100.0;                                                     //
float Kd = 0.35;                                                      //
// The desired angle to maintain                                      //
float desAng = 0.0;                                                   //
//
// The angle offset to trim the balance point                         //
float angleOffset = 0.0;                                              //


// State Space Values
//float K[4] = { -7, -1, -10, -0.35}; // 200 Hz version
float K[4] = { -3, -0.5, -25, -.4}; // 200 Hz version


//**********************************************************************
// TIMER RELATED (LOOP TIMES, ETC.)
//**********************************************************************
float controlLoopPeriod;                                              //
// timer for integration of gyro for angle                            //
unsigned long gyro_integral_timestep = 0;                             //
// Timer variable for loop time measurement                           //
unsigned long loopTimer = 0;                                              //
float loopTimeTotal = 0.0;                                                  //

boolean calibrateGyro = false;
boolean calibrateAccel = false;


//**********************************************************************
// MOTOR RELATED
//**********************************************************************
boolean motors_on = true;                                             //

//**********************************************************************
// BLE / FAULT / MESSAGE RELATED
//**********************************************************************
unsigned long debugRequestBLETimer;
boolean debugRequestBLE = true;
int debug_request_BLE_interval = 50;
boolean angleStream = false;
boolean gyroStream = false;
boolean accXStream = false;
boolean accZStream = false;
boolean encoderRStream = false;
boolean encoderLStream = false;
boolean wheelSpeedStream = false;
boolean errorStream = false;
boolean KpStream = false;
boolean KiStream = false;
boolean KdStream = false;
boolean desAngStream = false;
boolean loopTimeStream = false;
boolean motorOutputStream = false;



//**********************************************************************
// MISCELLANEOUS ITEMS
//**********************************************************************
// Universal trash bin for character reads                            //
char trash;                                                           //
//----------------------------------------------------------------------
//----------------------------------------------------------------------












//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// PROGRAM SETUP
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


void setup() {
  // Set up the UART for communicating with FTDI USB COM port
  Serial.begin(SERIAL_BAUDRATE);
  // Set up BLE serial
  BLE_SERIAL.begin(BLE_SERIAL_BAUDRATE);


  if (MOTORS_ON) {
    motors_on = true;
  }
  else {
    motors_on = false;
  }

  if (CALIBRATE_GYRO) {
    calibrateGyro = true;
  }
  if (CALIBRATE_ACCEL) {
    calibrateAccel = true;
  }
  // Set up the IMU with desired settings
  setupIMU();

  // Set up the motor driver interface pins
  setupMotors();
  // Set up the timer interrupt
  controlLoopPeriod = 1.0 / CONTROL_LOOP_FREQUENCY;
  delay(1000);
#ifdef DEBUG
  Serial.print(F("Setting control loop to: "));
  Serial.print((int)(controlLoopPeriod * 1000 * 1000));
  Serial.println(F(" us"));
#endif
  Timer1.initialize((int)(controlLoopPeriod * 1000 * 1000));
  Timer1.attachInterrupt(controlCalc);

  // initialize gyro timestep
  gyro_integral_timestep = micros();

} // end of setup



//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// PROGRAM LOOP
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


void loop() {

  // Get the 3 IMU values
  readIMU();

  // Get the current encoder count
  cli();
  tempEncCountM1 = -encCountM1;
  tempEncCountM2 = encCountM2;
  sei();
  lastMetersEncM1 = metersEncM1;
  metersEncM1 = tempEncCountM1 / OUTPUT_CPR * 2 * M_PI * R_WHEEL; // distance in meters
  lastMetersEncM2 = metersEncM2;
  metersEncM2 = tempEncCountM2 / OUTPUT_CPR * 2 * M_PI * R_WHEEL; // distance in meters
  distance = (metersEncM1 + metersEncM2) / 2.0; // average of the two

  #ifdef DEBUG
    Serial.println(velocity);
  #endif

  //See if anything is available on bluetooth
  checkForBLE();

  //Wrapper function for debugging main variables
  // over serial or BLE
  debugMainVariables();

  loopTimer = micros() - loopTimer;
  loopTimeTotal = (float)(loopTimer) * .001;

  //  Serial.println(loopTimeTotal);

}



//**********************************************************************
// CONTROL LOOP CALCULATION
//**********************************************************************
void controlCalc() {

  if (abs(angleCalc) < ACTIVE_ANGLE) {

    if (stateSpace) {

      //  velocity
      Xw4 = Xw3;
      Xw3 = Xw2;
      Xw2 = Xw1;
      Xw1 = Xw0;
      Xw0 = distance;
      // discrete time digital low pass filter
      velocity = (5 * Xw0 + 2 * Xw1 - 8 * Xw2 - 2 * Xw3 + 3 * Xw4) / (8 * controlLoopPeriod);
      // K[0] = xw
      // K[1] = xw_dot
      // K[2] = theta
      // K[3] = theta_dot
      output = -K[0] * distance;
      output -= K[1] * velocity;
      output += K[2] * angleCalc;
      output += K[3] * x_dot;
    }
    else {

      // exponential filtered derivative
      dAng = ((1 - ALPHA_D_TERM) * dAng) + (ALPHA_D_TERM * ((angleCalc - lastAngleCalc) * CONTROL_LOOP_FREQUENCY));

      // error calculation
      error = desAng - angleCalc;
      // Integral term calculation
      integralTerm += error * controlLoopPeriod * Ki;
      // Anti-windup
      integralTerm = constrain(integralTerm, -integralMax, integralMax);
      output = error * Kp + integralTerm - dAng * Kd;
    } // else - from if (stateSpace)


    // dead band motors off
    if (abs(angleCalc - desAng) < DEAD_BAND) {
      output = 0.0;
    }
    if (motors_on) {
      // set direction pins based on sign of output
      if (output < 0) {
        // set A Phase Pin High
        A_PHASE_PORT |= _BV(A_PHASE_PORT_PIN);
        //    digitalWrite(A_PHASE_PIN, HIGH);
        // set B Phase Pin Low
        B_PHASE_PORT &= ~_BV(B_PHASE_PORT_PIN);
        //    digitalWrite(B_PHASE_PIN, LOW);
      }
      else {
        // Set A Phase Pin Low
        A_PHASE_PORT &= ~_BV(A_PHASE_PORT_PIN);
        //    digitalWrite(A_PHASE_PIN, LOW);
        // Set B Phase Pin High
        B_PHASE_PORT |= _BV(B_PHASE_PORT_PIN);
        //    digitalWrite(B_PHASE_PIN, HIGH);
      }

      // scale output to PWM values
      output = floor(output / 5.0 * 255.0);
      output = constrain((int)output, -255, 255);
      output = constrain((PREAMP + abs((int)output)), 0, 255);

      analogWrite(B_EN_PIN, (int)output);
      analogWrite(A_EN_PIN, (int)output);
    } // if (motors_on)
  } // if angle < activeAngle
  else {
    analogWrite(B_EN_PIN, 0);
    analogWrite(A_EN_PIN, 0);
  }
} // end of controlCalc function



//**********************************************************************
// READ IMU DATA
//**********************************************************************
void readIMU(void) {
  // Start SPI hardware
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE3));

  float tempBuf = 0.0;

  // GET GYRO VALUE

  // Set ss as output (set low first)
  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  // get data for Gyro Y (LSB)
  IMU_Low = SPI.transfer16(OUTY_L_G);
  //Set as input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  // Set ss as output (set low first)
  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  // get data for Gyro Y (MSB)
  IMU_High = SPI.transfer16(OUTY_H_G);
  //Set as input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  tempBuf = (((int)(IMU_High << 8) | (byte(IMU_Low))) - gyroOffset);

  filteredGyro = (ALPHA_GYRO * lastFilteredGyro) + (ALPHA_GYRO * tempBuf);
  lastFilteredGyro = filteredGyro;

  // Save to array
  dataIMU[0] = (int)filteredGyro;


  // GET ACC X VALUE

  // Set ss as output (set low first)
  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  // get data for Accel X (LSB)
  IMU_Low = SPI.transfer16(OUTX_L_XL);
  //Set as input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  // Set ss as output (set low first)
  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  // get data for Accel X (MSB)
  IMU_High = SPI.transfer16(OUTX_H_XL);
  //Set as input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  // Save to array
  //  dataIMU[1] = ((IMU_High << 8) | (byte(IMU_Low)));
  dataIMU[1] = (int)((1 - ALPHA_ACC) * ((int)(IMU_High << 8) | (byte(IMU_Low))) + (ALPHA_ACC) * ((float)dataIMU[1]));

  //  Serial.println(dataIMU[1]);


  // GET ACC Z VALUE

  // Set ss as output (set low first)
  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  // get data for Accel Z (LSB)
  IMU_Low = SPI.transfer16(OUTZ_L_XL);
  //Set as input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  // Set ss as output (set low first)
  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  // get data for Accel Z (MSB)
  IMU_High = SPI.transfer16(OUTZ_H_XL);
  //Set as input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  // Save to array
  //  dataIMU[2] = ((IMU_High << 8) | (byte(IMU_Low)));// * (GYR_FS_RANGE / 32768.0);
  dataIMU[2] = (int)((1 - ALPHA_ACC) * ((int)((IMU_High << 8) | (byte(IMU_Low)))) + (ALPHA_ACC) * ((float)dataIMU[2]));

  // Turn off SPI hardware
  SPI.endTransaction();



  // Calculate the angle based on IMU inputs

  lastAngleAcc = angleAcc;
  //      angleAcc = -atan2(double(dataIMU[2]), -double(dataIMU[1]));
  angleAcc = ((1 - ALPHA_ACC) * (-atan2(double(dataIMU[2]), -double(dataIMU[1])))) + ((ALPHA_ACC) * lastAngleAcc);
  angleAcc -= angleOffset;

  lastAngleCalc = angleCalc;

  //angleCalc = (angleCalc + dataIMU[0] * .0065 * .0174532925 * (micros()-gyro_integral_timestep) * .001 * .001);
  angleCalc = (1 - ALPHA_COMPLEMENTARY) * angleAcc + (ALPHA_COMPLEMENTARY) * (angleCalc + dataIMU[0] * .00875 * .0174532925 * (micros() - gyro_integral_timestep) * .001 * .001); // - angleOffset2);

  x_dot = dataIMU[0] * .00875 * .01745329; // current calculated gyro in rad/s

  gyro_integral_timestep = micros();

} // end of readIMU function



//**********************************************************************
// FUNCTION TO SET UP LSM6DS3 IMU
//**********************************************************************
void setupIMU(void) {
  // Set up SS pin as input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);
  Serial.println(F("Attempting to set up LSM6 accel/gyro"));
  SPI.begin();
  //  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE3));


  // set direction to output
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  //  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  SPI.transfer16(CTRL8_XL_STARTUP);
  // set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);
  //  IMU_SS_PORT |= _BV(IMU_SS_PIN);
  //  SPI.end();

  delay(10);

  // set direction to output
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  //  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  SPI.transfer16(TAP_CFG_STARTUP);
  // set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);
  //  IMU_SS_PORT |= _BV(IMU_SS_PIN);
  //  SPI.end();


  delay(10);

  // set direction to output
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  //  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  SPI.transfer16(GYRO_STARTUP);
  // set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);
  //  IMU_SS_PORT |= _BV(IMU_SS_PIN);

  delay(10);

  IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  SPI.transfer16(ACC_STARTUP);
  // set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);
  //  IMU_SS_PORT |= _BV(IMU_SS_PIN);

  delay(10);

  // Read back the registers to see if they were set correctly
  word IMUTemp;
  // Set direction to output
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  IMUTemp = SPI.transfer16(ACC_STARTUP_READ);
  // Set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  delay(10);

  // Check for correctness
  if ((IMUTemp & 0x00FF) == (ACC_STARTUP & 0x00FF)) {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.println(F("Accelerometer startup register set"));
#endif
#endif
  }
  else {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Accelerometer startup register not properly set, result: "));
    Serial.println(IMUTemp, HEX);
#endif
#endif
  }

  IMUTemp = 0x0000;
  // Set direction to output
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  IMUTemp = SPI.transfer16(GYRO_STARTUP_READ);
  // Set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  delay(10);

  // Check for correctness
  if ((IMUTemp & 0x00FF) == (GYRO_STARTUP & 0x00FF)) {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.println(F("Gyro startup register set"));
#endif
#endif
  }
  else {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Gyro startup register not properly set, result: "));
    Serial.println(IMUTemp, HEX);
#endif
#endif
  }


  IMUTemp = 0x0000;
  // Set direction to output
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  IMUTemp = SPI.transfer16(CTRL8_XL_STARTUP_READ);
  // Set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  delay(10);

  // Check for correctness
  if ((IMUTemp & 0x00FF) == (CTRL8_XL_STARTUP & 0x00FF)) {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.println(F("CTRL8 startup register set"));
#endif
#endif
  }
  else {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("CTRL8 startup register not properly set, result: "));
    Serial.println(IMUTemp, HEX);
#endif
#endif
  }


  IMUTemp = 0x0000;
  // Set direction to output
  IMU_SS_DDR |= _BV(IMU_SS_PIN);
  IMUTemp = SPI.transfer16(TAP_CFG_STARTUP_READ);
  // Set direction to input
  IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

  delay(10);

  // Check for correctness
  if ((IMUTemp & 0x00FF) == (TAP_CFG_STARTUP & 0x00FF)) {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.println(F("TAP_CFG startup register set"));
#endif
#endif
  }
  else {
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("TAP_CFG startup register not properly set, result: "));
    Serial.println(IMUTemp, HEX);
#endif
#endif
  }










  if (calibrateGyro) {

#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.println(F("Reading gyro offset"));
#endif
#endif

    gyroOffset = 0.0;

    // Read the Gyro offset
    for (int i = 0; i < CALIBRATION_READINGS; i++) {
      // Set ss as output (set low first)
      IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
      IMU_SS_DDR |= _BV(IMU_SS_PIN);
      // get data for Gyro Y (LSB)
      IMU_Low = SPI.transfer16(OUTY_L_G);
      //Set as input
      IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

      // Set ss as output (set low first)
      IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
      IMU_SS_DDR |= _BV(IMU_SS_PIN);
      // get data for Gyro Y (MSB)
      IMU_High = SPI.transfer16(OUTY_H_G);
      //Set as input
      IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

      // Save to array
      gyroOffset += ((int)(IMU_High << 8) | (byte(IMU_Low)));
      delayMicroseconds(300);
      Serial.println(gyroOffset);
    }

    gyroOffset /= (float)(CALIBRATION_READINGS);

#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Gyro stationary: "));
    Serial.println(gyroOffset);
#endif
#endif

    EEPROM.write(GYRO_OFFSET_EEPROM_LSB, (byte)((int)(gyroOffset)));
    EEPROM.write(GYRO_OFFSET_EEPROM_MSB, (byte)(((int)(gyroOffset)) >> 8));

  }
  else {
    gyroOffset = ((EEPROM.read(GYRO_OFFSET_EEPROM_MSB) << 8) | (EEPROM.read(GYRO_OFFSET_EEPROM_LSB)));
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Reading gyro offset from EEPROM: "));
    Serial.println(gyroOffset);
#endif
#endif
  }

  if (calibrateAccel) {

#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.println(F("Reading accel X offset"));
#endif
#endif

    accelXOffset = 0.0;

    // Read the Gyro offset
    for (int i = 0; i < CALIBRATION_READINGS; i++) {
      // Set ss as output (set low first)
      IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
      IMU_SS_DDR |= _BV(IMU_SS_PIN);
      // get data for Gyro Y (LSB)
      IMU_Low = SPI.transfer16(OUTX_L_XL);
      //Set as input
      IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

      // Set ss as output (set low first)
      IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
      IMU_SS_DDR |= _BV(IMU_SS_PIN);
      // get data for Gyro Y (MSB)
      IMU_High = SPI.transfer16(OUTX_H_XL);
      //Set as input
      IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

      // Save to array
      accelXOffset += ((int)(IMU_High << 8) | (byte(IMU_Low)));
      delayMicroseconds(300);
      Serial.println(accelXOffset);
    }

    accelXOffset /= (float)(CALIBRATION_READINGS);

#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Accel-X stationary: "));
    Serial.println(accelXOffset);
#endif
#endif

    EEPROM.write(XL_X_OFFSET_EEPROM_LSB, (byte)((int)(accelXOffset)));
    EEPROM.write(XL_X_OFFSET_EEPROM_MSB, (byte)(((int)(accelXOffset)) >> 8));



#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.println(F("Reading accel Z offset"));
#endif
#endif

    accelZOffset = 0.0;

    // Read the ACcel offset
    for (int i = 0; i < CALIBRATION_READINGS; i++) {
      // Set ss as output (set low first)
      IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
      IMU_SS_DDR |= _BV(IMU_SS_PIN);
      // get data for Gyro Y (LSB)
      IMU_Low = SPI.transfer16(OUTZ_L_XL);
      //Set as input
      IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

      // Set ss as output (set low first)
      IMU_SS_PORT &= ~_BV(IMU_SS_PIN);
      IMU_SS_DDR |= _BV(IMU_SS_PIN);
      // get data for Gyro Y (MSB)
      IMU_High = SPI.transfer16(OUTZ_H_XL);
      //Set as input
      IMU_SS_DDR &= ~_BV(IMU_SS_PIN);

      // Save to array
      accelZOffset += ((int)(IMU_High << 8) | (byte(IMU_Low)));
      delayMicroseconds(300);
      Serial.println(accelZOffset);
    }

    accelZOffset /= (float)(CALIBRATION_READINGS);

#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Accel-Z stationary: "));
    Serial.println(accelZOffset);
#endif
#endif

    EEPROM.write(XL_Z_OFFSET_EEPROM_LSB, (byte)((int)(accelZOffset)));
    EEPROM.write(XL_Z_OFFSET_EEPROM_MSB, (byte)(((int)(accelZOffset)) >> 8));

  } // if (calibrateAccel)
  else {

    accelXOffset = ((EEPROM.read(XL_X_OFFSET_EEPROM_MSB) << 8) | (EEPROM.read(XL_X_OFFSET_EEPROM_LSB)));
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Reading accel X offset from EEPROM: "));
    Serial.println(accelXOffset);
#endif
#endif
    accelZOffset = ((EEPROM.read(XL_Z_OFFSET_EEPROM_MSB) << 8) | (EEPROM.read(XL_Z_OFFSET_EEPROM_LSB)));
#ifdef DEBUG
#ifdef INTRO_DEBUG
    Serial.print(F("Reading accel Z offset from EEPROM: "));
    Serial.println(accelZOffset);
#endif
#endif
  }


  SPI.endTransaction();
} // end of setupIMU function



//**********************************************************************
// SETUP MOTORS FUNCTION
//**********************************************************************
void setupMotors(void) {
  // Set up Mode PIN (PHASE-ENABLE mode)
  pinMode(DRV_MODE_PIN, OUTPUT);
  digitalWrite(DRV_MODE_PIN, HIGH);

  // Set up phase and enable pins as outputs
  pinMode(A_EN_PIN, OUTPUT);
  pinMode(A_PHASE_PIN, OUTPUT);
  pinMode(B_EN_PIN, OUTPUT);
  pinMode(B_PHASE_PIN, OUTPUT);

  // set up encoder pins as interrupts
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), encUpdate1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_B_PIN), encUpdate1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A_PIN), encUpdate2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_B_PIN), encUpdate2B, CHANGE);


  // Change PWM frequency to fast (30k)
  //  TCCR4B &= ~0x07;
  //  TCCR4B |= 0x01;
}


//**********************************************************************
// DEBUG MAIN VARIABLES (OVER BLE OR REGULAR SERIAL)
//**********************************************************************
void debugMainVariables(void) {
  if ((millis() - debugRequestBLETimer) >= debug_request_BLE_interval) {
    // Reset the timer
    debugRequestBLETimer = millis();
    // Provision to turn on debug values via BLE by setting a variable
    if (debugRequestBLE) {
      if (angleStream) {
        sendMsgBLE('Y', 1, calcMSB((int)(angleCalc * 1000.0)), calcLSB((int)(angleCalc * 1000.0)));
      }
      if (gyroStream) {
        sendMsgBLE('Y', 2, calcMSB(dataIMU[0]), calcLSB(dataIMU[0]));
      }
      if (accXStream) {
        sendMsgBLE('Y', 3, calcMSB(dataIMU[1]), calcLSB(dataIMU[1]));
      }
      if (accZStream) {
        sendMsgBLE('Y', 4, calcMSB(dataIMU[2]), calcLSB(dataIMU[2]));
      }
      if (encoderRStream) {
        sendMsgBLE('Y', 5, calcMSB((int)(metersEncM1 * 1000.0)), calcLSB((int)(metersEncM1 * 1000.0)));
      }
      if (encoderLStream) {
        sendMsgBLE('Y', 6, calcMSB((int)(metersEncM2 * 1000.0)), calcLSB((int)(metersEncM2 * 1000.0)));
      }
      if (wheelSpeedStream) {
        sendMsgBLE('Y', 7, calcMSB((int)(velocity * 1000.0)), calcLSB((int)(velocity * 1000.0)));
      }
      if (errorStream) {
        sendMsgBLE('Y', 8, calcMSB((int)(error * 1000.0)), calcLSB((int)(error * 1000.0)));
      }
      if (KpStream) {
        sendMsgBLE('Y', 9, calcMSB((int)(error * Kp * 100.0)), calcLSB((int)(error * Kp * 100.0)));
      }
      if (KiStream) {
        sendMsgBLE('Y', 10, calcMSB((int)(integralTerm * 100.0)), calcLSB((int)(integralTerm * 100.0)));
      }
      if (KdStream) {
        sendMsgBLE('Y', 11, calcMSB((int)(- dAng * Kd * 100.0)), calcLSB((int)(- dAng * Kd * 100.0)));
      }
      if (desAngStream) {
        sendMsgBLE('Y', 12, calcMSB((int)(desAng * 1000.0)), calcLSB((int)(desAng * 1000.0)));
      }
      if (loopTimeStream) {
        sendMsgBLE('Y', 13, calcMSB((int)(loopTimeTotal)), calcLSB((int)(loopTimeTotal)));
      }
      if (motorOutputStream) {
        sendMsgBLE('Y', 14, calcMSB(output), calcLSB(output));
      }
    } // if debugRequestBLE
  } // if the interval has been reached
} //End of debugMainVariables function





//**********************************************************************
// CHECK FOR BLE LIMITS AND REACT IF A GOOD MESSAGE RECEIVED
//**********************************************************************
void checkForBLE(void) {
  int startCode;
  int commandCode1;
  int commandCode2;
  int MSBin = 0;
  int LSBin = 0;
  int checkSumRecv = 0;
  byte checksumIn;
  int MSBout;
  int LSBout;

  //Check if serial data available from BLE
  if (BLE_SERIAL.available() >= 6) {
    startCode = BLE_SERIAL.read();

#ifdef DEBUG
#ifdef BLE_DEBUG
    Serial.print(F("start code = "));
    Serial.println(startCode);
#endif
#endif

    if (startCode == '*') {
      // The control character was read

#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.println(F("start code was correct"));
#endif
#endif
      commandCode1 = BLE_SERIAL.read();
#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.print(F("cc1: "));
      Serial.println(commandCode1);
#endif
#endif


      commandCode2 = BLE_SERIAL.read();
#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.print(F("cc2: "));
      Serial.println(commandCode2);
#endif
#endif

      MSBin = BLE_SERIAL.read();
#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.print(F("msb: "));
      Serial.println(MSBin);
#endif
#endif

      LSBin = BLE_SERIAL.read();
#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.print(F("lsb: "));
      Serial.println(LSBin);
#endif
#endif

      checkSumRecv = BLE_SERIAL.read();
#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.print(F("chsum recv'd: "));
      Serial.println(checkSumRecv);
#endif
#endif


      checksumIn = calcChecksum(commandCode1, commandCode2, MSBin, LSBin);

#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.print(F("checksum calc: "));
      Serial.println(checksumIn);
#endif
#endif
      if (checksumIn == checkSumRecv) {
        switch (commandCode1) {
          case 'A':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                desAng += ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '-':
                desAng -= ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '=':
                desAng = ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'C':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case 'G':
                calibrateGyro = true;
                break;
              case 'A':
                calibrateAccel = true;
                break;
              default:
                break;
            }
            break;
          case 'D':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                Kd += ((float)((MSBin << 8) | LSBin)) / 1000.0;
                break;
              case '-':
                Kd -= ((float)((MSBin << 8) | LSBin)) / 1000.0;
                break;
              case '=':
                Kd = ((float)((MSBin << 8) | LSBin)) / 1000.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'I':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                Ki += ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '-':
                Ki -= ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '=':
                Ki = ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'J':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                K[0] += ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '-':
                K[0] -= ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '=':
                K[0] = ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'K':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                K[1] += ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '-':
                K[1] -= ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '=':
                K[1] = ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'L':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                K[2] += ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '-':
                K[2] -= ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '=':
                K[2] = ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'M':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                K[3] += ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '-':
                K[3] -= ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '=':
                K[3] = ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'O':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                angleOffset += ((float)((MSBin << 8) | LSBin)) / 1000.0;
                break;
              case '-':
                angleOffset -= ((float)((MSBin << 8) | LSBin)) / 1000.0;
                break;
              case '=':
                angleOffset = ((float)((MSBin << 8) | LSBin)) / 1000.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'P':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case '+':
                Kp += ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '-':
                Kp -= ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              case '=':
                Kp = ((float)((MSBin << 8) | LSBin)) / 100.0;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'S':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case 'S':
                stateSpace = true;
                break;
              case 'P':
                stateSpace = false;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'U':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case 1:
                debug_request_BLE_interval = ((MSBin << 8) | LSBin);
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'W':
            MSBout = 0;
            LSBout = 0;
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case 'G':
                LSBout = ((byte)((int)(gyroOffset)));
                MSBout = ((byte)(((int)(gyroOffset)) >> 8));
                break;
              case 'A':
                LSBout = ((byte)((int)(accelXOffset)));
                MSBout = ((byte)(((int)(accelXOffset)) >> 8));
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          case 'X':
            echoRecvMsg(commandCode1, commandCode2, MSBin, LSBin);
            switch (commandCode2) {
              case 1:
                if (LSBin) {
                  angleStream = true;
                }
                else {
                  angleStream = false;
                }
                break;
              case 2:
                if (LSBin) {
                  gyroStream = true;
                }
                else {
                  gyroStream = false;
                }
                break;
              case 3:
                if (LSBin) {
                  accXStream = true;
                }
                else {
                  accXStream = false;
                }
                break;
              case 4:
                if (LSBin) {
                  accZStream = true;
                }
                else {
                  accZStream = false;
                }
                break;
              case 5:
                if (LSBin) {
                  encoderRStream = true;
                }
                else {
                  encoderRStream = false;
                }
                break;
              case 6:
                if (LSBin) {
                  encoderLStream = true;
                }
                else {
                  encoderLStream = false;
                }
                break;
              case 7:
                if (LSBin) {
                  wheelSpeedStream = true;
                }
                else {
                  wheelSpeedStream = false;
                }
                break;
              case 8:
                if (LSBin) {
                  errorStream = true;
                }
                else {
                  errorStream = false;
                }
                break;
              case 9:
                if (LSBin) {
                  KpStream = true;
                }
                else {
                  KpStream = false;
                }
                break;
              case 10:
                if (LSBin) {
                  KiStream = true;
                }
                else {
                  KiStream = false;
                }
                break;
              case 11:
                if (LSBin) {
                  KdStream = true;
                }
                else {
                  KdStream = false;
                }
                break;
              case 12:
                if (LSBin) {
                  desAngStream = true;
                }
                else {
                  desAngStream = false;
                }
                break;
              case 13:
                if (LSBin) {
                  loopTimeStream = true;
                }
                else {
                  loopTimeStream = false;
                }
                break;
              case 14:
                if (LSBin) {
                  motorOutputStream = true;
                }
                else {
                  motorOutputStream = false;
                }
                break;
              case 99:
                angleStream = false;
                gyroStream = false;
                accXStream = false;
                accZStream = false;
                encoderRStream = false;
                encoderLStream = false;
                wheelSpeedStream = false;
                errorStream = false;
                KpStream = false;
                KiStream = false;
                KdStream = false;
                desAngStream = false;
                loopTimeStream = false;
                motorOutputStream = false;
                break;
              default:
                break;
            } // switch (commandCode2)
            break;
          default:
            break;
        }// end of switch(commandCode1)

      } // checksum if statement
      else {
        //message is bad - send back error
#ifdef DEBUG
#ifdef BLE_DEBUG
        Serial.println(F("ERROR-Checksum did not match"));
#endif
#endif
      }
    }// if start code is equal to *
    else {
      //Something weird was received on serial line and should be ignored
#ifdef DEBUG
#ifdef BLE_DEBUG
      Serial.println(F("ERROR-Start code not '*' not received"));
#endif
#endif
      // Try clearing the serial data because it is not in sync
      //      Serial.flush();
      // Read until out of data, or until the next start code (*) is
      // discovered
      while (BLE_SERIAL.available()) {
        if ((BLE_SERIAL.peek() != '*') && (BLE_SERIAL.peek() != '#') && (BLE_SERIAL.peek() != '%')) {
          trash = BLE_SERIAL.read();
          //        delay(1);
        }
      }

    } // end of else ( from if start code is equal to *
  } // if serial available >= 6
} // end function


//**********************************************************************
// FUNCTION TO CALCULATE THE MSB OF 2-BYTE NUMBER
//**********************************************************************
//Calculate MSB
byte calcMSB(int inVal) {
  byte MSB;
  MSB = (byte) (inVal >> 8);
  return MSB;
}


//**********************************************************************
// FUNCTION TO CALCULATE THE LSB OF 2-BYTE NUMBER
//**********************************************************************
//Calculate LSB
byte calcLSB(int inVal) {
  byte LSB;
  LSB = (byte)(inVal & 0xFF);
  return LSB;
}


//**********************************************************************
// FUNCTION TO CALCULATE MESSAGE CHECKSUM FOR SENDING TO APP
//**********************************************************************
byte calcChecksum(int commandCode1, int commandCode2, byte MSB, byte LSB) {
  byte outVal;
  // calculate checksum
  outVal = (byte)(commandCode1) + (byte)(commandCode2) + MSB + LSB;
  return outVal;
}


//**********************************************************************
// FUNCTION TO ECHO BACK A RECEIVED MESSAGE
//**********************************************************************
void echoRecvMsg(char cmd, int cmdCode, byte MSBin, byte LSBin) {
#ifdef DEBUG
#ifdef BLE_DEBUG
  Serial.println(F("echo receive message called"));
#endif
#endif
  sendMsgBLE(cmd, cmdCode, MSBin, LSBin); // do not require echo/ack
}


//**********************************************************************
// FUNCTION TO SEND MSG VIA BLE AND CONFIRM TRANSMISSION SUCCESS
//**********************************************************************
void sendMsgBLE(char cmd, int cmdNum, byte MSBout, byte LSBout) {
  byte checksumOut;

  checksumOut = calcChecksum((int)(cmd), cmdNum, MSBout, LSBout);
  BLE_SERIAL.print('*');
  BLE_SERIAL.print(cmd);
  BLE_SERIAL.write((byte)(cmdNum));
  BLE_SERIAL.write(MSBout);
  BLE_SERIAL.write(LSBout);
  BLE_SERIAL.write(checksumOut);

#ifdef DEBUG
#ifdef BLE_DEBUG
  Serial.print(F("Writing out command: "));
  Serial.print(cmd);
  Serial.print(cmdNum);
  Serial.print(F(" over BLE"));
  Serial.println();
#endif
#endif


} // end of sendMsgBLE function


//**********************************************************************
// INTERRUPT ROUTINES
//**********************************************************************


//**********************************************************************
// ENCODER 1A
//**********************************************************************

void encUpdate1A(void) {
  // check for low to high transition triggering this interrupt
  if (M1_ENC_A_PORT_READ & _BV(M1_ENC_A_PORT_PIN)) {
    // if pin B is also high, CCW movement happened
    if (M1_ENC_B_PORT_READ & _BV(M1_ENC_B_PORT_PIN)) {
      encCountM1--;
    }
    else {
      encCountM1++;
    }
  }
  else {
    // high to low transition triggered interrupt
    if (M1_ENC_B_PORT_READ & _BV(M1_ENC_B_PORT_PIN)) {
      encCountM1++;
    }
    else {
      encCountM1--;
    }
  }
} // end of encUpdate1A

//**********************************************************************
// ENCODER 1B
//**********************************************************************

void encUpdate1B(void) {
  // check for low to high transition triggering this interrupt
  if (M1_ENC_B_PORT_READ & _BV(M1_ENC_B_PORT_PIN)) {
    // if pin A is also high, CW movement happened
    if (M1_ENC_A_PORT_READ & _BV(M1_ENC_A_PORT_PIN)) {
      encCountM1++;
    }
    else {
      encCountM1--;
    }
  }
  else {
    // high to low transition triggered interrupt
    if (M1_ENC_A_PORT_READ & _BV(M1_ENC_A_PORT_PIN)) {
      encCountM1--;
    }
    else {
      encCountM1++;
    }
  }
} // end of encUpdate1B

//**********************************************************************
// ENCODER 2A
//**********************************************************************

void encUpdate2A(void) {
  // check for low to high transition triggering this interrupt
  if (M2_ENC_A_PORT_READ & _BV(M2_ENC_A_PORT_PIN)) {
    // if pin B is also high, CCW movement happened
    if (M2_ENC_B_PORT_READ & _BV(M2_ENC_B_PORT_PIN)) {
      encCountM2--;
    }
    else {
      encCountM2++;
    }
  }
  else {
    // high to low transition triggered interrupt
    if (M2_ENC_B_PORT_READ & _BV(M2_ENC_B_PORT_PIN)) {
      encCountM2++;
    }
    else {
      encCountM2--;
    }
  }
} // end of encUpdate2A

//**********************************************************************
// ENCODER 2B
//**********************************************************************

void encUpdate2B(void) {
  // check for low to high transition triggering this interrupt
  if (M2_ENC_B_PORT_READ & _BV(M2_ENC_B_PORT_PIN)) {
    // if pin A is also high, CW movement happened
    if (M2_ENC_A_PORT_READ & _BV(M2_ENC_A_PORT_PIN)) {
      encCountM2++;
    }
    else {
      encCountM2--;
    }
  }
  else {
    // high to low transition triggered interrupt
    if (M2_ENC_A_PORT_READ & _BV(M2_ENC_A_PORT_PIN)) {
      encCountM2--;
    }
    else {
      encCountM2++;
    }
  }
} // end of encUpdate2B
