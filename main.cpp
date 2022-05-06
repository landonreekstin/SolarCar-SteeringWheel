/* 
Author: Landon Reekstin, Delwys Glokpor, Professor Brian Kamusinga
*/
#include <Arduino.h>
#include <FlexCAN_T4.h>   // CAN system lib
#include <canAddresses.h> // devices "yellow pages"
#include <string.h>
#include <math.h>
#include <genieArduino.h>
#include <SoftwareSerial.h>

/// enable CAN2.0 mode usage ///
// can2 port - used for primary driving functions (motor controller + driveMode+BMS)
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
// can1 port - used for secondary functions (gear change, horn, lights)
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // can1 port

// software serial management
SoftwareSerial screen(34, 35);

// led pin
int led = 13;
// broadcasting timer
IntervalTimer timer;
// for debug prints
bool DEBUG_PRINT = false;

//These allow for the mcu to be restarted
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

/****************
 * 
 * 
This are constants that do not change and are stored in flash to save ram at runtime
etch uses 33376 bytes (1%) of program storage space. Maximum is 2031616 bytes.
Global variables use 66228 bytes (12%) of dynamic memory, leaving 458060 bytes for local variables. Maximum is 524288 bytes.
* 
* 
********/

// Conversion Factors
#define MPH_MS_CONV 2.237
#define KPH_MS_CONV 3.6
#define METERS_TO_MILES_CONV 0.000621371
#define METERS_TO_KM_CONV 0.001
#define TIRE_DIAMETER_IN_METERS 0.62

// display adaptative constants
#define CUR_MULT 0.1
#define CELL_MULTI_VOLT .0001
#define SCREEN_ON 36 // screen power up
#define RESETLINE 33 // screen reset pin
bool SCREEN_RUNNING = false; // keep track of screen state

// global variables
int uni_Vel, uni_Amp; //universal write variables
int iCount = 0;
int iCellCtr = 0;
// true if regen is above min value
bool regenEnable = false;

// display variables
int speedSelect = 0;   // 0 for MPH | 1 for KPH
int currentSelect = 0; // 0 for net | 1 for MC
int mcSpeed = 0;
int Voltage = 0;
int Current = 0;
String currentMC0Errors = ""; // error string presets
String currentMC1Errors = "";
int currentMC0Limits = 0;
int currentMC1Limits = 0;
int CurrentDriveMode = 0;
int CurrentBATDISCurrent = 0;
int CurrentBATCHGCurrent = 0;
int currentOdometer = 0;
int currentPowerUse = 0;
int currentGear = 0;

// conversion unions
union fVals // float to byteArray
{
  byte b[4];
  float fVal;
} motorCurrent, BusCurrent, MotorRPM, busVolt, busCur, vel, mcCur, mcTemp, mcOdo, mcAh, dcCommand, mcVolt, mcRPM, bus1Volt, bus1Cur, vel1, mc1Cur, mc1Temp, mc1Odo, mc1Ah, dc1Command, mc1Volt, mc1RPM;

union inVals // int to byteArray
{
  byte b[2];
  int inVal;
} BATDISCurrent, BATCHGCurrent, ChargerCurrent, ChargerVoltage;

// Genie display object
Genie genie;

/******************************************
 *VFM presets
* 
*******************************************/
unsigned int VFM_GEAR_INDICATOR[8] = {
   0x00, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54, 0x64
}; //percentage out
String VFM_ERRORS[] = {
   "Sensor Error", "Power Error", "Motor Error"
};

/******************************************
 *MC presets
* 
*******************************************/
// error strings
String MC_ERRORS[] = {
   "HWOC", "SWOC", "DC BUS OV", "HALL", "WATCHDOG", 
   "CONFIG ERR", "15V RAIL UV", "DESAT....BAD!!"
};
// 1st motor status data
int limitFlags, errorFlags, activeMotor, canSendErr, canRecErr;
// 2nd motor status data
int limitFlags1, errorFlags1, activeMotor1, canSendErr1, canRecErr1;

/******************************************
 *Battery Pack presets
* 
*******************************************/
// battery pack data 1
int packVolt, packCur, packAHr;
// battery pack data 2
byte relayStatus, currentLimitStatus1, currentLimitStatus2;
int dischargeRelay, chargeRelay, iBPSCounter, dischargeLimit,
   chargeLimit, currentHold, BMSmalfunction, batSOC;
// battery pack data 3
int H_Volt, L_Volt, A_Volt;
//battery pack data 4
int H_Temp, L_Temp, A_Temp;


/******************************************
 *Gear shifting Variables 
* 
*******************************************/
int gearNum = 1;
unsigned long lastShiftTime = 0; // tracks shifting time
#define SHIFT_WAIT 2000.0 //length of time to wait before shifting gears.
#define MIN_GEAR 1
#define MAX_GEAR 4
#define GEAR_INTERVAL 1
#define MINIMUM_HIGVOLTAGE 90

bool motor_High_Voltage_ON = false;
int gearIncrement = 0;
// QUESTION: WHAT'S GOING ON?
unsigned int GEAR_CMD[8][8] = {
   {0x02, 0x00, 0x04, 0x50, 0x00, 0x00, 0x14, 0x14},
   // {0x02, 0x00, 0x74, 0x10, 0x00, 0x00, 0x14, 0x14},
   {0x02, 0x00, 0xE4, 0x10, 0x00, 0x00, 0x14, 0x14},
   //{0x02, 0x00, 0x54, 0x11, 0x00, 0x00, 0x14, 0x14},
   {0x02, 0x00, 0xC4, 0x11, 0x00, 0x00, 0x14, 0x14},
   // {0x02, 0x00, 0x34, 0x12, 0x00, 0x00, 0x14, 0x14},
   // {0x02, 0x00, 0xA4, 0x12, 0x00, 0x00, 0x14, 0x14},
   {0x02, 0x00, 0x24, 0x13, 0x00, 0x00, 0x14, 0x14}
};

// Pins
// Analog (Potentiometer)
#define ACCELERATOR_PIN 21
#define REGEN_PIN 20
#define ADC_RESOLUTION 10
#define ADC_READINGS_AVERAGE_NUM 10

// Digital (Pushbutton)
#define MOTOR_RESET_PIN 2 
#define REVERSE_PIN 8   // got reversed in wiring
#define FORWARD_PIN 15  // with this 
#define NEUTRAL_PIN 9 
#define HORN_PIN 17
#define CRUISE_SET_PIN 6  
#define LEFT_TURN_PIN 10
#define RIGHT_TURN_PIN 12
// digital inputs on new steering wheel
#define CRUISE_CONTROL_PIN 4 // or 5?


//VFM Control
#define UP_GEAR_PIN 11     // got reversed in wiring
#define DOWN_GEAR_PIN 14   // with this
#define HOME_GEAR_PIN 41

//current limits are set in the pack
#define CHARGE_LIMIT 65.0    //Same as the battery max charge current
#define DISCHARGE_LIMIT 65.0 //Same as the battery max discharge current
#define AVERAGE_VELOCITY (vel.fVal + vel1.fVal) / 2
#define AVERAGE_RPM (mcRPM + mc1RPM) / 2

//The slack allowed in the pedals to avoid signal noise
#define PEDAL_SLACK 2
#define REGEN_PEDAL_MAX 397
#define ACCELERATOR_PEDAL_MAX 287
#define REGEN_PEDAL_MIN 300
#define ACCELERATOR_PEDAL_MIN 186
//To limit gathering excessive speed in reverse
#define REVERSE_SPEED_SCALING_FACTOR 0.2
#define MAX_SPEED 70 // Max speed in MPH
#define TIRE_DIAMETER_IN_METERS 0.62
#define MAX_RPM (MAX_SPEED * 2.23694) / (TIRE_DIAMETER_IN_METERS * M_PI * 2) * 60

// Max speed about 70Mph, confirm with tire diameter

//sets the minimum speed for regen to be activated, prevents regen from being turned on if the car is stationary
#define MAX_REGEN_SPEED 5 //Minimum speed for regen to work in MPh
#define MINIMUM_REGEN_RPM (MAX_REGEN_SPEED * 2.23694) / (TIRE_DIAMETER_IN_METERS * M_PI * 2) * 60

// Regen values
#define MIN_REGEN 20

// driving function flags
bool brakeSignal = false;
bool DirectionFoward = true;
bool DirectionReverse = false;
bool motorReset = false;
bool DUAL_MOTOR = false;
bool bShowCells = false;
int acceleratorValue = 0; // value read from the  accelerator pot
int regenValue = 0;       // value read from regen pot
bool regenSignal = false;
char driveMode = 'N'; // Possible values: 'N' (Neutral/Park), 'F' (Forward), or 'R' (Reverse)
char OldDriveMode =' ';// Stores the old values



bool hornSignal = false;
bool leftTurnSignal = false;
bool rightTurnSignal = false;

/*********************************
 * 
 * VERY IMPORTANT TO SET THIS IF TESTING ON THE BENCH, IT DISABLES REGEN!!!!!!!!!!!!!!!
 * *********************************/
bool BENCH_POWERSUPPLY_TESTING = false;

/*********************************
 *  * VERY IMPORTANT TO SET THIS IF TESTING ON THE BENCH, IT DISABLES REGEN!!!!!!!!!!!!!!!
* 
* *********************************/

/*********************************
* display start up
* 
* *********************************/

/**
 * starts the screen by pulling the SCREEN_ON PIN HIGH.
 * flags the screen as running. works only if not
 * SCREEN_RUNNING
 */
void startScreen(){
   if (!SCREEN_RUNNING){
      digitalWrite(SCREEN_ON, HIGH);
      SCREEN_RUNNING = true; // flag screen turned on
   }
}

/**
 * kill the screeb and flag it as off. works only if 
 * SCREEN_RUNNING.
 */
void turnOffScreen(){
   if (SCREEN_RUNNING){
      digitalWrite(SCREEN_ON, LOW);
      SCREEN_RUNNING = false;
   }
}

/**
 * turn the screen off and then turn it back on.
 */
void restartScreen(){
   turnOffScreen();
   startScreen();
}

/*********************************
* utility methods for display
* 
* *********************************/

void checkRegen() {
  if (regenValue > MIN_REGEN) {
    regenEnable = true;
  }
}

/**
* changes the unit in which speed and amps are displayed
* on the display screen.
*/
void screenOptions()
{
   if (speedSelect) // true for MPH | false for KPH
    genie.WriteObject(GENIE_OBJ_STRINGS, 0, 1);
   else
    genie.WriteObject(GENIE_OBJ_STRINGS, 0, 0);

   if (currentSelect) // true for netA |  false mcA
    genie.WriteObject(GENIE_OBJ_STRINGS, 1, 0);
   else
    genie.WriteObject(GENIE_OBJ_STRINGS, 1, 1);
   // change to main screen
}

/**
* compute current speed and write it to display screen.
* lit LED "2" when velocity computed is negative.
*/
void calculateSpeed()
{
   // QUESTION is this comment still relevant?
   //for now this works OK but it needs to be redone so that it accounts for the fact that we have two motors.
   if (DUAL_MOTOR)
   {
      if (!speedSelect)
         uni_Vel = (mc1RPM.fVal + mcRPM.fVal) / 2.0 * TIRE_DIAMETER_IN_METERS * M_PI * 1 / 60.0 * MPH_MS_CONV;
      else
         uni_Vel = (mc1RPM.fVal + mcRPM.fVal) / 2.0 * TIRE_DIAMETER_IN_METERS * M_PI * 1 / 60.0 * KPH_MS_CONV;
   }
   else
   {
      if (!speedSelect)
         uni_Vel = mcRPM.fVal * 0.7 * TIRE_DIAMETER_IN_METERS * 1 / 60 * MPH_MS_CONV;
      else
         uni_Vel = mcRPM.fVal * 0.7 * TIRE_DIAMETER_IN_METERS * 1 / 60 * KPH_MS_CONV;
   }

   if (uni_Vel < 0)
      genie.WriteObject(GENIE_OBJ_USER_LED, 2, 1); // led is turned on when velocity is negative
   else
   {
      // turn LED off
      genie.WriteObject(GENIE_OBJ_USER_LED, 2, 0);
   }
   
   // write to screen
   genie.WriteObject(GENIE_OBJ_LED_DIGITS, 12, abs(uni_Vel * 10));
   // debug prints
   if(DEBUG_PRINT){
      Serial.print("SPEED : ");
      Serial.println(uni_Vel);
   }
}

/**
* compute universal current and write it to display screen.
* lit LED "4" when universal current computed is negative.
*/
void calculateCurrent()
{
   if (DUAL_MOTOR)
   {
      if (currentSelect == 0)
         uni_Amp = (mcCur.fVal + mc1Cur.fVal) * 10.0;
      else
         uni_Amp = busCur.fVal * 10.0;
   }
   else
   {
      if (currentSelect == 0)
         uni_Amp = mcCur.fVal * 10.0;
      else
         uni_Amp = busCur.fVal * 10.0;
   }

   // write pack Current
   if ((uni_Amp * CUR_MULT) < 0)
   {
      genie.WriteObject(GENIE_OBJ_USER_LED, 4, 1);
   } // led is turned on when pack current is negative
   else
   {
      genie.WriteObject(GENIE_OBJ_USER_LED, 4, 0);
   } // led is off when current is positive
   if (DEBUG_PRINT){
      Serial.print("Current sent to display:");
      Serial.println(abs(uni_Amp));
   }
   genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, abs(uni_Amp));

  //reseting values once written on screen
}

/**
* compute the travelled distance and the consumption of
* the car and send them to the display screen.
*/
void calculate_odo_and_poweruse()
{
   if (DUAL_MOTOR)
   {
      currentPowerUse = int((mc1Ah.fVal + mcAh.fVal) * 100);
      if (speedSelect == 0)
      {
         currentOdometer = int((mc1Odo.fVal + mcOdo.fVal) / 2 * METERS_TO_MILES_CONV * 10);
      }
      else
      {
         currentOdometer = int((mc1Odo.fVal + mcOdo.fVal) / 2 * METERS_TO_KM_CONV * 10);
      }
   }
   else
   {
      currentPowerUse = int(mcAh.fVal * 100);
      if (speedSelect == 0)
      {
         currentOdometer = (mcOdo.fVal * METERS_TO_MILES_CONV * 10);
      }
      else
      {
         currentOdometer = (mcOdo.fVal * METERS_TO_KM_CONV * 10);
      }
   }

   genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8, currentOdometer);
   genie.WriteObject(GENIE_OBJ_LED_DIGITS, 9, currentPowerUse);
   // debug print
   if (DEBUG_PRINT){
      Serial.print("Odometer in 0.1  : ");
      Serial.println(currentOdometer);
      Serial.print("Power use in 0.01 Amp Hours : ");
      Serial.println(currentPowerUse);
   }
}

/**
* sends an empty message to the motors base addresses
* which triggers the broadcast of motors' speed as a
* response
*/
void requestSpeed()
{
   CAN_message_t requestSpeedmsg;
   CAN_message_t requestSpeed1msg;
   requestSpeedmsg.id = MC_MOTOR_BASE;
   requestSpeed1msg.id = MC1_MOTOR_BASE;
   requestSpeedmsg.flags.remote = 1;
   requestSpeed1msg.flags.remote = 1;
   for (size_t i = 0; i < 8; i++)
   {
      requestSpeedmsg.buf[i] = 0x00;
      requestSpeed1msg.buf[i] = 0x00;
   }
   can2.write(requestSpeedmsg);
   can2.write(requestSpeed1msg);
}

/**
* 4dscreen main event handler method. responds to the inputs
* sent to the screen by the user via buttons and such.
*/
void screenEvents(void)
{
   genieFrame Event;
   genie.DequeueEvent(&Event);

   int slider_val = 15;

   //If the cmd received is from a Reported Event
   if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
   {
      if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) // If the reported message is from a Winbutton
      {
         if (Event.reportObject.index == 2) // If the reported message is from Current Button
         {
           // Write value to red LED
            speedSelect = !speedSelect;
            if (DEBUG_PRINT)
               Serial.println("Speed Button"); // Write value to red LED
         }
         else if (Event.reportObject.index == 3)
         {
            currentSelect = !currentSelect;
            if (DEBUG_PRINT)
               Serial.println("Current Button"); // Write value to red LED
         }
         screenOptions();
      }else if (Event.reportObject.object == GENIE_OBJ_SLIDER) // If the Reported Message is from a slider
      {
         int inputSlider = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb; // data of message is stored
         slider_val = (inputSlider * 14) / 100 + 1;
         if (DEBUG_PRINT){
            Serial.println(inputSlider);                                                        // Write value to red LED
            Serial.println(slider_val);
         }
         genie.WriteContrast(slider_val);
         // Write value to red LED
      }
   }
}

/*********************************
* utility methods for steering wheel
* 
* *********************************/

/**
* pretty dump the contents of a CAN message on a 
* serial port
*/
void printCanMessage(const CAN_message_t msg)
{
   Serial.print(" BUS: ");
   Serial.print(msg.bus, HEX);
   Serial.println(" , ");
   Serial.print(" TS: ");
   Serial.print(msg.timestamp);
   Serial.println(" , ");
   Serial.print(" ID: ");
   Serial.print(msg.id, HEX);
   Serial.println(" , ");
   Serial.print(" Buffer: ");
   for (uint8_t i = 0; i < msg.len; i++)
   {
    Serial.print(msg.buf[i], HEX);
   }
   Serial.print("\n");
}

/**
* changes the stator position of the motors by sending a
* VFM CAN message to motors; effectively changing gears. 
*/
void ChangeGear()
{
   if (mc1Volt.fVal > MINIMUM_HIGVOLTAGE || mcVolt.fVal > MINIMUM_HIGVOLTAGE)
   {
      if (millis() >= (lastShiftTime + SHIFT_WAIT))
      { // limit to how quickly you can shift up to prevent sliding from 1 -> 4 by holding button down for too long
         lastShiftTime = millis();
         gearNum = gearNum + gearIncrement;
         gearIncrement = 0; //resets the gear increment to 0
      }
      CAN_message_t Gear_MSG;
      Gear_MSG.flags.extended = 1; //Set the can ID to extended
      Gear_MSG.id = DC_VFM;
      for (uint8_t i = 0; i < 8; i++)
      {
         Gear_MSG.buf[i] = GEAR_CMD[gearNum-1][i];
      }
      can2.write(Gear_MSG); // changed to the same as motors
      if (DEBUG_PRINT){
         Serial.print("VMF Gear: ");
         Serial.println(gearNum);
      }
   }
}

/**
* Reads a canbus message and prints it to the serial port.
* made to decode any CAN message via a switch statement.
* the internal "printRawCanMessage" variable can be set to
* true if need is for bit version of the message for
* debugging purposes for example.
* @param msg a pointer to the CAN message to serial print.
*/
void readCanbus(const CAN_message_t &msg)
{
   // change to `true` when debugging CAN traffic,
   // otherwise keep `false`
   bool printRawCanMessage = false; 

   if (printRawCanMessage)
   {
     printCanMessage(msg); // dump message
   }

   // TODO: Add code here to listen to the BMS and set the 
   // CHARGE_LIMIT and the DischargeCurrent limit

   switch (msg.id)
   {
      case VFM: // display version
      {
         unsigned int VFM_Stator_displacement_percentage = 0x00;
         int gear = 0;
         VFM_Stator_displacement_percentage = msg.buf[0];
         // debug print
         if (DEBUG_PRINT){
            Serial.print("Current % stator displacement:");
            Serial.println(VFM_Stator_displacement_percentage);
         }

         for (int i = 0; i < 8; i++)
         {
            if (VFM_Stator_displacement_percentage == VFM_GEAR_INDICATOR[i])
            {
               gear = i + 1;
               if (currentGear != gear)
               {
                  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, i + 1);
                  currentGear = gear;
                  if (DEBUG_PRINT)
                     Serial.println(gear);
                  break;
               }
            }
         }
         // debug print errors
         if (DEBUG_PRINT)
            Serial.print("VFM Errors Present:");
         String errorMessage = "";
         int errorCount = 0;
         // record any error
         for (int i = 1; i < 4; i++)
         {
            if (!bitRead(msg.buf[1], i))
            {
               errorMessage = errorMessage + VFM_ERRORS[i - 1] + ": ";
               errorCount++;
            }
         }
         if (errorCount == 0) // no error encoutered
         {
            if (DEBUG_PRINT)
               Serial.println("None");
            genie.WriteStr(7, errorMessage);
         }else{
            genie.WriteStr(7, errorMessage);
            if (DEBUG_PRINT)
               Serial.println(errorMessage);
         }
         break;
      }
      case BAT_CURRENT_LIMITS:
         BATDISCurrent.b[0] = msg.buf[0];
         BATCHGCurrent.b[0] = msg.buf[1];
         
         // add-on from display
         if (BATCHGCurrent.inVal != CurrentBATCHGCurrent)
         {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 10, BATCHGCurrent.inVal);
            BATCHGCurrent.inVal = CurrentBATCHGCurrent;
         }
         if (CurrentBATDISCurrent != BATDISCurrent.inVal)
         {
            genie.WriteObject(GENIE_OBJ_LED_DIGITS, 11, BATDISCurrent.inVal);
            CurrentBATDISCurrent = BATDISCurrent.inVal;
         }
         break;
      // Tritium motor controller voltage data
      // 0-3: motor bus voltage
      // 4-7: motor bus amperage
      case MC_BUS: // display version
         for (int i = 0; i < 4; i++)
         {
            mcVolt.b[i] = msg.buf[i];
            mcCur.b[i] = msg.buf[i + 4];
         }
         if (DEBUG_PRINT){
            Serial.print("Motor Controller 0 Voltage :");
            Serial.println(mcVolt.fVal);
            Serial.print("Motor Controller 0 Current :");
            Serial.println(mcCur.fVal);
         }
         calculateCurrent();
         break;  
      case MC1_BUS: // display version
         for (int i = 0; i < 4; i++)
         {
            mc1Volt.b[i] = msg.buf[i];
            mc1Cur.b[i] = msg.buf[i + 4];
         }
         if (DEBUG_PRINT){
            Serial.print("Motor Controller 1 Voltage :");
            Serial.println(mc1Volt.fVal);
            Serial.print("Motor Controller 1 Current :");
            Serial.println(mc1Cur.fVal);
         }
         calculateCurrent();
         break;
      // Tritium motor controller velocity data
      // 0-3: motor rpm
      // 4-7: motor velocity (calculated by MC)
      case MC_VELOCITY:
         for (int i = 0; i < 4; i++)
         {
            mcRPM.b[i] = msg.buf[i];
            vel.b[i] = msg.buf[i + 4];
         }
         // debug print
         if (DEBUG_PRINT){
            Serial.print("Motor 0 velocity:  ");
            Serial.println(vel.fVal);
         }
         calculateSpeed(); // add-on from display
         break;
      case MC1_VELOCITY:
         for (int i = 0; i < 4; i++)
         {
            mc1RPM.b[i] = msg.buf[i];
            vel1.b[i] = msg.buf[i + 4];
         }
         
         // debug print
         if (DEBUG_PRINT){
            Serial.print("Motor 1 velocity: ");
            Serial.println(vel1.fVal);
         }
         calculateSpeed(); // add-on from display
         break;
      case BRAKE:
         if (msg.buf[0] == 0x42)
         {
            if (!brakeSignal)
            {
               if (DEBUG_PRINT){
                  Serial.println("Brake Signal is On");
               }
               brakeSignal = true;
            }
        }else{
           if (DEBUG_PRINT)
               Serial.println("Brake Signal is Off");
            brakeSignal = false;
        }
        break;
      // BROUGHT FROM DISPLAY
      // Tritium motor controller temp sensor data
      // 0-3: motor temp
      // 4-7: heatsink temp
      case MC_TEMP1:
         for (int i = 0; i < 4; i++)
         {
            mcTemp.b[i] = msg.buf[i + 4];
         }
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 14, (int)(mcTemp.fVal * 10));
         if (DEBUG_PRINT){
            Serial.print("Motor Controller 1 IPM Temperature :");
            Serial.println(mcTemp.fVal);
         }
         break;
      case MC1_TEMP1:
         for (int i = 0; i < 4; i++)
         {
            mc1Temp.b[i] = msg.buf[i + 4];
         }
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 15, (int)(mc1Temp.fVal * 10));
         if (DEBUG_PRINT){
            Serial.print("Motor Controller 1 IPM Temperature :");
            Serial.println(mc1Temp.fVal);
         }
         break;
      // Tritium motor controller status data
      // 7: Can receive error
      // 6: Can send error
      // 4-5: Active Motor
      // 2-3: Error Flags
      // 0-1: Limit Flags
      case MC_STATUS:
      {
         limitFlags = msg.buf[0];  //
         errorFlags = msg.buf[2];  //
         activeMotor = msg.buf[4]; //
         canSendErr = msg.buf[6];
         canRecErr = msg.buf[7];
         String mc0Errors = " ";
         for (size_t i = 0; i < 7; i++)
         {
            if (bitRead(errorFlags, i))
            {
               mc0Errors = mc0Errors + MC_ERRORS[i] + ": ";
            }
         }

         if (currentMC0Errors != mc0Errors)
         {
            genie.WriteStr(2, mc0Errors);
         }
         currentMC0Errors = mc0Errors;

         if (limitFlags != currentMC0Limits)
         {
            for (size_t i = 0; i < 7; i++)
            {
               if (bitRead(limitFlags, i))
               {
                  genie.WriteObject(GENIE_OBJ_STRINGS, 3, i + 1);
                  break;
               }
            }
            currentMC0Limits = limitFlags;
         }
         else if (limitFlags == 0)
         {
            genie.WriteObject(GENIE_OBJ_STRINGS, 3, 0);
         }
         break;
      }
      case MC1_STATUS:
      {
         limitFlags1 = msg.buf[0];  //
         errorFlags1 = msg.buf[2];  //
         activeMotor1 = msg.buf[4]; //
         canSendErr1 = msg.buf[6];
         canRecErr1 = msg.buf[7];

         String mc1Errors = " ";
         for (size_t i = 0; i < 7; i++)
         {
            if (bitRead(errorFlags1, i))
            {
              mc1Errors = mc1Errors + MC_ERRORS[i] + ":";
            }
         }

         if (currentMC1Errors != mc1Errors)
         {
            genie.WriteStr(2, mc1Errors);
         }
         currentMC1Errors = mc1Errors;
         if (limitFlags != currentMC1Limits)
         {
            for (size_t i = 0; i < 7; i++)
            {
               if (bitRead(limitFlags1, i))
               {
                  genie.WriteObject(GENIE_OBJ_STRINGS, 5, i + 1);
                  break;
               }
            }
         }
         else if (limitFlags1 == 0)
         {
         genie.WriteObject(GENIE_OBJ_STRINGS, 5, 0);
         }
         currentMC1Limits = limitFlags;
         break;
      }
      // Tritium motor controller Odo and Ah measurements - base address + 14
      // 22-25: Distance the vehicle has travelled since reset in m
      // 26-29: Charge flow into the controller DC bus from the time of reset in Ah
      case MC1_DIST:
         for (int i = 0; i < 4; i++)
         {
            mc1Odo.b[i] = msg.buf[i];
            mc1Ah.b[i] = msg.buf[i + 4];
         }
         calculate_odo_and_poweruse();
         break;
      case MC_DIST:
         // read data bytes
         for (int i = 0; i < 4; i++)
         {
            mcOdo.b[i] = msg.buf[i];
            mcAh.b[i] = msg.buf[i + 4];
         }
         calculate_odo_and_poweruse();
         break;
      // Orion BMS data pack 1/B
      // 0-1: pack current
      // 2-3: pack voltage
      // 5-6: Relay State
      /*
      This parameter contains the status (active state) of all 
      the BMS outputs and inputs. The format is as follows:
      Bit #0 (0x01): Discharge relay enabled
      Bit #1 (0x02): Charge relay enabled
      Bit #2 (0x04): Charger safety enabled
      Bit #3 (0x08): Malfunction indicator active (DTC status)
      Bit #4 (0x10): Multi-Purpose Input signal status
      Bit #5 (0x20): Always-on signal status
      Bit #6 (0x40): Is-Ready signal status
      Bit #7 (0x80): Is-Charging signal status

      Bit #8 (0x0100): Multi-Purpose Input #2 signal status
      Bit #9 (0x0200): Multi-Purpose Input #3 signal status
      Bit #10 (0x0400): RESERVED
      Bit #11 (0x0800): Multi-Purpose Output #2 signal status
      Bit #12 (0x1000): Multi-Purpose Output #3 signal status
      Bit #13 (0x2000): Multi-Purpose Output #4 signal status
      Bit #14 (0x4000): Multi-Purpose Enable signal status
      Bit #15 (0x8000): Multi-Purpose Output #1 signal status

      For all of these values, a "1' signifies that the signal or relay in question is enabled or active (high). A "0' indicates that it is inactive or off (low).
      */
      //7 : Checksum
      case BMS_PACK_1:
         // convert byte[2] to short int
         packCur = ((int)msg.buf[0] << 8) | (int)msg.buf[1];
         
         // display pack voltage
         packVolt = ((unsigned int)msg.buf[2] << 8) | (unsigned int)msg.buf[3];
         if (DEBUG_PRINT){
            Serial.print("Battery Pack Current:");
            Serial.println(packCur);
            Serial.print("Battery Pack Voltage");
            Serial.println(packVolt);
         }
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, packVolt);
         calculateCurrent();
         
         // display SOC as calculated by BMS
         if(batSOC!=msg.buf[4])
         {
            batSOC = msg.buf[4];
            genie.WriteObject(GENIE_OBJ_GAUGE, 0, int(batSOC*0.5));
            genie.WriteStr(10, String(round(batSOC*0.5))+"%");
         }
         if (DEBUG_PRINT){
            Serial.print("Battery Pack SOC:");
            Serial.println(String(batSOC*0.5) +"%");
         }

         //Relay Status
         relayStatus = msg.buf[6];
         dischargeRelay = bitRead(relayStatus, 0);
         if (dischargeRelay == 1)
         {
            genie.WriteObject(GENIE_OBJ_USER_LED, 0, 1); // led is turned on when discharge is on
            if (DEBUG_PRINT)
               Serial.print("discharge is on:");
         }
         else
         { 
            genie.WriteObject(GENIE_OBJ_USER_LED, 0, 0); // led is off when discharge is off
            if (DEBUG_PRINT)
               Serial.print("discharge is off:");
         }
         chargeRelay = bitRead(relayStatus, 1);
         if (chargeRelay == 1)
         {
            genie.WriteObject(GENIE_OBJ_USER_LED, 1, 1); // led is turned on when charge is on
            if (DEBUG_PRINT)
               Serial.print("charge is on:");
         }
         else
         {
            genie.WriteObject(GENIE_OBJ_USER_LED, 1, 0); // led is off when charge is off
            if (DEBUG_PRINT)
               Serial.print("charge is off:");
         }
         
         // display errors
         if (BMSmalfunction == 1)
            genie.WriteObject(GENIE_OBJ_USER_LED, 3, 1); // led is turned on when charge is on
         else
            genie.WriteObject(GENIE_OBJ_USER_LED, 3, 0); // led is off when charge is off
         break;
      // Orion BMS data pack 2/C
      // 0: QUESTION is there supposed to be more here?
      case BMS_PACK_2:
         packAHr = ((unsigned int)msg.buf[4] << 8) | (unsigned int)msg.buf[5];
         genie.WriteObject(GENIE_OBJ_LED_DIGITS,13, (int)(packAHr * 10));
         break;
      // Orion BMS data pack 3/D
      // QUESTION same
      case BMS_PACK_3:
         H_Volt = ((unsigned int)msg.buf[0] << 8) | (unsigned int)msg.buf[1];
         L_Volt = ((unsigned int)msg.buf[2] << 8) | (unsigned int)msg.buf[3];
         A_Volt = ((unsigned int)msg.buf[4] << 8) | (unsigned int)msg.buf[5];
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, (int)(H_Volt / 10));
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, (int)(L_Volt / 10));
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, (int)(A_Volt / 10));
         if (DEBUG_PRINT){
            Serial.print("Highest Cell volatge in 0.1mv: ");
            Serial.println(H_Volt);
            Serial.print("Lowest Cell Voltage in 0.1mv: ");
            Serial.println(L_Volt);
            Serial.print("Average Cell Voltage in 0.1mv: ");
            Serial.println(A_Volt);
         }
         break;
      // Orion BMS data pack 4/E
      //
      case BMS_PACK_4:
         H_Temp = (unsigned int)msg.buf[2];
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, H_Temp * 10);
         L_Temp = (unsigned int)msg.buf[3];
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, L_Temp);
         if (DEBUG_PRINT){
            Serial.print("Highest Cell Temperature: ");
            Serial.println(H_Temp);
            Serial.print("Lowest Cell Temperature: ");
            Serial.println(L_Temp);
         }
         break;
   } // switch END
}

/**
* reset motor controllers
*/
void motorResetCommand()
{
   CAN_message_t MC_Reset_MSG;
   MC_Reset_MSG.id = DC_RESET;
   for (size_t i = 0; i < 8; i++)
   {
      MC_Reset_MSG.buf[i] = 0x00;
   }
   can2.write(MC_Reset_MSG);
   // delay(250);
   if (DEBUG_PRINT)
      Serial.println("Motor controller Reset...........");
}

/**
* respond to steering wheel pedals input.
*/
void readPedals()
{
   // debug pack current
   if (DEBUG_PRINT){
      Serial.print("Pack_Discharge_Current_Limit: ");
      Serial.println(BATDISCurrent.inVal);
      Serial.print("Pack_Charge_Current_Limit: ");
      Serial.println(BATCHGCurrent.inVal);
   }
   
   // adjust calculations in case of bench setup
   if (BENCH_POWERSUPPLY_TESTING)
   {
      BATDISCurrent.inVal = 10;
   }
   
   // compute accelerator and regen values
   acceleratorValue = analogRead(ACCELERATOR_PIN) - PEDAL_SLACK;
   regenValue = analogRead(REGEN_PIN) - PEDAL_SLACK;

   if (regenSignal && ((mcRPM.fVal > MINIMUM_REGEN_RPM) || 
      (mc1RPM.fVal > MINIMUM_REGEN_RPM))){
      BusCurrent.fVal = (float)(BATCHGCurrent.inVal) / CHARGE_LIMIT;
      regenValue = min(regenValue, REGEN_PEDAL_MAX);

      if (regenValue > REGEN_PEDAL_MIN)
      {
         motorCurrent.fVal = (float)(regenValue - REGEN_PEDAL_MIN) / (REGEN_PEDAL_MAX - REGEN_PEDAL_MIN);
      }
      else
      {
         motorCurrent.fVal = 0.0;
      }
      if (BENCH_POWERSUPPLY_TESTING)
      {
         MotorRPM.fVal = MAX_RPM;
      }
      else
      {
         MotorRPM.fVal = 0;
      } //needs to be set to 0 for regen to work it has been set to 2000 so we don't blow up the power supply
   }

   //only accelerate if the brake signal has not been triggered
   if (!brakeSignal && !regenSignal)
   {
      acceleratorValue = min(acceleratorValue, ACCELERATOR_PEDAL_MAX);
      BusCurrent.fVal = (float)(BATDISCurrent.inVal) / DISCHARGE_LIMIT; //Maximum current allowed by BMS

      //Setting RPM
      if (driveMode == 'F')
      {
         MotorRPM.fVal = MAX_RPM;
      }
      else if (driveMode == 'R')
      {
         MotorRPM.fVal = -1 * MAX_RPM * REVERSE_SPEED_SCALING_FACTOR;
      }
      else if (driveMode == 'N')
      {
         MotorRPM.fVal = 0;
      }

      //setting acceleration/current
      if ((driveMode == 'F' || driveMode == 'R') && acceleratorValue > ACCELERATOR_PEDAL_MIN)
      {
         motorCurrent.fVal = (float)(acceleratorValue - ACCELERATOR_PEDAL_MIN) / (ACCELERATOR_PEDAL_MAX - ACCELERATOR_PEDAL_MIN);
      }
      else
      {
         motorCurrent.fVal = 0;
      }
   }
   
   // debug prints
   if (DEBUG_PRINT){
      Serial.print("Drive Mode = ");
      Serial.println(driveMode);
      Serial.print("accelerator = ");
      Serial.println(acceleratorValue);
      Serial.print("Regen = ");
      Serial.println(regenValue);
   }
}

/**
* build and send configuration CAN messages to the motors.
*/
void sendMotorCommands()
{
   if (motorReset)
   {
      motorResetCommand(); // send a reset message
      motorReset = false;  // and clear flag
   }
   
   // build steering wheel switches message
   CAN_message_t SwitchesMSG;
   SwitchesMSG.id = DC_SWITCH;
   // write driving modes status bits
   bitWrite(SwitchesMSG.buf[0], 0, (driveMode == 'R') ? 1 : 0); //Reverse
   bitWrite(SwitchesMSG.buf[0], 1, (driveMode == 'N') ? 1 : 0); //Neutral
   bitWrite(SwitchesMSG.buf[0], 2, regenSignal);                //Regen
   bitWrite(SwitchesMSG.buf[0], 3, (driveMode == 'F') ? 1 : 0); //Forward

   bitWrite(SwitchesMSG.buf[0], 4, 0);           //Accesories  Not used
   bitWrite(SwitchesMSG.buf[0], 5, 0);           //Run  Not used
   bitWrite(SwitchesMSG.buf[0], 6, 0);           //start Not Used
   bitWrite(SwitchesMSG.buf[0], 7, brakeSignal); //Brakes

   SwitchesMSG.buf[7] = 0x0F; // QUESTION no error check?
   for (uint8_t i = 1; i < 7; i++)
   {
      SwitchesMSG.buf[i] = 0x00; // fill with "0"s
   }

   can2.write(SwitchesMSG); // broadcast switches infos
   
   // build driver controls drive and power allocation messages
   readPedals();
   CAN_message_t DriverControlsDriveMSG;
   DriverControlsDriveMSG.id = DC_DRIVE;
   CAN_message_t DC_POWERCMD_MSG;
   DC_POWERCMD_MSG.id = DC_POWER;

   // drive command
   if (DEBUG_PRINT){
      Serial.print("Motor RPM:  "); // debug print 
      Serial.println(MotorRPM.fVal);// debug print
   }
   for (uint8_t i = 0; i < 4; i++)
   {
      DriverControlsDriveMSG.buf[i] = MotorRPM.b[i];
   }
   if (DEBUG_PRINT){
      Serial.print("Motor Current:  ");
      Serial.println(motorCurrent.fVal);
   }
   for (uint8_t i = 0; i < 4; i++)
   {
      DriverControlsDriveMSG.buf[i + 4] = motorCurrent.b[i];
   }
   can2.write(DriverControlsDriveMSG); // broadcast

   if (regenSignal)
   {
      BusCurrent.fVal = (float)(BATCHGCurrent.inVal) / CHARGE_LIMIT;
   }
   else
   {
      BusCurrent.fVal = (float)(BATDISCurrent.inVal) / DISCHARGE_LIMIT;
   }

   //BusCurrent.fVal = 0.5; //remove after connecting BMS

   for (uint8_t i = 0; i < 4; i++)
   {
      DC_POWERCMD_MSG.buf[i + 4] = BusCurrent.b[i];
   }

   can2.write(DC_POWERCMD_MSG);
}

/**
* build and send turn signals states to primary CAN.
*/ 
void send_Turn_signals() {
  CAN_message_t TurnSignal_MSG;
  TurnSignal_MSG.id = DC_TURN_SIGNALS;
  // QUESTION are these still correct?
  const int leftTurnCmd = 0x4C;  // LeftTurn
  const int rightTurnCmd = 0x52; // RightTurn

   if (leftTurnSignal || rightTurnSignal) {
      // since turnSignal is on rocker switch we make the assumption
      // that leftTurnSignal and rightTurnSignal will not both be true
      // so we only send one of each command.
      TurnSignal_MSG.buf[0] = leftTurnSignal ? leftTurnCmd : rightTurnCmd;
      // debug print
      if (DEBUG_PRINT){
         Serial.print("Turn Signal : ");
         Serial.println((char)TurnSignal_MSG.buf[0]);
      }
   }else {
      // deactivate signals
      TurnSignal_MSG.buf[0] = 0x00;
   }
  
   if (regenSignal) {
      TurnSignal_MSG.buf[1] = 0x42; // QUESTION this is brake light command?
      if (DEBUG_PRINT){
         Serial.print("Regening, turn on brake lights: ");
         Serial.println((char)TurnSignal_MSG.buf[1]);
      }
   }else {
      TurnSignal_MSG.buf[1] = 0x00;
   }

   for (size_t i = 2; i < 8; i++) {
      TurnSignal_MSG.buf[i] = 0x00;
   }
   // can1.write(TurnSignal_MSG);
   can2.write(TurnSignal_MSG);
   leftTurnSignal = false; //cancel the signal
   rightTurnSignal = false;
}

/**
* build and send horn state to primary CAN
*/
void sendHorn() {
   if (hornSignal) {
      CAN_message_t Horn_MSG;
      Horn_MSG.id = HORN;
      Horn_MSG.buf[0] = 0x32;
      for (size_t i = 1; i < 8; i++)
      {
         Horn_MSG.buf[i] = 0x00;
      }
      can2.write(Horn_MSG); // changed to can2
      if (DEBUG_PRINT)
         Serial.println("Horn Blowing");
   }
}

/*********************************
* input response & interupts methods
* 
**********************************/

/**
* change value of "gearIncrement" to a positive constant
* to allow increasing the gear. the maximum gear value
* achievable is "MAX_GEAR".
*/
void Upgear_func() {
   if (gearNum < MAX_GEAR) {
      gearIncrement = GEAR_INTERVAL;
   }
   if(DEBUG_PRINT)
      Serial.println("\n\nUPSHIFT!!!");
}

/**
* change value of "gearIncrement" to a negative constant
* to allow increasing the gear. the minimum gear value
* achievable is "MIN_GEAR".
*/
void Downgear_func() {
   if (gearNum > MIN_GEAR) {
      gearIncrement = -GEAR_INTERVAL;
   }
   if(DEBUG_PRINT)
      Serial.println("\n\nDOWNSHIFT!!!");
}

/**
* downgrade "gearNum" to "MIN_GEAR"
*/
void Homegear_func() {
   gearNum = MIN_GEAR;
   if(DEBUG_PRINT)
      Serial.println("\n\nGEAR TO 0!!!");
}

/**
* toggle "motorReset" flag
*/
void motorReset_func() {
   motorReset = !motorReset;
   if(DEBUG_PRINT)
      Serial.println("\n\nMOTOR RESET!!!");
}

/**
* flag left turn signal
*/
void Left_func() {
   leftTurnSignal = true;
   //if(DEBUG_PRINT)
      Serial.println("\n\nLEEEEEEEFFFFFFFTTTTTTT!!!");
}

/**
* flag right turn signal
*/
void Right_func() {
   rightTurnSignal = true;
   //if(DEBUG_PRINT)
      Serial.println("\n\nRIGGGGGGGGGHHHHHHHHTTTTTT!!!");
}

/**
* change drive mode to forward
*/
void Forward_Func() {
  // Don't allow switching into Forward if motor is spinning
   if (mcRPM.fVal < 50 || AVERAGE_VELOCITY > 0) {
      driveMode = 'F';
      genie.WriteObject(GENIE_OBJ_STRINGS, 6, 2);
   }
   // debug print
   //if (DEBUG_PRINT)
      Serial.println("\n\nGOING FORWARD!!!");
}

/**
* change drive mode to reverse
*/
void Reverse_Func() { // Don't allow switching into Reverse if motor is spinning
   if (mcRPM.fVal < 50 || AVERAGE_VELOCITY < 0) {
      driveMode = 'R';
   }
}
/*
*Change the drive mode displayed in the screen
*/
void DriveModeChange(){

if(OldDriveMode!=driveMode){
         OldDriveMode = driveMode;
if (driveMode=='F'){
      genie.WriteObject(GENIE_OBJ_STRINGS, 6, 2);
}else if (driveMode=='R'){
      genie.WriteObject(GENIE_OBJ_STRINGS, 6, 1);
}else if (driveMode=='N'){
      genie.WriteObject(GENIE_OBJ_STRINGS, 6, 0);
}

}

}
/**
* change drive mode to neutral
*/
void Neutral_Func() {
   driveMode = 'N';
}

/**
* toggle "regenSignal" flag
*/
void Brake_Func() {
   regenSignal = !regenSignal;
   if(DEBUG_PRINT)
      Serial.println("\n\nBRAKING AND REGENING!!!");
}

/**
* toggle "hornSignal" flag
*/
void Horn_Func() {
   hornSignal = !hornSignal;
   if(DEBUG_PRINT)
      Serial.println("\n\nHOOOOOOONNNNKKKKIIINNNNNNG!!!");
}

/*********************************
* broadcast bottleneck method
* 
**********************************/
/**
* helper that groups all methods that send CAN messages
* in a single call.
*/
void sendframe(){
   sendMotorCommands();
   ChangeGear();
   send_Turn_signals();
   sendHorn();
   // display writing already happens directly so no
   // need to include them in bottleneck
}

/*********************************
* Teensy runtime config
*
**********************************/

void setup(void) {
   analogReadRes(ADC_RESOLUTION);                 // set ADC resolution to this many bits
   analogReadAveraging(ADC_READINGS_AVERAGE_NUM); // average this many readings  //Serial.begin(115200);
   Serial.println("Principia Solar Car Driver controls");
   /****************************************************************************
    *     Switches setup All switches should have interupts attached to them.   *
    * **************************************************************************/
   // Digital Pin activation
   pinMode(MOTOR_RESET_PIN, INPUT_PULLUP);
   pinMode(REVERSE_PIN, INPUT_PULLUP);
   pinMode(FORWARD_PIN, INPUT_PULLUP);
   pinMode(NEUTRAL_PIN, INPUT_PULLUP);
   pinMode(HORN_PIN, INPUT_PULLUP);
   pinMode(CRUISE_SET_PIN, INPUT_PULLUP);
   // pinMode(regenEnable, INPUT_PULLUP);
   pinMode(LEFT_TURN_PIN, INPUT_PULLUP);
   pinMode(RIGHT_TURN_PIN, INPUT_PULLUP);
   pinMode(UP_GEAR_PIN, INPUT_PULLUP);
   pinMode(DOWN_GEAR_PIN, INPUT_PULLUP);
   pinMode(HOME_GEAR_PIN, INPUT_PULLUP);
   pinMode(SCREEN_ON, OUTPUT);

   // power screen and init 4D screen
   // digitalWrite(SCREEN_ON, HIGH); unneeded cause not used anymore
   Serial.println("Principia Solar Car Driver controls\0");
   Serial6.begin(200000); //connected to 4dscreen
   genie.Begin(Serial6);
   pinMode(RESETLINE, OUTPUT); // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
   digitalWrite(RESETLINE, 0); // Reset the Display
   delay(5000);
   digitalWrite(RESETLINE, 1); // unReset the Display
   delay(3300);                //let the display start up after the reset (This is important)
   Serial.println("Screen Reset");
   genie.AttachEventHandler(screenEvents);


   //Pin interupts
   attachInterrupt(REVERSE_PIN, Reverse_Func, FALLING);
   attachInterrupt(FORWARD_PIN, Forward_Func, FALLING);
   attachInterrupt(NEUTRAL_PIN, Neutral_Func, FALLING);
   attachInterrupt(MOTOR_RESET_PIN, motorReset_func, FALLING);
   attachInterrupt(HORN_PIN, Horn_Func, CHANGE);
   //attachInterrupt(CruiseSetPin, func, LOW);
   //attachInterrupt(regenEnable, Brake_Func, CHANGE);
   attachInterrupt(LEFT_TURN_PIN, Left_func, FALLING);
   attachInterrupt(RIGHT_TURN_PIN, Right_func, FALLING);
   attachInterrupt(UP_GEAR_PIN, Upgear_func, FALLING);
   attachInterrupt(DOWN_GEAR_PIN, Downgear_func, FALLING);
   attachInterrupt(HOME_GEAR_PIN, Homegear_func, FALLING);

   // CAN networks config
   can2.begin();
   can2.setBaudRate(500000); // 500kbps data rate
   can2.enableFIFO();
   can2.enableFIFOInterrupt();
   can2.onReceive(FIFO, readCanbus);
   can2.mailboxStatus();

   can1.begin();
   can1.setBaudRate(500000); // 250kbps data rate
   can1.enableFIFO();
   can1.enableFIFOInterrupt();
   can1.onReceive(FIFO, readCanbus);
   can1.mailboxStatus();

   timer.begin(sendframe, 50000); // Send frame every 50ms--100ms
   motorResetCommand();
}

void loop() { 
   // calls library each loop to process the queued responses
   // from the display
   genie.DoEvents(); 
   DriveModeChange();
   can2.events(); // handle message requests on channel 2
   can1.events(); // handle message requests on channel 1
   // wait 2 milliseconds before the next loop for the 
   // analog-to-digital converter to settle after the
   // last reading:
   delay(2);
}