//#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <MD_UISwitch.h>
#include <hd44780.h>
// #include <RCSwitch.h>
#include <EEPROM.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <PID_v1.h>

#define VERSION "GSR Cheese V5"

#define NO_COUNTDOWN  0

//Constants for the recipe - could be in EEPROM if need to expand or vary
#define HOLD_TIME 5400000  // 90 mins
#define STIR_TIME 300000 // 5 mins
#define REST_TIME 900000 // 15 mins
#define WAIT_TIME 900000 // 15 mins
#define TARGET_TEMP 31.5

// #define RCPin A1
#define ONE_WIRE_BUS A2



boolean relay = false;
boolean old_relay = true;
boolean beeping = false;

// initialise the switch structures
const uint8_t ANALOG_SWITCH_PIN = A0;       // switches connected to this pin

// These key values work for most LCD shields
MD_UISwitch_Analog::uiAnalogKeys_t kt[] =
{
  {  20, 20, 'R' },  // Right
  { 205, 20, 'U' },  // Up
  { 405, 20, 'D' },  // Down
  { 621, 20, 'L' },  // Left
  { 822, 20, 'S' },  // Select
};
MD_UISwitch_Analog Buttons(ANALOG_SWITCH_PIN, kt, ARRAY_SIZE(kt));


// initialize the library with the numbers of the interface pins
const int pin_RS = 8; // arduino pin wired to LCD RS
const int pin_EN = 9; // arduino pin wired to LCD EN
const int pin_d4 = 4; // arduino pin wired to LCD d4
const int pin_d5 = 5; // arduino pin wired to LCD d5
const int pin_d6 = 6; // arduino pin wired to LCD d7
const int pin_d7 = 7; // arduino pin wired to LCD d8

const int pin_BL = 10; // arduino pin wired to LCD backlight circuit
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

OneWire oneWire(ONE_WIRE_BUS);
const int ledPin = 13;
const int tonePin = 12;
const int ssrPin = 11;

// Create a thermocouple instance with software SPI on three
// analog pins.
#define MAXDO   17 //(a3)
#define MAXCS   18  //(a4)
#define MAXCLK  19 // (a5)

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;


volatile long onTime = 0;
unsigned long startTime = 0;

// pid tuning parameters
// one set for initial ramp up to SP
double Kp;
double Ki;
double Kd;

// and a second set of maintaining SP
double Kp2;
double Ki2;
double Kd2;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;
const int Kp2Address = 32;
const int Ki2Address = 40;
const int Kd2Address = 48;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000;
unsigned long windowStartTime;

// ************************************************
// DiSplay Variables and constants
// ************************************************

#define BUTTON_SHIFT BUTTON_SELECT

unsigned long lastButtonPressTime = 0; // last button press

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

enum heatingState { HEAT_AUTO = 0, HEAT_ON, HEAT_OFF};
heatingState heaterState = HEAT_AUTO;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, TUNE_P2, TUNE_I2, TUNE_D2, A };
operatingState opState = OFF;
//
// Process States
//
// HEATING
//		Enter - on entering opState RUN
//		Leave - up key (usually after beep indicating temperature reached)
//
// _HEATING
//		Enter - up key from previous state
//		Leave - up key (usually after starter and moulds added)
//
// HOLDING
//		Enter - up key from previous state
//		Leave - up key (usually after beep indicating elapsed time reached (90mins))
//
// _HOLDING
//		Enter - up key from previous state
//		Leave - up key (usually after rennet added)
//
// WAIT_FOR_FLOC
//		Enter - up key from previous state
//		Leave - on floc point being reached
//
//
// WAIT_FOR_CUT
//		Enter - key press on floc point being reached
//		Leave - on coagulation time reached
//
// CUT
//		Enter - key press after  coagulation time reached
//		Leave - up key (after cutting)
//
// WAIT
//    Enter - up key (after cutting)
//    Leave - on key after elapsed time (15min)
//
// STIR1
//		Enter - up key (after cutting)
//		Leave - on key after stir time elapsed (5min)
//
// STIR2
//		Enter - up key (after stiring)
//		Leave - on key after stir time elapsed (5min)
//
// STIR3
//		Enter - up key (after stiring)
//		Leave - on key after stir time elapsed (5min)
//
// REST
//    Enter - up key (after stiring)
//    Leave - on key after rest time elapsed (15min).
//
// MOULDS
//    Enter - on stir time elapsed (15min).
//    Leave - up key (after resting)
//
// DONE
//		Enter - up key (after stiring)
//		Leave - up key (after stiring)
//

enum processState { NOT_STARTED = 0, HEATING, _HEATING, HOLDING, _HOLDING, WAIT_FOR_FLOC, WAIT_FOR_CUT, CUT,
                    WAIT, STIR1, STIR2, STIR3, REST, MOULDS, DONE
                  };
processState procState = NOT_STARTED;
processState oldProcState = DONE;
#define NUMBER_OF_MODES 14
char heaterOnSymbol = '*';
char heaterOffSymbol = ' ';

void turnHeaterOn () {
  if (heaterState != HEAT_OFF)
    digitalWrite(ssrPin, HIGH); /*mySwitch.switchOn(1, 1);*/ lcd.setCursor(15, 1); lcd.print(heaterOnSymbol);
}

void turnHeaterOff () {
  if (heaterState != HEAT_ON)
    digitalWrite(ssrPin, LOW); /*mySwitch.switchOff(1, 1);*/ lcd.setCursor(15, 1); lcd.print(heaterOffSymbol);
}


double readTemperature () {
  static double input[5] = {0, 0, 0, 0, 0};
  short int number = 0;
  double temperature;
  float k = 0.1;
  static double oldTemperature = 0.0;

  temperature = thermocouple.readCelsius();
  if (isnan(temperature)) {
    /*Serial.println("Something wrong with thermocouple!");*/
    lcd.setCursor(7, 1);
    lcd.print(F("E"));
    temperature = oldTemperature; // Use last reading instead as this one is no good
  }

  if (oldTemperature != 0.0) temperature = k * temperature + (1 - k) * oldTemperature;
  oldTemperature = temperature;
  return (temperature);

}

void setup() {
  Serial.begin(230400);
  Buttons.begin();
  Buttons.enableDoublePress(true);
  Buttons.enableLongPress(false);
  Buttons.enableRepeat(false);
  Buttons.enableRepeatResult(true);
  //Buttons.setPressTime (100);

  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  // pinMode(RCPin, OUTPUT);
  pinMode(tonePin, OUTPUT);
  pinMode (ssrPin, OUTPUT);
  digitalWrite(tonePin, LOW);

  int i;
  for (i = 0; i < 120; i++) // When a frequency sound
  {
    digitalWrite (tonePin, HIGH) ; //send tone
    delay (1) ;
    digitalWrite (tonePin, LOW) ; //no tone
    delay (1) ;
  }

  digitalWrite(tonePin, HIGH);
  //delay(2000);

  //mySwitch.enableTransmit(RCPin);
  //mySwitch.setRepeatTransmit (3);

  // mySwitch.switchOn(1, 1);
  //int n = millis();
  turnHeaterOff();		// Ensure that we start with the output off
  // int t = millis() - n;

  lcd.begin(16, 2);
  lcd.print(F(VERSION));   lcd.setCursor(0, 1);   lcd.print(F("Temperature Control"));
  lcd.setCursor (0, 1);
  delay(2000);  // Splash screen

  // Initialize the PID and related variables
  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);
  Serial.print ("Kp ");
  Serial.println (Kp);
  Serial.print ("Ki ");
  Serial.println (Ki);
  Serial.print ("Kd ");
  Serial.println (Kd);
  //Serial.println ();
  Serial.println ("SetPoint timeChange Input Error Ki*Error outputSum Kp*dInput outputSum outputSum Kd*DInput Output");
  if (myPID.GetProportionalType() == P_ON_E) Serial.println ("Proportional on Error"); else Serial.println ("Proportional on Measurement");
  myPID.SetSampleTime(10000);
  myPID.SetOutputLimits(0, WindowSize);
  //Run timer2 interrupt every 15 ms
  noInterrupts();
  // TCCR2A = 0;
  // TCCR2B = 1 << CS22 | 1 << CS21 | 1 << CS20;

  //Timer2 Overflow Interrupt Enable
  // TIMSK2 |= 1 << TOIE2;

  // Run timer 1 200ms
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 10 Hz increments
  OCR1A = 49999; // = 16000000 / (64 * 5) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER1_COMPA_vect)
{
  if (opState == OFF)
  {
    relay = false;
  }
  else
  {
    DriveOutput();
  }
}

void loop() {

  /* while (true) {
    MD_UISwitch::keyResult_t s = Buttons.read();

    if (s == MD_UISwitch::KEY_PRESS) {
      lastButtonPressTime = millis();
      Serial.println (Buttons.getKey());
    }

    }*/
  // while ( Buttons.read() != 0) { }
  lcd.clear();
  //if (beeping == true) beep(); else digitalWrite(tonePin, HIGH);  // Need to force beep off


  //Serial.println (opState);

  switch (opState)   {
    case OFF:
      Off();
      break;
    case SETP:
      Tune_Sp();
      break;
    case RUN:
      Run();
      break;
    case TUNE_P:
      TuneP();
      break;
    case TUNE_I:
      TuneI();
      break;
    case TUNE_D:
      TuneD();
      break;
    case TUNE_P2:
      TuneP2();
      break;
    case TUNE_I2:
      TuneI2();
      break;
    case TUNE_D2:
      TuneD2();
      break;
  }
}

// ************************************************ // Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons() {
  MD_UISwitch::keyResult_t s = Buttons.read();
  if (beeping == true) beep(); else digitalWrite(tonePin, HIGH);  // Need to force beep off
  if (s == MD_UISwitch::KEY_PRESS) {
    lastButtonPressTime = millis();
    return Buttons.getKey();
  }
  return 0;
}


// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
  myPID.SetMode(MANUAL);

  lcd.clear();
  turnHeaterOff();   //make sure it is off

  lcd.setCursor(5, 0);
  lcd.print(F("GSR"));


  char button = ' ';

  while (true)
  { // Read the input:
    Input = readTemperature();
    lcd.setCursor(0, 1);
    lcd.print(Input, 1);
    lcd.print((char)223);
    lcd.print(F("C"));

    button = ReadButtons();
    if (button == 'U')
    {
      turnHeaterOn();
    }
    else if (button == 'D')
    {
      turnHeaterOff();
    }
    else if (button == 'R')
    {
      // Prepare to transition to the RUN state
      // Make sure manual override is cancelled
      turnHeaterOff();
      // Force the output to max if we are too cold to avoid slow ramp up and then turn the PID on
      if (Input < (Setpoint - 8)) Output = 10000;
      procState = HEATING;
      myPID.SetMode(AUTOMATIC);
      windowStartTime = millis();
      opState = RUN; // start control
      return;
    }
  }
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************
void Tune_Sp()
{
  // lcd.setBacklight(VIOLET);

  lcd.print(F("Set Point:"));
  char button = ' ';

  while (true) {
    button = ReadButtons();

    static float increment = 0.1;
    if (button == 'S')
    {
      if (increment == 0.1) increment = 1; else if (increment == 1) increment = 10; else increment = 0.1;
      lcd.setCursor (10, 0);
      lcd.print (F("x"));
      lcd.print (increment);
    }
    if (button == 'L')
    {
      opState = RUN;
      return;
    }
    if (button == 'R')
    {
      opState = TUNE_P;
      return;
    }
    if (button == 'U')
    {
      Setpoint += increment;
      delay(100);
    }
    if (button == 'D')
    {
      Setpoint -= increment;
      delay(100);
    }

    if ((millis() - lastButtonPressTime) > 5000)  // return to RUN after 5 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Setpoint, 1);
    lcd.print((char)223);
    DoControl();
  }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
  //   lcd.setBacklight(VIOLET);
  lcd.print(F("Set Kp"));

  char button = ' ';
  while (true) {
    button = ReadButtons();

    static float increment = 1.0;
    if (button == 'S')
    {
      if (increment == 1) increment = 10; else if (increment == 10) increment = 100; else increment = 1;
      lcd.setCursor (10, 0);
      lcd.print (F("x"));
      lcd.print (increment);
    }
    if (button == 'L')
    {
      opState = SETP;
      return;
    }
    if (button == 'R')
    {
      opState = TUNE_I;
      return;
    }
    if (button == 'U')
    {
      Kp += increment;
      delay(100);
    }
    if (button == 'D')
    {
      Kp -= increment;
      delay(100);
    }
    if ((millis() - lastButtonPressTime) > 5000)  // return to RUN after 5 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Kp);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
  lcd.print(F("Set Ki"));

  char button = ' ';
  while (true) {
    button = ReadButtons();
    static float increment = 0.01;
    if (button == 'S')
    {
      if (increment == 0.01) increment = 0.1; else if (increment == 0.1) increment = 1; else increment = 0.01;
      lcd.setCursor (10, 0);
      lcd.print (F("x"));
      lcd.print (increment);
    }
    if (button == 'L')
    {
      opState = TUNE_P;
      return;
    }
    if (button == 'R')
    {
      opState = TUNE_D;
      return;
    }
    if (button == 'U')
    {
      Ki += increment;
      delay(100);
    }
    if (button == 'D')
    {
      Ki -= increment;
      delay(100);
    }
    if ((millis() - lastButtonPressTime) > 5000)  // return to RUN after 5 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Ki);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
  lcd.print(F("Set Kd"));

  char button = ' ';
  while (true) {
    button = ReadButtons();

    static float increment = 0.1;
    if (button == 'S')
    {
      if (increment == 0.1) increment = 1; else if (increment == 1) increment = 10; else increment = 0.1;
      lcd.setCursor (10, 0);
      lcd.print (F("x"));
      lcd.print (increment);
    }
    if (button == 'L')
    {
      opState = TUNE_I;
      return;
    }
    if (button == 'R')
    {
      opState = TUNE_P2;
      return;
    }
    if (button == 'U')
    {
      Kd += increment;
      delay(100);
    }
    if (button == 'D')
    {
      Kd -= increment;
      delay(100);
    }
    if ((millis() - lastButtonPressTime) > 5000)  // return to RUN after 5 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Kd);
    lcd.print(F(" "));
    DoControl();

  }

}
//Second set of parameters
//
// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP2()
{
  lcd.print(F("Set Kp2"));

  char button = ' ';
  while (true) {
    button = ReadButtons();

    static float increment = 1.0;
    if (button == 'S')
    {
      if (increment == 1) increment = 10; else if (increment == 10) increment = 100; else increment = 1;
      lcd.setCursor (10, 0);
      lcd.print (F("x"));
      lcd.print (increment);
    }
    if (button == 'L')
    {
      opState = SETP;
      return;
    }
    if (button == 'R')
    {
      opState = TUNE_I2;
      return;
    }
    if (button == 'U')
    {
      Kp2 += increment;
      delay(100);
    }
    if (button == 'D')
    {
      Kp2 -= increment;
      delay(100);
    }
    if ((millis() - lastButtonPressTime) > 5000)  // return to RUN after 5 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Kp2);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI2()
{
  lcd.print(F("Set Ki2"));

  char button = ' ';
  while (true) {
    button = ReadButtons();

    static float increment = 0.01;
    if (button == 'S')
    {
      if (increment == 0.01) increment = 0.1; else if (increment == 0.1) increment = 1; else increment = 0.01;
      lcd.setCursor (10, 0);
      lcd.print (F("x"));
      lcd.print (increment);
    }
    if (button == 'L')
    {
      opState = TUNE_P2;
      return;
    }
    if (button == 'R')
    {
      opState = TUNE_D2;
      return;
    }
    if (button == 'U')
    {
      Ki2 += increment;
      delay(100);
    }
    if (button == 'D')
    {
      Ki2 -= increment;
      delay(100);
    }
    if ((millis() - lastButtonPressTime) > 5000)  // return to RUN after 5 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Ki2);
    lcd.print(" ");
    DoControl();
  }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD2()
{
  lcd.print(F("Set Kd2"));

  char button = ' ';

  while (true) {
    button = ReadButtons();


    static float increment = 0.1;
    if (button == 'S')
    {
      if (increment == 0.1) increment = 1; else if (increment == 1) increment = 10; else increment = 0.1;
      lcd.setCursor (10, 0);
      lcd.print (F("x"));
      lcd.print (increment);
    }
    if (button == 'L')
    {
      opState = TUNE_I2;
      return;
    }
    if (button == 'R')
    {
      opState = RUN;
      return;
    }
    if (button == 'U')
    {
      Kd2 += increment;
      delay(100);
    }
    if (button == 'D')
    {
      Kd2 -= increment;
      delay(100);
    }
    if ((millis() - lastButtonPressTime) > 5000)  // return to RUN after 5 seconds idle
    {
      opState = RUN;
      return;
    }
    lcd.setCursor(0, 1);
    lcd.print(Kd2);
    lcd.print(F(" "));
    DoControl();

  }

}

// ************************************************
// PID COntrol State
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
  static unsigned long runTime = NO_COUNTDOWN;
  static unsigned long rennetTime;
  static unsigned long flocPoint;
  static unsigned long now;
  static unsigned long remainingSeconds;
  static unsigned long secsWaitingForFloc;

  char buffer[6];

  old_relay = false;
  oldProcState = DONE; // Need to reforce display update on entry

  SaveParameters();
  myPID.SetTunings(Kp, Ki, Kd);
  startTime = millis();

  while (true) {
    char button = ' ';
    button = ReadButtons();
    /* if (button != 0) Serial.println (button);*/

    now = millis();
    remainingSeconds = (runTime - (now - startTime)) / 1000;

    if (runTime != NO_COUNTDOWN) {   // runTime of 0 is no countdown
      displayTime (remainingSeconds);
      if (remainingSeconds == 0) {
        beeping = true;
        runTime = NO_COUNTDOWN;
        lcd.setCursor (11, 0);
        lcd.print (F("--:--"));
      }
    }

    displayMode ();
    if ((procState == HEATING) && (Input > TARGET_TEMP)) beeping = true;

    // If we are waiting for floc point to occur, display the elapsed time and beep on the minuute after
    // 8 minutes have elapsed.
    if (procState == WAIT_FOR_FLOC) {
      secsWaitingForFloc = (now - rennetTime) / 1000;
      if ((secsWaitingForFloc >= 480) && (secsWaitingForFloc % 60 >= 0) && (secsWaitingForFloc % 60 < 3)) beeping = true;
      else beeping = false;
      displayTime (secsWaitingForFloc);
    }

    if (button == 'U')
    {
      beeping = false;
      procState = (processState)(((int)procState + 1) % NUMBER_OF_MODES);

      switch (procState) {
        case _HEATING:  //Up to temperature, wait for input to advance.  Change mode to Proportional on Error
          /*              myPID.SetMode(MANUAL);
                        myPID.SetTunings(Kp2, Ki2, Kd2, P_ON_E);
                        Serial.println ("----------------------------------------------");
                        Serial.print ("Kp2 ");
                        Serial.println (Kp2);
                        Serial.print ("Ki2 ");
                        Serial.println (Ki2);
                        Serial.print ("Kd2 ");
                        Serial.println (Kd2);
                        Serial.println ("SetPoint timeChange Input Error Ki*Error outputSum Kp*dInput outputSum outputSum Kd*DInput Output");

                        myPID.SetMode(AUTOMATIC);  */

          runTime = NO_COUNTDOWN;
          break;

        case HOLDING:
          runTime = HOLD_TIME;
          break;

        case _HOLDING:  // Adding Rennet at this point
          runTime = NO_COUNTDOWN;
          break;

        case WAIT_FOR_FLOC:   // Rennet added at start of this phase


          runTime = NO_COUNTDOWN;  // Waiting for operator input when floc has occured
          rennetTime = millis();
          Serial.print ("RennetTime: ");
          Serial.println (rennetTime);
          break;

        case WAIT_FOR_CUT: // Floc point has been reached.
          myPID.SetMode(MANUAL);  // Stop trying to control the temperature - it does not work with probe in curds
          Output = 0;
          turnHeaterOff();;   //make sure it is off
          flocPoint = millis();
          Serial.print ("FlocPoint: ");
          Serial.println (flocPoint);
          runTime = ((flocPoint - rennetTime) * 4);
          Serial.print ("RunTime ");
          Serial.println (runTime);
          break;

        case CUT: // Floc point has been reached.
          runTime = NO_COUNTDOWN;
          break;

        case WAIT:
          runTime = WAIT_TIME;
          break;

        case STIR1:
          runTime = STIR_TIME;
          break;

        case STIR2:
          runTime = STIR_TIME;
          break;

        case STIR3:
          runTime = STIR_TIME;
          break;

        case REST:
          runTime = REST_TIME;
          break;

        case MOULDS:
          runTime = NO_COUNTDOWN;
          break;

        case DONE:
          return;
      }
      startTime = millis();
    }
    else if (button == 'R')
    {
      opState = SETP;
      return;
    }
    else if (button == 'L')
    {
      opState = OFF;
      return;
    }
    else if (button == 'D')
    {
      switch (heaterState) {

        case HEAT_AUTO:
          heaterState = HEAT_ON;
          heaterOnSymbol = 'O';
          myPID.SetMode(MANUAL);
          turnHeaterOn();
          break;

        case HEAT_ON:
          heaterState = HEAT_OFF;
          heaterOffSymbol = '-';
          myPID.SetMode(MANUAL);
          turnHeaterOff();
          break;

        case HEAT_OFF:
          heaterState = HEAT_AUTO;
          relay = true;
          old_relay = false;
          myPID.SetMode(AUTOMATIC);
          heaterOnSymbol = '*';
          heaterOffSymbol = ' ';
          break;
      }
    }

    DoControl();

    // display the current temperature
    lcd.setCursor(0, 1);
    lcd.print(Input, 1);
    lcd.print((char)223);
    lcd.print(F("C"));

    // display output percentage
    float pct = map(Output, 0, WindowSize, 0, 1000);
    dtostrf(pct / 10, 3, 0, buffer);
    lcd.setCursor(9, 1);
    lcd.print (buffer);
    lcd.print("%");

    // periodically log to serial port in csv format
    if (millis() - lastLogTime > logInterval)
    {
      lastLogTime = millis();
      //   Serial.print(Input);
      //    Serial.print(",");
      //    Serial.println(Output);
    }

    if (old_relay != relay)
    {
      if (relay == false) {
        turnHeaterOff();;
      } else {
        turnHeaterOn();
      };
      old_relay = relay;
    }
    //  delay(100);
  }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{

  // Read the input:
  Input = readTemperature();
  if ((myPID.GetProportionalType() == P_ON_M) && (Input >= Setpoint)) {
    myPID.SetMode(MANUAL);
    myPID.SetTunings(Kp2, Ki2, Kd2, P_ON_E);
    Serial.println ("----------------------------------------------");
    Serial.print ("Kp2 ");
    Serial.println (Kp2);
    Serial.print ("Ki2 ");
    Serial.println (Ki2);
    Serial.print ("Kd2 ");
    Serial.println (Kd2);
    Serial.println ("SetPoint timeChange Input Error Ki*Error outputSum Kp*dInput outputSum outputSum Kd*DInput Output");
    if (myPID.GetProportionalType() == P_ON_E) Serial.println ("Proportional on Error"); else Serial.println ("Proportional on Measurement");
    myPID.SetMode(AUTOMATIC);
  }
  myPID.Compute();

  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output;
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime)))  //LA - Was 100 - 0.1sec
  {
    relay = true;
  }
  else
  {
    relay = false;
  }
}


// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  if (Setpoint != EEPROM_readDouble(SpAddress))
  {
    EEPROM_writeDouble(SpAddress, Setpoint);
  }
  if (Kp != EEPROM_readDouble(KpAddress))
  {
    EEPROM_writeDouble(KpAddress, Kp);
  }
  if (Ki != EEPROM_readDouble(KiAddress))
  {
    EEPROM_writeDouble(KiAddress, Ki);
  }
  if (Kd != EEPROM_readDouble(KdAddress))
  {
    EEPROM_writeDouble(KdAddress, Kd);
  }

  if (Kp2 != EEPROM_readDouble(Kp2Address))
  {
    EEPROM_writeDouble(Kp2Address, Kp2);
  }
  if (Ki2 != EEPROM_readDouble(Ki2Address))
  {
    EEPROM_writeDouble(Ki2Address, Ki2);
  }
  if (Kd2 != EEPROM_readDouble(Kd2Address))
  {
    EEPROM_writeDouble(Kd2Address, Kd2);
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
  Setpoint = EEPROM_readDouble(SpAddress);
  Kp = EEPROM_readDouble(KpAddress);
  Ki = EEPROM_readDouble(KiAddress);
  Kd = EEPROM_readDouble(KdAddress);
  Kp2 = EEPROM_readDouble(Kp2Address);
  Ki2 = EEPROM_readDouble(Ki2Address);
  Kd2 = EEPROM_readDouble(Kd2Address);

  // Use defaults if EEPROM values are invalid
  if (isnan(Setpoint))
  {
    Setpoint = 32;
  }
  if (isnan(Kp))
  {
    Kp = 4500;
  }
  if (isnan(Ki))
  {
    Ki = 10;
  }
  if (isnan(Kd))
  {
    Kd = 30;
  }
  if (isnan(Kp2))
  {
    Kp2 = 850;
  }
  if (isnan(Ki2))
  {
    Ki2 = 0.5;
  }
  if (isnan(Kd2))
  {
    Kd2 = 0.1;
  }

  /*  Initialise
    Setpoint = 32;
    Kp = 4500;
    Ki = 10;
    Kd = 30;
    Kp2 = 850;
    Ki2 = 0.5;
    Kd2 = 0.1;
  */

}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}

void beep ()
{
  static unsigned long lastTime;
  static boolean tonePinState = HIGH;

  unsigned long timeChange = (millis() - lastTime);

  if (timeChange > 500)
  {
    if (tonePinState == true) tonePinState = false; else tonePinState = true;
    digitalWrite (tonePin, tonePinState);
    lastTime = millis();
  }
}

void displayModeString (char *theText) {
  lcd.setCursor(0, 0);
  lcd.print (theText);
  lcd.setCursor(11, 0);
  lcd.print ("     ");
}

void displayMode ()
{
  if (oldProcState != procState) {
    switch (procState) {
      case HEATING:
        displayModeString("Heating ");
        break;

      case _HEATING:
        displayModeString ("Add Culture");
        break;

      case HOLDING:
        displayModeString ("Holding    ");
        break;

      case _HOLDING:
        displayModeString ("Add Rennet");
        break;

      case WAIT_FOR_FLOC:
        displayModeString("Waiting FP");
        break;

      case WAIT_FOR_CUT:
        displayModeString("Wait...    ");
        break;

      case CUT:
        displayModeString("Cut Now");
        break;

      case WAIT:
        displayModeString("Wait...    ");
        break;

      case STIR1:
        displayModeString ("STIR1  ");
        break;

      case STIR2:
        displayModeString ("Stir.");
        break;

      case STIR3:
        displayModeString ("STIR..");
        break;

      case REST:
        displayModeString("STIR...");
        break;

      case MOULDS:
        displayModeString ("Moulds  ");
        break;

      default:
        break;
    }
    oldProcState = procState;
  }
}

void displayTime (unsigned long numSeconds) {
  static unsigned long oldNumSeconds;
  char buffer[7];

  if (numSeconds != oldNumSeconds) {
    unsigned int mins = numSeconds / 60;
    unsigned int secs = numSeconds % 60;
    lcd.setCursor (11, 0);
    if (mins < 100) sprintf (buffer, "%2d:%02d", mins, secs); else sprintf (buffer, "%3dm ", mins);
    lcd.print (buffer);
    oldNumSeconds = numSeconds;
  }

}
