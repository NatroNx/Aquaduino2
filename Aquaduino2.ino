//Aquaduino
// v.1.1  -  21.08.15
// v.1.2  -  05.09.15
// v.1.3  -  12.09.15 - PWM Dimmung angepasst an MOSFET Steuerung hinter dem Netzteil
// v.1.3.1 - 25.09.15 - fixed a Bug in LightCalculation (false values when now.minute == nextlight.minute)
// v.1.3.2 - 04.10.2015 - feed Mode - ScreenUpdate AFTER switching Lights
// v.1.4  -  04.10.2015 - initial TVMode implemented
// v.1.4.1 - 13.10.2015 - started to implement LightModes for RGB
// v.1.4.2 - 17.10.2015 - RGB visualisation ready
// v.1.4.3 - 18.10.2015 - RGB finalized
// v.1.5   - 04.11.2015 - added PH Curve
// v.1.5.1 - 05.11.2015 - bugfixing
// v.1.5.2 - 07.11.2015 - bugfixing | implementing TempCurve
// v.1.5.3 - 21.11.2015 - implement MoonMode
// v.1.7.0 - 23.02.2016 - use less memory
// v.2.0.0(b) - 26.02.2016 - less f() macro - made troubles I never found.  Bugfixing. First full working build with ESP connect.
// v.2.0.1 - 04.03.2016 - bugfixing
// v.2.0.2 - 10.03.2016  - sendSerial with and without Feedback  - choose your weapons
// v.2.0.3 - 19.03.2016 - fixed RGBLight while feeding; better DrawCurve - PH & Temperature Curve
// v.2.0.4 - 20.03.2016 - implemented Refillbeep (beeper, beepnow, getDistance()
// v.2.0.5 - 25.03.2016 - compleded filling level routine, fixed lag in FeedMode, implemented Calibration Mode, reworked PH-Measurements
// v.2.0.7 - 30.04.2016 - changed the point where CO2 turns on/off, minor tweak in Filling, Forget HighLow Values in drawCurve
// v.2.0.8 - 01.07.2016 - added support of 12V Fans to cool the tank
// v.2.0.9 - 01.07.2016 - bugfixing - fan turned on and off when temp == tempUpperlimit
// v.2.1.0 - 06.11.2016 - bring the PH/Temp/Co2 to the Web (in progress)
// v.2.1.1 - 16.11.2016 - PH/TEMP/CO2/Fertilizer and WaterDistance to Webinterface
// v.2.1.2 - 08.12.2016 - changed rate PH can turn on/off to 15 minutes (from 60 seconds)


//needed Libraries

#include <Arduino.h>
#include <ctype.h>
#include <DallasTemperature.h> //needed for Temp
#include <HardwareSerial.h>
#include <OneWire.h>  //needed for Temp
#include <pins_arduino.h>
#include <Print.h>
#include <RCSwitch.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sx1509_library.h> // Include the SX1509 library
#include <UTFT.h>    //TFT
#include <UTouch.h> //Touch
#include <Wire.h>  // Wire.h library is required to use SX1509 lib
#include <WString.h>

#include "RTClib.h"
#include "SdFat.h"

#define ONE_WIRE_BUS 10 //needed for Temperature
#include <EEPROM.h>  // used to store and retrieve settings from memory
#include <tinyFAT.h> // used to acess the SD card
#include <UTFT_SdRaw.h>  // used to read .raw images from the SD card
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <NewPing.h>  //ultasonic range

#define PING_PIN 18 // Arduino pin for both trig and echo
#define debug 0

// Declare which fonts we will be using
extern uint8_t BigFont[];
extern uint8_t UbuntuBold[];
extern uint8_t SevenSegmentFull[];
extern uint8_t OCR_A_Extended_M[];

//bring it up - initialise
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
//UTFT myGLCD(ITDB50, 38, 39, 40, 41); //initialize TFT
UTFT myGLCD(ITDB50, 38, 39, 40, 41); //initialize TFT
UTouch myTouch(6, 5, 4, 3, 2); //initialize TFT
UTFT_SdRaw myFiles(&myGLCD);
RCSwitch mySwitch = RCSwitch();
NewPing sonar(PING_PIN, PING_PIN, 80); //80 is max measure in cm

/*  VARIABLEN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 alle globale Variablen - teilweise mit Startweten definiert
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define progMemBuffer 128

enum {
	tPhWert,
	tTemp,
	tcalculatedPWM,
	tcalculatedRed,
	tcalculatedGreen,
	tcalculatedBlue,
	tTVModeState,
	tcleaningInProcess,
	tmanualOverride,
	tMoonModeState,
	tpump1Value,
	tpump2Value,
	tlight230Value,
	tlight1Value,
	tlight2Value,
	tco2Value,
	theaterValue,
	tdPump1Value,
	tdPump2Value,
	tdPump3Value,
	tcoolValue,
	tnow,
	tpS,
	tpF,
	tpR,
	tpB,
	tpP,
	tcalculatedPWMnF,
	tcalculatedRednF,
	tcalculatedGreennF,
	tcalculatedBluenF,
	tPHValues,
	tTempValues,
	tCo2Values,
	tnpkFert,
	tnFert,
	tfeFert,
	tdst
};

const char PhWert_Char[] PROGMEM = "pH";
const char Temp_Char[] PROGMEM = "tE";
const char calculatedPWM_Char[] PROGMEM = "cP";
const char calculatedRed_Char[] PROGMEM = "cR";
const char calculatedGreen_Char[] PROGMEM = "cG";
const char calculatedBlue_Char[] PROGMEM = "cB";
const char TVModeState_Char[] PROGMEM = "tV";
const char cleaningInProcess_Char[] PROGMEM = "cI";
const char manualOverride_Char[] PROGMEM = "mO";
const char MoonModeState_Char[] PROGMEM = "mM";
const char pump1Value_Char[] PROGMEM = "p1";
const char pump2Value_Char[] PROGMEM = "p2";
const char light230Value_Char[] PROGMEM = "lV";
const char light1Value_Char[] PROGMEM = "l1";
const char light2Value_Char[] PROGMEM = "l2";
const char co2Value_Char[] PROGMEM = "cO";
const char heaterValue_Char[] PROGMEM = "hV";
const char dPump1Value_Char[] PROGMEM = "d1";
const char dPump2Value_Char[] PROGMEM = "d2";
const char dPump3Value_Char[] PROGMEM = "d3";
const char coolValue_Char[] PROGMEM = "cV";
const char now_Char[] PROGMEM = "nO";
const char processSlide_Char[] PROGMEM = "pS";
const char processRF_Char[] PROGMEM = "pF";
const char processRel_Char[] PROGMEM = "pR";
const char processBool_Char[] PROGMEM = "pB";
const char processPump_Char[] PROGMEM = "pP";
const char calculatedPWMnF_Char[] PROGMEM = "nP";   //PWM withouth feedback
const char calculatedRednF_Char[] PROGMEM = "nR";   //red withouth feedback
const char calculatedGreennF_Char[] PROGMEM = "nG";   //green withouth feedback
const char calculatedBluenF_Char[] PROGMEM = "nB";   //blue withouth feedback
const char PHValues_Char[] PROGMEM = "phS";
const char TempValues_Char[] PROGMEM = "tS";
const char Co2Values_Char[] PROGMEM = "cS";
const char npkFert_Char[] PROGMEM = "npkF";
const char nFert_Char[] PROGMEM = "nF";
const char feFert_Char[] PROGMEM = "feF";
const char dst_Char[] PROGMEM = "dst";

PGM_P const Char_table[] PROGMEM =
{	PhWert_Char, Temp_Char, calculatedPWM_Char, calculatedRed_Char, calculatedGreen_Char, calculatedBlue_Char, TVModeState_Char,
	cleaningInProcess_Char, manualOverride_Char, MoonModeState_Char, pump1Value_Char, pump2Value_Char,
	light230Value_Char, light1Value_Char, light2Value_Char, co2Value_Char, heaterValue_Char,
	dPump1Value_Char, dPump2Value_Char, dPump3Value_Char, coolValue_Char, now_Char, processSlide_Char, processRF_Char, processRel_Char,
	processBool_Char, processPump_Char, calculatedPWMnF_Char, calculatedRednF_Char, calculatedGreennF_Char, calculatedBluenF_Char, PHValues_Char,
	TempValues_Char, Co2Values_Char, npkFert_Char, nFert_Char, feFert_Char, dst_Char};

int charCount = sizeof(Char_table) / sizeof(Char_table[0]);

const byte numChars = 255;
char receivedChars[numChars];
static byte ndx = 0;
boolean newData = false;

unsigned long currentMillis; // get current millis
unsigned long prevMillisTouch = 0;
unsigned long prevMillis1sec = 0; //track 1 second
unsigned long prevMillis5sec = 0; // track 5 seconds for refreshing clock and temp
unsigned long prevMillis1min = 0; // track 60 seconds for refreshing
unsigned long prevMillis15min = 0; //track 15 minutes
// unsigned long stopMillis = 0; //stopwatch
//unsigned long standbyMillis=0;

RTC_DS1307 rtc;
const byte SX1509_ADDRESS = 0x3E;
sx1509Class sx1509(SX1509_ADDRESS);
const byte interruptPin = 2;
const byte resetPin = 8;
DateTime now;
DateTime cleanEnd;
DateTime adjustTimer;
DateTime lastFert;

DateTime tankClean;
byte tankCleandDays;
DateTime co2Bottle;
byte co2BottleDays;
DateTime cleanFilter1;
byte cleanFilter1Days;
DateTime cleanFilter2;
byte cleanFilter2Days;

DateTime TVModeStart;
boolean TVModeState = false;
float TVModeBrightness; // (20%)

DateTime MoonEnd;
boolean MoonModeState = false;
byte MoonRed;
byte MoonGreen;
byte MoonBlue;
byte MoonMinutes;

int x, y; //touched coordinates
int x2, y2;
String inputstring = "";            //a string to hold incoming data from the PC
String PhWertString = ""; //a string to hold the data from the Atlas Scientific product
boolean calibrate = false;

float PhWert = 7.01; //sting to float to calculate with it
float PHUpperLimit = 10;
float PHLowerLimit = 5;
float PHValues[96];
float TempValues[96];
float highestTemp = 26.05;
float lowestTemp = 25.95;
float highestPH = 7.01;
float lowestPH = 6.99;
boolean Co2Values[96];  //tracks if the CO2 enabled or disabled at that time

byte put_PHindex = 0;
byte put_TempIndex = 0;
byte get_index = 1;
/**<datatype> array [DIM_0_SIZE] [DIM_1_SIZE] = {
 //as many vals as dim1
 {val,val,val},
 {val,val,val}//as many rows as dim0
 };
 **/

boolean input_stringcomplete = false; //have we received all the data from the PC
boolean sensor_stringcomplete = false; //have we received all the data from the Atlas Scientific product
float Temp = 25.01;        //sting to float to calculate with it
float TempUpperLimit = 30;
float TempLowerLimit = 25;

//Fertilizer Stuff
float FDose[] = { 0, 0, 0 };
float FMax[] = { 0, 0, 0 };
float FLeft[] = { 0, 0, 0 };
float FRate[] = { 0, 0, 0 };
byte FSelect = 5;
unsigned long fertmillis = 0;

boolean MoF[] = { false, false, false };
boolean TuF[] = { false, false, false };
boolean WeF[] = { false, false, false };
boolean ThF[] = { false, false, false };
boolean FrF[] = { false, false, false };
boolean SaF[] = { false, false, false };
boolean SuF[] = { false, false, false };
boolean dayN[7] = { false, false, false, false, false, false, false };
boolean dayNPK[7] = { false, false, false, false, false, false, false };
boolean dayFE[7] = { false, false, false, false, false, false, false };
byte doseHour;
byte doseMinute;

//Powerschedule

byte powLightOnHour;
byte powLightOnMinute;
byte powLightOffHour;
byte powLightOffMinute;
byte powCo2OnHour;
byte powCo2OnMinute;
byte powCo2OffHour;
byte powCo2OffMinute;

byte screenOnHour;
byte screenOnMinute;
byte screenOffHour;
byte screenOffMinute;
byte standByMinutes = 10;
byte backlightPWM = 255;         // value for backlight  - 255 is full

int highestWaterDistance = 0;
float waterDistance = 0;
int lastWaterDistance = 0;
boolean beepActive = false;

int speakerPin = A8;
boolean beep = false;
byte counter = 0;
int freq = 0;
unsigned long milliSecondsToBeep = 0;
byte repeat = 0;
unsigned long beepWait = 0;

/*
 byte tankCleanMonth;
 byte tankCleanDay;
 byte co2BottleMonth;
 byte co2BottleDay;
 byte cleanFilter1Month;
 byte cleanFilter1Day;
 byte cleanFilter2Month;
 byte cleanFilter2Day;
 */

//days and month char for displaing at the top of screen
char *dayName[] = { "Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag",
		"Freitag", "Samstag" };
char *monthName[] = { "", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul",
		"Aug", "Sep", "Oct", "Nov", "Dec" };

const byte light2Pin = 2;
const byte light1Pin = 11; //not active
const byte dPump2Pin = 6;
const byte dPump1Pin = 7;
const byte dPump3Pin = 5;
const byte fanPin = 3;
const byte redPin = 12;
const byte greenPin = 13;
const byte bluePin = 11;
/**
 byte heaterPin = 12; //will be RF
 byte coolPin = 11; //will be RF
 byte co2Pin = 10; //will be RF
 byte pump1Pin = 13; //not used anymore RF now
 byte pump2Pin = 14; //not used anymore RF now
 byte light230Pin = 15; //not used anymore RF now
 **/

boolean pump1Value = false;
boolean pump2Value = false;
boolean light230Value = true;
boolean light1Value = true;
boolean light2Value = true;
boolean co2Value = true;
boolean heaterValue = false;
boolean dPump1Value = true;
boolean dPump2Value = true;
boolean dPump3Value = true;
boolean coolValue = true;
//boolean fanValue = true;

boolean pump1Clean = true;
boolean pump2Clean = true;
boolean light230Clean = true;
boolean light2Clean = true;
boolean co2Clean = true;
boolean heaterClean = true;
boolean coolClean = true;
byte cleanMinutes = 120;

boolean changeRF = false;

// Used for PWM and lightstuff

TimeSpan timeSinceLastLight;
TimeSpan timeToNextLight;
TimeSpan timeSinceLastLightRGB;
TimeSpan timeToNextLightRGB;
byte currentPWM = 0; //0 - which is NO light
byte nextPWM = 0;

const byte lightPwmPin = 9;            //the pin used for pwm
float calculatedPWM = 0;        //the value
float calculatedRed = 0;
float calculatedGreen = 0;
float calculatedBlue = 0;
const byte backlightPIN = 44;    // Pin 44 used for backlight

byte dispScreen = 0;       //current screen at start
byte lastScreen = 255;        //last screen

// screens are listed below
// 0-home, 1-cleaning, , 2-power, 3-extras, 4-lights,
// 5-clock, 6-feeding sched, 7-schedule, 8-heater
// 9-dosing, 10-pwer schedule, 11-schedule item, 13=ScreenScreen
//

//funkzeug
const unsigned long f11on = 1381717;  //Pump1 ON
const unsigned long f11off = 1381716;  //Pump1 OFF
const unsigned long f12on = 1394005;
const unsigned long f12off = 1394004;
const unsigned long f13on = 1397077; //MainLight ON
const unsigned long f13off = 1397076; //MainLight OFF
const unsigned long f14on = 1397845;
const unsigned long f14off = 1397844;
const unsigned long f21on = 4527445;   //Heater ON
const unsigned long f21off = 4527444;  //Heater FOFF
const unsigned long f22on = 4539733;   //CO2 ON
const unsigned long f22off = 4539732;  //CO2 OFF
const unsigned long f23on = 4542805;   //Coolpump ON
const unsigned long f23off = 4542804; //Coolpump OFF
const unsigned long f24on = 4543573; //Pump2 ON

const unsigned long f24off = 4543572;
const unsigned long f31on = 5313877;
const unsigned long f31off = 5313876;
const unsigned long f32on = 5326165;
const unsigned long f32off = 5326164;
const unsigned long f33on = 5329237;
const unsigned long f33off = 5329236;
const unsigned long f34on = 5330005;
const unsigned long f34off = 5330004;
const unsigned long f41on = 5510485;

const unsigned long f42on = 5522773;  //Pump2 ON
const unsigned long f42off = 5522772; //Pump2 Off

const unsigned long f43on = 5525845;
const unsigned long f43off = 5525844;
const unsigned long f44on = 5526613;
const unsigned long f44off = 5526612;

boolean cleaningInProcess = false;
boolean manualOverride = false;

//byte pwmDateAndValue[]={21,0,0,7,0,0,7,20,100,11,50,100,12,00,0,15,50,0,16,00,100,20,40,100};

typedef struct {
	byte Hour;
	byte Minute;
	float pwmValue;
} record_type;

record_type lightPWM[12];
byte lightScreenSet = 99;
byte RGBScreenSet = 99;

typedef struct {
	byte Hour;
	byte Minute;
	byte red;
	byte green;
	byte blue;
} recordRGB_type;

recordRGB_type lightRGB[12];

struct RGB {
	byte r;
	byte g;
	byte b;
};
typedef struct RGB Color;

//Farben
const Color col_white = { 255, 255, 255 };
const Color col_black = { 255, 255, 255 };
const Color col_blue = { 0, 0, 255 };
const Color col_red = { 255, 0, 0 };
const Color col_FertiN = { 45, 90, 255 };
const Color col_FertiNPK = { 25, 191, 13 };
const Color col_FertiFE = { 204, 17, 17 };

//Buttonkoordinaten Home
const short Button1Cord[] = { 0, 0, 0, 0 };
const short HomeButtonCoord[] = { 21, 613, 117, 710 };
const short FeedButtonCoord[] = { 133, 613, 230, 710 };
const short PowerButtonCoord[] = { 249, 613, 346, 710 };
const short SettingsButtonCoord[] = { 362, 613, 458, 710 };
const short LightUp[] = { 60, 320, 108, 368 };
const short LightDown[] = { 60, 450, 108, 498 };
const short BottomButtonCoord[] = { 215, 746, 263, 795 }; //used @ all screens

//Powerbuttoncords
const short Filter1Cord[] = { 20, 150, 94, 224 };
const short Filter2Cord[] = { 250, 150, 324, 224 };
const short Ligth1Cord[] = { 20, 234, 94, 308 };
const short Light2Cord[] = { 250, 234, 324, 308 };
const short Co2Cord[] = { 20, 318, 94, 392 };
const short HeaterCord[] = { 250, 318, 324, 392 };
const short CoolingCord[] = { 20, 402, 94, 476 };
const short AllOFFCord[] = { 20, 524, 94, 598 };
const short ResetCord[] = { 20, 608, 94, 682 };
const short CleanModeCord[] = { 250, 524, 324, 598 };
const short SpeakerCord[] = { 250, 608, 324, 682 };

//Settingbuttoncords
const short PowerSchedCord[] = { 68, 150, 142, 224 };
const short LightsCord[] = { 158, 150, 232, 224 };
const short CleanCord[] = { 248, 150, 322, 224 };
const short ScheCord[] = { 338, 150, 412, 224 };
const short ClockCord[] = { 68, 240, 142, 314 };
const short Co2SetCord[] = { 158, 240, 232, 314 };
const short HeatCord[] = { 248, 240, 322, 314 };
const short DoseCord[] = { 338, 240, 412, 314 };
const short ScreenCord[] = { 68, 330, 142, 404 };
const short RGBCord[] = { 158, 330, 232, 404 };
const short TVModeCord[] = { 248, 330, 322, 404 };
const short MoonModeCord[] = { 338, 330, 412, 404 };

/** more buttons if needed

 */

//Clockbuttoncords
const short HourUp[] = { 175, 220, 223, 268 };
const short HourDown[] = { 175, 273, 223, 321 };
const short MinuteUp[] = { 347, 220, 395, 268 };
const short MinuteDown[] = { 347, 273, 395, 321 };
const short DayUp[] = { 110, 391, 158, 439 };
const short DayDown[] = { 110, 444, 158, 492 };
const short MonthUp[] = { 220, 391, 268, 439 };
const short MonthDown[] = { 220, 444, 268, 492 };
const short YearUp[] = { 380, 391, 428, 439 };
const short YearDown[] = { 380, 444, 428, 492 };
const short SetClockCord[] = { 156, 600, 324, 652 };
//const short ResetCord[] = {20, 608 ,94, 682};

//POWERSCHEDULE BUTTONS
const short SetPowerSchedCord[] = { 300, 680, 465, 732 }; //also used for Light scenes
const short CancelPowerSchedCord[] = { 22, 680, 190, 732 }; //also used for light scenes
const short powLightOnHourUp[] = { 275, 110, 323, 158 };
const short powLightOnHourDown[] = { 275, 163, 323, 211 };
const short powLightOnMinuteUp[] = { 412, 110, 460, 158 };
const short powLightOnMinuteDown[] = { 412, 163, 460, 211 };

const short powLightOffHourUp[] = { 275, 240, 323, 288 };
const short powLightOffHourDown[] = { 275, 293, 323, 341 };
const short powLightOffMinuteUp[] = { 412, 240, 460, 288 };
const short powLightOffMinuteDown[] = { 412, 293, 460, 341 };

const short powCo2OnHourUp[] = { 275, 410, 323, 458 };
const short powCo2OnHourDown[] = { 275, 463, 323, 511 };
const short powCo2OnMinuteUp[] = { 412, 410, 460, 458 };
const short powCo2OnMinuteDown[] = { 412, 463, 460, 511 };
const short powCo2OffHourUp[] = { 275, 540, 323, 588 };
const short powCo2OffHourDown[] = { 275, 593, 323, 641 };
const short powCo2OffMinuteUp[] = { 412, 540, 460, 588 };
const short powCo2OffMinuteDown[] = { 412, 593, 460, 641 };

//Light Buttons (Disp 4)
const short LightMode1Cord[] = { 15, 135, 465, 195 }; //also used for dosing
const short LightMode2Cord[] = { 15, 225, 465, 285 }; //also used for dosing
const short LightMode3Cord[] = { 15, 315, 465, 375 }; //also used for dosing
const short LightMode4Cord[] = { 15, 405, 465, 465 };
const short LightMode5Cord[] = { 15, 495, 465, 555 };
const short LightMode6Cord[] = { 15, 585, 465, 645 };

//Dosing Button (Disp 9)
const short refillAllCord[] = { 110, 580, 370, 632 };
const short calibrateCord[] = { 22, 370, 282, 422 };

//DaySched for Dosing (91,92,93)
const short MoCord[] = { 18, 590, 66, 638 };
const short TuCord[] = { 84, 590, 132, 638 };
const short WeCord[] = { 150, 590, 198, 638 };
const short ThCord[] = { 216, 590, 264, 638 };
const short FrCord[] = { 282, 590, 330, 638 };
const short SaCord[] = { 348, 590, 396, 638 };
const short SoCord[] = { 414, 590, 462, 638 };

//RGBScreen
const short red1Up[] = { 138, 240, 186, 288 };
const short red1Down[] = { 138, 293, 186, 341 };

const short red2Up[] = { 138, 540, 186, 588 };
const short red2Down[] = { 138, 593, 186, 641 };

SdFat SD;
File Aquaduino;

/*EEPROM ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 locations for saved settings
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 0 - FDose[0] in ml          0 is N
 1 - FDose[1] in ml          1 is NPK
 2 - FDose[2] in ml          2 is FE
 3 - FMax[0] in ml
 4 - FMax[1] in ml
 5 - FMax[2] in ml
 6 - FLeft[0] in ml
 7 - NKPLeft in ml
 8 - FLeft[2] in ml
 powLightOnHour=EEPROM.read(9);
 powLightOnMinute=EEPROM.read(10);
 powLightOffHour=EEPROM.read(11);
 powLightOffMinute=EEPROM.read(12);
 powCo2OnHour=EEPROM.read(13);
 powCo2OnMinute=EEPROM.read(14);
 powCo2OffHour=EEPROM.read(15);
 powCo2OffMinute=EEPROM.read(16);
 lightPWM[0].Hour=EEPROM.read(20);
 lightPWM[0].Minute=EEPROM.read(21);
 lightPWM[0].pwmValue=EEPROM.read(22);
 lightPWM[1].Hour=EEPROM.read(23);
 lightPWM[1].Minute=EEPROM.read(24);
 lightPWM[1].pwmValue=EEPROM.read(25);
 lightPWM[2].Hour=EEPROM.read(26);
 lightPWM[2].Minute=EEPROM.read(27);
 lightPWM[2].pwmValue=EEPROM.read(28);
 lightPWM[3].Hour=EEPROM.read(29);
 lightPWM[3].Minute=EEPROM.read(30);
 lightPWM[3].pwmValue=EEPROM.read(31);
 lightPWM[4].Hour=EEPROM.read(32);
 lightPWM[4].Minute=EEPROM.read(33);
 lightPWM[4].pwmValue=EEPROM.read(34);
 lightPWM[5].Hour=EEPROM.read(35);
 lightPWM[5].Minute=EEPROM.read(36);
 lightPWM[5].pwmValue=EEPROM.read(37);
 lightPWM[6].Hour=EEPROM.read(38);
 lightPWM[6].Minute=EEPROM.read(39);
 lightPWM[6].pwmValue=EEPROM.read(40);
 lightPWM[7].Hour=EEPROM.read(41);
 lightPWM[7].Minute=EEPROM.read(42);
 lightPWM[7].pwmValue=EEPROM.read(43);
 lightPWM[8].Hour=EEPROM.read(44);
 lightPWM[8].Minute=EEPROM.read(45);
 lightPWM[8].pwmValue=EEPROM.read(46);
 lightPWM[9].Hour=EEPROM.read(47);
 lightPWM[9].Minute=EEPROM.read(48);
 lightPWM[9].pwmValue=EEPROM.read(49);
 lightPWM[10].Hour=EEPROM.read(50);
 lightPWM[10].Minute=EEPROM.read(51);
 lightPWM[10].pwmValue=EEPROM.read(52);
 lightPWM[11].Hour=EEPROM.read(53);
 lightPWM[11].Minute=EEPROM.read(54);
 lightPWM[11].pwmValue=EEPROM.read(55);
 pump1Clean=EEPROM.read(56);
 pump2Clean=EEPROM.read(57);
 light230Clean=EEPROM.read(58);
 light2Clean=EEPROM.read(59);
 co2Clean=EEPROM.read(60);
 heaterClean=EEPROM.read(61);
 coolClean=EEPROM.read(62);
 PHUpperLimit=EEPROM.read(63);
 PHLowerLimitEEPROM.read(64);
 TempUpperLimit=(EEPROM.read(65));
 TempLowerLimit=(EEPROM.read(66));
 FRate[0]=EEPROM.read(67);
 FRate[1]=EEPROM.read(68);
 FRate[2]=EEPROM.read(69);
 MoF[0]=EEPROM.read(70);
 MoF[1]=EEPROM.read(71);
 MoF[2]=EEPROM.read(72);
 TuF[0]=EEPROM.read(73);
 TuF[1]=EEPROM.read(74);
 TuF[2]=EEPROM.read(75);
 WeF[0]=EEPROM.read(76);
 WeF[1]=EEPROM.read(77);
 WeF[2]=EEPROM.read(78);
 ThF[0]=EEPROM.read(79);
 ThF[1]=EEPROM.read(80);
 ThF[2]=EEPROM.read(81);
 FrF[0]=EEPROM.read(82);
 FrF[1]=EEPROM.read(83);
 FrF[2]=EEPROM.read(84);
 SaF[0]=EEPROM.read(85);
 SaF[1]=EEPROM.read(86);
 SaF[2]=EEPROM.read(87);
 SuF[0]=EEPROM.read(88);
 SuF[1]=EEPROM.read(89);
 SuF[2]=EEPROM.read(90);
 cleanMinutes=EEPROM.read(91);
 doseHour=EEPROM.read(92);
 doseMinute=EEPROM.read(93);
 //tankCleanDay=EEPROM.read(94);
 /*tankCleanMonth=EEPROM.read(95);
 co2BottleDay=EEPROM.read(96);
 co2BottleMonth=EEPROM.read(97);
 cleanFilter1Day=EEPROM.read(98);
 cleanFilter1Month=EEPROM.read(99);
 cleanFilter2Day=EEPROM.read(100);
 cleanFilter2Month=EEPROM.read(101);
 tankCleandDays=EEPROM.read(102);
 co2BottleDays=EEPROM.read(103);
 cleanFilter1Days=EEPROM.read(104);
 cleanFilter2Days=EEPROM.read(105);
 screenOnHour=EEPROM.read(106);
 screenOffHour=EEPROM.read(107);
 screenOnMinute=EEPROM.read(108);
 screenOffMinute=EEPROM.read(109);
 standByMinutes=EEPROM.read(110);
 backlightPWM=EEPROM.read(111);
 lightRGB[0].Hour=EEPROM.read(112);
 lightRGB[0].Minute=EEPROM.read(113);
 lightRGB[0].red=EEPROM.read(114);
 lightRGB[0].green=EEPROM.read(115);
 lightRGB[0].blue=EEPROM.read(116);
 lightRGB[1].Hour=EEPROM.read(117);
 lightRGB[1].Minute=EEPROM.read(118);
 lightRGB[1].red=EEPROM.read(119);
 lightRGB[1].green=EEPROM.read(120);
 lightRGB[1].blue=EEPROM.read(121);
 lightRGB[2].Hour=EEPROM.read(122);
 lightRGB[2].Minute=EEPROM.read(123);
 lightRGB[2].red=EEPROM.read(124);
 lightRGB[2].green=EEPROM.read(125);
 lightRGB[2].blue=EEPROM.read(126);
 lightRGB[3].Hour=EEPROM.read(127);
 lightRGB[3].Minute=EEPROM.read(128);
 lightRGB[3].red=EEPROM.read(129);
 lightRGB[3].green=EEPROM.read(130);
 lightRGB[3].blue=EEPROM.read(131);
 lightRGB[4].Hour=EEPROM.read(132);
 lightRGB[4].Minute=EEPROM.read(133);
 lightRGB[4].red=EEPROM.read(134);
 lightRGB[4].green=EEPROM.read(135);
 lightRGB[4].blue=EEPROM.read(136);
 lightRGB[5].Hour=EEPROM.read(137);
 lightRGB[5].Minute=EEPROM.read(138);
 lightRGB[5].red=EEPROM.read(139);
 lightRGB[5].green=EEPROM.read(140);
 lightRGB[5].blue=EEPROM.read(141);
 lightRGB[6].Hour=EEPROM.read(142);
 lightRGB[6].Minute=EEPROM.read(143);
 lightRGB[6].red=EEPROM.read(144);
 lightRGB[6].green=EEPROM.read(145);
 lightRGB[6].blue=EEPROM.read(146);
 lightRGB[7].Hour=EEPROM.read(147);
 lightRGB[7].Minute=EEPROM.read(148);
 lightRGB[7].red=EEPROM.read(149);
 lightRGB[7].green=EEPROM.read(150);
 lightRGB[7].blue=EEPROM.read(151);
 lightRGB[8].Hour=EEPROM.read(152);
 lightRGB[8].Minute=EEPROM.read(153);
 lightRGB[8].red=EEPROM.read(154);
 lightRGB[8].green=EEPROM.read(155);
 lightRGB[8].blue=EEPROM.read(156);
 lightRGB[9].Hour=EEPROM.read(157);
 lightRGB[9].Minute=EEPROM.read(158);
 lightRGB[9].red=EEPROM.read(159);
 lightRGB[9].green=EEPROM.read(160);
 lightRGB[9].blue=EEPROM.read(161);
 lightRGB[10].Hour=EEPROM.read(162);
 lightRGB[10].Minute=EEPROM.read(163);
 lightRGB[10].red=EEPROM.read(164);
 lightRGB[10].green=EEPROM.read(165);
 lightRGB[10].blue=EEPROM.read(166);
 lightRGB[11].Hour=EEPROM.read(167);
 lightRGB[11].Minute=EEPROM.read(168);
 lightRGB[11].red=EEPROM.read(168);
 lightRGB[11].green=EEPROM.read(170);
 lightRGB[11].blue=EEPROM.read(171);
 MoonRed = EEPROM.read(172);
 MoonGreen = EEPROM.read(173);
 MoonBlue = EEPROM.read(174);
 MoonMinutes = EEPROM.read(175);



 voidSetup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 void Setup - initialisierung aller Werte und Libraries
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

void setup() {
	wdt_enable(WDTO_8S);  //enable watchdog - reload after 8 seconds
	file.setSSpin(53);
	file.initFAT(SPISPEED_VERYHIGH);
	// thsoot Aquaduino
	pinMode(53, OUTPUT);
	SD.begin(53);
	pinMode(44, OUTPUT);
	analogWrite(backlightPIN, 255);
	//reset Co2Values
	for (int i = 0; i < 96; i++) {
		Co2Values[i] = 1;
	}
	for (int i = 0; i < 96; i++) {
		//float rndF=random(690, 710);
		PHValues[i] = 7;
		TempValues[i] = 26;
	}

	Serial.begin(57600);
	Serial2.begin(9600);  //to ESP
	Serial.println(F("REBOOT FINISHED"));
#if debug
	{
		printMyValues();

	}
#endif

	//ph Stuff                                                      //set baud rate for the hardware serial port_0 to 38400
	Serial3.begin(9600);     //set baud rate for software serial port_3 to 38400
	inputstring.reserve(10); //set aside some bytes for receiving data from the PC
	PhWertString.reserve(30); //set aside some bytes for receiving data from Atlas Scientific product
	sensors.begin();    //temp Sensor
	//end

	//mySwtich initialisation

	mySwitch.enableTransmit(8); // Transmitter is connected to Arduino Pin #8
	mySwitch.enableReceive(4); // this is pin 19
	//mySwitch.setPulseLength(426);
	mySwitch.setRepeatTransmit(20);

	//Begin I2C - Relais und RTC
	Wire.begin();
	rtc.begin();
	DateTime now = rtc.now();
	lastFert = now.unixtime() - 82800;
	sx1509.init();  // Initialize the SX1509, does Wire.begin()
	/**  sx1509.pinDir(pump1Pin, OUTPUT);  // Set SX1509 pin 14 as an output
	 sx1509.pinDir(pump2Pin, OUTPUT);  //
	 sx1509.pinDir(light230Pin, OUTPUT);  //

	 sx1509.pinDir(co2Pin, OUTPUT);  //
	 sx1509.pinDir(heaterPin, OUTPUT);  //
	 sx1509.pinDir(coolPin, OUTPUT);  //
	 */
	sx1509.pinDir(light2Pin, OUTPUT);  //
	sx1509.pinDir(light1Pin, OUTPUT);  //
	sx1509.pinDir(dPump1Pin, OUTPUT);  //
	sx1509.pinDir(dPump2Pin, OUTPUT);  //
	sx1509.pinDir(dPump3Pin, OUTPUT);  //
	sx1509.pinDir(fanPin, OUTPUT);  //

	//END I2C
	//touchscreen
	myGLCD.InitLCD();
	myGLCD.clrScr();
	myGLCD.InitLCD(PORTRAIT);
	myTouch.InitTouch(PORTRAIT);
	myTouch.setPrecision(PREC_EXTREME);
	// get Values from EEPROM
	readPowerSchedule();
	readCleanSched();
	readPHValue();
	readTempValue();
	readFerti();
	readLightPWM();
	readLightRGB();
	readScreenScreen();
	readMoonMode();
	readTVMode();
	highestPH = PHUpperLimit;
	lowestPH = PHLowerLimit;

	timeToNextLight = now.unixtime() - (now.unixtime() - 86400 * 7); //set it to 7 days as fallback
	myGLCD.setFont(BigFont);
	myGLCD.setBackColor(0, 0, 0);
	drawScreen();
	processPump();
	analogWrite(redPin, 0);
	analogWrite(greenPin, 0);
	analogWrite(bluePin, 0);

	Aquaduino = SD.open("aqua.txt", FILE_WRITE);
	if (Aquaduino) {
		Aquaduino.println(
				F("Initialisation complete - Aquaduino up and running"));
		Aquaduino.println(
				F(
						"Date;Time;Temperature;PH;PWM;pump2Value;light230Value;light1Value=true;light2Value;co2Value;heaterValue;dPump1Value;dPump2Value;dPump3Value;coolValue;"));
		Aquaduino.close();
	}
#if debug
	{
		printMyValues();
		Serial.println(F("REBOOT FINISHED"));
	}
#endif

	beepNow(2000, 1, 2);
}

void loop() {
	wdt_reset();

	currentMillis = millis(); // get current millis
	if (myTouch.dataAvailable()) //read x,y when button is touched

	{ // make sure it's been .2 sec between touches
	  // if (currentMillis - prevMillisTouch > 0)
		if (dispScreen != 13) {
			analogWrite(backlightPIN, 255);
		}
		{
			myTouch.read();
			x = myTouch.getX();
			y = myTouch.getY();
			prevMillisTouch = currentMillis;

			switch (dispScreen) { // Caputre Buttons @ F ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			case 0:  // home screen
				if (((x >= HomeButtonCoord[0]) && (x <= HomeButtonCoord[2]))
						&& ((y >= HomeButtonCoord[1])
								&& (y <= HomeButtonCoord[3]))) // homebutton
						{
					waitForIt(HomeButtonCoord[0], HomeButtonCoord[1],
							HomeButtonCoord[2], HomeButtonCoord[3]);
					// lastScreen = 1; //we fake that we switched from another screen - this will FULL reload the HomeScreen on press on the Home button
					dispScreen = 0;
					drawScreen();
				} else if (((x >= FeedButtonCoord[0])
						&& (x <= FeedButtonCoord[2]))
						&& ((y >= FeedButtonCoord[1])
								&& (y <= FeedButtonCoord[3]))) {
					waitForIt(FeedButtonCoord[0], FeedButtonCoord[1],
							FeedButtonCoord[2], FeedButtonCoord[3]);
					manualOverride = true;
					light2Value = true;
					light1Value = true;
					processRelais();
					calculatedRed = 0;
					calculatedGreen = 0;
					calculatedBlue = 0;
					analogWrite(redPin, calculatedRed);
					analogWrite(greenPin, calculatedGreen);
					analogWrite(bluePin, calculatedBlue);
					dispScreen = 1;

					drawScreen();

				} else if (((x >= PowerButtonCoord[0])
						&& (x <= PowerButtonCoord[2]))
						&& ((y >= PowerButtonCoord[1])
								&& (y <= PowerButtonCoord[3]))) // powerbutton
						{
					waitForIt(PowerButtonCoord[0], PowerButtonCoord[1],
							PowerButtonCoord[2], PowerButtonCoord[3]);
					dispScreen = 2;
					drawScreen();
					getDistance();
				} else if (((x >= SettingsButtonCoord[0])
						&& (x <= SettingsButtonCoord[2]))
						&& ((y >= SettingsButtonCoord[1])
								&& (y <= SettingsButtonCoord[3]))) //  settings
						{
					waitForIt(SettingsButtonCoord[0], SettingsButtonCoord[1],
							SettingsButtonCoord[2], SettingsButtonCoord[3]);
					dispScreen = 3;
					drawScreen();
				} else if (((x >= BottomButtonCoord[0])
						&& (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // bottomResetButton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					cleaningInProcess = false;
					manualOverride = false;
					TVModeState = false;
					pump1Value = false;
					pump2Value = false;
					printDate(now, 5, 5);
					lightCalculator();
					drawScreen();
					AI();
					processPump();
				} else if (((x >= LightUp[0]) && (x <= LightUp[2]))
						&& ((y >= LightUp[1]) && (y <= LightUp[3]))) // pwmUp
						{ //waitForIt(LightUp[0], LightUp[1], LightUp[2], LightUp[3]);
					calculatedPWM += 2.55;
					if (calculatedPWM > 255) {
						calculatedPWM = 0;
					}
					if (calculatedPWM < 0) {
						calculatedPWM = 255;
					}

					manualOverride = true;
					drawPWM();
				} else if (((x >= LightDown[0]) && (x <= LightDown[2]))
						&& ((y >= LightDown[1]) && (y <= LightDown[3]))) // pwmDown
						{ //waitForIt(LightDown[0], LightDown[1], LightDown[2], LightDown[3]);
					manualOverride = true;
					calculatedPWM -= 2.55;
					if (calculatedPWM > 255) {
						calculatedPWM = 0;
					}
					if (calculatedPWM < 0) {
						calculatedPWM = 255;
					}

					drawPWM();
				}
				break;

				// Caputre Buttons @ FeedScreen ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			case 1:
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;

					drawScreen();

				}

				else if (((x >= ResetCord[0]) && (x <= ResetCord[2]))
						&& ((y >= ResetCord[1]) && (y <= ResetCord[3]))) // homebutton
						{
					waitForIt(ResetCord[0], ResetCord[1], ResetCord[2],
							ResetCord[3]);
					manualOverride = false;

					dispScreen = 0;
					drawScreen();

					//processRF();
					lightCalculator();
					AI();

				}

				break;

				// Caputre Buttons @ PowerScreen ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			case 2:
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= Filter1Cord[0]) && (x <= Filter1Cord[2]))
						&& ((y >= Filter1Cord[1]) && (y <= Filter1Cord[3]))) // homebutton
						{
					waitForIt(Filter1Cord[0], Filter1Cord[1], Filter1Cord[2],
							Filter1Cord[3]);
					manualOverride = true;
					pump1Value = !pump1Value;
					updatePowerIcons();
					if (!pump1Value) {
						mySwitch.send(f11on, 24);
					} else {
						mySwitch.send(f11off, 24);
					};

				} else if (((x >= Filter2Cord[0]) && (x <= Filter2Cord[2]))
						&& ((y >= Filter2Cord[1]) && (y <= Filter2Cord[3]))) // homebutton
						{
					waitForIt(Filter2Cord[0], Filter2Cord[1], Filter2Cord[2],
							Filter2Cord[3]);
					manualOverride = true;
					pump2Value = !pump2Value;
					updatePowerIcons();
					if (!pump2Value) {
						mySwitch.send(f42on, 24);
					} else {
						mySwitch.send(f42off, 24);
					}
				}

				else if (((x >= Ligth1Cord[0]) && (x <= Ligth1Cord[2]))
						&& ((y >= Ligth1Cord[1]) && (y <= Ligth1Cord[3]))) // homebutton
						{
					waitForIt(Ligth1Cord[0], Ligth1Cord[1], Ligth1Cord[2],
							Ligth1Cord[3]);
					manualOverride = true;
					light230Value = !light230Value;
					updatePowerIcons();
					if (!light230Value) {
						mySwitch.send(f13on, 24);
					} else {
						mySwitch.send(f13off, 24);
					}

				} else if (((x >= Light2Cord[0]) && (x <= Light2Cord[2]))
						&& ((y >= Light2Cord[1]) && (y <= Light2Cord[3]))) // homebutton
						{
					waitForIt(Light2Cord[0], Light2Cord[1], Light2Cord[2],
							Light2Cord[3]);
					manualOverride = true;
					light1Value = !light1Value;
					light2Value = !light2Value;
					updatePowerIcons();
					sx1509.writePin(light1Pin, light1Value);
					sx1509.writePin(light2Pin, light2Value);
				} else if (((x >= Co2Cord[0]) && (x <= Co2Cord[2]))
						&& ((y >= Co2Cord[1]) && (y <= Co2Cord[3]))) // homebutton
						{
					waitForIt(Co2Cord[0], Co2Cord[1], Co2Cord[2], Co2Cord[3]);
					manualOverride = true;
					co2Value = !co2Value;
					updatePowerIcons();
					if (!co2Value) {
						mySwitch.send(f22on, 24);
					} else {
						mySwitch.send(f22off, 24);
					}
				} else if (((x >= HeaterCord[0]) && (x <= HeaterCord[2]))
						&& ((y >= HeaterCord[1]) && (y <= HeaterCord[3]))) // homebutton
						{
					waitForIt(HeaterCord[0], HeaterCord[1], HeaterCord[2],
							HeaterCord[3]);
					manualOverride = true;
					heaterValue = !heaterValue;
					updatePowerIcons();
					if (!heaterValue) {
						mySwitch.send(f21on, 24);
					} else {
						mySwitch.send(f21off, 24);
					}

				} else if (((x >= CoolingCord[0]) && (x <= CoolingCord[2]))
						&& ((y >= CoolingCord[1]) && (y <= CoolingCord[3]))) // homebutton
						{
					waitForIt(CoolingCord[0], CoolingCord[1], CoolingCord[2],
							CoolingCord[3]);
					manualOverride = true;
					coolValue = !coolValue;
					processRelais();
					updatePowerIcons();

				} else if (((x >= AllOFFCord[0]) && (x <= AllOFFCord[2]))
						&& ((y >= AllOFFCord[1]) && (y <= AllOFFCord[3]))) // homebutton
						{
					waitForIt(AllOFFCord[0], AllOFFCord[1], AllOFFCord[2],
							AllOFFCord[3]);
					manualOverride = false;
					pump1Value = true;
					pump2Value = true;
					light230Value = true;
					light1Value = true;
					light2Value = true;
					co2Value = true;
					heaterValue = true;
					coolValue = true;
					updatePowerIcons();
					processPump();
					processRF();
					processPump();

				} else if (((x >= ResetCord[0]) && (x <= ResetCord[2]))
						&& ((y >= ResetCord[1]) && (y <= ResetCord[3]))) // Resetbutton
						{
					waitForIt(ResetCord[0], ResetCord[1], ResetCord[2],
							ResetCord[3]);
					manualOverride = false;
					cleaningInProcess = false;
					beepActive = false;
					TVModeState = false;
					pump1Value = false;
					pump2Value = false;
					dispScreen = 0;
					drawScreen();
					//processRF();
					lightCalculator();
					AI();
					processPump();

				}

				else if (((x >= CleanModeCord[0]) && (x <= CleanModeCord[2]))
						&& ((y >= CleanModeCord[1]) && (y <= CleanModeCord[3]))) // Cleanbutton
						{
					waitForIt(CleanModeCord[0], CleanModeCord[1],
							CleanModeCord[2], CleanModeCord[3]);
					CleanMode();
				} else if (((x >= SpeakerCord[0]) && (x <= SpeakerCord[2]))
						&& ((y >= SpeakerCord[1]) && (y <= SpeakerCord[3]))) // Cleanbutton
						{
					waitForIt(SpeakerCord[0], SpeakerCord[1], SpeakerCord[2],
							SpeakerCord[3]);
					beepActive = !beepActive;
					drawScreen();
				}

				break;

				// Caputre Buttons @ Settingsscreen ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			case 3:
				if (((x >= ClockCord[0]) && (x <= ClockCord[2]))
						&& ((y >= ClockCord[1]) && (y <= ClockCord[3]))) // homebutton
						{
					waitForIt(ClockCord[0], ClockCord[1], ClockCord[2],
							ClockCord[3]);
					dispScreen = 5;
					adjustTimer = rtc.now();
					adjustTimer = adjustTimer.unixtime() + 60
							- adjustTimer.second();
					drawScreen();
				} else if (((x >= LightsCord[0]) && (x <= LightsCord[2]))
						&& ((y >= LightsCord[1]) && (y <= LightsCord[3]))) // homebutton
						{
					waitForIt(LightsCord[0], LightsCord[1], LightsCord[2],
							LightsCord[3]);
					dispScreen = 4;
					drawScreen();
				} else if (((x >= CleanCord[0]) && (x <= CleanCord[2]))
						&& ((y >= CleanCord[1]) && (y <= CleanCord[3]))) // homebutton
						{
					waitForIt(CleanCord[0], CleanCord[1], CleanCord[2],
							CleanCord[3]);
					dispScreen = 6;
					drawScreen();
				} else if (((x >= ScheCord[0]) && (x <= ScheCord[2]))
						&& ((y >= ScheCord[1]) && (y <= ScheCord[3]))) // homebutton
						{
					waitForIt(ScheCord[0], ScheCord[1], ScheCord[2],
							ScheCord[3]);
					readReminder();
					dispScreen = 7;
					drawScreen();
				} else if (((x >= ScreenCord[0]) && (x <= ScreenCord[2]))
						&& ((y >= ScreenCord[1]) && (y <= ScreenCord[3]))) // homebutton
						{
					waitForIt(ScreenCord[0], ScreenCord[1], ScreenCord[2],
							ScreenCord[3]);
					readScreenScreen();
					dispScreen = 13;
					drawScreen();
				}

				else if (((x >= HeatCord[0]) && (x <= HeatCord[2]))
						&& ((y >= HeatCord[1]) && (y <= HeatCord[3]))) // homebutton
						{
					waitForIt(HeatCord[0], HeatCord[1], HeatCord[2],
							HeatCord[3]);
					dispScreen = 8;
					drawScreen();
				} else if (((x >= Co2SetCord[0]) && (x <= Co2SetCord[2]))
						&& ((y >= Co2SetCord[1]) && (y <= Co2SetCord[3]))) // homebutton
						{
					waitForIt(Co2SetCord[0], Co2SetCord[1], Co2SetCord[2],
							Co2SetCord[3]);
					dispScreen = 12;
					drawScreen();
				} else if (((x >= DoseCord[0]) && (x <= DoseCord[2]))
						&& ((y >= DoseCord[1]) && (y <= DoseCord[3]))) // homebutton
						{
					waitForIt(DoseCord[0], DoseCord[1], DoseCord[2],
							DoseCord[3]);
					dispScreen = 9;
					drawScreen();
				} else if (((x >= PowerSchedCord[0]) && (x <= PowerSchedCord[2]))
						&& ((y >= PowerSchedCord[1]) && (y <= PowerSchedCord[3]))) // homebutton
						{
					waitForIt(PowerSchedCord[0], PowerSchedCord[1],
							PowerSchedCord[2], PowerSchedCord[3]);
					dispScreen = 10;
					drawScreen();
				} else if (((x >= RGBCord[0]) && (x <= RGBCord[2]))
						&& ((y >= RGBCord[1]) && (y <= RGBCord[3]))) // homebutton
						{
					waitForIt(RGBCord[0], RGBCord[1], RGBCord[2], RGBCord[3]);
					dispScreen = 14;
					drawScreen();
				} else if (((x >= TVModeCord[0]) && (x <= TVModeCord[2]))
						&& ((y >= TVModeCord[1]) && (y <= TVModeCord[3]))) // homebutton
						{
					waitForIt(TVModeCord[0], TVModeCord[1], TVModeCord[2],
							TVModeCord[3]);
					dispScreen = 15;
					drawScreen();
					TVMode();
					drawScreen();
				} else if (((x >= MoonModeCord[0]) && (x <= MoonModeCord[2]))
						&& ((y >= MoonModeCord[1]) && (y <= MoonModeCord[3]))) // homebutton
						{
					waitForIt(MoonModeCord[0], MoonModeCord[1], MoonModeCord[2],
							MoonModeCord[3]);
					dispScreen = 16;
					drawScreen();
					MoonMode();

				}

				else if (((x >= BottomButtonCoord[0])
						&& (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				}
				break;

			case 4:  // Lights
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= LightMode1Cord[0]) && (x <= LightMode1Cord[2]))
						&& ((y >= LightMode1Cord[1]) && (y <= LightMode1Cord[3]))) {
					waitForIt(LightMode1Cord[0], LightMode1Cord[1],
							LightMode1Cord[2], LightMode1Cord[3]);
					dispScreen = 41;
					lightScreenSet = 0;
					drawScreen();
				} else if (((x >= LightMode2Cord[0]) && (x <= LightMode2Cord[2]))
						&& ((y >= LightMode2Cord[1]) && (y <= LightMode2Cord[3]))) {
					waitForIt(LightMode2Cord[0], LightMode2Cord[1],
							LightMode2Cord[2], LightMode2Cord[3]);
					dispScreen = 42;
					lightScreenSet = 2;
					drawScreen();
				} else if (((x >= LightMode3Cord[0]) && (x <= LightMode3Cord[2]))
						&& ((y >= LightMode3Cord[1]) && (y <= LightMode3Cord[3]))) {
					waitForIt(LightMode3Cord[0], LightMode3Cord[1],
							LightMode3Cord[2], LightMode3Cord[3]);
					dispScreen = 43;
					lightScreenSet = 4;
					drawScreen();
				} else if (((x >= LightMode4Cord[0]) && (x <= LightMode4Cord[2]))
						&& ((y >= LightMode4Cord[1]) && (y <= LightMode4Cord[3]))) {
					waitForIt(LightMode4Cord[0], LightMode4Cord[1],
							LightMode4Cord[2], LightMode4Cord[3]);
					dispScreen = 44;
					lightScreenSet = 6;
					drawScreen();
				} else if (((x >= LightMode5Cord[0]) && (x <= LightMode5Cord[2]))
						&& ((y >= LightMode5Cord[1]) && (y <= LightMode5Cord[3]))) {
					waitForIt(LightMode5Cord[0], LightMode5Cord[1],
							LightMode5Cord[2], LightMode5Cord[3]);
					dispScreen = 45;
					lightScreenSet = 8;
					drawScreen();
				} else if (((x >= LightMode6Cord[0]) && (x <= LightMode6Cord[2]))
						&& ((y >= LightMode6Cord[1]) && (y <= LightMode6Cord[3]))) {
					waitForIt(LightMode6Cord[0], LightMode6Cord[1],
							LightMode6Cord[2], LightMode6Cord[3]);
					dispScreen = 46;
					lightScreenSet = 10;
					drawScreen();
				}
				break;

			case 41:  //listen on lightscene1
			case 42:  //listen on lightscene2
			case 43:  //listen on lightscene3
			case 44:  //listen on lightscene4
			case 45:  //listen on lightscene5
			case 46:  //listen on lightscene6

				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= powLightOnHourUp[0])
						&& (x <= powLightOnHourUp[2]))
						&& ((y >= powLightOnHourUp[1])
								&& (y <= powLightOnHourUp[3]))) {
					waitForIt(powLightOnHourUp[0], powLightOnHourUp[1],
							powLightOnHourUp[2], powLightOnHourUp[3]);
					if ((lightPWM[lightScreenSet].Hour >= 23)
							&& (lightPWM[lightScreenSet].Hour <= 250)) {
						lightPWM[lightScreenSet].Hour = 0;
					} else {
						lightPWM[lightScreenSet].Hour++;
					}
					UpdateLightScene();
				} else if (((x >= powLightOnHourDown[0])
						&& (x <= powLightOnHourDown[2]))
						&& ((y >= powLightOnHourDown[1])
								&& (y <= powLightOnHourDown[3]))) {
					waitForIt(powLightOnHourDown[0], powLightOnHourDown[1],
							powLightOnHourDown[2], powLightOnHourDown[3]);
					lightPWM[lightScreenSet].Hour--;
					if (lightPWM[lightScreenSet].Hour >= 230) {
						lightPWM[lightScreenSet].Hour = 23;
					}
					UpdateLightScene();
				} else if (((x >= powLightOnMinuteUp[0])
						&& (x <= powLightOnMinuteUp[2]))
						&& ((y >= powLightOnMinuteUp[1])
								&& (y <= powLightOnMinuteUp[3]))) {
					waitForIt(powLightOnMinuteUp[0], powLightOnMinuteUp[1],
							powLightOnMinuteUp[2], powLightOnMinuteUp[3]);
					if ((lightPWM[lightScreenSet].Minute >= 59)
							&& (lightPWM[lightScreenSet].Minute <= 250)) {
						lightPWM[lightScreenSet].Minute = 0;
					} else {
						lightPWM[lightScreenSet].Minute++;
					}
					UpdateLightScene();
				} else if (((x >= powLightOnMinuteDown[0])
						&& (x <= powLightOnMinuteDown[2]))
						&& ((y >= powLightOnMinuteDown[1])
								&& (y <= powLightOnMinuteDown[3]))) {
					waitForIt(powLightOnMinuteDown[0], powLightOnMinuteDown[1],
							powLightOnMinuteDown[2], powLightOnMinuteDown[3]);
					lightPWM[lightScreenSet].Minute--;
					if (lightPWM[lightScreenSet].Minute >= 230) {
						lightPWM[lightScreenSet].Minute = 59;
					}
					UpdateLightScene();
				} else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);

					lightPWM[lightScreenSet].pwmValue += 2.55;
					if (lightPWM[lightScreenSet].pwmValue > 255) {
						lightPWM[lightScreenSet].pwmValue = 0;
					}
					if (lightPWM[lightScreenSet].pwmValue < 0) {
						lightPWM[lightScreenSet].pwmValue = 255;
					}

					UpdateLightScene();
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					lightPWM[lightScreenSet].pwmValue -= 2.55;
					if (lightPWM[lightScreenSet].pwmValue > 255) {
						lightPWM[lightScreenSet].pwmValue = 0;
					}
					if (lightPWM[lightScreenSet].pwmValue < 0) {
						lightPWM[lightScreenSet].pwmValue = 255;
					}
					UpdateLightScene();
				} else if (((x >= powCo2OnHourUp[0]) && (x <= powCo2OnHourUp[2]))
						&& ((y >= powCo2OnHourUp[1]) && (y <= powCo2OnHourUp[3]))) // homebutton
						{
					waitForIt(powCo2OnHourUp[0], powCo2OnHourUp[1],
							powCo2OnHourUp[2], powCo2OnHourUp[3]);
					if ((lightPWM[lightScreenSet + 1].Hour >= 23)
							&& (lightPWM[lightScreenSet + 1].Hour <= 250)) {
						lightPWM[lightScreenSet + 1].Hour = 0;
					} else {
						lightPWM[lightScreenSet + 1].Hour++;
					}
					UpdateLightScene();
				} else if (((x >= powCo2OnHourDown[0])
						&& (x <= powCo2OnHourDown[2]))
						&& ((y >= powCo2OnHourDown[1])
								&& (y <= powCo2OnHourDown[3]))) // homebutton
						{
					waitForIt(powCo2OnHourDown[0], powCo2OnHourDown[1],
							powCo2OnHourDown[2], powCo2OnHourDown[3]);
					lightPWM[lightScreenSet + 1].Hour--;
					if (lightPWM[lightScreenSet + 1].Hour >= 230) {
						lightPWM[lightScreenSet + 1].Hour = 23;
					}
					UpdateLightScene();
				} else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					if ((lightPWM[lightScreenSet + 1].Minute >= 59)
							&& (lightPWM[lightScreenSet + 1].Minute <= 250)) {
						lightPWM[lightScreenSet + 1].Minute = 0;
					} else {
						lightPWM[lightScreenSet + 1].Minute++;
					}
					UpdateLightScene();
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					lightPWM[lightScreenSet + 1].Minute--;
					if (lightPWM[lightScreenSet + 1].Minute >= 230) {
						lightPWM[lightScreenSet + 1].Minute = 59;
					}
					UpdateLightScene();
				} else if (((x >= powCo2OffMinuteUp[0])
						&& (x <= powCo2OffMinuteUp[2]))
						&& ((y >= powCo2OffMinuteUp[1])
								&& (y <= powCo2OffMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1],
							powCo2OffMinuteUp[2], powCo2OffMinuteUp[3]);
					lightPWM[lightScreenSet + 1].pwmValue += 2.55;
					if (lightPWM[lightScreenSet + 1].pwmValue > 255) {
						lightPWM[lightScreenSet + 1].pwmValue = 0;
					}
					if (lightPWM[lightScreenSet + 1].pwmValue < 0) {
						lightPWM[lightScreenSet + 1].pwmValue = 255;
					}
					UpdateLightScene();
				} else if (((x >= powCo2OffMinuteDown[0])
						&& (x <= powCo2OffMinuteDown[2]))
						&& ((y >= powCo2OffMinuteDown[1])
								&& (y <= powCo2OffMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1],
							powCo2OffMinuteDown[2], powCo2OffMinuteDown[3]);
					lightPWM[lightScreenSet + 1].pwmValue -= 2.55;
					if (lightPWM[lightScreenSet + 1].pwmValue > 255) {
						lightPWM[lightScreenSet + 1].pwmValue = 0;
					}
					if (lightPWM[lightScreenSet + 1].pwmValue < 0) {
						lightPWM[lightScreenSet + 1].pwmValue = 255;
					}
					UpdateLightScene();
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveLightPWM();
					dispScreen = 4;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readLightPWM();
					dispScreen = 4;
					drawScreen();
				}
				break;

				// Caputre Buttons @ ClockScreen ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			case 5:
				if (((x >= SetClockCord[0]) && (x <= SetClockCord[2]))
						&& ((y >= SetClockCord[1]) && (y <= SetClockCord[3]))) // homebutton
						{
					waitForIt(SetClockCord[0], SetClockCord[1], SetClockCord[2],
							SetClockCord[3]);
					rtc.adjust(adjustTimer);
					dispScreen = 3;
					drawScreen();
				} else if (((x >= HourUp[0]) && (x <= HourUp[2]))
						&& ((y >= HourUp[1]) && (y <= HourUp[3]))) // homebutton
						{
					waitForIt(HourUp[0], HourUp[1], HourUp[2], HourUp[3]);
					adjustTimer = adjustTimer.unixtime() + 3600;
					updateClockSettings();
				} else if (((x >= HourDown[0]) && (x <= HourDown[2]))
						&& ((y >= HourDown[1]) && (y <= HourDown[3]))) // homebutton
						{
					waitForIt(HourDown[0], HourDown[1], HourDown[2],
							HourDown[3]);
					adjustTimer = adjustTimer.unixtime() - 3600;
					updateClockSettings();
				} else if (((x >= MinuteUp[0]) && (x <= MinuteUp[2]))
						&& ((y >= MinuteUp[1]) && (y <= MinuteUp[3]))) // homebutton
						{
					waitForIt(MinuteUp[0], MinuteUp[1], MinuteUp[2],
							MinuteUp[3]);
					adjustTimer = adjustTimer.unixtime() + 60;
					updateClockSettings();
				} else if (((x >= MinuteDown[0]) && (x <= MinuteDown[2]))
						&& ((y >= MinuteDown[1]) && (y <= MinuteDown[3]))) // homebutton
						{
					waitForIt(MinuteDown[0], MinuteDown[1], MinuteDown[2],
							MinuteDown[3]);
					adjustTimer = adjustTimer.unixtime() - 60;
					updateClockSettings();
				} else if (((x >= DayUp[0]) && (x <= DayUp[2]))
						&& ((y >= DayUp[1]) && (y <= DayUp[3]))) // homebutton
						{
					waitForIt(DayUp[0], DayUp[1], DayUp[2], DayUp[3]);
					adjustTimer = adjustTimer.unixtime() + 86400;
					updateClockSettings();
				} else if (((x >= DayDown[0]) && (x <= DayDown[2]))
						&& ((y >= DayDown[1]) && (y <= DayDown[3]))) // homebutton
						{
					waitForIt(DayDown[0], DayDown[1], DayDown[2], DayDown[3]);
					adjustTimer = adjustTimer.unixtime() - 86400;
					updateClockSettings();
				} else if (((x >= MonthUp[0]) && (x <= MonthUp[2]))
						&& ((y >= MonthUp[1]) && (y <= MonthUp[3]))) // homebutton
						{
					waitForIt(MonthUp[0], MonthUp[1], MonthUp[2], MonthUp[3]);
					adjustTimer = adjustTimer.unixtime() + (30 * 86400);
					updateClockSettings();
				} else if (((x >= MonthDown[0]) && (x <= MonthDown[2]))
						&& ((y >= MonthDown[1]) && (y <= MonthDown[3]))) // homebutton
						{
					waitForIt(MonthDown[0], MonthDown[1], MonthDown[2],
							MonthDown[3]);
					adjustTimer = adjustTimer.unixtime() - (30 * 86400L);
					updateClockSettings();
				} else if (((x >= YearUp[0]) && (x <= YearUp[2]))
						&& ((y >= YearUp[1]) && (y <= YearUp[3]))) // homebutton
						{
					waitForIt(YearUp[0], YearUp[1], YearUp[2], YearUp[3]);
					adjustTimer = adjustTimer.unixtime() + (365 * 86400);
					updateClockSettings();
				} else if (((x >= YearDown[0]) && (x <= YearDown[2]))
						&& ((y >= YearDown[1]) && (y <= YearDown[3]))) // homebutton
						{
					waitForIt(YearDown[0], YearDown[1], YearDown[2],
							YearDown[3]);
					adjustTimer = adjustTimer.unixtime() - (365 * 86400);
					updateClockSettings();
				} else if (((x >= BottomButtonCoord[0])
						&& (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				}
				break;

			case 6:  // Clean Schedule
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= Filter1Cord[0]) && (x <= Filter1Cord[2]))
						&& ((y >= Filter1Cord[1]) && (y <= Filter1Cord[3]))) // homebutton
						{
					waitForIt(Filter1Cord[0], Filter1Cord[1], Filter1Cord[2],
							Filter1Cord[3]);
					pump1Clean = !pump1Clean;
					updateCleanSchedScreen();
				} else if (((x >= Filter2Cord[0]) && (x <= Filter2Cord[2]))
						&& ((y >= Filter2Cord[1]) && (y <= Filter2Cord[3]))) // homebutton
						{
					waitForIt(Filter2Cord[0], Filter2Cord[1], Filter2Cord[2],
							Filter2Cord[3]);
					pump2Clean = !pump2Clean;
					updateCleanSchedScreen();
				}

				else if (((x >= Ligth1Cord[0]) && (x <= Ligth1Cord[2]))
						&& ((y >= Ligth1Cord[1]) && (y <= Ligth1Cord[3]))) // homebutton
						{
					waitForIt(Ligth1Cord[0], Ligth1Cord[1], Ligth1Cord[2],
							Ligth1Cord[3]);
					light230Clean = !light230Clean;
					updateCleanSchedScreen();
				} else if (((x >= Light2Cord[0]) && (x <= Light2Cord[2]))
						&& ((y >= Light2Cord[1]) && (y <= Light2Cord[3]))) // homebutton
						{
					waitForIt(Light2Cord[0], Light2Cord[1], Light2Cord[2],
							Light2Cord[3]);
					light2Clean = !light2Clean;
					updateCleanSchedScreen();
				} else if (((x >= Co2Cord[0]) && (x <= Co2Cord[2]))
						&& ((y >= Co2Cord[1]) && (y <= Co2Cord[3]))) // homebutton
						{
					waitForIt(Co2Cord[0], Co2Cord[1], Co2Cord[2], Co2Cord[3]);
					co2Clean = !co2Clean;
					updateCleanSchedScreen();
				} else if (((x >= HeaterCord[0]) && (x <= HeaterCord[2]))
						&& ((y >= HeaterCord[1]) && (y <= HeaterCord[3]))) // homebutton
						{
					waitForIt(HeaterCord[0], HeaterCord[1], HeaterCord[2],
							HeaterCord[3]);
					heaterClean = !heaterClean;
					updateCleanSchedScreen();
				} else if (((x >= CoolingCord[0]) && (x <= CoolingCord[2]))
						&& ((y >= CoolingCord[1]) && (y <= CoolingCord[3]))) // homebutton
						{
					waitForIt(CoolingCord[0], CoolingCord[1], CoolingCord[2],
							CoolingCord[3]);
					coolClean = !coolClean;
					updateCleanSchedScreen();
				} else if (((x >= powCo2OffMinuteUp[0])
						&& (x <= powCo2OffMinuteUp[2]))
						&& ((y >= powCo2OffMinuteUp[1])
								&& (y <= powCo2OffMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1],
							powCo2OffMinuteUp[2], powCo2OffMinuteUp[3]);
					cleanMinutes++;
					quickUpdateCleanSchedScreen();
				} else if (((x >= powCo2OffMinuteDown[0])
						&& (x <= powCo2OffMinuteDown[2]))
						&& ((y >= powCo2OffMinuteDown[1])
								&& (y <= powCo2OffMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1],
							powCo2OffMinuteDown[2], powCo2OffMinuteDown[3]);
					cleanMinutes--;
					quickUpdateCleanSchedScreen();
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveCleanSched();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readCleanSched();
					dispScreen = 3;
					drawScreen();
				}
				break;

			case 7:  //  Reminder Screen

				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveReminder();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readReminder();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= Filter1Cord[0]) && (x <= Filter1Cord[2]))
						&& ((y >= Filter1Cord[1]) && (y <= Filter1Cord[3]))) // homebutton
						{
					actOnRealease(Filter1Cord[0], Filter1Cord[1],
							Filter1Cord[2], Filter1Cord[3]);
					tankCleandDays = (now.unixtime() - tankClean.unixtime())
							/ 86400;
					tankClean = now;

					updateRemindScreen();
				} else if (((x >= Ligth1Cord[0]) && (x <= Ligth1Cord[2]))
						&& ((y >= Ligth1Cord[1]) && (y <= Ligth1Cord[3]))) // homebutton
						{
					actOnRealease(Ligth1Cord[0], Ligth1Cord[1], Ligth1Cord[2],
							Ligth1Cord[3]);
					co2BottleDays = (now.unixtime() - co2Bottle.unixtime())
							/ 86400;
					co2Bottle = now;
					updateRemindScreen();
				} else if (((x >= Co2Cord[0]) && (x <= Co2Cord[2]))
						&& ((y >= Co2Cord[1]) && (y <= Co2Cord[3]))) // homebutton
						{
					actOnRealease(Co2Cord[0], Co2Cord[1], Co2Cord[2],
							Co2Cord[3]);
					cleanFilter1Days =
							(now.unixtime() - cleanFilter1.unixtime()) / 86400;
					cleanFilter1 = now;
					updateRemindScreen();
				} else if (((x >= CoolingCord[0]) && (x <= CoolingCord[2]))
						&& ((y >= CoolingCord[1]) && (y <= CoolingCord[3]))) // homebutton
						{
					actOnRealease(CoolingCord[0], CoolingCord[1],
							CoolingCord[2], CoolingCord[3]);
					cleanFilter2Days =
							(now.unixtime() - cleanFilter2.unixtime()) / 86400;
					cleanFilter2 = now;
					updateRemindScreen();
				}
				break;

			case 8:  //  heater
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);
					TempUpperLimit += 1;
					updateHeaterScreen();
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					TempUpperLimit -= 1;
					updateHeaterScreen();
				} else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					TempLowerLimit += 1;
					updateHeaterScreen();
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					TempLowerLimit -= 1;
					updateHeaterScreen();
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveTempValue();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readTempValue();
					dispScreen = 3;
					drawScreen();
				}
				break;

			case 9:  //  DOSE
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= LightMode1Cord[0]) && (x <= LightMode1Cord[2]))
						&& ((y >= LightMode1Cord[1]) && (y <= LightMode1Cord[3]))) {
					waitForIt(LightMode1Cord[0], LightMode1Cord[1],
							LightMode1Cord[2], LightMode1Cord[3]);
					dispScreen = 91;
					FSelect = 0;
					drawScreen();
				} else if (((x >= LightMode2Cord[0]) && (x <= LightMode2Cord[2]))
						&& ((y >= LightMode2Cord[1]) && (y <= LightMode2Cord[3]))) {
					waitForIt(LightMode2Cord[0], LightMode2Cord[1],
							LightMode2Cord[2], LightMode2Cord[3]);
					dispScreen = 91;
					FSelect = 1;
					drawScreen();
				} else if (((x >= LightMode3Cord[0]) && (x <= LightMode3Cord[2]))
						&& ((y >= LightMode3Cord[1]) && (y <= LightMode3Cord[3]))) {
					waitForIt(LightMode3Cord[0], LightMode3Cord[1],
							LightMode3Cord[2], LightMode3Cord[3]);
					dispScreen = 91;
					FSelect = 2;
					drawScreen();
				}

				else if (((x >= powCo2OnHourUp[0]) && (x <= powCo2OnHourUp[2]))
						&& ((y >= powCo2OnHourUp[1]) && (y <= powCo2OnHourUp[3]))) // homebutton
						{
					waitForIt(powCo2OnHourUp[0], powCo2OnHourUp[1],
							powCo2OnHourUp[2], powCo2OnHourUp[3]);
					if ((doseHour >= 23) && (doseHour <= 250)) {
						doseHour = 0;
					} else {
						doseHour++;
					}
					updateDoseScreen();
				} else if (((x >= powCo2OnHourDown[0])
						&& (x <= powCo2OnHourDown[2]))
						&& ((y >= powCo2OnHourDown[1])
								&& (y <= powCo2OnHourDown[3]))) // homebutton
						{
					waitForIt(powCo2OnHourDown[0], powCo2OnHourDown[1],
							powCo2OnHourDown[2], powCo2OnHourDown[3]);
					doseHour--;
					if (doseHour >= 230) {
						doseHour = 23;
					}
					updateDoseScreen();
				} else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					if ((doseMinute >= 59) && (doseMinute <= 250)) {
						doseMinute = 0;
					} else {
						doseMinute++;
					}
					updateDoseScreen();
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					doseMinute--;
					if (doseMinute >= 230) {
						doseMinute = 59;
					}
					updateDoseScreen();
				}

				else if (((x >= refillAllCord[0]) && (x <= refillAllCord[2]))
						&& ((y >= refillAllCord[1]) && (y <= refillAllCord[3]))) {
					waitForIt(refillAllCord[0], refillAllCord[1],
							refillAllCord[2], refillAllCord[3]);
					FLeft[0] = FMax[0];
					FLeft[1] = FMax[1];
					FLeft[2] = FMax[2];
					saveFerti();
					drawScreen();
				}
				break;

			case 91:  //  DOSE
			{
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= powLightOnMinuteUp[0])
						&& (x <= powLightOnMinuteUp[2]))
						&& ((y >= powLightOnMinuteUp[1])
								&& (y <= powLightOnMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOnMinuteUp[0], powLightOnMinuteUp[1],
							powLightOnMinuteUp[2], powLightOnMinuteUp[3]);
					FDose[FSelect] += 1;
					quickUpdateDoseScreenN(FSelect);
				} else if (((x >= powLightOnMinuteDown[0])
						&& (x <= powLightOnMinuteDown[2]))
						&& ((y >= powLightOnMinuteDown[1])
								&& (y <= powLightOnMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOnMinuteDown[0], powLightOnMinuteDown[1],
							powLightOnMinuteDown[2], powLightOnMinuteDown[3]);
					FDose[FSelect] -= 1;
					quickUpdateDoseScreenN(FSelect);
				} else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);
					FMax[FSelect] += 5;
					quickUpdateDoseScreenN(FSelect);
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					FMax[FSelect] -= 5;
					quickUpdateDoseScreenN(FSelect);
				} else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					FRate[FSelect] += 1;
					quickUpdateDoseScreenN(FSelect);
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					FRate[FSelect] -= 1;
					quickUpdateDoseScreenN(FSelect);
				} else if (((x >= MoCord[0]) && (x <= MoCord[2]))
						&& ((y >= MoCord[1]) && (y <= MoCord[3]))) // homebutton
						{
					waitForIt(MoCord[0], MoCord[1], MoCord[2], MoCord[3]);
					MoF[FSelect] = !MoF[FSelect];
					UpdateDoseScreenN(FSelect);
				} else if (((x >= TuCord[0]) && (x <= TuCord[2]))
						&& ((y >= TuCord[1]) && (y <= TuCord[3]))) // homebutton
						{
					waitForIt(TuCord[0], TuCord[1], TuCord[2], TuCord[3]);
					TuF[FSelect] = !TuF[FSelect];
					UpdateDoseScreenN(FSelect);
				} else if (((x >= WeCord[0]) && (x <= WeCord[2]))
						&& ((y >= WeCord[1]) && (y <= WeCord[3]))) // homebutton
						{
					waitForIt(WeCord[0], WeCord[1], WeCord[2], WeCord[3]);
					WeF[FSelect] = !WeF[FSelect];
					UpdateDoseScreenN(FSelect);
				} else if (((x >= ThCord[0]) && (x <= ThCord[2]))
						&& ((y >= ThCord[1]) && (y <= ThCord[3]))) // homebutton
						{
					waitForIt(ThCord[0], ThCord[1], ThCord[2], ThCord[3]);
					ThF[FSelect] = !ThF[FSelect];
					UpdateDoseScreenN(FSelect);
				} else if (((x >= FrCord[0]) && (x <= FrCord[2]))
						&& ((y >= FrCord[1]) && (y <= FrCord[3]))) // homebutton
						{
					waitForIt(FrCord[0], FrCord[1], FrCord[2], FrCord[3]);
					FrF[FSelect] = !FrF[FSelect];
					UpdateDoseScreenN(FSelect);
				} else if (((x >= SaCord[0]) && (x <= SaCord[2]))
						&& ((y >= SaCord[1]) && (y <= SaCord[3]))) // homebutton
						{
					waitForIt(SaCord[0], SaCord[1], SaCord[2], SaCord[3]);
					SaF[FSelect] = !SaF[FSelect];
					UpdateDoseScreenN(FSelect);
				} else if (((x >= SoCord[0]) && (x <= SoCord[2]))
						&& ((y >= SoCord[1]) && (y <= SoCord[3]))) // homebutton
						{
					waitForIt(SoCord[0], SoCord[1], SoCord[2], SoCord[3]);
					SuF[FSelect] = !SuF[FSelect];
					UpdateDoseScreenN(FSelect);
				} else if (((x >= calibrateCord[0]) && (x <= calibrateCord[2]))
						&& ((y >= calibrateCord[1]) && (y <= calibrateCord[3]))) // homebutton
						{
					waitForIt(calibrateCord[0], calibrateCord[1],
							calibrateCord[2], calibrateCord[3]);
					fertmillis = millis();
					switch (FSelect) {
					case 0: {
						/*dPump1Value=false;
						 processRF();
						 wdt_disable();
						 delay(60000);
						 dPump1Value=true;
						 processRF();
						 wdt_enable(WDTO_8S);
						 */
						wdt_disable();
						while (millis() - fertmillis
								< 1000 * FDose[0] / (FRate[0] / 60)) {
							dPump1Value = false;
							processRF();
						}
						dPump1Value = true;
						processRF();

						wdt_enable(WDTO_8S);
						break;

					}
					case 1: {
						wdt_disable();
						while (millis() - fertmillis
								< 1000 * FDose[1] / (FRate[1] / 60)) {
							dPump2Value = false;
							processRF();
						}
						dPump2Value = true;
						processRF();

						wdt_enable(WDTO_8S);
						break;
					}
					case 2: {
						wdt_disable();
						while (millis() - fertmillis
								< 1000 * FDose[2] / (FRate[2] / 60)) {
							dPump3Value = false;
							processRF();
						}
						dPump3Value = true;
						processRF();

						wdt_enable(WDTO_8S);
						break;
					}
					}
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveFerti();
					dispScreen = 9;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readFerti();
					dispScreen = 9;
					drawScreen();
				}
			}
				break;

			case 10:  //  Power Sched
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					readPowerSchedule();
					dispScreen = 0;
					drawScreen();
				} else if (((x >= powLightOnHourUp[0])
						&& (x <= powLightOnHourUp[2]))
						&& ((y >= powLightOnHourUp[1])
								&& (y <= powLightOnHourUp[3]))) // homebutton
						{
					waitForIt(powLightOnHourUp[0], powLightOnHourUp[1],
							powLightOnHourUp[2], powLightOnHourUp[3]);
					if ((powLightOnHour >= 23) && (powLightOnHour <= 250)) {
						powLightOnHour = 0;
					} else {
						powLightOnHour++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powLightOnHourDown[0])
						&& (x <= powLightOnHourDown[2]))
						&& ((y >= powLightOnHourDown[1])
								&& (y <= powLightOnHourDown[3]))) // homebutton
						{
					waitForIt(powLightOnHourDown[0], powLightOnHourDown[1],
							powLightOnHourDown[2], powLightOnHourDown[3]);
					powLightOnHour--;
					if (powLightOnHour >= 230) {
						powLightOnHour = 23;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powLightOnMinuteUp[0])
						&& (x <= powLightOnMinuteUp[2]))
						&& ((y >= powLightOnMinuteUp[1])
								&& (y <= powLightOnMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOnMinuteUp[0], powLightOnMinuteUp[1],
							powLightOnMinuteUp[2], powLightOnMinuteUp[3]);
					if ((powLightOnMinute >= 59) && (powLightOnMinute <= 250)) {
						powLightOnMinute = 0;
					} else {
						powLightOnMinute++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powLightOnMinuteDown[0])
						&& (x <= powLightOnMinuteDown[2]))
						&& ((y >= powLightOnMinuteDown[1])
								&& (y <= powLightOnMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOnMinuteDown[0], powLightOnMinuteDown[1],
							powLightOnMinuteDown[2], powLightOnMinuteDown[3]);
					powLightOnMinute--;
					if (powLightOnMinute >= 230) {
						powLightOnMinute = 59;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powLightOffHourUp[0])
						&& (x <= powLightOffHourUp[2]))
						&& ((y >= powLightOffHourUp[1])
								&& (y <= powLightOffHourUp[3]))) // homebutton
						{
					waitForIt(powLightOffHourUp[0], powLightOffHourUp[1],
							powLightOffHourUp[2], powLightOffHourUp[3]);
					if ((powLightOffHour >= 23) && (powLightOffHour <= 250)) {
						powLightOffHour = 0;
					} else {
						powLightOffHour++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powLightOffHourDown[0])
						&& (x <= powLightOffHourDown[2]))
						&& ((y >= powLightOffHourDown[1])
								&& (y <= powLightOffHourDown[3]))) // homebutton
						{
					waitForIt(powLightOffHourDown[0], powLightOffHourDown[1],
							powLightOffHourDown[2], powLightOffHourDown[3]);
					powLightOffHour--;
					if (powLightOffHour >= 230) {
						powLightOffHour = 23;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);
					if ((powLightOffMinute >= 59)
							&& (powLightOffMinute <= 250)) {
						powLightOffMinute = 0;
					} else {
						powLightOffMinute++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					powLightOffMinute--;
					if (powLightOffMinute >= 230) {
						powLightOffMinute = 59;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OnHourUp[0]) && (x <= powCo2OnHourUp[2]))
						&& ((y >= powCo2OnHourUp[1]) && (y <= powCo2OnHourUp[3]))) // homebutton
						{
					waitForIt(powCo2OnHourUp[0], powCo2OnHourUp[1],
							powCo2OnHourUp[2], powCo2OnHourUp[3]);
					if ((powCo2OnHour >= 23) && (powCo2OnHour <= 250)) {
						powCo2OnHour = 0;
					} else {
						powCo2OnHour++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OnHourDown[0])
						&& (x <= powCo2OnHourDown[2]))
						&& ((y >= powCo2OnHourDown[1])
								&& (y <= powCo2OnHourDown[3]))) // homebutton
						{
					waitForIt(powCo2OnHourDown[0], powCo2OnHourDown[1],
							powCo2OnHourDown[2], powCo2OnHourDown[3]);
					powCo2OnHour--;
					if (powCo2OnHour >= 230) {
						powCo2OnHour = 23;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					if ((powCo2OnMinute >= 59) && (powCo2OnMinute <= 250)) {
						powCo2OnMinute = 0;
					} else {
						powCo2OnMinute++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					powCo2OnMinute--;
					if (powCo2OnMinute >= 230) {
						powCo2OnMinute = 59;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OffHourUp[0])
						&& (x <= powCo2OffHourUp[2]))
						&& ((y >= powCo2OffHourUp[1])
								&& (y <= powCo2OffHourUp[3]))) // homebutton
						{
					waitForIt(powCo2OffHourUp[0], powCo2OffHourUp[1],
							powCo2OffHourUp[2], powCo2OffHourUp[3]);
					if ((powCo2OffHour >= 23) && (powCo2OffHour <= 250)) {
						powCo2OffHour = 0;
					} else {
						powCo2OffHour++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OffHourDown[0])
						&& (x <= powCo2OffHourDown[2]))
						&& ((y >= powCo2OffHourDown[1])
								&& (y <= powCo2OffHourDown[3]))) // homebutton
						{
					waitForIt(powCo2OffHourDown[0], powCo2OffHourDown[1],
							powCo2OffHourDown[2], powCo2OffHourDown[3]);
					powCo2OffHour--;
					if (powCo2OffHour >= 230) {
						powCo2OffHour = 23;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OffMinuteUp[0])
						&& (x <= powCo2OffMinuteUp[2]))
						&& ((y >= powCo2OffMinuteUp[1])
								&& (y <= powCo2OffMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1],
							powCo2OffMinuteUp[2], powCo2OffMinuteUp[3]);
					if ((powCo2OffMinute >= 59) && (powCo2OffMinute <= 250)) {
						powCo2OffMinute = 0;
					} else {
						powCo2OffMinute++;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= powCo2OffMinuteDown[0])
						&& (x <= powCo2OffMinuteDown[2]))
						&& ((y >= powCo2OffMinuteDown[1])
								&& (y <= powCo2OffMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1],
							powCo2OffMinuteDown[2], powCo2OffMinuteDown[3]);
					powCo2OffMinute--;
					if (powCo2OffMinute >= 230) {
						powCo2OffMinute = 59;
					}
					UpdatePowerSchedScreen();
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					savePowerSchedule();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readPowerSchedule();
					dispScreen = 3;
					drawScreen();
				}

				break;

			case 12:  //  Co2Screen Screen
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);
					PHUpperLimit += 0.01;
					updateCO2SetScreen();
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					PHUpperLimit -= 0.01;
					updateCO2SetScreen();
				} else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					PHLowerLimit += 0.01;
					updateCO2SetScreen();
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					PHLowerLimit -= 0.01;
					updateCO2SetScreen();
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					savePHValue();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readPHValue();
					dispScreen = 3;
					drawScreen();
				}
				break;

			case 13: //testscreen

				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					readScreenScreen();
					dispScreen = 0;
					drawScreen();
				} else if (((x >= powLightOnHourUp[0])
						&& (x <= powLightOnHourUp[2]))
						&& ((y >= powLightOnHourUp[1])
								&& (y <= powLightOnHourUp[3]))) // homebutton
						{
					waitForIt(powLightOnHourUp[0], powLightOnHourUp[1],
							powLightOnHourUp[2], powLightOnHourUp[3]);
					if ((screenOnHour >= 23) && (screenOnHour <= 250)) {
						screenOnHour = 0;
					} else {
						screenOnHour++;
					}
					UpdateScreenScreen();
				} else if (((x >= powLightOnHourDown[0])
						&& (x <= powLightOnHourDown[2]))
						&& ((y >= powLightOnHourDown[1])
								&& (y <= powLightOnHourDown[3]))) // homebutton
						{
					waitForIt(powLightOnHourDown[0], powLightOnHourDown[1],
							powLightOnHourDown[2], powLightOnHourDown[3]);
					screenOnHour--;
					if (screenOnHour >= 230) {
						screenOnHour = 23;
					}
					UpdateScreenScreen();
				} else if (((x >= powLightOnMinuteUp[0])
						&& (x <= powLightOnMinuteUp[2]))
						&& ((y >= powLightOnMinuteUp[1])
								&& (y <= powLightOnMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOnMinuteUp[0], powLightOnMinuteUp[1],
							powLightOnMinuteUp[2], powLightOnMinuteUp[3]);
					if ((screenOnMinute >= 59) && (screenOnMinute <= 250)) {
						screenOnMinute = 0;
					} else {
						screenOnMinute++;
					}
					UpdateScreenScreen();
				} else if (((x >= powLightOnMinuteDown[0])
						&& (x <= powLightOnMinuteDown[2]))
						&& ((y >= powLightOnMinuteDown[1])
								&& (y <= powLightOnMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOnMinuteDown[0], powLightOnMinuteDown[1],
							powLightOnMinuteDown[2], powLightOnMinuteDown[3]);
					screenOnMinute--;
					if (screenOnMinute >= 230) {
						screenOnMinute = 59;
					}
					UpdateScreenScreen();
				} else if (((x >= powLightOffHourUp[0])
						&& (x <= powLightOffHourUp[2]))
						&& ((y >= powLightOffHourUp[1])
								&& (y <= powLightOffHourUp[3]))) // homebutton
						{
					waitForIt(powLightOffHourUp[0], powLightOffHourUp[1],
							powLightOffHourUp[2], powLightOffHourUp[3]);
					if ((screenOffHour >= 23) && (screenOffHour <= 250)) {
						screenOffHour = 0;
					} else {
						screenOffHour++;
					}
					UpdateScreenScreen();
				} else if (((x >= powLightOffHourDown[0])
						&& (x <= powLightOffHourDown[2]))
						&& ((y >= powLightOffHourDown[1])
								&& (y <= powLightOffHourDown[3]))) // homebutton
						{
					waitForIt(powLightOffHourDown[0], powLightOffHourDown[1],
							powLightOffHourDown[2], powLightOffHourDown[3]);
					screenOffHour--;
					if (screenOffHour >= 230) {
						screenOffHour = 23;
					}
					UpdateScreenScreen();
				} else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);
					if ((screenOffMinute >= 59) && (screenOffMinute <= 250)) {
						screenOffMinute = 0;
					} else {
						screenOffMinute++;
					}
					UpdateScreenScreen();
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) // homebutton
						{
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					screenOffMinute--;
					if (screenOffMinute >= 230) {
						screenOffMinute = 59;
					}
					UpdateScreenScreen();
				}

				else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					if ((standByMinutes >= 59) && (standByMinutes <= 250)) {
						standByMinutes = 0;
					} else {
						standByMinutes++;
					}
					UpdateScreenScreen();
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					standByMinutes--;
					if (standByMinutes >= 230) {
						standByMinutes = 59;
					}
					UpdateScreenScreen();
				} else if (((x >= powCo2OffMinuteUp[0])
						&& (x <= powCo2OffMinuteUp[2]))
						&& ((y >= powCo2OffMinuteUp[1])
								&& (y <= powCo2OffMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1],
							powCo2OffMinuteUp[2], powCo2OffMinuteUp[3]);
					backlightPWM++;
					analogWrite(backlightPIN, backlightPWM);
					UpdateScreenScreen();
				} else if (((x >= powCo2OffMinuteDown[0])
						&& (x <= powCo2OffMinuteDown[2]))
						&& ((y >= powCo2OffMinuteDown[1])
								&& (y <= powCo2OffMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1],
							powCo2OffMinuteDown[2], powCo2OffMinuteDown[3]);
					backlightPWM--;
					analogWrite(backlightPIN, backlightPWM);
					UpdateScreenScreen();
				} else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveScreenScreen();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readScreenScreen();
					dispScreen = 3;
					drawScreen();
				}

				break;

			case 14:  // RGBLights
				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= LightMode1Cord[0]) && (x <= LightMode1Cord[2]))
						&& ((y >= LightMode1Cord[1]) && (y <= LightMode1Cord[3]))) {
					waitForIt(LightMode1Cord[0], LightMode1Cord[1],
							LightMode1Cord[2], LightMode1Cord[3]);
					dispScreen = 141;
					RGBScreenSet = 0;
					drawScreen();
				} else if (((x >= LightMode2Cord[0]) && (x <= LightMode2Cord[2]))
						&& ((y >= LightMode2Cord[1]) && (y <= LightMode2Cord[3]))) {
					waitForIt(LightMode2Cord[0], LightMode2Cord[1],
							LightMode2Cord[2], LightMode2Cord[3]);
					dispScreen = 142;
					RGBScreenSet = 2;
					drawScreen();
				} else if (((x >= LightMode3Cord[0]) && (x <= LightMode3Cord[2]))
						&& ((y >= LightMode3Cord[1]) && (y <= LightMode3Cord[3]))) {
					waitForIt(LightMode3Cord[0], LightMode3Cord[1],
							LightMode3Cord[2], LightMode3Cord[3]);
					dispScreen = 143;
					RGBScreenSet = 4;
					drawScreen();
				} else if (((x >= LightMode4Cord[0]) && (x <= LightMode4Cord[2]))
						&& ((y >= LightMode4Cord[1]) && (y <= LightMode4Cord[3]))) {
					waitForIt(LightMode4Cord[0], LightMode4Cord[1],
							LightMode4Cord[2], LightMode4Cord[3]);
					dispScreen = 144;
					RGBScreenSet = 6;
					drawScreen();
				} else if (((x >= LightMode5Cord[0]) && (x <= LightMode5Cord[2]))
						&& ((y >= LightMode5Cord[1]) && (y <= LightMode5Cord[3]))) {
					waitForIt(LightMode5Cord[0], LightMode5Cord[1],
							LightMode5Cord[2], LightMode5Cord[3]);
					dispScreen = 145;
					RGBScreenSet = 8;
					drawScreen();
				} else if (((x >= LightMode6Cord[0]) && (x <= LightMode6Cord[2]))
						&& ((y >= LightMode6Cord[1]) && (y <= LightMode6Cord[3]))) {
					waitForIt(LightMode6Cord[0], LightMode6Cord[1],
							LightMode6Cord[2], LightMode6Cord[3]);
					dispScreen = 146;
					RGBScreenSet = 10;
					drawScreen();
				}
				break;

			case 141:  //listen on lightscene1
			case 142:  //listen on lightscene2
			case 143:  //listen on lightscene3
			case 144:  //listen on lightscene4
			case 145:  //listen on lightscene5
			case 146:  //listen on lightscene6

				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				} else if (((x >= powLightOnHourUp[0])
						&& (x <= powLightOnHourUp[2]))
						&& ((y >= powLightOnHourUp[1])
								&& (y <= powLightOnHourUp[3]))) {
					waitForIt(powLightOnHourUp[0], powLightOnHourUp[1],
							powLightOnHourUp[2], powLightOnHourUp[3]);
					if ((lightRGB[RGBScreenSet].Hour >= 23)
							&& (lightRGB[RGBScreenSet].Hour <= 250)) {
						lightRGB[RGBScreenSet].Hour = 0;
					} else {
						lightRGB[RGBScreenSet].Hour++;
					}
					UpdateRGBSceneTOP();
				} else if (((x >= powLightOnHourDown[0])
						&& (x <= powLightOnHourDown[2]))
						&& ((y >= powLightOnHourDown[1])
								&& (y <= powLightOnHourDown[3]))) {
					waitForIt(powLightOnHourDown[0], powLightOnHourDown[1],
							powLightOnHourDown[2], powLightOnHourDown[3]);
					lightRGB[RGBScreenSet].Hour--;
					if (lightRGB[RGBScreenSet].Hour >= 230) {
						lightRGB[RGBScreenSet].Hour = 23;
					}
					UpdateRGBSceneTOP();
				} else if (((x >= powLightOnMinuteUp[0])
						&& (x <= powLightOnMinuteUp[2]))
						&& ((y >= powLightOnMinuteUp[1])
								&& (y <= powLightOnMinuteUp[3]))) {
					waitForIt(powLightOnMinuteUp[0], powLightOnMinuteUp[1],
							powLightOnMinuteUp[2], powLightOnMinuteUp[3]);
					if ((lightRGB[RGBScreenSet].Minute >= 59)
							&& (lightRGB[RGBScreenSet].Minute <= 250)) {
						lightRGB[RGBScreenSet].Minute = 0;
					} else {
						lightRGB[RGBScreenSet].Minute++;
					}
					UpdateRGBSceneTOP();
				} else if (((x >= powLightOnMinuteDown[0])
						&& (x <= powLightOnMinuteDown[2]))
						&& ((y >= powLightOnMinuteDown[1])
								&& (y <= powLightOnMinuteDown[3]))) {
					waitForIt(powLightOnMinuteDown[0], powLightOnMinuteDown[1],
							powLightOnMinuteDown[2], powLightOnMinuteDown[3]);
					lightRGB[RGBScreenSet].Minute--;
					if (lightRGB[RGBScreenSet].Minute >= 230) {
						lightRGB[RGBScreenSet].Minute = 59;
					}
					UpdateRGBSceneTOP();
				}

				else if (((x >= red1Up[0]) && (x <= red1Up[2]))
						&& ((y >= red1Up[1]) && (y <= red1Up[3]))) {
					waitForIt(red1Up[0], red1Up[1], red1Up[2], red1Up[3]);
					lightRGB[RGBScreenSet].red += 1;
					UpdateRGBSceneTOP();
					analogWrite(redPin, lightRGB[RGBScreenSet].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet].blue);

				} else if (((x >= red1Down[0]) && (x <= red1Down[2]))
						&& ((y >= red1Down[1]) && (y <= red1Down[3]))) {
					waitForIt(red1Down[0], red1Down[1], red1Down[2],
							red1Down[3]);
					lightRGB[RGBScreenSet].red -= 1;
					analogWrite(redPin, lightRGB[RGBScreenSet].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet].blue);
					UpdateRGBSceneTOP();
				}

				else if (((x >= powLightOffHourUp[0])
						&& (x <= powLightOffHourUp[2]))
						&& ((y >= powLightOffHourUp[1])
								&& (y <= powLightOffHourUp[3]))) {
					waitForIt(powLightOffHourUp[0], powLightOffHourUp[1],
							powLightOffHourUp[2], powLightOffHourUp[3]);
					lightRGB[RGBScreenSet].green += 1;
					analogWrite(redPin, lightRGB[RGBScreenSet].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet].blue);
					UpdateRGBSceneTOP();
				} else if (((x >= powLightOffHourDown[0])
						&& (x <= powLightOffHourDown[2]))
						&& ((y >= powLightOffHourDown[1])
								&& (y <= powLightOffHourDown[3]))) {
					waitForIt(powLightOffHourDown[0], powLightOffHourDown[1],
							powLightOffHourDown[2], powLightOffHourDown[3]);
					lightRGB[RGBScreenSet].green -= 1;
					analogWrite(redPin, lightRGB[RGBScreenSet].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet].blue);
					UpdateRGBSceneTOP();
				}

				else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) {
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);
					lightRGB[RGBScreenSet].blue += 1;
					analogWrite(redPin, lightRGB[RGBScreenSet].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet].blue);
					UpdateRGBSceneTOP();
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) {
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					lightRGB[RGBScreenSet].blue -= 1;
					analogWrite(redPin, lightRGB[RGBScreenSet].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet].blue);
					UpdateRGBSceneTOP();
				}

				else if (((x >= powCo2OnHourUp[0]) && (x <= powCo2OnHourUp[2]))
						&& ((y >= powCo2OnHourUp[1]) && (y <= powCo2OnHourUp[3]))) // homebutton
						{
					waitForIt(powCo2OnHourUp[0], powCo2OnHourUp[1],
							powCo2OnHourUp[2], powCo2OnHourUp[3]);
					if ((lightRGB[RGBScreenSet + 1].Hour >= 23)
							&& (lightRGB[RGBScreenSet + 1].Hour <= 250)) {
						lightRGB[RGBScreenSet + 1].Hour = 0;
					} else {
						lightRGB[RGBScreenSet + 1].Hour++;
					}
					UpdateRGBSceneBOT();
				} else if (((x >= powCo2OnHourDown[0])
						&& (x <= powCo2OnHourDown[2]))
						&& ((y >= powCo2OnHourDown[1])
								&& (y <= powCo2OnHourDown[3]))) // homebutton
						{
					waitForIt(powCo2OnHourDown[0], powCo2OnHourDown[1],
							powCo2OnHourDown[2], powCo2OnHourDown[3]);
					lightRGB[RGBScreenSet + 1].Hour--;
					if (lightRGB[RGBScreenSet + 1].Hour >= 230) {
						lightRGB[RGBScreenSet + 1].Hour = 23;
					}
					UpdateRGBSceneBOT();
				} else if (((x >= powCo2OnMinuteUp[0])
						&& (x <= powCo2OnMinuteUp[2]))
						&& ((y >= powCo2OnMinuteUp[1])
								&& (y <= powCo2OnMinuteUp[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1],
							powCo2OnMinuteUp[2], powCo2OnMinuteUp[3]);
					if ((lightRGB[RGBScreenSet + 1].Minute >= 59)
							&& (lightRGB[RGBScreenSet + 1].Minute <= 250)) {
						lightRGB[RGBScreenSet + 1].Minute = 0;
					} else {
						lightRGB[RGBScreenSet + 1].Minute++;
					}
					UpdateRGBSceneBOT();
				} else if (((x >= powCo2OnMinuteDown[0])
						&& (x <= powCo2OnMinuteDown[2]))
						&& ((y >= powCo2OnMinuteDown[1])
								&& (y <= powCo2OnMinuteDown[3]))) // homebutton
						{
					waitForIt(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1],
							powCo2OnMinuteDown[2], powCo2OnMinuteDown[3]);
					lightRGB[RGBScreenSet + 1].Minute--;
					if (lightRGB[RGBScreenSet + 1].Minute >= 230) {
						lightRGB[RGBScreenSet + 1].Minute = 59;
					}
					UpdateRGBSceneBOT();
				}

				else if (((x >= red2Up[0]) && (x <= red2Up[2]))
						&& ((y >= red2Up[1]) && (y <= red2Up[3]))) {
					waitForIt(red2Up[0], red2Up[1], red2Up[2], red2Up[3]);
					lightRGB[RGBScreenSet + 1].red += 1;
					analogWrite(redPin, lightRGB[RGBScreenSet + 1].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet + 1].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet + 1].blue);
					UpdateRGBSceneBOT();
				} else if (((x >= red2Down[0]) && (x <= red2Down[2]))
						&& ((y >= red2Down[1]) && (y <= red2Down[3]))) {
					waitForIt(red2Down[0], red2Down[1], red2Down[2],
							red2Down[3]);
					lightRGB[RGBScreenSet + 1].red -= 1;
					analogWrite(redPin, lightRGB[RGBScreenSet + 1].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet + 1].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet + 1].blue);
					UpdateRGBSceneBOT();
				}

				else if (((x >= powCo2OffHourUp[0]) && (x <= powCo2OffHourUp[2]))
						&& ((y >= powCo2OffHourUp[1])
								&& (y <= powCo2OffHourUp[3]))) {
					waitForIt(powCo2OffHourUp[0], powCo2OffHourUp[1],
							powCo2OffHourUp[2], powCo2OffHourUp[3]);
					lightRGB[RGBScreenSet + 1].green += 1;
					analogWrite(redPin, lightRGB[RGBScreenSet + 1].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet + 1].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet + 1].blue);
					UpdateRGBSceneBOT();
				} else if (((x >= powCo2OffHourDown[0])
						&& (x <= powCo2OffHourDown[2]))
						&& ((y >= powCo2OffHourDown[1])
								&& (y <= powCo2OffHourDown[3]))) {
					waitForIt(powCo2OffHourDown[0], powCo2OffHourDown[1],
							powCo2OffHourDown[2], powCo2OffHourDown[3]);
					lightRGB[RGBScreenSet + 1].green -= 1;
					analogWrite(redPin, lightRGB[RGBScreenSet + 1].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet + 1].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet + 1].blue);
					UpdateRGBSceneBOT();
				}

				else if (((x >= powCo2OffMinuteUp[0])
						&& (x <= powCo2OffMinuteUp[2]))
						&& ((y >= powCo2OffMinuteUp[1])
								&& (y <= powCo2OffMinuteUp[3]))) {
					waitForIt(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1],
							powCo2OffMinuteUp[2], powCo2OffMinuteUp[3]);
					lightRGB[RGBScreenSet + 1].blue += 1;
					analogWrite(redPin, lightRGB[RGBScreenSet + 1].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet + 1].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet + 1].blue);
					UpdateRGBSceneBOT();
				} else if (((x >= powCo2OffMinuteDown[0])
						&& (x <= powCo2OffMinuteDown[2]))
						&& ((y >= powCo2OffMinuteDown[1])
								&& (y <= powCo2OffMinuteDown[3]))) {
					waitForIt(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1],
							powCo2OffMinuteDown[2], powCo2OffMinuteDown[3]);
					lightRGB[RGBScreenSet + 1].blue -= 1;
					analogWrite(redPin, lightRGB[RGBScreenSet + 1].red);
					analogWrite(greenPin, lightRGB[RGBScreenSet + 1].green);
					analogWrite(bluePin, lightRGB[RGBScreenSet + 1].blue);
					UpdateRGBSceneBOT();
				}

				else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveLightRGB();
					dispScreen = 14;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readLightRGB();
					dispScreen = 14;
					drawScreen();
				}
				break;

			case 16: //moonmode

				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				}

				else if (((x >= powLightOnMinuteUp[0])
						&& (x <= powLightOnMinuteUp[2]))
						&& ((y >= powLightOnMinuteUp[1])
								&& (y <= powLightOnMinuteUp[3]))) {
					waitForIt(powLightOnMinuteUp[0], powLightOnMinuteUp[1],
							powLightOnMinuteUp[2], powLightOnMinuteUp[3]);
					MoonMinutes++;
					UpdateMoonScreen();
				} else if (((x >= powLightOnMinuteDown[0])
						&& (x <= powLightOnMinuteDown[2]))
						&& ((y >= powLightOnMinuteDown[1])
								&& (y <= powLightOnMinuteDown[3]))) {
					waitForIt(powLightOnMinuteDown[0], powLightOnMinuteDown[1],
							powLightOnMinuteDown[2], powLightOnMinuteDown[3]);
					MoonMinutes--;
					UpdateMoonScreen();
				}

				else if (((x >= red1Up[0]) && (x <= red1Up[2]))
						&& ((y >= red1Up[1]) && (y <= red1Up[3]))) {
					waitForIt(red1Up[0], red1Up[1], red1Up[2], red1Up[3]);
					MoonRed += 1;
					UpdateMoonScreen();
					analogWrite(redPin, MoonRed);
					analogWrite(greenPin, MoonGreen);
					analogWrite(bluePin, MoonBlue);

				} else if (((x >= red1Down[0]) && (x <= red1Down[2]))
						&& ((y >= red1Down[1]) && (y <= red1Down[3]))) {
					waitForIt(red1Down[0], red1Down[1], red1Down[2],
							red1Down[3]);
					MoonRed -= 1;
					analogWrite(redPin, MoonRed);
					analogWrite(greenPin, MoonGreen);
					analogWrite(bluePin, MoonBlue);
					UpdateMoonScreen();
				}

				else if (((x >= powLightOffHourUp[0])
						&& (x <= powLightOffHourUp[2]))
						&& ((y >= powLightOffHourUp[1])
								&& (y <= powLightOffHourUp[3]))) {
					waitForIt(powLightOffHourUp[0], powLightOffHourUp[1],
							powLightOffHourUp[2], powLightOffHourUp[3]);
					MoonGreen += 1;
					analogWrite(redPin, MoonRed);
					analogWrite(greenPin, MoonGreen);
					analogWrite(bluePin, MoonBlue);
					UpdateMoonScreen();
				} else if (((x >= powLightOffHourDown[0])
						&& (x <= powLightOffHourDown[2]))
						&& ((y >= powLightOffHourDown[1])
								&& (y <= powLightOffHourDown[3]))) {
					waitForIt(powLightOffHourDown[0], powLightOffHourDown[1],
							powLightOffHourDown[2], powLightOffHourDown[3]);
					MoonGreen -= 1;
					analogWrite(redPin, MoonRed);
					analogWrite(greenPin, MoonGreen);
					analogWrite(bluePin, MoonBlue);
					UpdateMoonScreen();
				}

				else if (((x >= powLightOffMinuteUp[0])
						&& (x <= powLightOffMinuteUp[2]))
						&& ((y >= powLightOffMinuteUp[1])
								&& (y <= powLightOffMinuteUp[3]))) {
					waitForIt(powLightOffMinuteUp[0], powLightOffMinuteUp[1],
							powLightOffMinuteUp[2], powLightOffMinuteUp[3]);
					MoonBlue += 1;
					analogWrite(redPin, MoonRed);
					analogWrite(greenPin, MoonGreen);
					analogWrite(bluePin, MoonBlue);
					UpdateMoonScreen();
				} else if (((x >= powLightOffMinuteDown[0])
						&& (x <= powLightOffMinuteDown[2]))
						&& ((y >= powLightOffMinuteDown[1])
								&& (y <= powLightOffMinuteDown[3]))) {
					waitForIt(powLightOffMinuteDown[0],
							powLightOffMinuteDown[1], powLightOffMinuteDown[2],
							powLightOffMinuteDown[3]);
					MoonBlue -= 1;
					analogWrite(redPin, MoonRed);
					analogWrite(greenPin, MoonGreen);
					analogWrite(bluePin, MoonBlue);
					UpdateMoonScreen();
				}

				else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveMoonMode();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readMoonMode();
					dispScreen = 3;
					drawScreen();
				}
				break;

			case 15: //TVMode

				if (((x >= BottomButtonCoord[0]) && (x <= BottomButtonCoord[2]))
						&& ((y >= BottomButtonCoord[1])
								&& (y <= BottomButtonCoord[3]))) // homebutton
						{
					waitForIt(BottomButtonCoord[0], BottomButtonCoord[1],
							BottomButtonCoord[2], BottomButtonCoord[3]);
					dispScreen = 0;
					drawScreen();
				}

				else if (((x >= powLightOnMinuteUp[0])
						&& (x <= powLightOnMinuteUp[2]))
						&& ((y >= powLightOnMinuteUp[1])
								&& (y <= powLightOnMinuteUp[3]))) {
					waitForIt(powLightOnMinuteUp[0], powLightOnMinuteUp[1],
							powLightOnMinuteUp[2], powLightOnMinuteUp[3]);
					TVModeBrightness += 2.55;
					if (TVModeBrightness > 255) {
						TVModeBrightness = 0;
					}
					if (TVModeBrightness < 0) {
						TVModeBrightness = 255;
					}
					UpdateTVScreen();
					analogWrite(lightPwmPin, TVModeBrightness);
				} else if (((x >= powLightOnMinuteDown[0])
						&& (x <= powLightOnMinuteDown[2]))
						&& ((y >= powLightOnMinuteDown[1])
								&& (y <= powLightOnMinuteDown[3]))) {
					waitForIt(powLightOnMinuteDown[0], powLightOnMinuteDown[1],
							powLightOnMinuteDown[2], powLightOnMinuteDown[3]);
					TVModeBrightness -= 2.55;
					if (TVModeBrightness > 255) {
						TVModeBrightness = 0;
					}
					if (TVModeBrightness < 0) {
						TVModeBrightness = 255;
					}
					UpdateTVScreen();
					analogWrite(lightPwmPin, TVModeBrightness);
				}

				else if (((x >= SetPowerSchedCord[0])
						&& (x <= SetPowerSchedCord[2]))
						&& ((y >= SetPowerSchedCord[1])
								&& (y <= SetPowerSchedCord[3]))) // homebutton
						{
					waitForIt(SetPowerSchedCord[0], SetPowerSchedCord[1],
							SetPowerSchedCord[2], SetPowerSchedCord[3]);
					saveTVMode();
					dispScreen = 3;
					drawScreen();
				} else if (((x >= CancelPowerSchedCord[0])
						&& (x <= CancelPowerSchedCord[2]))
						&& ((y >= CancelPowerSchedCord[1])
								&& (y <= CancelPowerSchedCord[3]))) // homebutton
						{
					waitForIt(CancelPowerSchedCord[0], CancelPowerSchedCord[1],
							CancelPowerSchedCord[2], CancelPowerSchedCord[3]);
					readTVMode();
					dispScreen = 3;
					drawScreen();
				}
				break;

			}
		}
	}
	/*sendCommand("Temp",String(Temp));
	 sendCommand("PhWert",String(PhWert));
	 sendCommand("heaterValue",String(heaterValue));
	 */

	if (!myTouch.dataAvailable()) {
		beeper();
		// Serial.println("beep");
		recvWithStartEndMarkers();
		//  Serial.println("rec");
		useNewData();
		//   Serial.println("usenewdata");

		if (currentMillis - prevMillis1sec > 2000) //every 2 seconds update our data
				{
			prevMillis1sec = millis();
			//Serial.println(millis());
			if (cleaningInProcess) {
				getDistance();
			}

		}

		if (currentMillis - prevMillis5sec > 5000) //if 5 seconds are over update our data
				{
			prevMillis5sec = millis();
			getPHValue();
			lightCalculator();
			UpdateClockAndLight();
			//writeToRingBuffer();  //only for debug

			if (changeRF) {
				processRF();
			}
		}

		if (currentMillis - prevMillis1min > 60000) //every 60 seconds update our data
				{
			prevMillis1min = millis();
			getDistance();
			GetTemperature();
			if (dispScreen < 1) {
				drawPhPwmNotifications();
			}
			printDate(now, 5, 5);
			if (currentMillis - prevMillisTouch < (standByMinutes * 60000)) //wenn die Letzte berÃ¼hrung kleiner als die eingestellten StandbyMinutes
					{
				analogWrite(backlightPIN, 255);  //display auf volle Helligkeit
			} else //wenn keine BerÃ¼hrung - AI
			{
				AI();
				fertilize();
				DateTime CompareScreenOnTime(now.year(), now.month(), now.day(),
						int(screenOnHour), int(screenOnMinute), 0);
				DateTime CompareScreenOffTime(now.year(), now.month(),
						now.day(), int(screenOffHour), int(screenOffMinute), 0);
				if (now.unixtime() < CompareScreenOnTime.unixtime()
						|| now.unixtime() > CompareScreenOffTime.unixtime()) {
					analogWrite(backlightPIN, 0);
				} else {
					analogWrite(backlightPIN, backlightPWM);
				}
			}

		}
		if (currentMillis - prevMillis15min > 900000) //every 15 minutes write to file
				{
			prevMillis15min = millis();
			writeFile();
			writeToRingBuffer();
			processPH();
			drawScreen();
		}

		if (mySwitch.available()) // get RF Data
		{
			processRFInput();
			mySwitch.resetAvailable();
		}
	}
}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// SCREENS
// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

void drawScreen() {
	if (dispScreen != lastScreen) {
		myGLCD.clrScr(); //clear everything on screenchange
	}
	lastScreen = dispScreen;
	//selection which screen to draw
	printDate(now, 5, 5);
	switch (dispScreen) {
	case 0:  // home screen
		//     stopMillis=millis();
		HomeScreen();
		drawPhPwmNotifications();
		drawFertilizer();
		printDate(now, 5, 5);
		//     stopMillis=millis()-stopMillis;
		//     myGLCD.printNumI(stopMillis, 40, 550);

		break;
	case 1:  // feed screen
		FeedScreen();
		break;
	case 2:  // power screen
		PowerScreen();
		break;
	case 3:  // settings screen
		SettingsScreen();
		break;
	case 4:  // lights screen
		LightsScreen();
		break;
	case 41:
	case 42:
	case 43:
	case 44:
	case 45:
	case 46:
		LightScene();
		break;
	case 5:  // clock screen
		ClockScreen();
		break;
	case 6:  // CleanSchedScreen
		CleanSchedScreen();
		updateCleanSchedScreen();
		quickUpdateCleanSchedScreen();
		break;
	case 7:  // RemindScreen
		RemindScreen();
		break;
	case 8:  // HeaterScreen
		HeaterScreen();
		updateHeaterScreen();
		break;
	case 9:  // DoseScreen
		DoseScreen();
		updateDoseScreen();
		break;
	case 91:  // DoseScreen
		DoseScreenN();
		UpdateDoseScreenN(FSelect);
		quickUpdateDoseScreenN(FSelect);
		break;

	case 10:  // DoseScreen
		PowerSchedScreen();
		break;
	case 11:  // DoseScreen
		ScreenScreen();
		break;
	case 12:  // DoseScreen
		Co2SetScreen();
		updateCO2SetScreen();
		break;
	case 13:
		ScreenScreen();
		break;
	case 14:
		RGBScreen();
		break;
	case 141:
	case 142:
	case 143:
	case 144:
	case 145:
	case 146:
		RGBScene();
		break;
	case 15:
		TVScreen();
		break;
	case 16:
		MoonScreen();
		break;
	}
}

/*  DRAW ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 draw Methoden zur Darstellung auf dem Touchscreen
 screens are listed below
 0-home, 1-cleaning, , 2-power, 3-extras, 4-lights
 5-clock, 6-feeding sched, 7-schedule, 8-heater
 9-dosing, 10-pwer schedule, 11-schedule item, 13=TestScreen
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

void printCoords() //for troubleshooting
{
	myGLCD.setFont(BigFont);
	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(x, RIGHT, 20, 3, 48);
	myGLCD.printNumI(y, RIGHT, 40, 3, 48);
	myGLCD.drawPixel(x, y);
}

//draw write Date to an char Array and draw Time and Date
void printDate(DateTime printnow, int tx, int ty) {
	char chDate[25], tmpChar[5];
	strcat(chDate, "     ");
	chDate[0] = '\0';
	strcat(chDate, dayName[printnow.dayOfWeek()]);
	strcat(chDate, ",");
	itoa(printnow.day(), tmpChar, 10);
	strcat(chDate, tmpChar);
	strcat(chDate, ".");
	strcat(chDate, monthName[printnow.month()]);
	strcat(chDate, "   ");
	myGLCD.setFont(BigFont);
	myGLCD.setColor(255, 255, 255);
	myGLCD.print(chDate, tx, ty);            //Display date
	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(printnow.hour(), 400, ty, 2, 48);
	myGLCD.print(F(":"), 430, ty);
	myGLCD.printNumI(printnow.minute(), 445, ty, 2, 48);
	//printCoords();             //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~for TSHOOT
	myGLCD.setColor(0, 0, 255);
	// myGLCD.fillRoundRect (Button1Cord[0], Button1Cord[1], Button1Cord[2], Button1Cord[3]);
	myGLCD.drawLine(Button1Cord[0], Button1Cord[1], Button1Cord[2],
			Button1Cord[1]);
	myGLCD.drawLine(Button1Cord[2], Button1Cord[1], Button1Cord[2],
			Button1Cord[3]);  // -
	myGLCD.drawLine(Button1Cord[0], Button1Cord[1], Button1Cord[0],
			Button1Cord[3]); // links senkrecht
	myGLCD.drawLine(Button1Cord[0], Button1Cord[3], Button1Cord[2],
			Button1Cord[3]);   //

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 0. draw Homescreen
void HomeScreen() { //Statusline TOP
	myGLCD.clrScr();
	drawCurve();
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(36, 0, 255);
	myGLCD.print(F("HOME       "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60home.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP
	myGLCD.drawLine(30, 270, 450, 270); //2nd line
	//icons - switchin with state

	updateHomeScreen();

	myGLCD.setColor(255, 255, 255);
	// lightmode
	myFiles.load(LightUp[0], LightUp[1], 48, 48, "48up.raw");
	myFiles.load(LightDown[0], LightDown[1], 48, 48, "48down.raw");
	//myFiles.load(90, 85, 120, 102, "1therm.raw");
	//draw the fertilizer
	myGLCD.drawLine(30, 570, 450, 570); //3rd line
	myFiles.load(21, 615, 438, 96, "DockNew.raw");
	myGLCD.setFont(BigFont);
	myGLCD.print(F("Home"), 37, 720);
	myGLCD.print(F("Feed"), 149, 720);
	myGLCD.print(F("Power"), 256, 720);
	myGLCD.print(F("Setting"), 357, 720);

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "ResetF.raw");

}

void drawCurve() {
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(340, 123, 450, 123);  //midline
	myGLCD.drawLine(407, 120, 407, 126);  //scaleline
	myGLCD.drawLine(364, 120, 364, 126); //scaleline
	myGLCD.setColor(60, 60, 60);
	myGLCD.drawLine(340, 83, 450, 83); //grey limitlines
	myGLCD.drawLine(340, 164, 450, 164);  //grey limitlines

	myGLCD.setColor(0, 0, 255);
	for (int i = 0; i < 62; i++) {
		int firstIndex = (i + put_TempIndex) % 62;
		int secondIndex = (i + put_TempIndex + 1) % 62;
		if (highestTemp < TempValues[secondIndex]) {
			highestTemp = TempValues[secondIndex];
		} else if (lowestTemp > TempValues[secondIndex]) {
			lowestTemp = TempValues[secondIndex];
		}

		//lower Border = PHLower Limit   | upper Border = PhLowerLimit

		myGLCD.drawLine(340 + 1.77 * i,
				123
						+ ((((highestTemp + lowestTemp) / 2)
								- TempValues[firstIndex])
								/ ((highestTemp - lowestTemp) / 2) * 40),
				340 + 1.77 * (i + 1),
				123
						+ (((highestTemp + lowestTemp) / 2)
								- TempValues[secondIndex])
								/ ((highestTemp - lowestTemp) / 2) * 40);
	}

	//PH~~~~~~~~~~~~
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(280, 222, 450, 222);  //midline
	myGLCD.drawLine(407, 219, 407, 225);  //scalline
	myGLCD.drawLine(364, 219, 364, 225); //scalline
	myGLCD.drawLine(321, 219, 321, 225); //scalline

	myGLCD.setColor(80, 70, 80);
	myGLCD.setColor(60, 60, 60);
	myGLCD.drawLine(280, 182, 450, 182);  //grey limitlines
	myGLCD.drawLine(280, 262, 450, 262); //grey limitlines

	highestPH = PHUpperLimit;
	lowestPH = PHLowerLimit;
	for (int i = 0; i < 95; i++) {
		int firstIndex = (i + put_PHindex) % 96;
		int secondIndex = (i + put_PHindex + 1) % 96;
		/* Serial.print(i);
		 Serial.print("   ");
		 Serial.print(highestPH);

		 Serial.print(" < ");
		 Serial.print(PHValues[secondIndex]);
		 Serial.print("   ");
		 */
		if (highestPH < PHValues[secondIndex]) {
			highestPH = PHValues[secondIndex];
			// Serial.println("new HIGH");
		} else if (lowestPH > PHValues[secondIndex]) {
			lowestPH = PHValues[secondIndex];
			//  Serial.println("new LOW");
		} else
			Serial.println();
		if (Co2Values[firstIndex]) {
			myGLCD.setColor(255, 0, 0);
		} else {
			myGLCD.setColor(0, 255, 0);
		}
		//lower Border = PHLower Limit   | upper Border = PhLowerLimit
		myGLCD.drawLine(280 + 1.77 * i,
				222
						+ ((((highestPH + lowestPH) / 2) - PHValues[firstIndex])
								/ ((highestPH - lowestPH) / 2) * 40),
				280 + 1.77 * (i + 1),
				222
						+ (((highestPH + lowestPH) / 2) - PHValues[secondIndex])
								/ ((highestPH - lowestPH) / 2) * 40);
	}

	myGLCD.setColor(255, 255, 255);
	myGLCD.setFont(BigFont);
	myGLCD.printNumF(highestPH, 2, 285, 185);
	myGLCD.printNumF(lowestPH, 2, 285, 244);
	myGLCD.printNumF(highestTemp, 2, 340, 89);
	myGLCD.printNumF(lowestTemp, 2, 340, 148);

}

//PHUpperLimit+0.1-PHLowerLimit-0.1
// scale = PHUpperLimit +0.1 - PHLowerLimit - 0.1

void updateHomeScreen() {
	wdt_reset();
	if (dispScreen < 1) {
		if (!pump1Value) {
			myFiles.load(340, 290, 48, 48, "3filt_N.raw");
		} else {
			myFiles.load(340, 290, 48, 48, "3filt_F.raw");
		}
		if (!pump2Value) {
			myFiles.load(393, 290, 48, 48, "3filt_N.raw");
		} else {
			myFiles.load(393, 290, 48, 48, "3filt_F.raw");
		}
		if (!light230Value) {
			myFiles.load(340, 343, 48, 48, "3light_N.raw");
		} else {
			myFiles.load(340, 343, 48, 48, "3light_F.raw");
		}
		if (!light2Value) {
			myFiles.load(393, 343, 48, 48, "3light_N.raw");
		} else {
			myFiles.load(393, 343, 48, 48, "3light_F.raw");
		}
		if (!co2Value) {
			myFiles.load(340, 396, 48, 48, "3co2_N.raw");
		} else {
			myFiles.load(340, 396, 48, 48, "3co2_F.raw");
		}
		if (!heaterValue) {
			myFiles.load(393, 396, 48, 48, "3heat_N.raw");
		} else {
			myFiles.load(393, 396, 48, 48, "3heat_F.raw");
		}
		if (!coolValue) {
			myFiles.load(340, 449, 48, 48, "3circ_N.raw");
		} else {
			myFiles.load(340, 449, 48, 48, "3circ_F.raw");
		}
		if (!dPump1Value) {
			myFiles.load(393, 449, 48, 48, "1nN.raw");
		} else {
			myFiles.load(393, 449, 48, 48, "1nF.raw");
		}
		if (!dPump2Value) {
			myFiles.load(340, 502, 48, 48, "1npkN.raw");
		} else {
			myFiles.load(340, 502, 48, 48, "1npkF.raw");
		}
		if (!dPump3Value) {
			myFiles.load(393, 502, 48, 48, "1feN.raw");
		} else {
			myFiles.load(393, 502, 48, 48, "1feF.raw");
		}
		if (Temp >= TempUpperLimit) {
			myFiles.load(26, 85, 105, 90, "1thermR.raw");
		} else {
			myFiles.load(26, 85, 105, 90, "1therm.raw");
		}
	}
}

void drawFertilizer() //methode zur Berechnung der DÃ¼ngermenge sowie Darstellung der DÃ¼ngerreserve
{
	if (dispScreen < 1) {
		myGLCD.setFont(BigFont);
		myFiles.load(166, 370, 46, 130, "1ferts.raw");
		myGLCD.setColor(col_FertiN.r, col_FertiN.g, col_FertiN.b);
		myGLCD.fillRect(178,
				(484
						- ((100 / (FMax[0] / FDose[0]))
								* ((FLeft[0] / (FDose[0]))))), 199, 484);
		myGLCD.printNumI((FLeft[0] / FDose[0]), 164, 348, 2);
		myGLCD.print("N", 180, 330, 2);

		myFiles.load(217, 370, 46, 130, "1ferts.raw");
		myGLCD.setColor(col_FertiNPK.r, col_FertiNPK.g, col_FertiNPK.b);
		myGLCD.fillRect(229,
				(484
						- ((100 / (FMax[1] / FDose[1]))
								* ((FLeft[1] / (FDose[1]))))), 250, 484);
		myGLCD.printNumI((FLeft[1] / FDose[1]), 217, 348, 2);
		myGLCD.print("NPK", 218, 330, 2);

		myFiles.load(268, 370, 46, 130, "1ferts.raw");
		myGLCD.setColor(col_FertiFE.r, col_FertiFE.g, col_FertiFE.b);
		myGLCD.fillRect(280,
				(484
						- ((100 / (FMax[2] / FDose[2]))
								* ((FLeft[2] / (FDose[2]))))), 301, 484);
		myGLCD.printNumI((FLeft[2] / FDose[2]), 267, 348, 2);
		myGLCD.print("FE", 275, 330, 2);

	}
}

void drawPhPwmNotifications() {
	if (Temp >= TempUpperLimit) {
		myFiles.load(26, 85, 105, 90, "1thermR.raw");
	}

	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.setFont(SevenSegmentFull);
	myGLCD.print("o", 300, 80);              // Degree icon

	myGLCD.printNumF(Temp, 2, 135, 95); //245
	myGLCD.setColor(0, 255, 0);

	myGLCD.print(F("PH:  "), 40, 210);
	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumF(PhWert, 2, 135, 210);

	if (cleaningInProcess) {
		myGLCD.setFont(BigFont);
		myGLCD.setColor(col_red.r, col_red.g, col_red.b);
		myGLCD.print(F("CLEANING IN PROGRESS"), 5, 545);
	} else if (manualOverride) {
		myGLCD.setFont(BigFont);
		myGLCD.setColor(col_red.r, col_red.g, col_red.b);
		myGLCD.print(F("MANUAL OVERRIDE     "), 5, 545);
	} else if (TVModeState) {
		myGLCD.setFont(BigFont);
		myGLCD.setColor(col_red.r, col_red.g, col_red.b);
		myGLCD.print(F("TV MODE active         "), 5, 545);
	} else if (MoonModeState) {
		myGLCD.setFont(BigFont);
		myGLCD.setColor(col_red.r, col_red.g, col_red.b);
		myGLCD.print(F("Moonligt ends @"), 5, 545);
		myGLCD.setColor(col_white.r, col_white.g, col_white.b);
		myGLCD.printNumI(MoonEnd.hour(), 310 - 55, 545, 2, 48);
		myGLCD.print(F(":"), 360 - 70, 545);
		myGLCD.printNumI(MoonEnd.minute(), 380 - 75, 545, 2, 48);

	} else {
		myGLCD.print(F("         "), 5, 500);
	}

	drawPWM();

}

void drawPWM() {
	myGLCD.setFont(UbuntuBold);  //every SetFont Call takes 100ms!
	myGLCD.setColor(255, int(calculatedPWM), 0);

	/**debugging
	 myGLCD.printNumI(calculatedPWM, 27,392,3);  //debugging
	 myGLCD.printNumI(newPWM, 27,440,3);
	 myGLCD.printNumI(oldPWM, 27,468,3);
	 myGLCD.printNumI(timeToNextLight.totalseconds(), 27,500,5);
	 myGLCD.printNumI(timeSinceLastLight.totalseconds(), 27,530,5);
	 // myGLCD.printNumI(helpSpan.totalseconds(), 27,560,3);
	 */

	if (calculatedPWM >= 25.5) {
		// myGLCD.printNumI(int(100-calculatedPWM*100/180+75/1.8), 23,392,3);
		myGLCD.printNumI(int(calculatedPWM * 100 / 255), 23, 392, 3); //live

	} else {
		myGLCD.printNumI(int(calculatedPWM * 100 / 255), 27, 392, 3);
	}
	myGLCD.print(F("%"), 108, 392, 3);

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//1. FeedScreen
void FeedScreen() { //Statusline TOP
	wdt_reset();
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 219, 0);
	myGLCD.print(F("FEEDING     "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Clea.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP
	myGLCD.print(F("FEEDING"), 20, 136);
	myGLCD.print(F("in progress..."), 135, 180);
	myGLCD.print(F("RESET"), 110, 635);
	myFiles.load(20, 608, 74, 74, "74Reset.raw");

	if (!pump1Value) {
		myFiles.load(340, 290, 48, 48, "3filt_N.raw");
	} else {
		myFiles.load(340, 290, 48, 48, "3filt_F.raw");
	}
	if (!pump2Value) {
		myFiles.load(393, 290, 48, 48, "3filt_N.raw");
	} else {
		myFiles.load(393, 290, 48, 48, "3filt_F.raw");
	}
	if (!light230Value) {
		myFiles.load(340, 343, 48, 48, "3light_N.raw");
	} else {
		myFiles.load(340, 343, 48, 48, "3light_F.raw");
	}
	if (!light2Value) {
		myFiles.load(393, 343, 48, 48, "3light_N.raw");
	} else {
		myFiles.load(393, 343, 48, 48, "3light_F.raw");
	}
	if (!co2Value) {
		myFiles.load(340, 396, 48, 48, "3co2_N.raw");
	} else {
		myFiles.load(340, 396, 48, 48, "3co2_F.raw");
	}
	if (!heaterValue) {
		myFiles.load(393, 396, 48, 48, "3heat_N.raw");
	} else {
		myFiles.load(393, 396, 48, 48, "3heat_F.raw");
	}
	wdt_reset();
	if (!dPump1Value) {
		myFiles.load(340, 449, 48, 48, "1nN.raw");
	} else {
		myFiles.load(340, 449, 48, 48, "1nF.raw");
	}
	if (!dPump2Value) {
		myFiles.load(393, 449, 48, 48, "1npkN.raw");
	} else {
		myFiles.load(393, 449, 48, 48, "1npkF.raw");
	}
	if (!dPump3Value) {
		myFiles.load(340, 502, 48, 48, "1feN.raw");
	} else {
		myFiles.load(340, 502, 48, 48, "1feF.raw");
	}
	if (!coolValue) {
		myFiles.load(393, 502, 48, 48, "3circ_N.raw");
	} else {
		myFiles.load(393, 502, 48, 48, "3circ_F.raw");
	}

	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myFiles.load(80, 310, 184, 184, "2feeding.raw");
	myGLCD.setFont(BigFont);

	/*  myGLCD.print("Feed mode ends at:", 20, 520);
	 myGLCD.setFont(UbuntuBold);
	 myGLCD.printNumI(cleanEnd.hour(), 110,564,2,48);
	 myGLCD.print(F(":"), 160,564);
	 myGLCD.printNumI(cleanEnd.minute(), 180,564,2,48);
	 **/

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 2. Power
void PowerScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(219, 0, 0);
	myGLCD.print(F("POWER       "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Powe.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	//icons starts here
	//  myFiles.load(40, 180,74,74, "74LED_N.raw");
	//icosn end here
	updatePowerIcons();

	//text for the buttons
	myGLCD.setFont(BigFont);
	myGLCD.setColor(col_red.r, col_red.g, col_red.b);
	if (manualOverride) {
		myGLCD.print(F("Manual Override is active"), 20, 100);
	} else {
		myGLCD.print(F("                         "), 20, 100);
	}
	if (cleaningInProcess) {
		myGLCD.print(F("Cleaning ends @"), 20, 125);
		myGLCD.setColor(col_white.r, col_white.g, col_white.b);
		myGLCD.printNumI(cleanEnd.hour(), 310 - 30, 125, 2, 48);
		myGLCD.print(F(":"), 360 - 30, 125);
		myGLCD.printNumI(cleanEnd.minute(), 380 - 30, 125, 2, 48);

	} else {
		myGLCD.print(F("                         "), 20, 125);
	}
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.print(F("Filter 1"), 110, 180);
	myGLCD.print(F("Filter 2"), 334, 180);
	myGLCD.print(F("Light 1"), 110, 264);
	myGLCD.print(F("Light 2"), 334, 264);
	myGLCD.print(F("CO2"), 110, 348);
	myGLCD.print(F("Heater"), 334, 348);
	myGLCD.print(F("Cooling"), 110, 432);
	myGLCD.print(F("ALL OFF"), 110, 550);
	myGLCD.print(F("RESET"), 110, 635);
	myGLCD.print(F("CLEAN"), 334, 550);
	myGLCD.print(F("MUTE"), 334, 635);

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");
}

void updatePowerIcons() {
	if (!pump1Value) {
		myFiles.load(20, 150, 74, 74, "74Filt_N.raw");
	} else {
		myFiles.load(20, 150, 74, 74, "74Filt_F.raw");
	}
	if (!pump2Value) {
		myFiles.load(250, 150, 74, 74, "74Filt_N.raw");
	} else {
		myFiles.load(250, 150, 74, 74, "74Filt_F.raw");
	}
	if (!light230Value) {
		myFiles.load(20, 234, 74, 74, "74LED_N.raw");
	} else {
		myFiles.load(20, 234, 74, 74, "74LED_F.raw");
	}
	if (!light2Value) {
		myFiles.load(250, 234, 74, 74, "74LED_N.raw");
	} else {
		myFiles.load(250, 234, 74, 74, "74LED_F.raw");
	}
	if (!co2Value) {
		myFiles.load(20, 318, 74, 74, "74CO2_N.raw");
	} else {
		myFiles.load(20, 318, 74, 74, "74CO2_F.raw");
	}
	if (!heaterValue) {
		myFiles.load(250, 318, 74, 74, "74heat_N.raw");
	} else {
		myFiles.load(250, 318, 74, 74, "74heat_F.raw");
	}
	if (!coolValue) {
		myFiles.load(20, 402, 74, 74, "74Fan_N.raw");
	} else {
		myFiles.load(20, 402, 74, 74, "74Fan_F.raw");
	}
	if (beepActive) {
		myFiles.load(250, 608, 74, 74, "74spk_N.raw");
	} else {
		myFiles.load(250, 608, 74, 74, "74spk_F.raw");
	}

	myFiles.load(20, 524, 74, 74, "74OFF.raw");
	myFiles.load(20, 608, 74, 74, "74Reset.raw");
	myFiles.load(250, 524, 74, 74, "74CleanN.raw"); //Cleanmode
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 3. Settings
void SettingsScreen() { //Statusline TOP
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(255, 36, 0);
	myGLCD.print(F("SETTINGS      "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Sett.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);

	//end of >TOP
	myFiles.load(PowerSchedCord[0], PowerSchedCord[1], 74, 74, "74Powe.raw");
	myFiles.load(LightsCord[0], LightsCord[1], 74, 74, "74Ligh.raw");
	myFiles.load(CleanCord[0], CleanCord[1], 74, 74, "74CleanN.raw");
	myFiles.load(ScheCord[0], ScheCord[1], 74, 74, "74Sche.raw");
	myFiles.load(ClockCord[0], ClockCord[1], 74, 74, "74Cloc.raw");
	myFiles.load(Co2SetCord[0], Co2SetCord[1], 74, 74, "74Co2_N.raw");
	myFiles.load(HeatCord[0], HeatCord[1], 74, 74, "74Heat.raw");
	myFiles.load(DoseCord[0], DoseCord[1], 74, 74, "74Dose.raw");
	myFiles.load(ScreenCord[0], ScreenCord[1], 74, 74, "74Scree.raw");
	myFiles.load(RGBCord[0], RGBCord[1], 74, 74, "74RGB.raw");
	myFiles.load(TVModeCord[0], TVModeCord[1], 74, 74, "74TV.raw");
	myFiles.load(MoonModeCord[0], MoonModeCord[1], 74, 74, "74Moon.raw");

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 4. Lights
void LightsScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(182, 0, 255);
	myGLCD.print(F("LIGHTS            "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Ligh.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setFont(OCR_A_Extended_M);

	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("Light Scene 1"), 15, 140);
	myGLCD.printNumI(lightPWM[0].Hour, 255, 140, 2, 48);
	myGLCD.print(F(":"), 290, 140);
	myGLCD.printNumI(lightPWM[0].Minute, 305, 140, 2, 48);
	myGLCD.print(F(" - "), 340, 140);
	myGLCD.printNumI(lightPWM[1].Hour, 385, 140, 2, 48);
	myGLCD.print(F(":"), 420, 140);
	myGLCD.printNumI(lightPWM[1].Minute, 435, 140, 2, 48);
	myGLCD.setColor(182, 0, 255);
	myGLCD.printNumI(int(lightPWM[0].pwmValue * 100 / 255), 274, 170, 3);
	myGLCD.print(F("%"), 320, 170);
	myGLCD.printNumI(int(lightPWM[1].pwmValue * 100 / 255), 404, 170, 3);
	myGLCD.print(F("%"), 450, 170);

	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("Light Scene 2"), 15, 230);
	myGLCD.printNumI(lightPWM[2].Hour, 255, 230, 2, 48);
	myGLCD.print(F(":"), 290, 230);
	myGLCD.printNumI(lightPWM[2].Minute, 305, 230, 2, 48);
	myGLCD.print(F(" - "), 340, 230);
	myGLCD.printNumI(lightPWM[3].Hour, 385, 230, 2, 48);
	myGLCD.print(F(":"), 420, 230);
	myGLCD.printNumI(lightPWM[3].Minute, 435, 230, 2, 48);
	myGLCD.setColor(182, 0, 255);
	myGLCD.printNumI(int(lightPWM[2].pwmValue * 100 / 255), 274, 260, 3);
	//int(calculatedPWM*100/255
	myGLCD.print(F("%"), 320, 260);
	myGLCD.printNumI(int(lightPWM[3].pwmValue * 100 / 255), 404, 260, 3);
	myGLCD.print(F("%"), 450, 260);

	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("Light Scene 3"), 15, 320);
	myGLCD.printNumI(lightPWM[4].Hour, 255, 320, 2, 48);
	myGLCD.print(F(":"), 290, 320);
	myGLCD.printNumI(lightPWM[4].Minute, 305, 320, 2, 48);
	myGLCD.print(F(" - "), 340, 140);
	myGLCD.printNumI(lightPWM[5].Hour, 385, 320, 2, 48);
	myGLCD.print(F(":"), 420, 320);
	myGLCD.printNumI(lightPWM[5].Minute, 435, 320, 2, 48);
	myGLCD.setColor(182, 0, 255);
	myGLCD.printNumI(int(lightPWM[4].pwmValue * 100 / 255), 274, 350, 3);
	myGLCD.print(F("%"), 320, 350);
	myGLCD.printNumI(int(lightPWM[5].pwmValue * 100 / 255), 404, 350, 3);
	myGLCD.print(F("%"), 450, 350);

	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("Light Scene 4"), 15, 410);
	myGLCD.printNumI(lightPWM[6].Hour, 255, 410, 2, 48);
	myGLCD.print(F(":"), 290, 410);
	myGLCD.printNumI(lightPWM[6].Minute, 305, 410, 2, 48);
	myGLCD.print(F(" - "), 340, 410);
	myGLCD.printNumI(lightPWM[7].Hour, 385, 410, 2, 48);
	myGLCD.print(F(":"), 420, 410);
	myGLCD.printNumI(lightPWM[7].Minute, 435, 410, 2, 48);
	myGLCD.setColor(182, 0, 255);
	myGLCD.printNumI(int(lightPWM[6].pwmValue * 100 / 255), 274, 440, 3);
	myGLCD.print(F("%"), 320, 440);
	myGLCD.printNumI(int(lightPWM[7].pwmValue * 100 / 255), 404, 440, 3);
	myGLCD.print(F("%"), 450, 440);

	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("Light Scene 5"), 15, 500);
	myGLCD.printNumI(lightPWM[8].Hour, 255, 500, 2, 48);
	myGLCD.print(F(":"), 290, 500);
	myGLCD.printNumI(lightPWM[8].Minute, 305, 500, 2, 48);
	myGLCD.print(F(" - "), 340, 500);
	myGLCD.printNumI(lightPWM[9].Hour, 385, 500, 2, 48);
	myGLCD.print(F(":"), 420, 500);
	myGLCD.printNumI(lightPWM[9].Minute, 435, 500, 2, 48);
	myGLCD.setColor(182, 0, 255);
	myGLCD.printNumI(int(lightPWM[8].pwmValue * 100 / 255), 274, 530, 3);
	myGLCD.print(F("%"), 320, 530);
	myGLCD.printNumI(int(lightPWM[9].pwmValue * 100 / 255), 404, 530, 3);
	myGLCD.print(F("%"), 450, 530);

	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("Light Scene 6"), 15, 590);
	myGLCD.printNumI(lightPWM[10].Hour, 255, 590, 2, 48);
	myGLCD.print(F(":"), 290, 590);
	myGLCD.printNumI(lightPWM[10].Minute, 305, 590, 2, 48);
	myGLCD.print(F(" - "), 340, 590);
	myGLCD.printNumI(lightPWM[11].Hour, 385, 590, 2, 48);
	myGLCD.print(F(":"), 420, 590);
	myGLCD.printNumI(lightPWM[11].Minute, 435, 590, 2, 48);
	myGLCD.setColor(182, 0, 255);
	myGLCD.printNumI(int(lightPWM[10].pwmValue * 100 / 255), 274, 620, 3);
	myGLCD.print(F("%"), 320, 620);
	myGLCD.printNumI(int(lightPWM[11].pwmValue * 100 / 255), 404, 620, 3);
	myGLCD.print(F("%"), 450, 620);

	/*
	 myGLCD.print("Light Scene 2  07:20 - 11:50" , 15,230);
	 myGLCD.print(" 90%" , 400,260);
	 myGLCD.print("Light Scene 3  12:00 - 15:50" , 15,320);
	 myGLCD.print("  0%" , 400,350);
	 myGLCD.print("Light Scene 4  12:00 - 15:50" , 15,410);
	 myGLCD.print("100%" , 400,440);
	 myGLCD.print("Light Scene 5  00:00 - 00:00" , 15,500);
	 myGLCD.print("  0%" , 400,530);
	 myGLCD.print("Light Scene 6  00:00 - 00:00" , 15,590);
	 myGLCD.print("  0%" , 400,620);
	 */
	/*
	 myFiles.load(ClockCord[0], ClockCord[1],74,74, "74Cloc.raw");
	 myFiles.load(LightsCord[0], LightsCord[1],74,74, "74Ligh.raw");
	 myFiles.load(CleanCord[0], CleanCord[1],74,74, "74Clea.raw");
	 myFiles.load(ScheCord[0], ScheCord[1],74,74, "74Sche.raw");
	 myFiles.load(ScreenCord[0], ScreenCord[1],74,74, "74Remi.raw");
	 myFiles.load(HeatCord[0], HeatCord[1],74,74, "74Heat.raw");
	 myFiles.load(Co2SetCord[0], Co2SetCord[1],74,74, "74Co2_N.raw");
	 myFiles.load(DoseCord[0], DoseCord[1],74,74, "74Dose.raw");
	 */
	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 41 . LightScene1
void LightScene() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(182, 0, 255);
	myGLCD.print(F("LIGHT SCENE"), 77, 40);
	myGLCD.printNumI(dispScreen - 40, 360, 40);
	myFiles.load(6, 24, 60, 60, "60Ligh.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setColor(182, 0, 255);
	myGLCD.print(F("SCENE ON"), 20, 100);
	myGLCD.print(F("BRIGHTNESS"), 20, 230);
	myGLCD.print(F("SCENE OFF"), 20, 400);
	myGLCD.print(F("BRIGHTNESS"), 20, 530);

	myFiles.load(powLightOnHourUp[0], powLightOnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powLightOnHourDown[0], powLightOnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOnMinuteUp[0], powLightOnMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOnMinuteDown[0], powLightOnMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(powCo2OnHourUp[0], powCo2OnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnHourDown[0], powCo2OnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");
	UpdateLightScene();
	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void UpdateLightScene() {
	myGLCD.setFont(UbuntuBold);

	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(lightPWM[lightScreenSet].Hour, 220, 145, 2, 48);
	myGLCD.printNumI(lightPWM[lightScreenSet].Minute, 360, 145, 2, 48);
	//myGLCD.printNumI(int(100-(lightPWM[lightScreenSet].pwmValue*100)/255), 335,275,3);
	myGLCD.printNumI(int(lightPWM[lightScreenSet].pwmValue * 100 / 255), 335,
			275, 3);

	myGLCD.printNumI(lightPWM[lightScreenSet + 1].Hour, 220, 445, 2, 48);
	myGLCD.printNumI(lightPWM[lightScreenSet + 1].Minute, 360, 445, 2, 48);
	//myGLCD.printNumI(int(100-(lightPWM[lightScreenSet+1].pwmValue*100)/255), 335,575,3);
	myGLCD.printNumI(int(lightPWM[lightScreenSet + 1].pwmValue * 100 / 255),
			335, 575, 3);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 14. RGB

void RGBScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 255, 132);
	myGLCD.print(F("RGB            "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60RGB.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setFont(OCR_A_Extended_M);

	for (int i = 0; i <= 5; i++) {
		myGLCD.setColor(255, 255, 255);

		myGLCD.print(F("RGB Scene"), 15, 140 + (i * 90));
		myGLCD.printNumI(i + 1, 180, 140 + (i * 90));
		myGLCD.printNumI(lightRGB[i * 2].Hour, 255, 140 + (i * 90), 2, 48);
		myGLCD.print(F(":"), 290, 140 + (i * 90));
		myGLCD.printNumI(lightRGB[i * 2].Minute, 305, 140 + (i * 90), 2, 48);
		myGLCD.print(F(" - "), 340, 140 + (i * 90));
		myGLCD.print(F(" - "), 218, 170 + (i * 90));
		myGLCD.print(F("|"), 61, 170 + (i * 90)); //white pipes between RGB values
		myGLCD.print(F("|"), 403, 170 + (i * 90));
		myGLCD.print(F("|"), 342, 170 + (i * 90));
		myGLCD.print(F("|"), 122, 170 + (i * 90));
		myGLCD.printNumI(lightRGB[i * 2 + 1].Hour, 385, 140 + (i * 90), 2, 48);
		myGLCD.print(F(":"), 420, 140 + (i * 90));

		myGLCD.printNumI(lightRGB[i * 2 + 1].Minute, 435, 140 + (i * 90), 2,
				48);
		if (int(lightRGB[i * 2].red) + int(lightRGB[i * 2].green)
				+ int(lightRGB[i * 2].blue) < 60) {
			myGLCD.setColor(100, 100, 100);
		} else {
			myGLCD.setColor(lightRGB[i * 2].red, lightRGB[i * 2].green,
					lightRGB[i * 2].blue);
		}
		myGLCD.printNumI(int(lightRGB[i * 2].red), 15, 170 + (i * 90), 3);

		myGLCD.printNumI(int(lightRGB[i * 2].green), 76, 170 + (i * 90), 3);

		myGLCD.printNumI(int(lightRGB[i * 2].blue), 137, 170 + (i * 90), 3);

		if (int(lightRGB[i * 2 + 1].red) + int(lightRGB[i * 2 + 1].green)
				+ int(lightRGB[i * 2 + 1].blue) < 60) {
			myGLCD.setColor(100, 100, 100);
		} else {
			myGLCD.setColor(lightRGB[i * 2 + 1].red, lightRGB[i * 2 + 1].green,
					lightRGB[i * 2 + 1].blue);
		}
		myGLCD.printNumI(int(lightRGB[i * 2 + 1].red), 296, 170 + (i * 90), 3);

		myGLCD.printNumI(int(lightRGB[i * 2 + 1].green), 357, 170 + (i * 90),
				3);

		myGLCD.printNumI(int(lightRGB[i * 2 + 1].blue), 418, 170 + (i * 90), 3);

	}

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void RGBScene() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 255, 132);
	myGLCD.print(F("RGB SCENE"), 77, 40);
	myGLCD.printNumI(dispScreen - 140, 360, 40);
	myFiles.load(6, 24, 60, 60, "60Ligh.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setColor(182, 0, 255);
	myGLCD.print(F("SCENE ON"), 20, 100);
	myGLCD.print(F("RGB"), 20, 230);
	myGLCD.print(F("SCENE OFF"), 20, 400);
	myGLCD.print(F("RGB"), 20, 530);

	myFiles.load(powLightOnHourUp[0], powLightOnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powLightOnHourDown[0], powLightOnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOnMinuteUp[0], powLightOnMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOnMinuteDown[0], powLightOnMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(red1Up[0], red1Up[1], 48, 48, "48up.raw"); //red 1 up
	myFiles.load(red1Down[0], red1Down[1], 48, 48, "48up.raw"); //red 1 down
	myFiles.load(powLightOffHourUp[0], powLightOffHourUp[1], 48, 48,
			"48up.raw"); //green 1  up
	myFiles.load(powLightOffHourDown[0], powLightOffHourDown[1], 48, 48,
			"48down.raw"); //green 1 down
	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw"); //blue 1 up
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw"); //blue 1 down

	myFiles.load(powCo2OnHourUp[0], powCo2OnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnHourDown[0], powCo2OnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(red2Up[0], red2Up[1], 48, 48, "48up.raw"); //red 2 up
	myFiles.load(red2Down[0], red2Down[1], 48, 48, "48up.raw"); //red 2 down
	myFiles.load(powCo2OffHourUp[0], powCo2OffHourUp[1], 48, 48, "48up.raw"); //green 2  up
	myFiles.load(powCo2OffHourDown[0], powCo2OffHourDown[1], 48, 48,
			"48down.raw"); //green 2 down
	myFiles.load(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1], 48, 48,
			"48up.raw"); //blue 2 up
	myFiles.load(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1], 48, 48,
			"48down.raw"); //blue 2 down

	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	UpdateRGBSceneTOP();
	UpdateRGBSceneBOT();
	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void MoonScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(135, 171, 255);
	myGLCD.print(F("MOONLIGHT MODE"), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Moon.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setColor(135, 171, 255);
	myGLCD.print(F("Minutes"), 20, 100);
	myGLCD.print("RGB", 20, 230);

	myFiles.load(powLightOnMinuteUp[0], powLightOnMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOnMinuteDown[0], powLightOnMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(red1Up[0], red1Up[1], 48, 48, "48up.raw"); //red 1 up
	myFiles.load(red1Down[0], red1Down[1], 48, 48, "48up.raw"); //red 1 down
	myFiles.load(powLightOffHourUp[0], powLightOffHourUp[1], 48, 48,
			"48up.raw"); //green 1  up
	myFiles.load(powLightOffHourDown[0], powLightOffHourDown[1], 48, 48,
			"48down.raw"); //green 1 down
	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw"); //blue 1 up
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw"); //blue 1 down

	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	UpdateMoonScreen();

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void UpdateMoonScreen() {
	myGLCD.setFont(UbuntuBold);

	myGLCD.setColor(255, 255, 255);

	myGLCD.printNumI(MoonMinutes, 335, 145, 3);
	if ((MoonRed + MoonGreen + MoonBlue) < 60) {
		myGLCD.setColor(100, 100, 100);
	} else {
		myGLCD.setColor(MoonRed, MoonGreen, MoonBlue);
	}
	myGLCD.printNumI(MoonRed, 49, 275, 3);
	myGLCD.printNumI(MoonGreen, 192, 275, 3);
	myGLCD.printNumI(MoonBlue, 335, 275, 3);

}

void TVScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 97, 23);
	myGLCD.print(F("TV MODE"), 77, 40);
	myFiles.load(6, 24, 60, 60, "60TV.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP
	myGLCD.setColor(0, 97, 23);
	myGLCD.print(F("Brightness"), 20, 100);

	myFiles.load(powLightOnMinuteUp[0], powLightOnMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOnMinuteDown[0], powLightOnMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	UpdateTVScreen();

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void UpdateTVScreen() {
	myGLCD.setFont(UbuntuBold);

	myGLCD.setColor(255, 255, 255);

	myGLCD.printNumI(int(TVModeBrightness * 100 / 255), 335, 145, 3);

}

void UpdateRGBSceneTOP() {
	myGLCD.setFont(UbuntuBold);

	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(lightRGB[RGBScreenSet].Hour, 220, 145, 2, 48);
	myGLCD.printNumI(lightRGB[RGBScreenSet].Minute, 360, 145, 2, 48);
	if (int(lightRGB[RGBScreenSet].red) + int(lightRGB[RGBScreenSet].green)
			+ int(lightRGB[RGBScreenSet].blue) < 60) {
		myGLCD.setColor(100, 100, 100);
	} else {
		myGLCD.setColor(lightRGB[RGBScreenSet].red,
				lightRGB[RGBScreenSet].green, lightRGB[RGBScreenSet].blue);
	}
	myGLCD.printNumI(int(lightRGB[RGBScreenSet].red), 49, 275, 3);
	myGLCD.printNumI(int(lightRGB[RGBScreenSet].green), 192, 275, 3);
	myGLCD.printNumI(int(lightRGB[RGBScreenSet].blue), 335, 275, 3);

}

void UpdateRGBSceneBOT() {
	myGLCD.setFont(UbuntuBold);

	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(lightRGB[RGBScreenSet + 1].Hour, 220, 445, 2, 48);
	myGLCD.printNumI(lightRGB[RGBScreenSet + 1].Minute, 360, 445, 2, 48);

	if (int(lightRGB[RGBScreenSet + 1].red)
			+ int(lightRGB[RGBScreenSet + 1].green)
			+ int(lightRGB[RGBScreenSet + 1].blue) < 60) {
		myGLCD.setColor(100, 100, 100);
	} else {
		myGLCD.setColor(lightRGB[RGBScreenSet + 1].red,
				lightRGB[RGBScreenSet + 1].green,
				lightRGB[RGBScreenSet + 1].blue);
	}
	myGLCD.printNumI(int(lightRGB[RGBScreenSet + 1].red), 49, 575, 3);
	myGLCD.printNumI(int(lightRGB[RGBScreenSet + 1].green), 192, 575, 3);
	myGLCD.printNumI(int(lightRGB[RGBScreenSet + 1].blue), 335, 575, 3);

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 5. Settings
void ClockScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 109, 255);
	myGLCD.print(F("CLOCK        "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Cloc.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	updateClockSettings();
	//end of >TOP
	//get the time

	myFiles.load(HourUp[0], HourUp[1], 48, 48, "48up.raw");
	myFiles.load(HourDown[0], HourDown[1], 48, 48, "48down.raw");
	myFiles.load(MinuteUp[0], MinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(MinuteDown[0], MinuteDown[1], 48, 48, "48down.raw");

	myFiles.load(DayUp[0], DayUp[1], 48, 48, "48up.raw");
	myFiles.load(DayDown[0], DayDown[1], 48, 48, "48down.raw");
	myFiles.load(MonthUp[0], MonthUp[1], 48, 48, "48up.raw");
	myFiles.load(MonthDown[0], MonthDown[1], 48, 48, "48down.raw");
	myFiles.load(YearUp[0], YearUp[1], 48, 48, "48up.raw");
	myFiles.load(YearDown[0], YearDown[1], 48, 48, "48down.raw");
	myFiles.load(SetClockCord[0], SetClockCord[1], 168, 52, "6set.raw");

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void updateClockSettings() {
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(255, 255, 255);
	//end of >TOP
	//get the time
	myGLCD.printNumI(adjustTimer.hour(), 100, 260, 2, 48);
	myGLCD.printNumI(adjustTimer.minute(), 270, 260, 2, 48);
	myGLCD.printNumI(adjustTimer.day(), 50, 424, 2, 48);
	myGLCD.printNumI(adjustTimer.month(), 170, 424, 2, 48);
	myGLCD.printNumI(adjustTimer.year(), 280, 424, 2, 48);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 6 . CleanSched
void CleanSchedScreen() { //Statusline TOP
	wdt_reset();
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 219, 0);
	myGLCD.print(F("CLEANING   "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60CleanN.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setFont(BigFont);

	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.print(F("Filter 1"), 110, 180);
	myGLCD.print(F("Filter 2"), 334, 180);
	myGLCD.print(F("Light 1"), 110, 264);
	myGLCD.print(F("Light 2"), 334, 264);
	myGLCD.print(F("CO2"), 110, 348);
	myGLCD.print(F("Heater"), 334, 348);
	myGLCD.print(F("Cooling"), 110, 432);

	myFiles.load(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");
	myGLCD.setFont(UbuntuBold);
	myGLCD.print(F("Minutes"), 70, 575);
	/*
	 */
	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void updateCleanSchedScreen() {
	wdt_reset();
	if (!pump1Clean) {
		myFiles.load(20, 150, 74, 74, "74Filt_N.raw");
	} else {
		myFiles.load(20, 150, 74, 74, "74Filt_F.raw");
	}
	if (!pump2Clean) {
		myFiles.load(250, 150, 74, 74, "74Filt_N.raw");
	} else {
		myFiles.load(250, 150, 74, 74, "74Filt_F.raw");
	}
	if (!light230Clean) {
		myFiles.load(20, 234, 74, 74, "74LED_N.raw");
	} else {
		myFiles.load(20, 234, 74, 74, "74LED_F.raw");
	}
	wdt_reset();
	if (!light2Clean) {
		myFiles.load(250, 234, 74, 74, "74LED_N.raw");
	} else {
		myFiles.load(250, 234, 74, 74, "74LED_F.raw");
	}
	if (!co2Clean) {
		myFiles.load(20, 318, 74, 74, "74CO2_N.raw");
	} else {
		myFiles.load(20, 318, 74, 74, "74CO2_F.raw");
	}
	if (!heaterClean) {
		myFiles.load(250, 318, 74, 74, "74heat_N.raw");
	} else {
		myFiles.load(250, 318, 74, 74, "74heat_F.raw");
	}
	if (!coolClean) {
		myFiles.load(20, 402, 74, 74, "74Fan_N.raw");
	} else {
		myFiles.load(20, 402, 74, 74, "74Fan_F.raw");
	}

}

void quickUpdateCleanSchedScreen() {
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 219, 0);
	myGLCD.printNumI(cleanMinutes, 330, 575, 3);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 7. Schedule
void RemindScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(219, 0, 170);
	myGLCD.print(F("REMINDER            "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Sche.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setFont(OCR_A_Extended_M);
	myGLCD.print(F("Cleaned Tank"), 120, 155);
	myGLCD.print(F("days ago"), 265, 190);
	myGLCD.print("(", 190, 190);
	myGLCD.print(")", 248, 190);

	myGLCD.print(F("new CO2 bottle"), 120, 239);
	myGLCD.print(F("days ago"), 265, 274);
	myGLCD.print("(", 190, 274);
	myGLCD.print(")", 248, 274);

	myGLCD.print(F("Cleaned Filter 1"), 120, 323);
	myGLCD.print(F("days ago"), 265, 358);
	myGLCD.print("(", 190, 358);
	myGLCD.print(")", 248, 358);

	myGLCD.print(F("Cleaned Filter 2"), 120, 407);
	myGLCD.print(F("days ago"), 265, 442);
	myGLCD.print("(", 190, 442);
	myGLCD.print(")", 248, 442);

	myFiles.load(20, 150, 74, 74, "74Clea.raw");
	myFiles.load(20, 234, 74, 74, "74CO2_N.raw");
	myFiles.load(20, 318, 74, 74, "74Filt_N.raw");
	myFiles.load(20, 402, 74, 74, "74Filt_N.raw");
	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	updateRemindScreen();

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

/*
 myGLCD.printNumI(printnow.hour(), 140,190,3,48);
 myGLCD.print(F(":"), 430,ty);
 myGLCD.printNumI(printnow.minute(), 445,ty,2,48);
 */

void updateRemindScreen() {
	myGLCD.setFont(OCR_A_Extended_M);
	myGLCD.setColor(219, 0, 170);
	// myGLCD.printNumI(((now.unixtime()-tankClean.unixtime())/86400), 110,190,3);
	// myGLCD.printNumI(tankCleandDays, 125,190,3);
	//myGLCD.printNumI(((now.unixtime()-tankClean.unixtime())/86400), 105,190,3);
	myGLCD.printNumI(((now.unixtime() - tankClean.unixtime()) / 86400), 135,
			190, 3);
	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(tankCleandDays, 200, 190, 3);

	//myGLCD.print("66", 140,274);
	//  myGLCD.printNumI(((now.unixtime()-co2Bottle.unixtime())/1), 130,274,3);
	myGLCD.setColor(219, 0, 170);
	myGLCD.printNumI(((now.unixtime() - co2Bottle.unixtime()) / 86400), 130,
			274, 3);
	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(co2BottleDays, 200, 274, 3);

	myGLCD.setColor(219, 0, 170);
	myGLCD.printNumI(((now.unixtime() - cleanFilter1.unixtime()) / 86400), 130,
			358, 3);
	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(cleanFilter1Days, 200, 358, 3);

	myGLCD.setColor(219, 0, 170);
	myGLCD.printNumI(((now.unixtime() - cleanFilter2.unixtime()) / 86400), 130,
			442, 3);
	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(cleanFilter2Days, 200, 442, 3);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 8. Heater
void HeaterScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(219, 0, 85);
	myGLCD.print(F("HEATER              "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Heat.raw");

	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP
	myGLCD.print(F("UPPER LIMIT"), 20, 215);
	myGLCD.print(F("LOWER LIMIT"), 20, 385);
	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");
	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void updateHeaterScreen() {
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.printNumF(TempUpperLimit, 2, 280, 275);
	myGLCD.printNumF(TempLowerLimit, 2, 280, 445);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 9. Dose
void DoseScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(73, 36, 0);
	myGLCD.print(F("DOSING             "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Dose.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP

	myGLCD.setFont(OCR_A_Extended_M);

	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("Macro "), 15, 140);

	myGLCD.setColor(col_FertiN.r, col_FertiN.g, col_FertiN.b);
	myGLCD.print("N", 100, 140);
	myGLCD.printNumI(FDose[0], 335, 140);
	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("ml/day"), 370, 140);

	myGLCD.setColor(col_FertiN.r, col_FertiN.g, col_FertiN.b);
	myGLCD.printNumI(int(FLeft[0] / FDose[0]), 244, 170, 3);
	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("doses left"), 305, 170);

	myGLCD.print("Macro", 15, 230);

	myGLCD.setColor(col_FertiNPK.r, col_FertiNPK.g, col_FertiNPK.b);
	myGLCD.print("NPK", 100, 230);
	myGLCD.printNumI(FDose[1], 335, 230);
	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("ml/day"), 370, 230);

	myGLCD.setColor(col_FertiNPK.r, col_FertiNPK.g, col_FertiNPK.b);
	myGLCD.printNumI(int(FLeft[1] / FDose[1]), 244, 260, 3);
	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("doses left"), 305, 260);

	myGLCD.print("Micro", 15, 320);
	myGLCD.setColor(col_FertiFE.r, col_FertiFE.g, col_FertiFE.b);
	myGLCD.print("FE", 100, 320);
	myGLCD.printNumI(FDose[2], 335, 320);
	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("ml/day"), 370, 320);

	myGLCD.setColor(col_FertiFE.r, col_FertiFE.g, col_FertiFE.b);
	myGLCD.printNumI(int(FLeft[2] / FDose[2]), 244, 350, 3);
	myGLCD.setColor(255, 255, 255);
	myGLCD.print(F("doses left"), 305, 350);

	myGLCD.setFont(UbuntuBold);
	myGLCD.print("TIME", 10, 445);
	myFiles.load(powCo2OnHourUp[0], powCo2OnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnHourDown[0], powCo2OnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(refillAllCord[0], refillAllCord[1], 260, 52, "260Ref.raw");

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void updateDoseScreen() {
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(73, 36, 0);
	myGLCD.printNumI(doseHour, 220, 445, 2, 48);
	myGLCD.printNumI(doseMinute, 360, 445, 2, 48);
}

// 91. DoseScreeen for N
void DoseScreenN() {
	myGLCD.setFont(UbuntuBold);

	myFiles.load(6, 24, 60, 60, "60Dose.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP
	myGLCD.setFont(OCR_A_Extended_M);

	myGLCD.print(F("MO  TU  WE"), 25, 550);
	myGLCD.print(F("TH  FR  SA  SO"), 225, 550);
	myGLCD.setFont(UbuntuBold);
	myGLCD.print(F("dose/day"), 20, 145);
	myGLCD.print(F("ml"), 345, 145);

	myGLCD.print(F("volume"), 20, 275);
	myGLCD.print("ml", 345, 275);

	myGLCD.print(F("ml/min"), 260, 445);
	//445
	myFiles.load(calibrateCord[0], calibrateCord[1], 260, 52, "260Cal.raw");
	myFiles.load(powLightOnMinuteUp[0], powLightOnMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOnMinuteDown[0], powLightOnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");
}

void UpdateDoseScreenN(byte FS) {
	if (MoF[FS]) {
		myFiles.load(MoCord[0], MoCord[1], 48, 48, "48chec.raw");
	} else {
		myFiles.load(MoCord[0], MoCord[1], 48, 48, "48unch.raw");
	}
	if (TuF[FS]) {
		myFiles.load(TuCord[0], TuCord[1], 48, 48, "48chec.raw");
	} else {
		myFiles.load(TuCord[0], TuCord[1], 48, 48, "48unch.raw");
	}
	if (WeF[FS]) {
		myFiles.load(WeCord[0], WeCord[1], 48, 48, "48chec.raw");
	} else {
		myFiles.load(WeCord[0], WeCord[1], 48, 48, "48unch.raw");
	}
	if (ThF[FS]) {
		myFiles.load(ThCord[0], ThCord[1], 48, 48, "48chec.raw");
	} else {
		myFiles.load(ThCord[0], ThCord[1], 48, 48, "48unch.raw");
	}
	if (FrF[FS]) {
		myFiles.load(FrCord[0], FrCord[1], 48, 48, "48chec.raw");
	} else {
		myFiles.load(FrCord[0], FrCord[1], 48, 48, "48unch.raw");
	}
	if (SaF[FS]) {
		myFiles.load(SaCord[0], SaCord[1], 48, 48, "48chec.raw");
	} else {
		myFiles.load(SaCord[0], SaCord[1], 48, 48, "48unch.raw");
	}
	if (SuF[FS]) {
		myFiles.load(SoCord[0], SoCord[1], 48, 48, "48chec.raw");
	} else {
		myFiles.load(SoCord[0], SoCord[1], 48, 48, "48unch.raw");
	}
}

void quickUpdateDoseScreenN(byte FS) {
	myGLCD.setFont(UbuntuBold);
	switch (FS) {
	case 0:
		myGLCD.setColor(73, 36, 0);
		myGLCD.print("MACRO", 77, 40);
		myGLCD.setColor(col_FertiN.r, col_FertiN.g, col_FertiN.b);
		myGLCD.print("N", 215, 40);

		break;
	case 1:
		myGLCD.setColor(73, 36, 0);
		myGLCD.print("MACRO", 77, 40);
		myGLCD.setColor(col_FertiNPK.r, col_FertiNPK.g, col_FertiNPK.b);
		myGLCD.print("NPK", 215, 40);

		break;
	case 2:
		myGLCD.setColor(73, 36, 0);
		myGLCD.print("MICRO", 77, 40);
		myGLCD.setColor(col_FertiFE.r, col_FertiFE.g, col_FertiFE.b);
		myGLCD.print("FE", 215, 40);

		break;
	}
	myGLCD.printNumI(FDose[FS], 270, 145, 3);
	myGLCD.printNumI(FMax[FS], 270, 275, 3);
	myGLCD.printNumI(FRate[FS], 185, 445, 3);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 10. Powerschedscreen
void PowerSchedScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(219, 0, 0);
	myGLCD.print(F("POWER           "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Powe.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	UpdatePowerSchedScreen();
	//end of >TOP

	myGLCD.setColor(219, 0, 0);
	myGLCD.print(F("LIGHTS ON"), 20, 100);
	myGLCD.print(F("LIGHTS OFF"), 20, 230);
	myGLCD.print(F("CO2 ON"), 20, 400);
	myGLCD.print(F("CO2 OFF"), 20, 530);
	myFiles.load(powLightOnHourUp[0], powLightOnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powLightOnHourDown[0], powLightOnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOnMinuteUp[0], powLightOnMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOnMinuteDown[0], powLightOnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOffHourUp[0], powLightOffHourUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffHourDown[0], powLightOffHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnHourUp[0], powCo2OnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnHourDown[0], powCo2OnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OffHourUp[0], powCo2OffHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OffHourDown[0], powCo2OffHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void UpdatePowerSchedScreen() {
	myGLCD.setFont(UbuntuBold);

	myGLCD.setColor(255, 255, 255);
	myGLCD.printNumI(powLightOnHour, 220, 145, 2, 48);
	myGLCD.printNumI(powLightOnMinute, 360, 145, 2, 48);
	myGLCD.printNumI(powLightOffHour, 220, 275, 2, 48);
	myGLCD.printNumI(powLightOffMinute, 360, 275, 2, 48);

	myGLCD.printNumI(powCo2OnHour, 220, 445, 2, 48);
	myGLCD.printNumI(powCo2OnMinute, 360, 445, 2, 48);
	myGLCD.printNumI(powCo2OffHour, 220, 575, 2, 48);
	myGLCD.printNumI(powCo2OffMinute, 360, 575, 2, 48);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 13. Dose
void ScreenScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 75, 450, 75);
	myGLCD.print(F("SCREEN         "), 77, 40);
	myFiles.load(6, 24, 60, 60, "60Scree.raw");
	myGLCD.setColor(255, 0, 0);

	//end of >TOP

	myGLCD.print(F("SCREEN ON"), 20, 100);
	myGLCD.print(F("SCREEN OFF"), 20, 230);
	myGLCD.print("MINUTES", 20, 390);
	myGLCD.print("BRIGHTNESS", 20, 510);
	myGLCD.setFont(BigFont);
	myGLCD.print("to Standby", 200, 400);
	myGLCD.print("in Standby", 265, 520);
	myFiles.load(powLightOnHourUp[0], powLightOnHourUp[1], 48, 48, "48up.raw");
	myFiles.load(powLightOnHourDown[0], powLightOnHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOnMinuteUp[0], powLightOnMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOnMinuteDown[0], powLightOnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOffHourUp[0], powLightOffHourUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffHourDown[0], powLightOffHourDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OffMinuteUp[0], powCo2OffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powCo2OffMinuteDown[0], powCo2OffMinuteDown[1], 48, 48,
			"48down.raw");

	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");
	UpdateScreenScreen();

}

void UpdateScreenScreen()

{
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(255, 255, 255);

	myGLCD.printNumI(screenOnHour, 220, 145, 2, 48);
	myGLCD.printNumI(screenOnMinute, 360, 145, 2, 48);
	myGLCD.printNumI(screenOffHour, 220, 275, 2, 48);
	myGLCD.printNumI(screenOffMinute, 360, 275, 2, 48);

	myGLCD.printNumI(standByMinutes, 360, 445, 2, 48);
	myGLCD.printNumI(backlightPWM, 335, 575, 3);

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 12. Co2 Settings
void Co2SetScreen() { //Statusline TOP

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 182, 16);
	myGLCD.print("CO2 SETTINGS     ", 77, 40);
	myFiles.load(6, 24, 60, 60, "60Co2.raw");

	myGLCD.drawLine(66, 75, 450, 75);
	//end of >TOP
	/*
	 myFiles.load(ClockCord[0], ClockCord[1],74,74, "74Cloc.raw");
	 myFiles.load(LightsCord[0], LightsCord[1],74,74, "74Ligh.raw");
	 myFiles.load(CleanCord[0], CleanCord[1],74,74, "74Clea.raw");
	 myFiles.load(ScheCord[0], ScheCord[1],74,74, "74Sche.raw");
	 myFiles.load(ScreenCord[0], ScreenCord[1],74,74, "74Remi.raw");
	 myFiles.load(HeatCord[0], HeatCord[1],74,74, "74Heat.raw");
	 myFiles.load(Co2SetCord[0], Co2SetCord[1],74,74, "74Co2_N.raw");
	 myFiles.load(DoseCord[0], DoseCord[1],74,74, "74Dose.raw");
	 */

	myGLCD.print("UPPER LIMIT", 20, 215);
	myGLCD.print("LOWER LIMIT", 20, 385);
	myFiles.load(powLightOffMinuteUp[0], powLightOffMinuteUp[1], 48, 48,
			"48up.raw");
	myFiles.load(powLightOffMinuteDown[0], powLightOffMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(powCo2OnMinuteUp[0], powCo2OnMinuteUp[1], 48, 48, "48up.raw");
	myFiles.load(powCo2OnMinuteDown[0], powCo2OnMinuteDown[1], 48, 48,
			"48down.raw");
	myFiles.load(CancelPowerSchedCord[0], CancelPowerSchedCord[1], 168, 52,
			"6cancel.raw");
	myFiles.load(SetPowerSchedCord[0], SetPowerSchedCord[1], 168, 52,
			"6set.raw");

	//footer starts here
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.drawLine(30, 770, 196, 770);
	myGLCD.drawLine(284, 770, 450, 770);
	myFiles.load(216, 746, 48, 48, "HomeBot.raw");

}

void updateCO2SetScreen() {
	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(col_white.r, col_white.g, col_white.b);
	myGLCD.printNumF(PHUpperLimit, 2, 280, 275);
	myGLCD.printNumF(PHLowerLimit, 2, 280, 445);
}

// 13. draw TestScreen
void TestScreen() { //Statusline TOP
	myGLCD.setColor(255, 255, 255);

	myGLCD.setFont(UbuntuBold);
	myGLCD.setColor(0, 0, 255);
	myGLCD.print("HOME       ", 77, 40);
	myFiles.load(6, 24, 60, 60, "1home.raw");
	myGLCD.setColor(255, 255, 255);
	myGLCD.drawLine(66, 81, 450, 81);
	//end of >TOP
	myFiles.load(0, 0, 480, 800, "256Col.raw");
	/*draw Button 1
	 myGLCD.setColor(0, 0, 255);
	 myGLCD.fillRoundRect (Button1Cord[0], Button1Cord[1], Button1Cord[2], Button1Cord[3]);
	 myGLCD.setColor(255, 255, 255);
	 myGLCD.drawRoundRect (Button1Cord[0], Button1Cord[1], Button1Cord[2], Button1Cord[3]);
	 myGLCD.setBackColor (0, 0, 255);
	 myGLCD.print("Button 1", CENTER, 740);
	 myGLCD.setBackColor (0, 0, 0);
	 */ //Ende Button 1
	/*temporÃ¤r Werte anzeige
	 myGLCD.setColor(0, 0, 0); //to delete the pwm field every update
	 myGLCD.fillRect(330,370,375,390); //to delete the pwm field every Update
	 myGLCD.setColor(255, 255, 255);
	 myGLCD.print("Temperatur:", 10, 70);
	 myGLCD.printNumF(Temp,2, 300, 70);
	 myGLCD.print("PH:", 10, 90);
	 myGLCD.printNumF(PhWert,2, 315, 90);
	 myGLCD.print("Relais", 10, 130);
	 myGLCD.print("pump1Pin",  10, 150);
	 myGLCD.print(String(pump1Value),  365, 150);
	 myGLCD.print("pump2Pin",  10, 170);
	 myGLCD.print(String(pump2Value),  365, 170);
	 myGLCD.print("light230Pin",  10, 190);
	 myGLCD.print(String(light230Value),  365, 190);
	 myGLCD.print("light2Pin",  10, 210);
	 myGLCD.print(String(light2Value),  365, 210);
	 myGLCD.print("co2Pin",  10, 230);
	 myGLCD.print(String(co2Value),  365, 230);
	 myGLCD.print("heaterPin",  10, 250);
	 myGLCD.print(String(heaterValue),  365, 250);
	 myGLCD.print("dPump1Pin",  10, 270);
	 myGLCD.print(String(dPump1Value),  365, 270);
	 myGLCD.print("dPump2Pin",  10, 290);
	 myGLCD.print(String(dPump2Value),  365, 290);
	 myGLCD.print("dPump3Pin",  10, 310);
	 myGLCD.print(String(dPump3Value),  365, 310);
	 myGLCD.print("coolPin",  10, 330);
	 myGLCD.print(String(coolValue),  365, 330);
	 myGLCD.print("lightPwmValue",  10, 370);
	 myGLCD.print(String(lightPwmValue*100/255),  330, 370);
	 myGLCD.print(F("%"),  375, 370);
	 */
}

void waitForIt(int x1, int y1, int x2, int y2) // Draw a red frame while a button is touched
		{ /*
		 myGLCD.setColor(255, 0, 0);
		 myGLCD.drawRoundRect (x1, y1, x2, y2);
		 while (myTouch.dataAvailable())
		 myTouch.read();
		 //  x=myTouch.getX();
		 //  y=myTouch.getY();
		 myGLCD.setColor(0, 0, 0);
		 myGLCD.drawRoundRect (x1, y1, x2, y2);
		 //drawScreen();
		 */
}

void actOnRealease(int x1, int y1, int x2, int y2) // Draw a red frame while a button is touched
		{
	myGLCD.setColor(255, 0, 0);
	myGLCD.drawRoundRect(x1, y1, x2, y2);
	while (myTouch.dataAvailable())
		myTouch.read();
	x = myTouch.getX();
	y = myTouch.getY();

	myGLCD.setColor(0, 0, 0);
	myGLCD.drawRoundRect(x1, y1, x2, y2);
	drawScreen();

}

/*
 * 
 * PROCESS
 * 
 */

/*  METHODS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 eigene Methoden zur Berechnung und AusfÃ¼rhung
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

void beepNow(int frequency, unsigned long millisToBeep, int count) {
	beep = true;
	counter = 0;
	freq = frequency;
	milliSecondsToBeep = millisToBeep;
	repeat = count;
}

void beeper()

{
	if (counter < repeat) {
		if (millis() - beepWait >= 500) {
			{
				int beepcount = 0;
				unsigned long previousBeepMillis;
				byte highlow = 0;
				boolean beepBool = false;
				unsigned long startmillis = 0;
				beepBool = true;
				startmillis = millis();

				while (beepBool) {
					if (beepcount < ((100000 * milliSecondsToBeep) / freq)) {
						if ((micros() - previousBeepMillis) >= freq) {
							highlow = (highlow + 128) % 256;
							analogWrite(speakerPin, highlow); // Toggle the LED on Pin 13
							previousBeepMillis = micros();
							beepcount++;

						}
					} else {
						beepBool = false;
						beepcount = 0;

					}

				}
				counter++;
				beepWait = millis();
			}
		}

	} else {
		beep = false;
	}
}

void getDistance() {
	float uS = sonar.ping_median(6); // Send ping, get ping time in microseconds (uS).
	waterDistance = (uS / 57) - 5.2;
//waterDistance= ((uS * (331.3 + 0.606 * Temp)) / 20000)-1;  //microseconds * (331.3 + 0.606 * temp)) / 2;  minus 2 cm für Nullsetzung
	if (dispScreen == 2) {
		myGLCD.setColor(col_red.r, col_red.g, col_red.b);
		myGLCD.print(F("Water missing:"), 250, 410);
		myGLCD.print(F("l"), 335, 430);
		myGLCD.print(F("% ("), 320, 450);
		myGLCD.print(F("cm)"), 425, 450);
		myGLCD.setColor(col_white.r, col_white.g, col_white.b);
		myGLCD.printNumF(waterDistance * 9.405, 1, 250, 430, '.', 5); //1 cm = 9,405 Liter  - 47,5 (innen) x 198 (innen)  * 57 (unterkante glassteg)
		myGLCD.printNumF(waterDistance * 1.754, 1, 250, 450, '.', 4);
//myGLCD.printNumF(uS, 1, 250, 450,'.',4);
//myGLCD.printNumI(int(waterDistance), 370, 450,2);
		myGLCD.printNumF(waterDistance, 1, 377, 450, '.', 3);
	}

	if (waterDistance > highestWaterDistance) {
		highestWaterDistance = waterDistance;
	}
	if (cleaningInProcess && waterDistance < highestWaterDistance - 1
			&& beepActive) {
		if (waterDistance < lastWaterDistance) {
			beepNow(200, 1, waterDistance);
			lastWaterDistance = waterDistance;
		} else if (waterDistance < 1) {
			beepNow(200, 1, 1);

		}

	}

}

void processRF() { //Aquaduino.print(".");

	//who is WHO:
	/*
	 f11on //Pump1 ON
	 f24on  //Pump2 ON
	 f13on //MainLight ON
	 f21on    //Heater ON
	 f22on    //CO2 ON
	 f23on //Coolpump ON

	 */
	//erst funken
	//dann schalten

	///XXXXXXXXXX
	//VALUES ARE ALSO USED IN THE POWERSCREEN()
	//XXXXXXXX
	if (!light230Value) {
		mySwitch.send(f13on, 24);
	} else {
		mySwitch.send(f13off, 24);
	}

	if (!heaterValue) {
		mySwitch.send(f21on, 24);
	} else {
		mySwitch.send(f21off, 24);
	}
	if (!co2Value) {
		mySwitch.send(f22on, 24);
	} else {
		mySwitch.send(f22off, 24);
	}
	/**  if (!coolValue)
	 {
	 mySwitch.send(f23on, 24);
	 }
	 else
	 {
	 mySwitch.send(f23off, 24);
	 }*/
	processRelais();

	// sx1509.writePin(co2Pin, co2Value);
	// sx1509.writePin(heaterPin, heaterValue);

	// sx1509.writePin(coolPin, coolValue);
	changeRF = false;
}

void processRelais() {
	sx1509.writePin(light1Pin, light1Value);
	sx1509.writePin(light2Pin, light2Value);
	sx1509.writePin(dPump1Pin, dPump1Value);
	sx1509.writePin(dPump2Pin, dPump2Value);
	sx1509.writePin(dPump3Pin, dPump3Value);
	sx1509.writePin(fanPin, coolValue);
}
void processPump() //2 times cause sometimes it doesnt switch
{
	if (!pump1Value) {
		mySwitch.send(f11on, 24);
	} else {
		mySwitch.send(f11off, 24);
	}
	//delay(100);  v.1.7
	if (!pump2Value) {
		mySwitch.send(f42on, 24);
	} else {
		mySwitch.send(f42off, 24);
	}
}

void UpdateClockAndLight() {
	now = rtc.now();
	//calculatedPWM=calculatedPWM+255;
	analogWrite(lightPwmPin, calculatedPWM);
	if (dispScreen != 141 && dispScreen != 142 && dispScreen != 143
			&& dispScreen != 144 && dispScreen != 145 && dispScreen != 146
			&& dispScreen != 16) {
		analogWrite(redPin, calculatedRed);
		analogWrite(greenPin, calculatedGreen);
		analogWrite(bluePin, calculatedBlue);
	}
}

void processRFInput() {
	switch (mySwitch.getReceivedValue()) {
	case f44on: {
		TVMode();
		lightCalculator();
		UpdateClockAndLight();
		AI();
		break;
	}
	case f44off: {
		TVModeState = false;
		lightCalculator();
		UpdateClockAndLight();
		AI();
		break;
	}
	case f43on: {
		MoonMode();
		lightCalculator();
		UpdateClockAndLight();
		AI();
		break;
	}
	case f43off: {
		MoonModeState = false;
		lightCalculator();
		UpdateClockAndLight();
		AI();
		break;
	}
	}
}

/* void SerialOutput()    //for debugging - not called
 {


 Serial.println("XXXXXX Serial Output XXXXXXXXX");
 Serial.print("Time: ");

 Serial.print("PH: ");
 Serial.print(PhWert);
 Serial.print("          Temp:  ");
 Serial.println(Temp);
 Serial.println();



 }
 */
void serialEvent() {
	inputstring = Serial.readStringUntil(13);
	input_stringcomplete = true;
}

void serialEvent3() {
	PhWertString = Serial3.readStringUntil(13);
	sensor_stringcomplete = true;
}

void GetTemperature() {
	sensors.requestTemperatures();
	Temp = sensors.getTempCByIndex(0);
}

void MoonMode() {
	if (!MoonModeState) {
		MoonEnd = now.unixtime() + (60 * MoonMinutes);
	}
	MoonModeState = true;
	light230Value = false;

	processRF();
	lightCalculator();
	UpdateClockAndLight();
	drawScreen();
}

void CleanMode() {
	if (!cleaningInProcess) {
		cleanEnd = now.unixtime() + (60 * cleanMinutes);
	}
	beepActive = true;
	lastWaterDistance = waterDistance;
	highestWaterDistance = 0;
	lastWaterDistance = 60;
	cleaningInProcess = true;
	calculatedPWM = 229.5;
	pump1Value = pump1Clean;
	pump2Value = pump2Clean;
	light230Value = light230Clean;
	light1Value = light2Clean;
	light2Value = light2Clean;
	co2Value = co2Clean;
	heaterValue = heaterClean;
	coolValue = coolClean;
	drawScreen();
	processRF();
	processPump();

}

void writeToRingBuffer() {
	TempValues[put_TempIndex] = Temp;
	put_TempIndex = (put_TempIndex + 1) % 96;

	PHValues[put_PHindex] = PhWert;
	Co2Values[put_PHindex] = co2Value;
	put_PHindex = (put_PHindex + 1) % 96;
}

void getPHValue() {
	if (input_stringcomplete) { //if a string from the PC has been received in its entirety
		if (inputstring.equalsIgnoreCase("calibrateON")) {
			Serial.println(F("Calibration ON"));
			calibrate = true;
		} else if (inputstring.equalsIgnoreCase("calibrateOFF")) {
			Serial.println(F("Calibration OFF"));
			calibrate = false;
		}
		Serial3.print(inputstring); //send that string to the Atlas Scientific product
		Serial3.print('\r');
		inputstring = "";                                    //clear the string:
		input_stringcomplete = false; //reset the flag used to tell if we have received a completed string from the PC
	}

	if (sensor_stringcomplete) { //if a string from the Atlas Scientific product has been received in its entierty

		if (isdigit(PhWertString[0])) {
			PhWert = PhWertString.toFloat();
			if (calibrate) {
				Serial.println(PhWert);
			}
		}

		/*
		 char FloatBufferForPH[5];
		 PhWertString.toCharArray(FloatBufferForPH, sizeof(FloatBufferForPH));
		 PhWert = atof(FloatBufferForPH);      //String is ready - set the PhWert

		 ONLY UNCOMMENT FOR PH DEBUGGING
		 Serial.println(PhWertString);
		 Serial.println(PhWert);
		 */
		PhWertString = "";        //clear the string:

		sensor_stringcomplete = false; //reset the flag used to tell if we have received a completed string from the Atlas Scientific product
	}

}

void TVMode() //to start the TV Mode. the TV Mode resets as soon as the CalculatedPWM hit 0
{
	TVModeStart = now;
	TVModeState = true;
	//TVModeBrightness is the Point Brightness starts
}

void lightCalculator() {
	if (!manualOverride && !cleaningInProcess) {
		TimeSpan helpSpan = now.unixtime() - (now.unixtime() - 86400 * 7); //set it to 7 days as fallback
		timeSinceLastLight = now.unixtime() - (now.unixtime() + 86400 * 7); //set it to -7 days as fallback
		timeToNextLight = now.unixtime() - (now.unixtime() - 86400 * 7); //set it to 7 days as fallback
		float oldPWM = 255;
		float newPWM = 255;

		//for RGB
		TimeSpan helpSpanRGB = now.unixtime() - (now.unixtime() - 86400 * 7); //set it to 7 days as fallback
		timeSinceLastLightRGB = now.unixtime() - (now.unixtime() + 86400 * 7); //set it to -7 days as fallback
		timeToNextLightRGB = now.unixtime() - (now.unixtime() - 86400 * 7); //set it to 7 days as fallback
		float oldRed = 0;
		float newRed = 0;
		float oldBlue = 0;
		float newBlue = 0;
		float oldGreen = 0;
		float newGreen = 0;

		//int helpSpanSeconds;
		for (int i = 0; i < 12; i++) {
			DateTime helpDTRGB(now.year(), now.month(), now.day(),
					int(lightRGB[i].Hour), int(lightRGB[i].Minute), 0);
			helpSpanRGB = helpDTRGB - now;
			DateTime helpDT(now.year(), now.month(), now.day(),
					int(lightPWM[i].Hour), int(lightPWM[i].Minute), 0);
			helpSpan = helpDT - now;

			//RGB

			if (timeToNextLightRGB.totalseconds() > helpSpanRGB.totalseconds()
					&& int(helpSpanRGB.totalseconds() >= 0)) {
				timeToNextLightRGB = helpSpanRGB;
				newRed = int(lightRGB[i].red);
				newGreen = int(lightRGB[i].green);
				newBlue = int(lightRGB[i].blue);
			}
			if (timeSinceLastLightRGB.totalseconds()
					< helpSpanRGB.totalseconds()
					&& int(helpSpanRGB.totalseconds() < 0)) {
				timeSinceLastLightRGB = helpSpanRGB;
				oldRed = int(lightRGB[i].red);
				oldGreen = int(lightRGB[i].green);
				oldBlue = int(lightRGB[i].blue);
			}

			//PWM for white light

			if (!TVModeState) {
				if (timeToNextLight.totalseconds() > helpSpan.totalseconds()
						&& int(helpSpan.totalseconds() >= 0)) {
					timeToNextLight = helpSpan;
					newPWM = int(lightPWM[i].pwmValue);
				}
				if (timeSinceLastLight.totalseconds() < helpSpan.totalseconds()
						&& int(helpSpan.totalseconds() < 0)) {
					timeSinceLastLight = helpSpan;
					oldPWM = int(lightPWM[i].pwmValue);
				}
			} else {
				if (timeToNextLight.totalseconds() > helpSpan.totalseconds()
						&& int(helpSpan.totalseconds() >= 0)
						&& int(lightPWM[i].pwmValue) < 3) //find the first point with 0 light  (<2)
								{
					timeToNextLight = helpSpan;
					newPWM = 0;
				}
				if (timeSinceLastLight.totalseconds() < helpSpan.totalseconds()
						&& int(helpSpan.totalseconds() < 0)) {
					timeSinceLastLight = helpSpan;
					if (oldPWM < 2) {
						oldPWM = int(lightPWM[i].pwmValue);
					} else {
						oldPWM = TVModeBrightness;
					}

					timeSinceLastLight = TVModeStart - now;
				}

				if (calculatedPWM < 3) //disable TVMode once hitting 0
						{
					TVModeState = false;
				}
			}

			//RGB
			if (!MoonModeState && !manualOverride) {
				calculatedRed =
						oldRed
								+ (int(
										((oldRed - newRed)
												/ ((timeToNextLightRGB.totalseconds())
														+ abs(
																timeSinceLastLightRGB.totalseconds())))
												* timeSinceLastLightRGB.totalseconds()));
				calculatedGreen =
						oldGreen
								+ (int(
										((oldGreen - newGreen)
												/ ((timeToNextLightRGB.totalseconds())
														+ abs(
																timeSinceLastLightRGB.totalseconds())))
												* timeSinceLastLightRGB.totalseconds()));
				calculatedBlue =
						oldBlue
								+ (int(
										((oldBlue - newBlue)
												/ ((timeToNextLightRGB.totalseconds())
														+ abs(
																timeSinceLastLightRGB.totalseconds())))
												* timeSinceLastLightRGB.totalseconds()));
				calculatedPWM =
						oldPWM
								+ (int(
										((oldPWM - newPWM)
												/ ((timeToNextLight.totalseconds())
														+ abs(
																timeSinceLastLight.totalseconds())))
												* timeSinceLastLight.totalseconds()));
			} else if (MoonModeState) {
				calculatedRed = MoonRed;
				calculatedGreen = MoonGreen;
				calculatedBlue = MoonBlue;
				calculatedPWM = 0;
			}

			//white

			/*
			 if (calculatedPWM > 90) //over 35% light - coolpump on
			 {
			 coolValue = false;
			 }
			 else
			 {
			 coolValue = true;
			 }
			 */

			/*

			 Serial.print("i = ");
			 Serial.print(i);
			 Serial.print("   ");
			 Serial.print(helpSpan.days(), DEC);
			 Serial.print("d    ");
			 Serial.print(helpSpan.hours(), DEC);
			 Serial.print(':');
			 Serial.print(helpSpan.minutes(), DEC);
			 Serial.print("      PWM:");
			 Serial.println(lightPWM[i].pwmValue);


			 Serial.print("Shortest TIME:    ");
			 Serial.print("   ");
			 Serial.print(timeToNextLight.days(), DEC);
			 Serial.print("d    ");
			 Serial.print(timeToNextLight.hours(), DEC);
			 Serial.print(':');
			 Serial.println(timeToNextLight.minutes(), DEC);

			 Serial.print("Shortest NEGATIVE TIME:    ");
			 Serial.print("   ");
			 Serial.print(timeSinceLastLight.days(), DEC);
			 Serial.print("d    ");
			 Serial.print(timeSinceLastLight.hours(), DEC);
			 Serial.print(':');
			 Serial.println(timeSinceLastLight.minutes(), DEC);

			 Serial.print("calculated PWM:    ");
			 Serial.println(calculatedPWM);



			 Serial.println(i);
			 Serial.print(timeToNextLight.days(), DEC);
			 Serial.print(' ');
			 Serial.print(timeToNextLight.hours(), DEC);
			 Serial.print(':');
			 Serial.println(timeToNextLight.minutes(), DEC);
			 Serial.println(currentPWM);
			 */
		}
	} else  //if manualoverride is active - turn off the RGB light
	{/*
	 calculatedRed = 0;
	 calculatedGreen = 0;
	 calculatedBlue = 0;
	 */
	}

}
/*
 lightPWM[0].startHour=21;
 lightPWM[0].startMinute=0;
 lightPWM[0].stopHour=6;
 lightPWM[0].stopMinute=40;
 lightPWM[0].pwmValue=255;

 */

void AI() {
	wdt_reset();

	if (!manualOverride && !cleaningInProcess && !MoonModeState) {
		DateTime CompareLightOnTime(now.year(), now.month(), now.day(),
				int(powLightOnHour), int(powLightOnMinute), 0);
		DateTime CompareLightOffTime(now.year(), now.month(), now.day(),
				int(powLightOffHour), int(powLightOffMinute), 0);
		DateTime CompareScreenOffTime(now.year(), now.month(), now.day(),
				int(screenOnHour), int(screenOnMinute), 0);
		DateTime CompareScreenOnTime(now.year(), now.month(), now.day(),
				int(screenOffHour), int(screenOffMinute), 0);
		pump1Value = false;
		pump2Value = false;
		processPump();
		if (Temp >= TempUpperLimit + 1) {
			if (coolValue) {
				coolValue = false;
				changeRF = true;
			}
		} else if (Temp < TempUpperLimit + 0.5) {
			if (!coolValue) {
				coolValue = true;
				changeRF = true;
			}
		}

		//turn lights on or off
		if (now.unixtime() > CompareLightOnTime.unixtime()
				&& now.unixtime() < CompareLightOffTime.unixtime()) {
			light230Value = false;
			light1Value = false;
			light2Value = false;
		} else {
			light230Value = true;
			light1Value = true;
			light2Value = true;
		}
		//turn Heater on or off
		if (Temp < TempLowerLimit) {
			heaterValue = false;
		} else if (TempUpperLimit < Temp) {
			heaterValue = true;
		}

		processRF();
		updateHomeScreen();
	}

	else if (!manualOverride && (now.unixtime() >= cleanEnd.unixtime())
			&& !MoonModeState) {
		cleaningInProcess = false;
		pump1Value = false;
		pump2Value = false;
		processPump();
		dispScreen = 0;
		drawScreen();
		AI();
		//processRF();
		lightCalculator();
		processPump();
	} else if (now.unixtime() >= MoonEnd.unixtime()) {
		MoonModeState = false;
	}

}

void processPH() //turn CO2 ON or OFF
{
	DateTime CompareCO2OnTime(now.year(), now.month(), now.day(),
			int(powCo2OnHour), int(powCo2OnMinute), 0);
	DateTime CompareCO2OffTime(now.year(), now.month(), now.day(),
			int(powCo2OffHour), int(powCo2OffMinute), 0);

	if (now.unixtime() > CompareCO2OnTime.unixtime()
			&& now.unixtime() < CompareCO2OffTime.unixtime()) {
		if (PhWert > PHUpperLimit) {
			co2Value = false;
		} else if (PhWert < PHLowerLimit) {
			co2Value = true;
		}
	} else {
		co2Value = true;
	}
	processRF();
}
void writeFile() {
	Aquaduino = SD.open("aqua.txt", FILE_WRITE);
	/*    if (Aquaduino)
	 { Aquaduino.print(now.day(), DEC);
	 Aquaduino.print(".");
	 Aquaduino.print(now.month(), DEC);
	 Aquaduino.print(".");
	 Aquaduino.print(now.year(), DEC);
	 Aquaduino.print(";");
	 Aquaduino.print(now.hour(), DEC);
	 Aquaduino.print(':');
	 Aquaduino.print(now.minute(), DEC);
	 Aquaduino.print(':');
	 Aquaduino.print(now.second(), DEC);
	 Aquaduino.print(";");
	 Aquaduino.print(Temp);
	 Aquaduino.print(";");
	 Aquaduino.print(PhWert);
	 Aquaduino.print(";");
	 Aquaduino.print(calculatedPWM);
	 Aquaduino.print(";");
	 Aquaduino.print(pump1Value);
	 Aquaduino.print(";");
	 Aquaduino.print(pump2Value);
	 Aquaduino.print(";");
	 Aquaduino.print(light230Value);
	 Aquaduino.print(";");
	 Aquaduino.print(light1Value);
	 Aquaduino.print(";");
	 Aquaduino.print(light2Value);
	 Aquaduino.print(";");
	 Aquaduino.print(co2Value);
	 Aquaduino.print(";");
	 Aquaduino.print(heaterValue);
	 Aquaduino.print(";");
	 Aquaduino.print(dPump1Value);
	 Aquaduino.print(";");
	 Aquaduino.print(dPump2Value);
	 Aquaduino.print(";");
	 Aquaduino.print(dPump3Value);
	 Aquaduino.print(";");
	 Aquaduino.print(coolValue);
	 Aquaduino.println(";");
	 //Aquaduino.println();
	 Aquaduino.close();
	 }*/
}
/*
 void writeFertToCard()
 {Aquaduino = SD.open("fert.txt", FILE_WRITE);
 if (Aquaduino)
 {  Aquaduino.print(lastFert.day(), DEC);
 Aquaduino.print(".");
 Aquaduino.print(lastFert.month(), DEC);
 Aquaduino.print(".");
 Aquaduino.print(lastFert.year(), DEC);
 Aquaduino.print(";");
 Aquaduino.print(lastFert.hour(), DEC);
 Aquaduino.print(':');
 Aquaduino.print(lastFert.minute(), DEC);
 Aquaduino.print(':');
 Aquaduino.print(lastFert.second(), DEC);
 Aquaduino.print(";");
 }
 }




 */

void fertilize() {
	if (now.unixtime() - lastFert.unixtime() > 82800) // no fertilizing for 23 hours (82800) - just to be sure
			{
		if ((now.hour() == doseHour) && (now.minute() >= doseMinute)
				&& (now.minute() <= doseMinute + 10)) //timeframe of 10 Minutes to fertilize - in case that another task runs at the exact dose time
				{
			lastFert = now.unixtime();
			Aquaduino = SD.open("fert.txt", FILE_WRITE);
			if (Aquaduino)   //write to file
			{
				Aquaduino.print(lastFert.day(), DEC);
				Aquaduino.print(".");
				Aquaduino.print(lastFert.month(), DEC);
				Aquaduino.print(".");
				Aquaduino.print(lastFert.year(), DEC);
				Aquaduino.print(";");
				Aquaduino.print(lastFert.hour(), DEC);
				Aquaduino.print(':');
				Aquaduino.print(lastFert.minute(), DEC);
				Aquaduino.print(':');
				Aquaduino.print(lastFert.second(), DEC);
				Aquaduino.print("; ");
				Aquaduino.print("Fertilizing ");
			}
			fertmillis = millis();

			//fertilize N
			if (dayN[now.dayOfWeek()] && (FLeft[0] - FDose[0] >= 0)) {
				Aquaduino.print("N:");
				Aquaduino.print(FDose[0]);
				FLeft[0] = FLeft[0] - FDose[0];
				fertmillis = millis();

				wdt_disable();

				while (millis() - fertmillis < 1000 * FDose[0] / (FRate[0] / 60)) {
					dPump1Value = false;
					processRF();
					if (dispScreen < 1) {
						myFiles.load(393, 449, 48, 48, "1nN.raw");
					}
				}
				dPump1Value = true;
				processRF();

				wdt_enable(WDTO_8S);
				updateHomeScreen();
			}
			//fertilize NPK
			if (dayNPK[now.dayOfWeek()] && (FLeft[1] - FDose[1] >= 0)) {
				Aquaduino.print("  NPK: ");
				Aquaduino.print(FDose[1]);
				FLeft[1] = FLeft[1] - FDose[1];
				fertmillis = millis();
				wdt_disable();
				while (millis() - fertmillis < 1000 * FDose[1] / (FRate[1] / 60)) {
					dPump2Value = false;
					processRF();
					if (dispScreen < 1) {
						myFiles.load(340, 502, 48, 48, "1npkN.raw");
					}
				}
				dPump2Value = true;
				processRF();
				wdt_enable(WDTO_8S);
				updateHomeScreen();
			}
			//fertilize FE
			if (dayFE[now.dayOfWeek()] && (FLeft[2] - FDose[2] >= 0)) {
				Aquaduino.print("  FE: ");
				Aquaduino.print(FDose[2]);
				FLeft[2] = FLeft[2] - FDose[2];
				fertmillis = millis();
				wdt_disable();
				while (millis() - fertmillis < 1000 * FDose[2] / (FRate[2] / 60)) {
					dPump3Value = false;
					processRF();
					if (dispScreen < 1) {
						myFiles.load(393, 502, 48, 48, "1feN.raw");
					}
				}
				dPump3Value = true;
				processRF();
				wdt_enable(WDTO_8S);
				updateHomeScreen();

			}
			Aquaduino.println();
			Aquaduino.close();
			drawFertilizer();
			saveFerti();

		}
	}
}

void readLightPWM() {
	lightPWM[0].Hour = EEPROM.read(20);
	lightPWM[0].Minute = EEPROM.read(21);
	lightPWM[0].pwmValue = EEPROM.read(22);
	lightPWM[1].Hour = EEPROM.read(23);
	lightPWM[1].Minute = EEPROM.read(24);
	lightPWM[1].pwmValue = EEPROM.read(25);
	lightPWM[2].Hour = EEPROM.read(26);
	lightPWM[2].Minute = EEPROM.read(27);
	lightPWM[2].pwmValue = EEPROM.read(28);
	lightPWM[3].Hour = EEPROM.read(29);
	lightPWM[3].Minute = EEPROM.read(30);
	lightPWM[3].pwmValue = EEPROM.read(31);
	lightPWM[4].Hour = EEPROM.read(32);
	lightPWM[4].Minute = EEPROM.read(33);
	lightPWM[4].pwmValue = EEPROM.read(34);
	lightPWM[5].Hour = EEPROM.read(35);
	lightPWM[5].Minute = EEPROM.read(36);
	lightPWM[5].pwmValue = EEPROM.read(37);
	lightPWM[6].Hour = EEPROM.read(38);
	lightPWM[6].Minute = EEPROM.read(39);
	lightPWM[6].pwmValue = EEPROM.read(40);
	lightPWM[7].Hour = EEPROM.read(41);
	lightPWM[7].Minute = EEPROM.read(42);
	lightPWM[7].pwmValue = EEPROM.read(43);
	lightPWM[8].Hour = EEPROM.read(44);
	lightPWM[8].Minute = EEPROM.read(45);
	lightPWM[8].pwmValue = EEPROM.read(46);
	lightPWM[9].Hour = EEPROM.read(47);
	lightPWM[9].Minute = EEPROM.read(48);
	lightPWM[9].pwmValue = EEPROM.read(49);
	lightPWM[10].Hour = EEPROM.read(50);
	lightPWM[10].Minute = EEPROM.read(51);
	lightPWM[10].pwmValue = EEPROM.read(52);
	lightPWM[11].Hour = EEPROM.read(53);
	lightPWM[11].Minute = EEPROM.read(54);
	lightPWM[11].pwmValue = EEPROM.read(55);
}

void saveLightPWM() {
	EEPROM.write(20, lightPWM[0].Hour);
	EEPROM.write(21, lightPWM[0].Minute);
	EEPROM.write(22, byte(lightPWM[0].pwmValue));
	EEPROM.write(23, lightPWM[1].Hour);
	EEPROM.write(24, lightPWM[1].Minute);
	EEPROM.write(25, byte(lightPWM[1].pwmValue));
	EEPROM.write(26, lightPWM[2].Hour);
	EEPROM.write(27, lightPWM[2].Minute);
	EEPROM.write(28, byte(lightPWM[2].pwmValue));
	EEPROM.write(29, lightPWM[3].Hour);
	EEPROM.write(30, lightPWM[3].Minute);
	EEPROM.write(31, byte(lightPWM[3].pwmValue));
	EEPROM.write(32, lightPWM[4].Hour);
	EEPROM.write(33, lightPWM[4].Minute);
	EEPROM.write(34, byte(lightPWM[4].pwmValue));
	EEPROM.write(35, lightPWM[5].Hour);
	EEPROM.write(36, lightPWM[5].Minute);
	EEPROM.write(37, byte(lightPWM[5].pwmValue));
	EEPROM.write(38, lightPWM[6].Hour);
	EEPROM.write(39, lightPWM[6].Minute);
	EEPROM.write(40, byte(lightPWM[6].pwmValue));
	EEPROM.write(41, lightPWM[7].Hour);
	EEPROM.write(42, lightPWM[7].Minute);
	EEPROM.write(43, byte(lightPWM[7].pwmValue));
	EEPROM.write(44, lightPWM[8].Hour);
	EEPROM.write(45, lightPWM[8].Minute);
	EEPROM.write(46, byte(lightPWM[8].pwmValue));
	EEPROM.write(47, lightPWM[9].Hour);
	EEPROM.write(48, lightPWM[9].Minute);
	EEPROM.write(49, byte(lightPWM[9].pwmValue));
	EEPROM.write(50, lightPWM[10].Hour);
	EEPROM.write(51, lightPWM[10].Minute);
	EEPROM.write(52, byte(lightPWM[10].pwmValue));
	EEPROM.write(53, lightPWM[11].Hour);
	EEPROM.write(54, lightPWM[11].Minute);
	EEPROM.write(55, byte(lightPWM[11].pwmValue));

}

void readMoonMode() {
	MoonRed = EEPROM.read(172);
	MoonGreen = EEPROM.read(173);
	MoonBlue = EEPROM.read(174);
	MoonMinutes = EEPROM.read(175);
}

void saveMoonMode() {
	EEPROM.write(172, MoonRed);
	EEPROM.write(173, MoonGreen);
	EEPROM.write(174, MoonBlue);
	EEPROM.write(175, MoonMinutes);

}

void readTVMode() {
	TVModeBrightness = EEPROM.read(176);

}

void saveTVMode() {
	EEPROM.write(176, TVModeBrightness);

}

void readLightRGB() {
	lightRGB[0].Hour = EEPROM.read(112);
	lightRGB[0].Minute = EEPROM.read(113);
	lightRGB[0].red = EEPROM.read(114);
	lightRGB[0].green = EEPROM.read(115);
	lightRGB[0].blue = EEPROM.read(116);
	lightRGB[1].Hour = EEPROM.read(117);
	lightRGB[1].Minute = EEPROM.read(118);
	lightRGB[1].red = EEPROM.read(119);
	lightRGB[1].green = EEPROM.read(120);
	lightRGB[1].blue = EEPROM.read(121);
	lightRGB[2].Hour = EEPROM.read(122);
	lightRGB[2].Minute = EEPROM.read(123);
	lightRGB[2].red = EEPROM.read(124);
	lightRGB[2].green = EEPROM.read(125);
	lightRGB[2].blue = EEPROM.read(126);
	lightRGB[3].Hour = EEPROM.read(127);
	lightRGB[3].Minute = EEPROM.read(128);
	lightRGB[3].red = EEPROM.read(129);
	lightRGB[3].green = EEPROM.read(130);
	lightRGB[3].blue = EEPROM.read(131);
	lightRGB[4].Hour = EEPROM.read(132);
	lightRGB[4].Minute = EEPROM.read(133);
	lightRGB[4].red = EEPROM.read(134);
	lightRGB[4].green = EEPROM.read(135);
	lightRGB[4].blue = EEPROM.read(136);
	lightRGB[5].Hour = EEPROM.read(137);
	lightRGB[5].Minute = EEPROM.read(138);
	lightRGB[5].red = EEPROM.read(139);
	lightRGB[5].green = EEPROM.read(140);
	lightRGB[5].blue = EEPROM.read(141);
	lightRGB[6].Hour = EEPROM.read(142);
	lightRGB[6].Minute = EEPROM.read(143);
	lightRGB[6].red = EEPROM.read(144);
	lightRGB[6].green = EEPROM.read(145);
	lightRGB[6].blue = EEPROM.read(146);
	lightRGB[7].Hour = EEPROM.read(147);
	lightRGB[7].Minute = EEPROM.read(148);
	lightRGB[7].red = EEPROM.read(149);
	lightRGB[7].green = EEPROM.read(150);
	lightRGB[7].blue = EEPROM.read(151);
	lightRGB[8].Hour = EEPROM.read(152);
	lightRGB[8].Minute = EEPROM.read(153);
	lightRGB[8].red = EEPROM.read(154);
	lightRGB[8].green = EEPROM.read(155);
	lightRGB[8].blue = EEPROM.read(156);
	lightRGB[9].Hour = EEPROM.read(157);
	lightRGB[9].Minute = EEPROM.read(158);
	lightRGB[9].red = EEPROM.read(159);
	lightRGB[9].green = EEPROM.read(160);
	lightRGB[9].blue = EEPROM.read(161);
	lightRGB[10].Hour = EEPROM.read(162);
	lightRGB[10].Minute = EEPROM.read(163);
	lightRGB[10].red = EEPROM.read(164);
	lightRGB[10].green = EEPROM.read(165);
	lightRGB[10].blue = EEPROM.read(166);
	lightRGB[11].Hour = EEPROM.read(167);
	lightRGB[11].Minute = EEPROM.read(168);
	lightRGB[11].red = EEPROM.read(168);
	lightRGB[11].green = EEPROM.read(170);
	lightRGB[11].blue = EEPROM.read(171);
}

void saveLightRGB() {
	EEPROM.write(112, lightRGB[0].Hour);
	EEPROM.write(113, lightRGB[0].Minute);
	EEPROM.write(114, lightRGB[0].red);
	EEPROM.write(115, lightRGB[0].green);
	EEPROM.write(116, lightRGB[0].blue);
	EEPROM.write(117, lightRGB[1].Hour);
	EEPROM.write(118, lightRGB[1].Minute);
	EEPROM.write(119, lightRGB[1].red);
	EEPROM.write(120, lightRGB[1].green);
	EEPROM.write(121, lightRGB[1].blue);
	EEPROM.write(122, lightRGB[2].Hour);
	EEPROM.write(123, lightRGB[2].Minute);
	EEPROM.write(124, lightRGB[2].red);
	EEPROM.write(125, lightRGB[2].green);
	EEPROM.write(126, lightRGB[2].blue);
	EEPROM.write(127, lightRGB[3].Hour);
	EEPROM.write(128, lightRGB[3].Minute);
	EEPROM.write(129, lightRGB[3].red);
	EEPROM.write(130, lightRGB[3].green);
	EEPROM.write(131, lightRGB[3].blue);
	EEPROM.write(132, lightRGB[4].Hour);
	EEPROM.write(133, lightRGB[4].Minute);
	EEPROM.write(134, lightRGB[4].red);
	EEPROM.write(135, lightRGB[4].green);
	EEPROM.write(136, lightRGB[4].blue);
	EEPROM.write(137, lightRGB[5].Hour);
	EEPROM.write(138, lightRGB[5].Minute);
	EEPROM.write(139, lightRGB[5].red);
	EEPROM.write(140, lightRGB[5].green);
	EEPROM.write(141, lightRGB[5].blue);
	EEPROM.write(142, lightRGB[6].Hour);
	EEPROM.write(143, lightRGB[6].Minute);
	EEPROM.write(144, lightRGB[6].red);
	EEPROM.write(145, lightRGB[6].green);
	EEPROM.write(146, lightRGB[6].blue);
	EEPROM.write(147, lightRGB[7].Hour);
	EEPROM.write(148, lightRGB[7].Minute);
	EEPROM.write(149, lightRGB[7].red);
	EEPROM.write(150, lightRGB[7].green);
	EEPROM.write(151, lightRGB[7].blue);
	EEPROM.write(152, lightRGB[8].Hour);
	EEPROM.write(153, lightRGB[8].Minute);
	EEPROM.write(154, lightRGB[8].red);
	EEPROM.write(155, lightRGB[8].green);
	EEPROM.write(156, lightRGB[8].blue);
	EEPROM.write(157, lightRGB[9].Hour);
	EEPROM.write(158, lightRGB[9].Minute);
	EEPROM.write(159, lightRGB[9].red);
	EEPROM.write(160, lightRGB[9].green);
	EEPROM.write(161, lightRGB[9].blue);
	EEPROM.write(162, lightRGB[10].Hour);
	EEPROM.write(163, lightRGB[10].Minute);
	EEPROM.write(164, lightRGB[10].red);
	EEPROM.write(165, lightRGB[10].green);
	EEPROM.write(166, lightRGB[10].blue);
	EEPROM.write(167, lightRGB[11].Hour);
	EEPROM.write(168, lightRGB[11].Minute);
	EEPROM.write(169, lightRGB[11].red);
	EEPROM.write(170, lightRGB[11].green);
	EEPROM.write(171, lightRGB[11].blue);
}

void readCleanSched() {
	pump1Clean = EEPROM.read(56);
	pump2Clean = EEPROM.read(57);
	light230Clean = EEPROM.read(58);
	light2Clean = EEPROM.read(59);
	co2Clean = EEPROM.read(60);
	heaterClean = EEPROM.read(61);
	coolClean = EEPROM.read(62);
	cleanMinutes = EEPROM.read(91);
}

void saveCleanSched() {
	EEPROM.write(56, pump1Clean);
	EEPROM.write(57, pump2Clean);
	EEPROM.write(58, light230Clean);
	EEPROM.write(59, light2Clean);
	EEPROM.write(60, co2Clean);
	EEPROM.write(61, heaterClean);
	EEPROM.write(62, coolClean);
	EEPROM.write(91, cleanMinutes);
}

void readPHValue() {
	PHUpperLimit = (EEPROM.read(63));
	PHLowerLimit = (EEPROM.read(64));
	PHUpperLimit = PHUpperLimit / 100 + 6;
	PHLowerLimit = PHLowerLimit / 100 + 6;
}

void savePHValue() {
	EEPROM.write(63, (PHUpperLimit - 6) * 100);
	EEPROM.write(64, (PHUpperLimit - 6) * 100);
}

void readTempValue() {
	TempUpperLimit = (EEPROM.read(65));
	TempLowerLimit = (EEPROM.read(66));

}

void saveTempValue() {
	EEPROM.write(65, TempUpperLimit);
	EEPROM.write(66, TempLowerLimit);
}

void readFerti() {
	FDose[0] = EEPROM.read(0);
	FDose[1] = EEPROM.read(1);
	FDose[2] = EEPROM.read(2);
	FMax[0] = (EEPROM.read(3) * 5); //max 1,2 Liter max amount
	FMax[1] = (EEPROM.read(4) * 5); //max 1,2 Liter max amount
	FMax[2] = (EEPROM.read(5) * 5); //max 1,2 Liter max amount
	FLeft[0] = (EEPROM.read(6) * 5); //max 1,2 Liter max amount
	FLeft[1] = (EEPROM.read(7) * 5); //max 1,2 Liter max amount
	FLeft[2] = (EEPROM.read(8) * 5); //max 1,2 Liter max amount
	FRate[0] = EEPROM.read(67);
	FRate[1] = EEPROM.read(68);
	FRate[2] = EEPROM.read(69);
	MoF[0] = EEPROM.read(70);
	MoF[1] = EEPROM.read(71);
	MoF[2] = EEPROM.read(72);
	TuF[0] = EEPROM.read(73);
	TuF[1] = EEPROM.read(74);
	TuF[2] = EEPROM.read(75);
	WeF[0] = EEPROM.read(76);
	WeF[1] = EEPROM.read(77);
	WeF[2] = EEPROM.read(78);
	ThF[0] = EEPROM.read(79);
	ThF[1] = EEPROM.read(80);
	ThF[2] = EEPROM.read(81);
	FrF[0] = EEPROM.read(82);
	FrF[1] = EEPROM.read(83);
	FrF[2] = EEPROM.read(84);
	SaF[0] = EEPROM.read(85);
	SaF[1] = EEPROM.read(86);
	SaF[2] = EEPROM.read(87);
	SuF[0] = EEPROM.read(88);
	SuF[1] = EEPROM.read(89);
	SuF[2] = EEPROM.read(90);
	doseHour = EEPROM.read(92);
	doseMinute = EEPROM.read(93);

	dayN[1] = MoF[0];
	dayN[2] = TuF[0];
	dayN[3] = WeF[0];
	dayN[4] = ThF[0];
	dayN[5] = FrF[0];
	dayN[6] = SaF[0];
	dayN[0] = SuF[0];

	dayNPK[1] = MoF[1];
	dayNPK[2] = TuF[1];
	dayNPK[3] = WeF[1];
	dayNPK[4] = ThF[1];
	dayNPK[5] = FrF[1];
	dayNPK[6] = SaF[1];
	dayNPK[0] = SuF[1];

	dayFE[1] = MoF[2];
	dayFE[2] = TuF[2];
	dayFE[3] = WeF[2];
	dayFE[4] = ThF[2];
	dayFE[5] = FrF[2];
	dayFE[6] = SaF[2];
	dayFE[0] = SuF[2];

}

void saveFerti() {
	EEPROM.write(0, FDose[0]);
	EEPROM.write(1, FDose[1]);
	EEPROM.write(2, FDose[2]);
	EEPROM.write(3, int(FMax[0] / 5));
	EEPROM.write(4, int(FMax[1] / 5));
	EEPROM.write(5, int(FMax[2] / 5));
	EEPROM.write(6, int(FLeft[0] / 5));
	EEPROM.write(7, int(FLeft[1] / 5));
	EEPROM.write(8, int(FLeft[2] / 5));
	EEPROM.write(67, FRate[0]);
	EEPROM.write(68, FRate[1]);
	EEPROM.write(69, FRate[2]);
	EEPROM.write(70, MoF[0]);
	EEPROM.write(71, MoF[1]);
	EEPROM.write(72, MoF[2]);
	EEPROM.write(73, TuF[0]);
	EEPROM.write(74, TuF[1]);
	EEPROM.write(75, TuF[2]);
	EEPROM.write(76, WeF[0]);
	EEPROM.write(77, WeF[1]);
	EEPROM.write(78, WeF[2]);
	EEPROM.write(79, ThF[0]);
	EEPROM.write(80, ThF[1]);
	EEPROM.write(81, ThF[2]);
	EEPROM.write(82, FrF[0]);
	EEPROM.write(83, FrF[1]);
	EEPROM.write(84, FrF[2]);
	EEPROM.write(85, SaF[0]);
	EEPROM.write(86, SaF[1]);
	EEPROM.write(87, SaF[2]);
	EEPROM.write(88, SuF[0]);
	EEPROM.write(89, SuF[1]);
	EEPROM.write(90, SuF[2]);
	EEPROM.write(92, doseHour);
	EEPROM.write(93, doseMinute);

	dayN[1] = MoF[0];
	dayN[2] = TuF[0];
	dayN[3] = WeF[0];
	dayN[4] = ThF[0];
	dayN[5] = FrF[0];
	dayN[6] = SaF[0];
	dayN[0] = SuF[0];

	dayNPK[1] = MoF[1];
	dayNPK[2] = TuF[1];
	dayNPK[3] = WeF[1];
	dayNPK[4] = ThF[1];
	dayNPK[5] = FrF[1];
	dayNPK[6] = SaF[1];
	dayNPK[0] = SuF[1];

	dayFE[1] = MoF[2];
	dayFE[2] = TuF[2];
	dayFE[3] = WeF[2];
	dayFE[4] = ThF[2];
	dayFE[5] = FrF[2];
	dayFE[6] = SaF[2];
	dayFE[0] = SuF[2];
}

void readReminder() { //tankCleanDay=EEPROM.read(94);
	/*tankCleanMonth=EEPROM.read(95);
	 co2BottleDay=EEPROM.read(96);
	 co2BottleMonth=EEPROM.read(97);
	 cleanFilter1Day=EEPROM.read(98);
	 cleanFilter1Month=EEPROM.read(99);
	 cleanFilter2Day=EEPROM.read(100);
	 cleanFilter2Month=EEPROM.read(101);
	 */
	tankClean = DateTime(now.year(), EEPROM.read(95), EEPROM.read(94),
			now.hour(), now.minute(), 0);
	co2Bottle = DateTime(now.year(), EEPROM.read(97), EEPROM.read(96),
			now.hour(), now.minute(), 0);
	cleanFilter1 = DateTime(now.year(), EEPROM.read(99), EEPROM.read(98),
			now.hour(), now.minute(), 0);
	cleanFilter2 = DateTime(now.year(), EEPROM.read(101), EEPROM.read(100),
			now.hour(), now.minute(), 0);
	tankCleandDays = EEPROM.read(102);
	co2BottleDays = EEPROM.read(103);
	cleanFilter1Days = EEPROM.read(104);
	cleanFilter2Days = EEPROM.read(105);
	// DateTime helpDT (now.year(),now.month(),now.day(),int(lightPWM[i].Hour),int(lightPWM[i].Minute),0);
}

void saveReminder() {
	EEPROM.write(94, tankClean.day());
	EEPROM.write(95, tankClean.month());
	EEPROM.write(96, co2Bottle.day());
	EEPROM.write(97, co2Bottle.month());
	EEPROM.write(98, cleanFilter1.day());
	EEPROM.write(99, cleanFilter1.month());
	EEPROM.write(100, cleanFilter2.day());
	EEPROM.write(101, cleanFilter2.month());
	EEPROM.write(102, tankCleandDays);
	EEPROM.write(103, co2BottleDays);
	EEPROM.write(104, cleanFilter1Days);
	EEPROM.write(105, cleanFilter2Days);

}

void readPowerSchedule() {
	powLightOnHour = EEPROM.read(9);
	powLightOnMinute = EEPROM.read(10);
	powLightOffHour = EEPROM.read(11);
	powLightOffMinute = EEPROM.read(12);
	powCo2OnHour = EEPROM.read(13);
	powCo2OnMinute = EEPROM.read(14);
	powCo2OffHour = EEPROM.read(15);
	powCo2OffMinute = EEPROM.read(16);
}

void savePowerSchedule() {
	EEPROM.write(9, powLightOnHour);
	EEPROM.write(10, powLightOnMinute);
	EEPROM.write(11, powLightOffHour);
	EEPROM.write(12, powLightOffMinute);
	EEPROM.write(13, powCo2OnHour);
	EEPROM.write(14, powCo2OnMinute);
	EEPROM.write(15, powCo2OffHour);
	EEPROM.write(16, powCo2OffMinute);
}

void readScreenScreen() {
	screenOnHour = EEPROM.read(106);
	screenOffHour = EEPROM.read(107);
	screenOnMinute = EEPROM.read(108);
	screenOffMinute = EEPROM.read(109);
	standByMinutes = EEPROM.read(110);
	backlightPWM = EEPROM.read(111);
}

void saveScreenScreen() {
	EEPROM.write(106, screenOnHour);
	EEPROM.write(107, screenOffHour);
	EEPROM.write(108, screenOnMinute);
	EEPROM.write(109, screenOffMinute);
	EEPROM.write(110, standByMinutes);
	EEPROM.write(111, backlightPWM);
}

/*
 * 
 * SERIAL IN OUT
 * 
 */

void recvWithStartEndMarkers() {
	static boolean recvInProgress = false;
	char startMarker = '<';
	char endMarker = '>';
	char rc;
	while (Serial2.available() > 0 && newData == false) {
		rc = Serial2.read();
		if (recvInProgress == true) {
			if (rc != endMarker) {
				receivedChars[ndx] = rc;
				ndx++;
				if (ndx >= numChars) {
					ndx = numChars - 1;
				}
			} else {
				receivedChars[ndx] = '\0'; // terminate the string
				recvInProgress = false;
				newData = true;
			}
		} else if (rc == startMarker) {
			recvInProgress = true;
		}
	}
}

void useNewData() {
	if (newData == true) {
		String StringFromSerial = "";
		StringFromSerial = receivedChars;
		//Serial.println("dbg: "+StringFromSerial);
		parseCommand(StringFromSerial);
		ndx = 0;
		newData = false;
	}
}

void sendCommand(String variable, String sendCom) {
	String sendThis = "<toESP|" + variable + "_" + sendCom + "|";
	int CheckSum = sendThis.length() - 1;
	sendThis += '*';
	if (CheckSum < 10) {
		sendThis += "00";
	} else if (CheckSum < 100) {
		sendThis += 0;
	}
	sendThis += CheckSum;
	sendThis += ">";
	char charBuf[650];
	sendThis.toCharArray(charBuf, 648);
	Serial2.println(charBuf);
# if debug
	{	Serial.print(F("sendCommand DebugLine: "));
		Serial.println(charBuf);
	}
#endif
}

void sendSerial(String sendCom) {
	String sendThis = "<" + sendCom + "|";
	int CheckSum = sendThis.length() - 1;
	sendThis += '*';
	if (CheckSum < 10) {
		sendThis += "00";
	} else if (CheckSum < 100) {
		sendThis += 0;
	}
	sendThis += CheckSum;
	sendThis += ">";
	char charBuf[650];
	sendThis.toCharArray(charBuf, 648);
	Serial2.println(charBuf);

# if debug
	{	Serial.print(F("sendSerial DebugLine: "));
		Serial.print(charBuf);
	}
#endif
	delay(10);
}

void parseCommand(String com) {
	int sentChecksum =
			(com.substring(com.indexOf("*") + 1, com.length())).toInt();
	int calculatedCheckSum = com.indexOf("*");
	if (sentChecksum == calculatedCheckSum) {
		String part1;
		// part1 = com.substring(0, com.indexOf("|"));
		if (com.substring(0, com.indexOf("|")).equalsIgnoreCase("toMega")) {
			for (int i = com.indexOf("|"); i < com.lastIndexOf("|");
					i = com.indexOf("|", i + 1)) {
				String part1 = com.substring(i + 1, com.indexOf("|", i + 1));
				int cmdID = findCommand(
						part1.substring(0, part1.indexOf(F("_"))).c_str());
				switch (cmdID) { //case tPhWert : {PhWert=(part1.substring(part1.indexOf(F("_"))+1,part1.length())).toFloat();  break;}
								 //case tTemp : {Temp=(part1.substring(part1.indexOf(F("_"))+1,part1.length())).toFloat();    break;}
				case tcalculatedPWM: {
					calculatedPWM = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toFloat();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedPWM));
					break;
				}
				case tcalculatedRed: {
					calculatedRed = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toFloat();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedRed));
					break;
				}
				case tcalculatedGreen: {
					calculatedGreen =
							(part1.substring(part1.indexOf(F("_")) + 1,
									part1.length())).toFloat();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedGreen));
					break;
				}
				case tcalculatedBlue: {
					calculatedBlue = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toFloat();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedBlue));
					break;
				}
				case tTVModeState: {
					if ((part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt()) {
						TVMode();
					} else {
						TVModeState = 0;
					}
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(TVModeState));
					break;
				}
				case tcleaningInProcess: {
					if ((part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt()) {
						CleanMode();
					} else {
						cleaningInProcess = 0;
					}
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(cleaningInProcess));
					break;
				}
				case tmanualOverride: {
					manualOverride = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(manualOverride));
					break;
				}
				case tMoonModeState: {
					if ((part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt()) {
						MoonMode();
					} else {
						MoonModeState = 0;
					}
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(MoonModeState));
					break;
				}
				case tpump1Value: {
					pump1Value = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(pump1Value));
					break;
				}
				case tpump2Value: {
					pump2Value = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(pump2Value));
					break;
				}
				case tlight230Value: {
					light230Value = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(light230Value));
					break;
				}
				case tlight1Value: {
					light1Value = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					light2Value = light1Value;
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(light1Value));
					break;
				}
					//case tlight2Value : {light2Value = (part1.substring(part1.indexOf(F("_")) + 1, part1.length())).toInt();sendCommand(part1.substring(0, part1.indexOf(F("_"))), String(light2Value));break;}
				case tco2Value: {
					co2Value = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(co2Value));
					break;
				}
				case theaterValue: {
					heaterValue = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(heaterValue));
					break;
				}
					// case tdPump1Value : {dPump1Value = (part1.substring(part1.indexOf(F("_")) + 1, part1.length())).toInt();sendCommand(part1.substring(0, part1.indexOf(F("_"))), String(dPump1Value));break;}
					//case tdPump2Value : {dPump2Value = (part1.substring(part1.indexOf(F("_")) + 1, part1.length())).toInt();sendCommand(part1.substring(0, part1.indexOf(F("_"))), String(dPump2Value));break;}
					// case tdPump3Value : {dPump3Value = (part1.substring(part1.indexOf(F("_")) + 1, part1.length())).toInt();sendCommand(part1.substring(0, part1.indexOf(F("_"))), String(dPump3Value));break;}
				case tcoolValue: {
					coolValue = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toInt();
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(coolValue));
					break;
				}
					//ase tnow : {rtc.adjust(DateTime);sendCommand(part1.substring(0, part1.indexOf(F("_"))), String(now.unixtime()));break;}
				case tnow: {
					now = now.unixtime() - now.unixtime()
							+ (part1.substring(part1.indexOf(F("_")) + 1,
									part1.length()).toInt());
					rtc.adjust(now.unixtime());
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(now.unixtime()));
					break;
				}
				case tpS: {
					UpdateClockAndLight();
					break;
				}
				case tpF: {
					processRF();
					break;
				}
				case tpR: {
					processRelais();
					break;
				}
				case tpB: {
					lightCalculator();
					AI();
					break;
				}
				case tpP: {
					processPump();
					break;
				}
				case tcalculatedPWMnF: {
					calculatedPWM = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toFloat();
					break;
				}
				case tcalculatedRednF: {
					calculatedRed = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toFloat();
					break;
				}
				case tcalculatedGreennF: {
					calculatedGreen =
							(part1.substring(part1.indexOf(F("_")) + 1,
									part1.length())).toFloat();
					break;
				}
				case tcalculatedBluenF: {
					calculatedBlue = (part1.substring(part1.indexOf(F("_")) + 1,
							part1.length())).toFloat();
					break;
				}

				}

			}
#if debug
			{
				printMyValues();
			}
#endif
		} else if (com.substring(0, com.indexOf("|")).equalsIgnoreCase(
				"toESP")) {
			for (int i = com.indexOf("|"); i < com.lastIndexOf("|");
					i = com.indexOf("|", i + 1)) {
				String part1 = com.substring(i + 1, com.indexOf("|", i + 1));
				int cmdID = findCommand(
						part1.substring(0, part1.indexOf(F("_"))).c_str());
				switch (cmdID) {
				case tPhWert: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(PhWert));
					break;
				}
				case tTemp: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(Temp));
					break;
				}
				case tcalculatedPWM: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedPWM));
					break;
				}
				case tcalculatedRed: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedRed));
					break;
				}
				case tcalculatedGreen: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedGreen));
					break;
				}
				case tcalculatedBlue: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(calculatedBlue));
					break;
				}
				case tTVModeState: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(TVModeState));
					break;
				}
				case tcleaningInProcess: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(cleaningInProcess));
					break;
				}
				case tmanualOverride: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(manualOverride));
					break;
				}
				case tMoonModeState: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(MoonModeState));
					break;
				}
				case tpump1Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(pump1Value));
					break;
				}
				case tpump2Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(pump2Value));
					break;
				}
				case tlight230Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(light230Value));
					break;
				}
				case tlight1Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(light1Value));
					break;
				}
				case tlight2Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(light2Value));
					break;
				}
				case tco2Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(co2Value));
					break;
				}
				case theaterValue: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(heaterValue));
					break;
				}
				case tdPump1Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(dPump1Value));
					break;
				}
				case tdPump2Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(dPump2Value));
					break;
				}
				case tdPump3Value: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(dPump3Value));
					break;
				}
				case tcoolValue: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(coolValue));
					break;
				}
				case tnow: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(now.unixtime()));
					break;
				}
				case tPHValues: {
					for (int io = 0; io < 4; io++) // auf 3 x Senden, da der CharBuffer sonst übergeht (max 255 - siehe numchar)
							{
						String sendString = String(io * 24) + ","
								+ String(
										PHValues[(put_PHindex + io * 24) % 96]);
						for (int ic = 1; ic < 25; ic++) {
							sendString =
									sendString + ";" + String(io * 24 + ic)
											+ ","
											+ String(
													PHValues[(put_PHindex
															+ io * 24 + ic) % 96]);
						}
						sendCommand(part1.substring(0, part1.indexOf(F("_"))),
								sendString);
					}
					break;
				}
				case tTempValues: {
					for (int io = 0; io < 4; io++) // auf 3 x Senden, da der CharBuffer sonst übergeht (max 255 - siehe numchar)
							{
						String sendString =
								String(io * 24) + ","
										+ String(
												TempValues[(put_PHindex
														+ io * 24) % 96]);
						for (int ic = 1; ic < 25; ic++) {
							sendString = sendString + ";" + String(io * 24 + ic)
									+ ","
									+ String(
											TempValues[(put_PHindex + io * 24
													+ ic) % 96]);
						}
						sendCommand(part1.substring(0, part1.indexOf(F("_"))),
								sendString);
					}
					break;
				}

				case tCo2Values: {
					//sendCommand("PhINDEX: ", String(put_PHindex));
					//String sendString=String(String(0)+","+String(Co2Values[put_PHindex]));
					for (int io = 0; io < 4; io++) // auf 3 x Senden, da der CharBuffer sonst übergeht (max 255 - siehe numchar)
							{
						String sendString =
								String(io * 24) + ","
										+ String(
												Co2Values[(put_PHindex + io * 24)
														% 96]);
						for (int ic = 1; ic < 25; ic++) { //sendCommand(String(io*24+ic)+ " ", String(Co2Values[(put_PHindex+io*24+ic)%96])+ " " + String((put_PHindex+io*24+ic)%96));
							sendString = sendString + ";" + String(io * 24 + ic)
									+ ","
									+ String(
											Co2Values[(put_PHindex + io * 24
													+ ic) % 96]);
						}

						sendCommand(part1.substring(0, part1.indexOf(F("_"))),
								sendString);
					}
					break;
				}

				case tnpkFert: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(int(FLeft[0] / FDose[0])));
					break;
				}
				case tnFert: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(int(FLeft[1] / FDose[1])));
					break;
				}
				case tfeFert: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(int(FLeft[2] / FDose[2])));
					break;
				}
				case tdst: {
					sendCommand(part1.substring(0, part1.indexOf(F("_"))),
							String(waterDistance));
					break;
				}

				}
			}
		}

		else if (com.substring(0, com.indexOf("|")).equalsIgnoreCase(
				"updateMe")) {
			parseCommand(
					F(
							"toESP|pH|tE|tV|cI|mO|mM|cP|cR|cG|cB|p1|p2|lV|l1|l2|cO|hV|d1|d2|d3|cV|nO|*72"));
			parseCommand(F("toESP|phS|cS|tS|*16"));
		} else {
#if debug
			{	Serial.println(F("Recieved: "));
				Serial.println(com);
			}
#endif
		}
	} else {
#if debug
		{	Serial.println(F("ErrorOnSerial"));
			Serial.println(com);
			Serial.println(F("-------"));}
#endif
	}
}

int findCommand(const char* searchText) {
	int startCount = 0;
	int foundIndex = -1; // -1 = not found
	while (startCount < charCount) {
		if (strcmp_P(searchText,
				(const char*) pgm_read_ptr(Char_table + startCount)) == 0) {
			foundIndex = startCount;
			break;
		}
		startCount++;
	}
	return foundIndex;
}

const char* readProgmem(int input) {
	char buffer[20];
	strcpy_P(buffer, (char*) pgm_read_word(&(Char_table[input]))); // Necessary casts and dereferencing, just copy.
	return buffer;
//Serial.println(buffer);
}

void printMyValues() {
#if debug
	{/**
	 Serial.println(F("-----Date and Time--------"));
	 Serial.println(F("NOW: "));
	 Serial.print(now.hour(), DEC);
	 Serial.print(':');
	 Serial.print(now.minute(), DEC);
	 Serial.print(':');
	 Serial.print(now.second(), DEC);
	 Serial.print(' ');
	 Serial.print(now.day(), DEC);
	 Serial.print('.');
	 Serial.print(now.month(), DEC);
	 Serial.print('.');
	 Serial.println(now.year(), DEC);
	 Serial.println(F("------PH and Temp--------"));
	 Serial.println(F("PhWert: "));
	 Serial.println(PhWert);
	 Serial.println(F("Temp: "));
	 Serial.println(Temp);
	 Serial.println(F("-----------Modes---------"));
	 Serial.print(F("TVModeState: "));
	 Serial.println(TVModeState);
	 Serial.print(F("cleaningInProcess: "));
	 Serial.println(cleaningInProcess);
	 Serial.print(F("manualOverride: "));
	 Serial.println(manualOverride);
	 Serial.print(F("MoonModeState: "));
	 Serial.println(MoonModeState);
	 Serial.println(F("--------Lights----------"));
	 Serial.print(F("calculatedPWM: "));
	 Serial.println(calculatedPWM);
	 Serial.print(F("calculatedRed: "));
	 Serial.println(calculatedRed);
	 Serial.print(F("calculatedGreen: "));
	 Serial.println(calculatedGreen);
	 Serial.print(F("calculatedBlue: "));
	 Serial.println(calculatedBlue);
	 Serial.println(F("-------Booleans--------"));
	 Serial.print(F("pump1Value: "));
	 Serial.println(pump1Value);
	 Serial.print(F("pump2Value: "));
	 Serial.println(pump2Value);
	 Serial.print(F("light230Value: "));
	 Serial.println(light230Value);
	 Serial.print(F("light1Value: "));
	 Serial.println(light1Value);
	 Serial.print(F("light2Value: "));
	 Serial.println(light2Value);
	 Serial.print(F("co2Value: "));
	 Serial.println(co2Value);
	 Serial.print(F("heaterValue: "));
	 Serial.println(heaterValue);
	 Serial.print(F("dPump1Value: "));
	 Serial.println(dPump1Value);
	 Serial.print(F("dPump2Value: "));
	 Serial.println(dPump2Value);
	 Serial.print(F("dPump3Value: "));
	 Serial.println(dPump3Value);
	 Serial.print(F("coolValue: "));
	 Serial.println(coolValue);
	 Serial.println(F("----------------------"));*/
	}
#endif
}

