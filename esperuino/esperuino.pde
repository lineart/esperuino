
/* The circuit:
 * LCD RS pin to digital pin 4
 * LCD Enable pin to digital pin 5
 * LCD D4 pin to digital pin 6
 * LCD D5 pin to digital pin 7
 * LCD D6 pin to digital pin 8
 * LCD D7 pin to digital pin 9
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 10)
 */

// include the library code:
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <IRremote.h>

// Time constants
#define SEC    1000UL
#define MIN    60000UL
#define HOUR   3600000UL

// LCD pinout
#define COLS 20
#define ROWS 4
#define LCDSIZE COLS, ROWS
#define RS 4
#define EN 5
#define D1 6
#define D2 7
#define D3 8
#define D4 9
#define BLIGHT 10

//IR Remote Control
#define IRPIN 12
#define FM    0xffa25d
#define FP    0xff629d
#define REL   0xffe21d
#define CHM   0xff22dd
#define CHP   0xff02fd
#define EQ    0xffc23d
#define REW   0xffe01f
#define FWD   0xffa857
#define PP    0xff906f
#define RD0    0xff6897
#define MINUS 0xff9867
#define PLUS  0xffb04f
#define RD1    0xff30cf
#define RD2    0xff18e7
#define RD3    0xff7a85
#define RD4    0xff10ef
#define RD5    0xff38c7
#define RD6    0xff5aa5
#define RD7    0xff42bd
#define RD8    0xff4ab5
#define RD9    0xff52ad
#define RPT   0xffffffff

// --- ECU REQUEST ---
#define REQ_MSG_ID   0xF4
#define REQ_MSG_LEN  0x56
#define REQ_MODE_NUM 0x01
#define REQ_CHK_SUM  0xB5

// Info indexes in data starting from 0
#define PROMIDA             data[0]
#define PROMIDB             data[1]
#define MALFA               data[2]
#define MALFB               data[3]
#define TEMP_COOLANT        (float) data[4] * .75 - 40  // (C) Coolant temperature
#define TEMP_COOLANT_START  (float) data[5] * .75 - 40  // (C) Coolant startup temperature
#define THROTTLE_SEN        (float) data[6] * .02       // (V) Throttle position sensor
#define THROTTLE_POS        (int)   data[7] / 2.55      // (%) Throttle position
#define RPM                 (int)   data[8] * 25        // (1/min) Engine speed
//#define REF_PULSE_DELAY_M   data[9]
//#define REF_PULSE_DELAY_L   data[10]
#define SPEED               data[11]                    // (km/h) Speed
//#define SPEED_L             data[12]                  // NOT USED
#define OXYGEN_SEN          (int)   data[13] * 4.44     // (mV) from 0 to 1130. Normal: ~500 mV
//#define  data[14]
//#define  data[15] 
//#define  data[16]
//#define LEARN               data[17]
//#define BLM                 data[18]
//#define CLOSED_LOOP_INT     data[19]
//#define IAC_MOTOR_POS       data[20]
//#define IAC_MOTOR_POS_DES   data[21]
#define RPM_DES             (int) data[22] * 12.5
#define RPM_IDLE_DES        (int) data[23] * 12.5
#define BAROMETER           data[24]
#define MAP_SEN             data[25]
//#define  data[26]
//#define  data[27]
#define TEMP                (float) data[28] * .75 - 40 // (C) Atmosphere temperature
//#define  data[29]
#define VOLTAGE             (float) data[30] * .1       // (V) Boart voltage
#define SPARK_ADV           data[31]                    // UNKNOWN
#define SPARK_ADV_M         data[32]
#define SPARK_ADV_L         data[33]
#define INJ_PULSE           (float) (data[34] * 256 + INJ_PULSE_L) * 0.01526 // (ms) Injection pulse width
#define INJ_PULSE_L         data[35]
#define AIR_FUEL            (float) data[36] * .1       // () Air to Fuel ratio
#define AIR_FUEL_CRANK      (float) data[37] * .1       // () Air to Fuel ratio while cranking
#define TIME                (int) data[38] * 256 + TIME_L // (s) Time Engine started
#define TIME_L data[39]
#define FUEL_LEVEL          (int) data[40] / 2.55       // (%) Fuel level
//#define  data[41]
//#define  data[42]
//#define  data[43]
//#define  data[44]
//#define  data[45]
//#define  data[46]
//#define  data[47]
//#define  data[48]
//#define  data[49]
//#define  data[50]
//#define  data[51]
//#define  data[52]
//#define  data[53]
//#define  data[54]
//#define  data[55]
//#define  data[56]
//#define  data[57]
//#define  data[58]
//#define  data[59]

#define HEADER 7     // response header lenght
#define SIZE   60    // response data lenght
#define RESPLEN 5    // index of MSG LENGTH in response


// Engine Parameters
#define INJ_PERF 0.1 // Injector Performance (l/min)

// Parameters definitions macros
#define FUEL_CONSUMP_LPH   (double) INJ_PULSE * 4 * RPM * 1.667e-5 * INJ_PERF * 60 // per hour
#define FUEL_CONSUMP_LPKM  (double) FUEL_CONSUMP_LPH / SPEED * 100  // per 100km

#define BRIGHTNESS  (byte) pow(2, brightness%9) -1  // LCD Backlight brightness (0-255)

// ECU IF speed
#define ECU_SPD  8192

// Display regions:     COL,        ROW, LENGTH 
#define D_START         10
#define D_TIME          D_START+5,  3
#define D_TEMP          D_START,    0, 4
#define D_CONSUMP       D_START,    1, 4
#define D_TRIP_CONSUMP  D_START+5,  1, 4
#define D_TRIP_DIST     D_START,    2, 8
#define D_DBG           D_START,    2

// addresses in ROM 
#define R_FUEL     1000
#define R_ING_PERF 1001

#define R_BLIGHT   1019
#define ODO0  1020
#define ODO1  1021
#define ODO2  1022
#define ODO3  1023

// requests delay
#define REQ_DLY 250UL
// update displayed info
#define UPD      500UL
#define CAN_UPDATE  millis() - timeStamp > UPD

// UI and Menus
#define BIGD_AREA 9
#define BIGD_SIZE 3

#define CATSIZE 3

byte  category; // position of category
byte  menu[CATSIZE]; 
byte  menuSize[] = {5, 1, 3};
char *catText[]  = {"Date & Time", "Display", "Car"}; // Textual representation
char *menuText[][5] = { {"Hour", "Minute", "Day", "Month", "Year"},
                                {"Brightness"},
                                {"Fuel Level", "Injector Perf", "Odometer",} };

//#define TEST
//=============== VARIABLES DECLARATION ===================
LiquidCrystal lcd(RS, EN, D1, D2, D3, D4); // initialize the LCD with the numbers of the interface pins

IRrecv irrecv(IRPIN);  // Init IR receiver
decode_results irCode; // define IR result storage

byte cache, data[SIZE], brightness;

boolean start, running, confMode, categoryActive, menuActive;

unsigned long odometer, timeStamp, reqStamp;
unsigned long tripStamp;

double fuel, tripDistance, tripConsump, oldConsump;

unsigned int oldSpeed, cnt;


//===== SETUP =====
void setup() {
  lcdSetup(); // Create custom chars & init LCD
  
  irrecv.enableIRIn(); // Start the receiver

  // Loading params from EEPROM
  brightness = EEPROM.read(R_BLIGHT);
  odometer   = EEPROM.read(ODO3)*16777216 + EEPROM.read(ODO2)*65536 + EEPROM.read(ODO1)*256 + EEPROM.read(ODO0); 
  fuel       = EEPROM.read(R_FUEL)*0.2;
  
  // Set LCD backlight
  pinMode(BLIGHT, OUTPUT);
  analogWrite(BLIGHT, BRIGHTNESS);
  
  // Init vars
  start = running = confMode = categoryActive = menuActive = false;
  category = 0;
  for (byte i = 0; i < CATSIZE; i++)
    menu[i] = 0;
  
  timeStamp = tripStamp = millis();
  
  tripDistance = 0;
  tripConsump  = 0;
  
  oldSpeed   = 0;
  oldConsump = 0;

  Serial.begin(ECU_SPD);
}

//===== LOOP =====
void loop() {
  irControl(); // Infrared Remote Control 

  talkToECU(); // Send request and read response when needed
  
  #ifdef TEST
  test();      // Make fake ECU response
  #endif

  if (start) {
    double distDelta = (double) (oldSpeed + SPEED)*1000/2 * (millis()-tripStamp)/HOUR;
    double consDelta = (double) (oldConsump + FUEL_CONSUMP_LPH)/2 * (millis()-tripStamp)/HOUR;
    tripStamp  = millis();
    
    tripDistance += distDelta;
    tripConsump  += consDelta;
    odometer     += distDelta;
    fuel         -= consDelta;
    
    oldSpeed   = SPEED;
    oldConsump = FUEL_CONSUMP_LPH;
  } else { tripStamp  = millis(); } // If engine OFF

  if (! confMode) {
    if (CAN_UPDATE) {
      bigNum( SPEED );      

      if ( SPEED < 40 )
        printDoubleAt( FUEL_CONSUMP_LPH, D_CONSUMP );
      else 
        printDoubleAt( FUEL_CONSUMP_LPKM, D_CONSUMP );
  
      printDecAt( TEMP_COOLANT, D_START, 0, 3 ); lcd.write(223);
      printDoubleAt( fuel, D_START + 5, 0, 4 ); lcd.print("L");    
      printDoubleAt( tripConsump, D_TRIP_CONSUMP ); lcd.print("L");
      printDoubleAt( tripDistance/1000.0, D_TRIP_DIST ); lcd.print("km");
      lcd.setCursor(D_TIME);
      timeDisplay(0);
      
      timeStamp = millis();
    }
  }


}
// ======================================================


void irControl() {
  if (irrecv.decode(&irCode)) {
    if (irCode.value == EQ) {
      confMode = !confMode || categoryActive;         // Enter/Exit/Stay conf mode
      categoryActive = categoryActive && menuActive;  // Exit/Stay category
      menuActive = false;                             // Exit menu
      lcd.clear();
    }  
    
    if (confMode) {      
      lcd.clear();
      if (categoryActive) {
        if (menuActive) { 
          setParam();
        } else {
          showMenu();
        }
      } else {
        showCategory();
      }

    } else {
      if (irCode.value == PLUS) {
        brightness++;
        analogWrite(BLIGHT, BRIGHTNESS);
      }
      if (irCode.value == MINUS) {
        brightness--;
        analogWrite(BLIGHT, BRIGHTNESS);
      }
      if (irCode.value == PP) {
        lcdSetup();
      }
    }
    irrecv.resume(); // Receive the next value
  }  
}

void showCategory() {
  if (irCode.value == FWD) category++; 
  if (irCode.value == REW) category--;
  if (category < 0 || category >= CATSIZE) category = 0;
  printCategory();
  if (irCode.value == PP) {
    categoryActive = true;
  } 
}

void showMenu() {
  if (irCode.value == FWD) menu[category]++; 
  if (irCode.value == REW) menu[category]--; 
  if (menu[category] < 0 || menu[category] >= menuSize[category]) menu[category] = 0;
  printMenu();
  if (irCode.value == PP) {
    menuActive = true;
  } 
}

void printCategory () {
  lcd.print("Settings");
  int shift = -1;
  if (category <= 0) {
    shift++;
  } else if (category >= CATSIZE -1 && CATSIZE > 2) {
    shift--;
  } 
  for (byte i = 0; i < 3; i++) {
    if (category + shift +i >= CATSIZE) break;
    lcd.setCursor(0, 1+i);
    if (shift + i == 0)
      lcd.print("* ");
    else
      lcd.print("  ");
    lcd.print(catText[category + shift +i]);
  }
}

void printMenu() {
  lcd.print(catText[category]);
  int shift = -1;
  if (menu[category] <= 0) {
    shift++;
  } else if (menu[category] >= menuSize[category] -1 && menuSize[category] > 2) {
    shift--;
  } 
  for (byte i = 0; i < 3; i++) {
    if (menu[category] + shift +i >= menuSize[category]) break;
    lcd.setCursor(0, 1+i);
    if (shift + i == 0)
      lcd.print("* ");
    else
      lcd.print("  ");
    lcd.print(menuText[category][menu[category] + shift +i]);
  }
}

void setParam() {
  lcd.print(menuText[category][menu[category]]);
  lcd.print(" - ");
  switch (category) {
    case 0:
      switch (menu[category]) {
        case 0:
          if (irCode.value == PLUS) setTime((hour()+1)%24, minute(), second(), day(), month(), year());
          if (irCode.value == MINUS) setTime((hour()-1)%24, minute(), second(), day(), month(), year());
          lcd.print(hour()); break;
        case 1:
          if (irCode.value == PLUS) setTime(hour(), (minute()+1)%60, 0, day(), month(), year());
          if (irCode.value == MINUS) setTime(hour(), (minute()-1)%60, 0, day(), month(), year());
          lcd.print(minute()); break;
        case 2:
          if (irCode.value == PLUS) setTime(hour(), minute(), second(), day()%31+1, month(), year());
          if (irCode.value == MINUS) setTime(hour(), minute(), second(), day()%31-1, month(), year());
          lcd.print(day()); break;
        case 3:
          if (irCode.value == PLUS) setTime(hour(), minute(), second(), day(), month()%12+1, year());
          if (irCode.value == MINUS) setTime(hour(), minute(), second(), day(), month()%12-1, year());
          lcd.print(month()); break;
        case 4:
          if (irCode.value == PLUS) setTime(hour(), minute(), second(), day(), month(), year()+1);
          if (irCode.value == MINUS) setTime(hour(), minute(), second(), day(), month(), year()-1);
          lcd.print(year()); break;
      } break;
    case 1:
      switch (menu[category]) {
        case 0:
          lcd.print(BRIGHTNESS); break;
      } break;
    case 2:
      switch (menu[category]) {
        case 0: // Fuel Level
          if (irCode.value == PLUS) fuel += 0.5;
          if (irCode.value == MINUS) fuel -= 0.5;
          lcd.print(fuel); break;
        case 1: // Inj Perf
          if (irCode.value == PLUS) fuel += 0.05;
          if (irCode.value == MINUS) fuel -= 0.05;
          lcd.print(INJ_PERF); break;
        case 2: // Odometer
          if (irCode.value == PLUS) odometer += 100;
          if (irCode.value == MINUS) odometer -= 100;
          if (irCode.value == REL) odometer = 0;
          if (irCode.value == RD0) odometer *= 10;
          if (irCode.value == RD1) { odometer *= 10; odometer += 1; }
          if (irCode.value == RD2) { odometer *= 10; odometer += 2; }
          if (irCode.value == RD3) { odometer *= 10; odometer += 3; }
          if (irCode.value == RD4) { odometer *= 10; odometer += 4; }
          if (irCode.value == RD5) { odometer *= 10; odometer += 5; }
          if (irCode.value == RD6) { odometer *= 10; odometer += 6; }
          if (irCode.value == RD7) { odometer *= 10; odometer += 7; }
          if (irCode.value == RD8) { odometer *= 10; odometer += 8; }
          if (irCode.value == RD9) { odometer *= 10; odometer += 9; }
          lcd.print(odometer); break;
      } break;
  }
}

void talkToECU() {
  // Sending REQUEST to ECU
  if ( reqStamp != millis()/REQ_DLY && Serial.available() == 0 ) {
    sendECUrequest();
    reqStamp = millis()/REQ_DLY;
    cnt = 0; // reset response byte index
  }

  // Reading ECU response
  if (Serial.available() > 0) {        
    cache = Serial.read(); 
    if (cnt == HEADER) {
      if (cache == PROMIDA)
        start = true;
      else
        start = false;
    }
    if (cnt >= HEADER && cnt - HEADER <= SIZE) 
      data[cnt - HEADER] = cache;
    cnt++;
  }
}

void sendECUrequest() {
  Serial.write(REQ_MSG_ID);
  Serial.write(REQ_MSG_LEN);
  Serial.write(REQ_MODE_NUM);
  Serial.write(REQ_CHK_SUM);
}

void test() {
  start = true;
  if (CAN_UPDATE) {
    for(short i = 0; i < SIZE; ++i)
      data[i] = data[i]+1;
    lcd.setCursor(D_DBG);
    lcd.write(SPEED);
  }  
}

void timeDisplay(boolean date) {
  // digital clock display of the time
  if (! date) {
    if(hour()<10) lcd.print("0");
    lcd.print(hour());
    lcd.print(":");
    if(minute() < 10) lcd.print('0');
    lcd.print(minute());
//    lcd.print(":");
//    if(second() < 10) lcd.print('0');
//    lcd.print(second());
  } else {
    lcd.print(day());
    lcd.print("/");
    lcd.print(month());
    lcd.print("/");
    lcd.print(year());
  }
}

void printDecAt(long data, short col, short row, short len) {
    String str = String(data);
    len -= str.length();
    lcd.setCursor(col, row);
    while(len > 0) {
      lcd.print(" ");
      --len;
    }
    lcd.print(str);
}

void printDoubleAt(double data, short col, short row, short len) {
    String str = String( long(round(data*10)) );
    len -= str.length() +1;
    lcd.setCursor(col, row);
    while(len > 0) {
      lcd.print(" ");
      --len;
    }
    lcd.print( str.substring(0, str.length() -1) + "." + str.substring(str.length() -1) );
}

void printHexAt(byte data, short col, short row, short len) {
    len -= 2;
    lcd.setCursor(col, row);
    while(len > 0) {
      lcd.print(" ");
      --len;
    }
    lcd.print(data, HEX);
}

void bigNum(int n) {
  byte mtx[ROWS][BIGD_AREA];
  for (byte i = 0; i < ROWS; ++i)
    for (byte j = 0; j < BIGD_AREA; ++j)
      mtx[i][j] = 0x20;

  if (n < 10) { bigDigit(n, 3, mtx); }
  else if (n < 100) { bigDigit(n/10, 1, mtx); bigDigit(n%10, 5, mtx); }
  else if (n < 200) { bigDigit(n/100, -1, mtx); bigDigit(n%100/10, 2, mtx); bigDigit(n%10, 6, mtx); }
  else { bigDigit(n/100, 0, mtx); bigDigit(n%100/10, 3, mtx); bigDigit(n%10, 6, mtx); }
  
  for (byte i = 0; i < ROWS; ++i) {
    lcd.setCursor(0, i);
    for (byte j = 0; j < BIGD_AREA; ++j)
      lcd.write(mtx[i][j]);
  }

}

byte bigDigit(int d, int disp, byte mtx[][BIGD_AREA]) {
  byte s = BIGD_SIZE;
  byte m[ROWS][BIGD_SIZE];
  switch(d) {
    case 0:
      m = {
        {0x00, 0x04, 0x01},
        {0xff, 0x20, 0xff},
        {0xff, 0x20, 0xff},
        {0x02, 0x05, 0x03}
      };
      break;
    case 1:
      //s = 1;
      m = {
        {0x20, 0x00, 0x20},
        {0x20, 0xff, 0x20},
        {0x20, 0xff, 0x20},
        {0x20, 0xff, 0x20}
      };
      break;
    case 2:
      m = {
        {0x00, 0x04, 0x01},
        {0x04, 0x20, 0xff},
        {0x20, 0x00, 0x03},
        {0x00, 0xff, 0x05}
      };
      break;
    case 3:
      m = {
        {0x00, 0x04, 0x01},
        {0x20, 0x05, 0x03},
        {0x20, 0x20, 0x01},
        {0x02, 0x05, 0x03}
      };
      break;
    case 4:
      m = {
        {0x01, 0x20, 0x00},
        {0xff, 0x20, 0xff},
        {0x04, 0x04, 0xff},
        {0x20, 0x20, 0x02}
      };
      break;
    case 5:
      m = {
        {0xff, 0x04, 0x04},
        {0xff, 0x04, 0x01},
        {0x20, 0x20, 0xff},
        {0x02, 0x05, 0x03}
      };
      break;
    case 6:
      m = {
        {0x00, 0x04, 0x01},
        {0xff, 0x04, 0x01},
        {0xff, 0x20, 0xff},
        {0x02, 0x05, 0x03}
      };
      break;
    case 7:
      m = {
        {0xff, 0x04, 0xff},
        {0x20, 0x20, 0xff},
        {0x20, 0x20, 0xff},
        {0x20, 0x20, 0x02}
      };
      break;
    case 8:
      m = {
        {0x00, 0x04, 0x01},
        {0x02, 0x05, 0x03},
        {0x00, 0x20, 0x01},
        {0x02, 0x05, 0x03}
      };
      break;
    case 9:
      m = {
        {0x00, 0x04, 0x01},
        {0xff, 0x20, 0xff},
        {0x02, 0x05, 0xff},
        {0x02, 0x05, 0x03}
      };
      break;
    //default:
  }
  for (byte i = 0; i < ROWS; ++i)
    for (byte j = 0; j < s; ++j)
      if (disp+j >= 0)
        mtx[i][disp+j] = m[i][j];
}

byte ch1[] = {0, 3, 7, 15, 15, 31, 31, 31};
byte ch2[] = {0, 24, 28, 30, 30, 31, 31, 31};
byte ch3[] = {31, 31, 31, 15, 15, 7, 3, 0};
byte ch4[] = {31, 31, 31, 30, 30, 28, 24, 0};
byte ch5[] = {31, 31, 31, 31, 31, 0, 0, 0};
byte ch6[] = {0, 0, 0, 31, 31, 31, 31, 31};

void lcdSetup() {
  lcd.createChar(0, ch1);
  lcd.createChar(1, ch2);
  lcd.createChar(2, ch3);
  lcd.createChar(3, ch4);
  lcd.createChar(4, ch5);
  lcd.createChar(5, ch6);
  lcd.begin(LCDSIZE); // set up the LCD's number of columns and rows 
}


