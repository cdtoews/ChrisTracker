#include <SPI.h>
#include <stdint.h>
#include <BLEPeripheral.h>

#include <nrf_nvic.h>//interrupt controller stuff
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <WInterrupts.h>
#include "Adafruit_GFX.h"
#include "SSD1306.h"
#include <TimeLib.h>
#include <nrf.h>
#include "count_steps.h"
#include "count_steps.c"
#include "i2csoft.h"

#define wdt_reset() NRF_WDT->RR[0] = WDT_RR_RR_Reload
#define wdt_enable(timeout) \
  NRF_WDT->CONFIG = NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos); \
  NRF_WDT->CRV = (32768*timeout)/1000; \
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  \
  NRF_WDT->TASKS_START = 1

Adafruit_SSD1306 display(128, 32, &SPI, 28, 4, 29);

boolean debug = false;
#define  CTversion "0.4"

#define sleepDelay 10000
#define clockDelay 5000
#define BUTTON_PIN              30
#define refreshRate 100

int menu = 0;
int lastMenu = 0;
volatile bool buttonPressed = false;
long startbutton;
unsigned long sleepTime, displayRefreshTime;
volatile bool sleeping = false;
int timezone;
int steps;
int steps1;
String serialNr = "235246472";
String versionNr = "110.200.051";
String btversionNr = "100.016.051";
String msgText;
boolean gotoBootloader = false;
boolean vibrationMode;

char msg[200];
char subChar[50];
char currentDOW[6];
const char msgPrefix[5] = "    ";
boolean doneScrolling = true;
const int scrollWaitMS = 50;
const int stepsPerChar = 11;
int currentCharStep = 0;
int currentCharInt = 0;
long lastMS = millis();
int charScrollStep = 7;
int quoteIndex;
const int quoteSize = 95;

String bleSymbol = " ";
int contrast;

const char msgTextToIgnore[2][30] =
{
  "Voice is doing work",
  "1234567890xxx"
};



BLEPeripheral                   blePeripheral           = BLEPeripheral();
BLEService                      batteryLevelService     = BLEService("190A");
BLECharacteristic   TXchar        = BLECharacteristic("0002", BLENotify, 20);
BLECharacteristic   RXchar        = BLECharacteristic("0001", BLEWriteWithoutResponse, 20);

BLEService                      batteryLevelService1     = BLEService("190B");
BLECharacteristic   TXchar1        = BLECharacteristic("0004", BLENotify, 20);
BLECharacteristic   RXchar1        = BLECharacteristic("0003", BLEWriteWithoutResponse, 20);

#define N_GRAINS     3 // Number of grains of sand
#define WIDTH        127 // Display width in pixels
#define HEIGHT       32 // Display height in pixels
#define MAX_FPS      150 // Maximum redraw rate, frames/second
int totalScore = 0;


// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
  uint16_t pos;
} grain[N_GRAINS];

uint32_t        prevTime   = 0;      // Used for frames-per-second throttle
uint16_t         backbuffer = 0, img[WIDTH * HEIGHT]; // Internal 'map' of pixels

#ifdef __cplusplus
extern "C" {
#endif

#define LF_FREQUENCY 32768UL
#define SECONDS(x) ((uint32_t)((LF_FREQUENCY * x) + 0.5))
#define wakeUpSeconds 120
void RTC2_IRQHandler(void)
{
  volatile uint32_t dummy;
  if (NRF_RTC2->EVENTS_COMPARE[0] == 1)
  {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->CC[0] = NRF_RTC2->COUNTER +  SECONDS(wakeUpSeconds);
    dummy = NRF_RTC2->EVENTS_COMPARE[0];
    dummy;
    //powerUp();
  }
}

void initRTC2() {

  NVIC_SetPriority(RTC2_IRQn, 15);
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);

  NRF_RTC2->PRESCALER = 0;
  NRF_RTC2->CC[0] = SECONDS(wakeUpSeconds);
  NRF_RTC2->INTENSET = RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos;
  NRF_RTC2->EVTENSET = RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;
  NRF_RTC2->TASKS_START = 1;
}
#ifdef __cplusplus
}
#endif

void powerUp() {
  if (sleeping) {
    sleeping = false;
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.display();
    if (debug)Serial.begin(115200);

    delay(5);
  }
  sleepTime = millis();
}

void powerDown() {
  totalScore = 0;
  if (!sleeping) {
    if (debug)NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled;
    sleeping = true;

    digitalWrite(28, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(29, LOW);
    digitalWrite(4, LOW);
    NRF_SAADC ->ENABLE = 0; //disable ADC
    NRF_PWM0  ->ENABLE = 0; //disable all pwm instance
    NRF_PWM1  ->ENABLE = 0;
    NRF_PWM2  ->ENABLE = 0;
  }
}

void charge() {
  if (sleeping)menu = 88;
  powerUp();
}

void buttonHandler() {
  if (!sleeping) buttonPressed = true;
  //let's try just going back to the last option
  //else menu = 0;
  menu = lastMenu; //lastMenu remembers the last menu item user selected
  powerUp();
}

void acclHandler() {
  ReadRegister(0x17);
  if (sleeping) {
    menu = 77;
    powerUp();
  }
}

void blePeripheralConnectHandler(BLECentral& central) {
  if (debug)Serial.println("BLEconnected");
  menu = 0;
  powerUp();
  bleSymbol = "B";
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  if (debug)Serial.println("BLEdisconnected");
  menu = 0;
  powerUp();
  bleSymbol = " ";
}

String answer = "";
String tempCmd = "";
int tempLen = 0, tempLen1;
boolean syn;

void characteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  char remoteCharArray[21];
  tempLen1 = characteristic.valueLength();
  tempLen = tempLen + tempLen1;
  memset(remoteCharArray, 0, sizeof(remoteCharArray));
  memcpy(remoteCharArray, characteristic.value(), tempLen1);
  tempCmd = tempCmd + remoteCharArray;
  if (tempCmd[tempLen - 2] == '\r' && tempCmd[tempLen - 1] == '\n') {
    answer = tempCmd.substring(0, tempLen - 2);
    tempCmd = "";
    tempLen = 0;
    if (debug)Serial.print("RxBle: ");
    if (debug)Serial.println(answer);
    filterCmd(answer);
  }
}

void filterCmd(String Command) {
  if (Command == "AT+BOND") {
    sendBLEcmd("AT+BOND:OK");
  } else if (Command == "AT+ACT") {
    sendBLEcmd("AT+ACT:0");
  } else if (Command.substring(0, 7) == "BT+UPGB") {
    gotoBootloader = true;
  } else if (Command.substring(0, 8) == "BT+RESET") {
    if (gotoBootloader)NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  } else if (Command.substring(0, 7) == "AT+RUN=") {
    sendBLEcmd("AT+RUN:" + Command.substring(7));
  } else if (Command.substring(0, 8) == "AT+USER=") {
    sendBLEcmd("AT+USER:" + Command.substring(8));
  }  else if (Command.substring(0, 7) == "AT+REC=") {
    sendBLEcmd("AT+REC:" + Command.substring(7));
  } else if (Command.substring(0, 8) == "AT+PUSH=" && !ignoreMsg(Command)) {
    sendBLEcmd("AT+PUSH:OK");
    menu = 99;
    powerUp();
    handlePush(Command.substring(8));
  } else if (Command.substring(0, 9) == "AT+MOTOR=") {
    sendBLEcmd("AT+MOTOR:" + Command.substring(9));
  } else if (Command.substring(0, 8) == "AT+DEST=") {
    sendBLEcmd("AT+DEST:" + Command.substring(8));
  } else if (Command.substring(0, 9) == "AT+ALARM=") {
    sendBLEcmd("AT+ALARM:" + Command.substring(9));
  } else if (Command.substring(0, 13) == "AT+HRMONITOR=") {
    sendBLEcmd("AT+HRMONITOR:" + Command.substring(13));
  } else if (Command.substring(0, 13) == "AT+FINDPHONE=") {
    sendBLEcmd("AT+FINDPHONE:" + Command.substring(13));
  } else if (Command.substring(0, 13) == "AT+ANTI_LOST=") {
    sendBLEcmd("AT+ANTI_LOST:" + Command.substring(13));
  } else if (Command.substring(0, 9) == "AT+UNITS=") {
    sendBLEcmd("AT+UNITS:" + Command.substring(9));
  } else if (Command.substring(0, 11) == "AT+HANDSUP=") {
    sendBLEcmd("AT+HANDSUP:" + Command.substring(11));
  } else if (Command.substring(0, 7) == "AT+SIT=") {
    sendBLEcmd("AT+SIT:" + Command.substring(7));
  } else if (Command.substring(0, 7) == "AT+LAN=") {
    sendBLEcmd("AT+LAN:ERR");
  } else if (Command.substring(0, 14) == "AT+TIMEFORMAT=") {
    sendBLEcmd("AT+TIMEFORMAT:" + Command.substring(14));
  } else if (Command == "AT+BATT") {
    sendBLEcmd("AT+BATT:" + String(getBatteryLevel()));
  } else if (Command == "BT+VER") {
    sendBLEcmd("BT+VER:" + btversionNr);
  } else if (Command == "AT+VER") {
    sendBLEcmd("AT+VER:" + versionNr);
  } else if (Command == "AT+SN") {
    sendBLEcmd("AT+SN:" + serialNr);
  } else if (Command.substring(0, 10) == "AT+DISMOD=") {
    sendBLEcmd("AT+DISMOD:" + Command.substring(10));
  } else if (Command.substring(0, 7) == "AT+LAN=") {
    sendBLEcmd("AT+LAN:ERR");
  } else if (Command.substring(0, 10) == "AT+MOTOR=1") {
    sendBLEcmd("AT+MOTOR:1" + Command.substring(10));
    digitalWrite(25, HIGH);
    delay(300);
    digitalWrite(25, LOW);
  } else if (Command.substring(0, 12) == "AT+CONTRAST=") {
    contrast = Command.substring(12).toInt();
  } else if (Command.substring(0, 6) == "AT+DT=") {
    SetDateTimeString(Command.substring(6));
    sendBLEcmd("AT+DT:" + GetDateTimeString());
  } else if (Command.substring(0, 5) == "AT+DT") {
    sendBLEcmd("AT+DT:" + GetDateTimeString());
  } else if (Command.substring(0, 12) == "AT+TIMEZONE=") {
    timezone = Command.substring(12).toInt();
    sendBLEcmd("AT+TIMEZONE:" + String(timezone));
  } else if (Command.substring(0, 11) == "AT+TIMEZONE") {
    sendBLEcmd("AT+TIMEZONE:" + String(timezone));
  } else if (Command == "AT+STEPSTORE") {
    sendBLEcmd("AT+STEPSTORE:OK");
  } else if (Command == "AT+TOPACE=1") {
    sendBLEcmd("AT+TOPACE:OK");
    sendBLEcmd("NT+TOPACE:" + String(steps));
  } else if (Command == "AT+TOPACE=0") {
    sendBLEcmd("AT+TOPACE:" + String(steps));
  } else if (Command == "AT+DATA=0") {
    sendBLEcmd("AT+DATA:0,0,0,0");
  } else if (Command.substring(0, 8) == "AT+PACE=") {
    steps1 = Command.substring(8).toInt();
    sendBLEcmd("AT+PACE:" + String(steps1));
  } else if (Command == "AT+PACE") {
    sendBLEcmd("AT+PACE:" + String(steps1));
  } else if (Command == "AT+DATA=1") {
    sendBLEcmd("AT+DATA:0,0,0,0");
  } else if (Command.substring(0, 7) == "AT+SYN=") {
    if (Command.substring(7) == "1") {
      sendBLEcmd("AT+SYN:1");
      syn = true;
    } else {
      sendBLEcmd("AT+SYN:0");
      syn = false;
    }
  }

}

void sendBLEcmd(String Command) {
  if (debug)Serial.print("TxBle: ");
  if (debug)Serial.println(Command);
  Command = Command + "\r\n";
  while (Command.length() > 0) {
    const char* TempSendCmd;
    String TempCommand = Command.substring(0, 20);
    TempSendCmd = &TempCommand[0];
    TXchar.setValue(TempSendCmd);
    TXchar1.setValue(TempSendCmd);
    Command = Command.substring(20);
  }
}

boolean ignoreMsg(String msgText) {
  char msgArray[msgText.length()];
  msgText.toCharArray(msgArray, msgText.length());
  //msgTextToIgnore
  int msgCount = sizeof(msgTextToIgnore) / sizeof(msgTextToIgnore[0]);
  for (int i = 0; i < msgCount; i++) {
    //char eachIgnoreable = msgTextToIgnore[i];
    if (strstr (msgArray, msgTextToIgnore[i])) {
      return true;
    }
  }
  return false;
}

String GetDateTimeString() {
  String datetime = String(year());
  if (month() < 10) datetime += "0";
  datetime += String(month());
  if (day() < 10) datetime += "0";
  datetime += String(day());
  if (hour() < 10) datetime += "0";
  datetime += String(hour());
  if (minute() < 10) datetime += "0";
  datetime += String(minute());
  return datetime;
}

void SetDateTimeString(String datetime) {
  int year = datetime.substring(0, 4).toInt();
  int month = datetime.substring(4, 6).toInt();
  int day = datetime.substring(6, 8).toInt();
  int hr = datetime.substring(8, 10).toInt();
  int min = datetime.substring(10, 12).toInt();
  int sec = datetime.substring(12, 14).toInt();
  setTime( hr, min, sec, day, month, year);
}

void handlePush(String pushMSG) {
  int commaIndex = pushMSG.indexOf(',');
  int secondCommaIndex = pushMSG.indexOf(',', commaIndex + 1);
  int lastCommaIndex = pushMSG.indexOf(',', secondCommaIndex + 1);
  String MsgText = pushMSG.substring(commaIndex + 1, secondCommaIndex);
  int timeShown = pushMSG.substring(secondCommaIndex + 1, lastCommaIndex).toInt();
  int SymbolNr = pushMSG.substring(lastCommaIndex + 1).toInt();
  msgText = MsgText;
  if (debug)Serial.println("MSGtext: " + msgText);
  if (debug)Serial.println("symbol: " + String(SymbolNr));
}

int getBatteryLevel() {
  return map(analogRead(3), 500, 715, 0, 100);
}

void setup() {
  randomSeed(analogRead(3));
  quoteIndex = random(0, 90);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(3, INPUT);
  if (digitalRead(BUTTON_PIN) == LOW) {
    NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  }
  pinMode(2, INPUT);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(15, INPUT);
  if (debug)Serial.begin(115200);
  wdt_enable(5000);
  blePeripheral.setLocalName("DS-D6");
  blePeripheral.setAdvertisingInterval(555);
  blePeripheral.setAppearance(0x0000);
  blePeripheral.setConnectable(true);
  blePeripheral.setDeviceName("ATCDSD6");
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService.uuid());
  blePeripheral.addAttribute(batteryLevelService);
  blePeripheral.addAttribute(TXchar);
  blePeripheral.addAttribute(RXchar);
  RXchar.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService1.uuid());
  blePeripheral.addAttribute(batteryLevelService1);
  blePeripheral.addAttribute(TXchar1);
  blePeripheral.addAttribute(RXchar1);
  RXchar1.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  blePeripheral.begin();
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(15), acclHandler, RISING);
  NRF_GPIO->PIN_CNF[15] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[15] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  attachInterrupt(digitalPinToInterrupt(2), charge, RISING);
  NRF_GPIO->PIN_CNF[2] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[2] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  display.begin(SSD1306_SWITCHCAPVCC);
  delay(100);
  display.clearDisplay();
  // display.setFont(&FreeSerifItalic9pt7b);
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println("D6 Emulator");
  display.display();
  digitalWrite(25, LOW);
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  initRTC2();
  initi2c();
  initkx023();

  uint8_t i, j, bytes;
  memset(img, 0, sizeof(img)); // Clear the img[] array
  for (i = 0; i < N_GRAINS; i++) { // For each sand grain...
    do {
      grain[i].x = random(WIDTH  * 256); // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for (j = 0; (j < i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                              ((grain[i].y / 256) != (grain[j].y / 256))); j++);
    } while (j < i); // Keep retrying until a clear spot is found
    img[(grain[i].y / 256) * WIDTH + (grain[i].x / 256)] = 255; // Mark it
    grain[i].pos = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
  }
}

void loop() {
  blePeripheral.poll();
  wdt_reset();
  if (sleeping) {
    sd_app_evt_wait();
    sd_nvic_ClearPendingIRQ(SD_EVT_IRQn);
  } else {

    switch (menu) {
      case 0:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu0();
        }
        break;
      //      case 1:
      //        if (millis() - displayRefreshTime > refreshRate) {
      //          displayRefreshTime = millis();
      //          displayMenu1();
      //        }
      //        break;
      case 2:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu2();
        }
        break;
      case 3:
        if (millis() - displayRefreshTime > scrollWaitMS) {
          displayRefreshTime = millis();
          doneScrolling = false;
          displayMenu3();
        }
        break;
      case 4:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu4();
        }
        break;
      case 5:
        displayMenu5();
        break;
      case 7:
        displayMenu77();
        break;
      case 77:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu77();
        }
        break;
      case 88:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu88();
        }
        break;
      case 99:
        if (millis() - displayRefreshTime > refreshRate) {
          displayRefreshTime = millis();
          displayMenu99();
        }
        break;
    }
    if (buttonPressed) {
      buttonPressed = false;
      switch (menu) {
        case -1:
          menu = 0;
          lastMenu = 0;
          break;
        case 0:
          menu = 2;
          lastMenu = 2;
          break;
        //        case 1:
        //          menu = 2;
        //          break;
        case 2:
          menu = 3;
          lastMenu = 3;
          break;
        case 3:
          //reset msg;
          msg[0] = 0;
          doneScrolling = true;
          menu = 4;
          lastMenu = 4;
          break;
        case 4:
          startbutton = millis();
          while (!digitalRead(BUTTON_PIN)) {}
          if (millis() - startbutton > 1000) {
            delay(100);
            int err_code = sd_power_gpregret_set(0x01);
            sd_nvic_SystemReset();
            while (1) {};
          } else {
            menu = 5;
            lastMenu = 5;
          }
          break;
        case 5:
          totalScore = 0;
          menu = 7;
          lastMenu = 7;
          break;
        case 7:
          menu = 0;
          lastMenu = 0;
          break;
        case 77:
          menu = lastMenu;
          break;
        case 88:
          menu = lastMenu;
          break;
        case 99:
          digitalWrite(25, LOW);
          menu = lastMenu;
          break;
      }
    }
    switch (menu) {
      case 0:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 1:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 2:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 3:
        if (millis() - sleepTime > 1000 && doneScrolling ) powerDown();
        break;
      case 4:
        if (millis() - sleepTime > sleepDelay ) powerDown();
        break;
      case 5:
        if (millis() - sleepTime > 20000 ) {
          //let's quickly display the score
          display.clearDisplay();
          display.setTextSize(2);
          display.setCursor(0, 0);
          display.print(totalScore);
          totalScore = 0;
          display.display();
          display.setTextSize(1);
          delay(2000);
          powerDown();
        }
        break;
      case 7:
        if (millis() - sleepTime > clockDelay ) powerDown();
        break;
      case 77:
        if (millis() - sleepTime > clockDelay ) powerDown();
        break;
      case 88:
        if (millis() - sleepTime > 3000 ) powerDown();
        break;
      case 99:
        if (millis() - sleepTime > 6000 ) {
          digitalWrite(25, LOW);
          powerDown();
        }
        break;
    }
  }
}

//display time & Battery
void displayMenu0() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(" Time and Batt:");
  //let's get time string together
  String timeString = "";
  int tempHour = hour();
  if (tempHour > 12) {
    tempHour -= 12;
  }
  timeString += String(tempHour) + ":";
  if (minute() < 10) timeString += "0";
  timeString += String(minute()) + ":";
  if (second() < 10) timeString += "0";
  timeString += String(second());

  display.println(timeString);
  display.print(getBatteryLevel());
  display.print("%  ");
  display.println(analogRead(3));
  display.print(contrast);
  char tmp[16];
  sprintf(tmp, "%04X", NRF_FICR->DEVICEADDR[1] & 0xffff);
  String MyID = tmp;
  sprintf(tmp, "%08X", NRF_FICR->DEVICEADDR[0]);
  MyID += tmp;

  display.print(" mac: " + MyID);
  display.display();
}

//void displayMenu1() {
//  display.setRotation(0);
//  display.clearDisplay();
//  display.setCursor(0, 0);
//  display.println("Manual Mac:");
//  char tmp[16];
//  sprintf(tmp, "%04X", NRF_FICR->DEVICEADDR[1] & 0xffff);
//  String MyID = tmp;
//  sprintf(tmp, "%08X", NRF_FICR->DEVICEADDR[0]);
//  MyID += tmp;
//  display.println(MyID);
//  display.display();
//}

//Display last push msg
void displayMenu2() {
  display.setRotation(0);
  //  uint8_t res[6];
  //  softbeginTransmission(0x1F);
  //  softwrite(0x06);
  //  softendTransmission();
  //  softrequestFrom(0x1F , 6);
  //  res[0] = softread();
  //  res[1] = softread();
  //  res[2] = softread();
  //  res[3] = softread();
  //  res[4] = softread();
  //  res[5] = softread();
  //  byte x = (int16_t)((res[1] << 8) | res[0]) / 128;
  //  byte y = (int16_t)((res[3] << 8) | res[2]) / 128;
  //  byte z = (int16_t)((res[5] << 8) | res[4]) / 128;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("MSG:");
  display.setTextSize(2);
  display.println(msgText);
  //  display.print(x);
  //  display.print(",");
  //  display.print(y);
  //  display.print(",");
  //  display.println(z);
  display.setTextSize(1);
  display.display();
}

char * getSubChar(char original[], int startingPoint, int length) {
  //char result[length];
  for (int i = 0; i < length; i++) {
    if (strlen(original) < startingPoint + i) {
      break;
    }
    subChar[i] = original[startingPoint + i];
  }
  subChar[sizeof(subChar) - 1] = 0;
  return subChar;
}

//Displaying Quotes
void displayMenu3() {
  if ((millis() - lastMS) > scrollWaitMS ) {
    //let's take care of scrolling here
    if (msg[0] == 0 ) {
      //we are just starting, so get the msg"
      doneScrolling = false;
      getRandomQuote(); //loads quote into msg
      currentCharStep = -30;// make it start out a little to make it easier to read
      currentCharInt = 0;
      //strcpy(msg, tempMsg);
      //msg = getRandomQuote();
    }

    display.setRotation(0);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Message of the Moment");
    //let's test
    //    display.setCursor(0,10);
    //    display.print(msg);
    //    display.display();
    //    return;

    //let's determine if we will clip a character, or move the cursor;

    //increase the step
    currentCharStep += charScrollStep;

    if (currentCharStep >= stepsPerChar) {
      //clip msg and decrease currentCharStep
      currentCharInt++;
      currentCharStep -= stepsPerChar;
    }
    String thisLine;
    int width = 10;


    if (strlen(msg) < width) {
      width = strlen(msg);
      thisLine = msg;

    } else {
      thisLine = getSubChar(msg, currentCharInt, width);
    }
    if (msg[currentCharInt] == 0) {
      doneScrolling = true;
      msg[0] = 0;
    }
    display.setCursor((currentCharStep * -1), 10);
    display.setTextSize(2);
    display.print(thisLine);
    display.display();
    display.setTextSize(1);
    //reset lastMS
    lastMS = millis();
  }

}


//Bootloader
void displayMenu4() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Howdy From Arduino");
  display.print("v: ");
  display.println(CTversion);
  display.println("Hold for Bootloader");
  display.display();
}

//shows the time, with shake, or menu
void displayMenu77() {
  display.setRotation(3);
  display.clearDisplay();
  //  display.setCursor(0, 0);
  //  display.println(bleSymbol);
  display.setCursor(0, 11);
  display.print(getBatteryLevel());
  display.println("%");

  int tempHour = hour();
  if (tempHour > 12) {
    tempHour -= 12;
  }
  if (tempHour == 0) {
    tempHour = 12;
  }
  String stringHour;
  if (tempHour < 10) {
    stringHour = " " + String(tempHour);
  } else {
    stringHour = String(tempHour);
  }
  display.setTextSize(3);
  display.setCursor(-2, 27);
  display.print(stringHour.charAt(0));
  display.setCursor(14, 29);
  display.print(stringHour.charAt(1));



  String stringMinute;
  if (minute() < 10) {
    stringMinute = "0" + String(minute());
  } else {
    stringMinute = String(minute());
  }
  display.setCursor(-2, 60);
  display.print(stringMinute.charAt(0));
  display.setCursor(14, 62);
  display.print(stringMinute.charAt(1));
  display.setTextSize(1);
  display.setCursor(6, 87);
  if (isAM()) {
    display.print("AM");
  } else {
    display.print("PM");
  }
  display.setCursor(0, 99);
  display.print(dow());

  display.setCursor(0, 111);
  if (month() < 10) display.print(" ");
  display.print(month());
  display.print("/");
  if (day() < 10) display.print("0");
  display.print(day());
  display.display();
}

//charge menu
void displayMenu88() {
  display.setRotation(3);
  display.clearDisplay();
  display.setCursor(0, 30);
  display.println("Charge");
  display.display();
}

//this is the notification
void displayMenu99() {
  display.setRotation(0);
  digitalWrite(25, vibrationMode);
  if (vibrationMode)vibrationMode = false; else vibrationMode = true;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);// set this to 2 becuase otherwise it's too small
  display.println(msgText);
  display.setTextSize(1);
  display.display();
}

//The Game
void displayMenu5() {
  display.setRotation(0);
  uint32_t t;
  while (((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

  uint8_t res[6];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F , 6);
  res[0] = softread();
  res[1] = softread();
  res[2] = softread();
  res[3] = softread();
  res[4] = softread();
  res[5] = softread();
  int x = (int16_t)((res[1] << 8) | res[0]);
  int y = (int16_t)((res[3] << 8) | res[2]);
  int z = (int16_t)((res[5] << 8) | res[4]);

  float accelX = y;
  float accelY = -x;
  float accelZ = z;
  int16_t ax = -accelY / 256,      // Transform accelerometer axes
          ay =  accelX / 256,      // to grain coordinate space
          az = abs(accelZ) / 2048; // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1;         // Range of random motion to add back in

  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for (int i = 0; i < N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    v2 = (int32_t)grain[i].vx * grain[i].vx + (int32_t)grain[i].vy * grain[i].vy;
    if (v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0 * (float)grain[i].vx / v); // Maintain heading
      grain[i].vy = (int)(256.0 * (float)grain[i].vy / v); // Limit magnitude
    }
  }

  uint16_t        i, bytes, oldidx, newidx, delta;
  int16_t        newx, newy;

  for (i = 0; i < N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if (newx > MAX_X) {              // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if (newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if (newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if (newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y / 256) * WIDTH + (grain[i].x / 256); // Prior pixel #
    newidx = (newy      / 256) * WIDTH + (newx      / 256); // New pixel #
    if ((oldidx != newidx) && // If grain is moving to a new pixel...
        img[newidx]) {       // but if that pixel is already occupied...
      delta = abs(newidx - oldidx); // What direction when blocked?
      if (delta == 1) {           // 1 pixel left or right)
        newx         = grain[i].x;  // Cancel X motion
        grain[i].vx /= -2;          // and bounce X velocity (Y is OK)
        newidx       = oldidx;      // No pixel change
      } else if (delta == WIDTH) { // 1 pixel up or down
        newy         = grain[i].y;  // Cancel Y motion
        grain[i].vy /= -2;          // and bounce Y velocity (X is OK)
        newidx       = oldidx;      // No pixel change
      } else { // Diagonal intersection is more tricky...
        if ((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
          if (!img[newidx]) { // That pixel's free!  Take it!  But...
            newy         = grain[i].y; // Cancel Y motion
            grain[i].vy /= -2;         // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newx         = grain[i].x; // Cancel X motion
              grain[i].vx /= -2;         // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
          if (!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x; // Cancel X motion
            grain[i].vy /= -2;         // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 255;  // Set new spot
    grain[i].pos = newidx;
  }

  display.clearDisplay();
  //draw box
  //
  int margin = 5;
  int boxWidth = WIDTH - (2 * margin);
  int boxHeight = HEIGHT - (2 * margin);
  display.drawRect(margin, margin, boxWidth, boxHeight, WHITE);
  int thisScore = 0;
  for (i = 0; i < N_GRAINS; i++) {
    int yPos = grain[i].pos / WIDTH;
    int xPos = grain[i].pos % WIDTH;
    //display.drawPixel(xPos , yPos, WHITE);
    int boxWidth = 4;
    int halfBox = boxWidth / 2;
    display.fillRect(xPos - halfBox , yPos - halfBox , boxWidth, boxWidth, WHITE);
    //need to determine if grain is inside box
    int pointMargin = margin + halfBox + 1;
    if (
      (xPos > pointMargin && yPos > pointMargin)
      &&
      (xPos < (WIDTH - pointMargin) && yPos < (HEIGHT - pointMargin) )
    ) {
      thisScore++;

    }

  }
  totalScore += thisScore;

  display.setCursor(0, 0);
  display.print(totalScore);
  display.display();
}

char * getRandomQuote() {

  quoteIndex++;
  //char thisMsg[154];
  //strcpy(msg, msgPrefix);
  //msg
  switch (quoteIndex) {
    case 0:
      strncpy( msg, "It is better to have loved and lost than just to have lost.", sizeof(msg) );
      break;
    case 1:
      strncpy( msg, "It is bad luck to be superstitious.", sizeof(msg) );
      break;
    case 2:
      strncpy( msg, "If it jams, force it. If it breaks, it needed replacement anyway.", sizeof(msg) );
      break;
    case 3:
      strncpy( msg, "Always remember that you are unique. Just like everyone else.", sizeof(msg) );
      break;
    case 4:
      strncpy( msg, "A bachelor is a guy who is footloose and fiancee free.", sizeof(msg) );
      break;
    case 5:
      strncpy( msg, "If Yoda a great Jedi master he is, why not a good sentence construct can he?", sizeof(msg) );
      break;
    case 6:
      strncpy( msg, "People will remember you better if you always wear the same outfit.", sizeof(msg) );
      break;
    case 7:
      strncpy( msg, "All things are possible except for skiing through a revolving door.", sizeof(msg) );
      break;
    case 8:
      strncpy( msg, "Only two of my personalities are schizophrenic, but one of them is paranoid and the other one is out to get him.", sizeof(msg) );
      break;
    case 9:
      strncpy( msg, "If you love a thing of beauty, set it free. If it doesn't come back to you, hunt it down and kill it.", sizeof(msg) );
      break;
    case 10:
      strncpy( msg, "I may be schizophrenic, but at least I'll always have each other.", sizeof(msg) );
      break;
    case 11:
      strncpy( msg, "Give your child mental blocks for Christmas.", sizeof(msg) );
      break;
    case 12:
      strncpy( msg, "I stayed up all night playing poker with tarot cards. I got a full house and four people died.", sizeof(msg) );
      break;
    case 13:
      strncpy( msg, "It is impossible to make anything foolproof because fools are so ingenious.", sizeof(msg) );
      break;
    case 14:
      strncpy( msg, "Those who can't write, write manuals.", sizeof(msg) );
      break;
    case 15:
      strncpy( msg, "The brain is a wonderful organ:  it starts working the moment you get up in the morning, and does not stop until you get to school.", sizeof(msg) );
      break;
    case 16:
      strncpy( msg, "You'd be paranoid too if everybody hated you.", sizeof(msg) );
      break;
    case 17:
      strncpy( msg, "All generalities are false.", sizeof(msg) );
      break;
    case 18:
      strncpy( msg, "I'd give my right arm to be ambidextrous.", sizeof(msg) );
      break;
    case 19:
      strncpy( msg, "Duct tape is like the Force.  It has a light side, and a dark side, and it holds the universe together.", sizeof(msg) );
      break;
    case 20:
      strncpy( msg, "I used to think I was indecisive, but now I'm not so sure.", sizeof(msg) );
      break;
    case 21:
      strncpy( msg, "Cole's Law:  Thinly sliced cabbage.", sizeof(msg) );
      break;
    case 22:
      strncpy( msg, "Things are more like they used to be than they are now.", sizeof(msg) );
      break;
    case 23:
      strncpy( msg, "There is so much sand in Northern Africa that if it were spread out it would completely cover the Sahara Desert.", sizeof(msg) );
      break;
    case 24:
      strncpy( msg, "It has been said that we only use 15% of our brain.  I wonder what we do with the other 75%?", sizeof(msg) );
      break;
    case 25:
      strncpy( msg, "If you want your spouse to listen and pay strict attention to every word you say, talk in your sleep.", sizeof(msg) );
      break;
    case 26:
      strncpy( msg, "If you have a difficult task, give it to someone lazy ... that person will find an easier way to do it.", sizeof(msg) );
      break;
    case 27:
      strncpy( msg, "You know it's going to be a bad day when your car horn goes off accidentally and remains stuck as you follow a group of Hell's Angels on the freeway.", sizeof(msg) );
      break;
    case 28:
      strncpy( msg, "The way to make a small fortune in the commodities market is to start with a large fortune.", sizeof(msg) );
      break;
    case 29:
      strncpy( msg, "A closed mouth gathers no feet.", sizeof(msg) );
      break;
    case 30:
      strncpy( msg, "A journey of a thousand miles begins with a cash advance.", sizeof(msg) );
      break;
    case 31:
      strncpy( msg, "A king's castle is his home.", sizeof(msg) );
      break;
    case 32:
      strncpy( msg, "A penny saved is ridiculous.", sizeof(msg) );
      break;
    case 33:
      strncpy( msg, "All that glitters has a high refractive index.", sizeof(msg) );
      break;
    case 34:
      strncpy( msg, "Ambition a poor excuse for not having enough sense to be lazy.", sizeof(msg) );
      break;
    case 35:
      strncpy( msg, "Anarchy is better that no government at all.", sizeof(msg) );
      break;
    case 36:
      strncpy( msg, "Any small object when dropped will hide under a larger object.", sizeof(msg) );
      break;
    case 37:
      strncpy( msg, "Automobile - A mechanical device that runs up hills and down people.", sizeof(msg) );
      break;
    case 38:
      strncpy( msg, "Be moderate where pleasure is concerned, avoid fatigue.", sizeof(msg) );
      break;
    case 39:
      strncpy( msg, "Brain -- the apparatus with which we think that we think.", sizeof(msg) );
      break;
    case 40:
      strncpy( msg, "BATCH - A group, kinda like a herd.", sizeof(msg) );
      break;
    case 41:
      strncpy( msg, "omputer modelers simulate it first.", sizeof(msg) );
      break;
    case 42:
      strncpy( msg, "Computer programmers don't byte, they nybble a bit.", sizeof(msg) );
      break;
    case 43:
      strncpy( msg, "Computer programmers know how to use their hardware.", sizeof(msg) );
      break;
    case 44:
      strncpy( msg, "Computers are not intelligent.  They only think they are.", sizeof(msg) );
      break;
    case 45:
      strncpy( msg, "Courage is your greatest present need.", sizeof(msg) );
      break;
    case 46:
      strncpy( msg, "Death is life's way of telling you you've been fired.", sizeof(msg) );
      break;
    case 47:
      strncpy( msg, "Death is Nature's way of saying 'slow down'.", sizeof(msg) );
      break;
    case 48:
      strncpy( msg, "Do something unusual today.  Accomplish work on the computer.", sizeof(msg) );
      break;
    case 49:
      strncpy( msg, "Don't force it, get a larger hammer.", sizeof(msg) );
      break;
    case 50:
      strncpy( msg, "Don't hate yourself in the morning -- sleep till noon.", sizeof(msg) );
      break;
    case 51:
      strncpy( msg, "Drive defensively -- buy a tank.", sizeof(msg) );
      break;
    case 52:
      strncpy( msg, "Earn cash in your spare time -- blackmail friends.", sizeof(msg) );
      break;
    case 53:
      strncpy( msg, "Entropy isn't what it used to be.", sizeof(msg) );
      break;
    case 54:
      strncpy( msg, "Fairy tales: horror stories for children to get them use to reality.", sizeof(msg) );
      break;
    case 55:
      strncpy( msg, "Familiarity breeds children.", sizeof(msg) );
      break;
    case 56:
      strncpy( msg, "God didn't create the world in 7 days.  He pulled an all-nighter on the 6th.", sizeof(msg) );
      break;
    case 57:
      strncpy( msg, "Going the speed of light is bad for your age.", sizeof(msg) );
      break;
    case 58:
      strncpy( msg, "He who hesitates is sometimes saved.", sizeof(msg) );
      break;
    case 59:
      strncpy( msg, "Health is merely the slowest possible rate at which one can die.", sizeof(msg) );
      break;
    case 60:
      strncpy( msg, "Help support helpless victims of computer error.", sizeof(msg) );
      break;
    case 61:
      strncpy( msg, "Herblock's Law: if it is good, they will stop making it.", sizeof(msg) );
      break;
    case 62:
      strncpy( msg, "History does not repeat itself, -- historians merely repeat each other.", sizeof(msg) );
      break;
    case 63:
      strncpy( msg, "If you don't change your direction, you may end up where you were headed.", sizeof(msg) );
      break;
    case 64:
      strncpy( msg, "If you're not part of the solution, be part of the problem!", sizeof(msg) );
      break;
    case 65:
      strncpy( msg, "In the field of observation, chance favors only the prepared minds.", sizeof(msg) );
      break;
    case 66:
      strncpy( msg, "It is a miracle that curiosity survives formal education.  Albert Einstein", sizeof(msg) );
      break;
    case 67:
      strncpy( msg, "It works better if you plug it in.", sizeof(msg) );
      break;
    case 68:
      strncpy( msg, "It's not hard to meet expenses, they're everywhere.", sizeof(msg) );
      break;
    case 69:
      strncpy( msg, "Jury -- Twelve people who determine which client has the better lawyer.", sizeof(msg) );
      break;
    case 70:
      strncpy( msg, "KODACLONE - duplicating film.", sizeof(msg) );
      break;
    case 71:
      strncpy( msg, "Let not the sands of time get in your lunch.", sizeof(msg) );
      break;
    case 72:
      strncpy( msg, "Life is what happens to you while you are planning to do something else.", sizeof(msg) );
      break;
    case 73:
      strncpy( msg, "Lynch's Law: When the going gets tough, everyone leaves.", sizeof(msg) );
      break;
    case 74:
      strncpy( msg, "Mediocrity thrives on standardization.", sizeof(msg) );
      break;
    case 75:
      strncpy( msg, "MOP AND GLOW - Floor wax used by Three Mile Island cleanup team.", sizeof(msg) );
      break;
    case 76:
      strncpy( msg, "Never lick a gift horse in the mouth.", sizeof(msg) );
      break;
    case 77:
      strncpy( msg, "Old MacDonald had an agricultural real estate tax abatement.", sizeof(msg) );
      break;
    case 78:
      strncpy( msg, "Quoting one is plagiarism.  Quoting many is research.", sizeof(msg) );
      break;
    case 79:
      strncpy( msg, "Reality's the only obstacle to happiness.", sizeof(msg) );
      break;
    case 80:
      strncpy( msg, "Screw up your life, you've screwed everything else up.", sizeof(msg) );
      break;
    case 81:
      strncpy( msg, "Silver's law:   If Murphy's law can go wrong it will.", sizeof(msg) );
      break;
    case 82:
      strncpy( msg, "Some grow with responsibility, others just swell.", sizeof(msg) );
      break;
    case 83:
      strncpy( msg, "The attention span of a computer is as long as its electrical cord.", sizeof(msg) );
      break;
    case 84:
      strncpy( msg, "The only difference between a rut and a grave is the depth.", sizeof(msg) );
      break;
    case 85:
      strncpy( msg, "The road to to success is always under construction.", sizeof(msg) );
      break;
    case 86:
      strncpy( msg, "Those who can't write, write help files.", sizeof(msg) );
      break;
    case 87:
      strncpy( msg, "To be, or not to be, those are the parameters.", sizeof(msg) );
      break;
    case 88:
      strncpy( msg, "To err is human, to really foul things up requires a computer.", sizeof(msg) );
      break;
    case 89:
      strncpy( msg, "Today is the last day of your life so far.", sizeof(msg) );
      break;
    case 90:
      strncpy( msg, "TRAPEZOID - A device for catching zoids.", sizeof(msg) );
      break;
    case 91:
      strncpy( msg, "Wasting time is an important part of life.", sizeof(msg) );
      break;
    case 92:
      strncpy( msg, "When all else fails, read the instructions.", sizeof(msg) );
      break;
    case 93:
      strncpy( msg, "When in doubt, don't bother.", sizeof(msg) );
      break;
    case 94:
      strncpy( msg, "When in doubt, ignore it.", sizeof(msg) );
      quoteIndex = -1;
      break;
  }


  msg[sizeof(msg) - 1] = 0;
  return msg;



}

char *  dow() {
  long t = now();
  int dayNum = int( ((t / 86400) + 4) % 7);

  //strncpy( a, "Hello", sizeof(a) );


  //char result[6];
  switch (dayNum) {
    case 0:
      strncpy( currentDOW, "SUN", sizeof(currentDOW) );
      //result = "SUN";
      break;
    case 1:
      strncpy( currentDOW, "MON", sizeof(currentDOW) );
      break;
    case 2:
      strncpy( currentDOW, "TUES", sizeof(currentDOW) );
      break;
    case 3:
      strncpy( currentDOW, "WED", sizeof(currentDOW) );
      break;
    case 4:
      strncpy( currentDOW, "THURS", sizeof(currentDOW) );
      break;
    case 5:
      strncpy( currentDOW, "FRI", sizeof(currentDOW) );
      break;
    case 6:
      strncpy( currentDOW, "SAT", sizeof(currentDOW) );
      break;

  }
  currentDOW[sizeof(currentDOW) - 1] = 0;
  return currentDOW;
}
