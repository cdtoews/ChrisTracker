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

#define sleepDelay 12000
#define BUTTON_PIN              30
#define refreshRate 100

int menu;
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

String msg = "";
boolean doneScrolling = true;
int scrollWaitMS = 1;
int stepsPerChar = 10;
int currentCharStep = 0;
long lastMS = millis();
int charScrollStep = 5;
int quoteIndex;
int quoteSize = 95;

String bleSymbol = " ";
int contrast;

BLEPeripheral                   blePeripheral           = BLEPeripheral();
BLEService                      batteryLevelService     = BLEService("190A");
BLECharacteristic   TXchar        = BLECharacteristic("0002", BLENotify, 20);
BLECharacteristic   RXchar        = BLECharacteristic("0001", BLEWriteWithoutResponse, 20);

BLEService                      batteryLevelService1     = BLEService("190B");
BLECharacteristic   TXchar1        = BLECharacteristic("0004", BLENotify, 20);
BLECharacteristic   RXchar1        = BLECharacteristic("0003", BLEWriteWithoutResponse, 20);

#define N_GRAINS     1 // Number of grains of sand
#define WIDTH        127 // Display width in pixels
#define HEIGHT       32 // Display height in pixels
#define MAX_FPS      150 // Maximum redraw rate, frames/second
int totalScore;


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
  else menu = 0;
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
  } else if (Command.substring(0, 8) == "AT+PUSH=") {
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
  totalScore = 0;
  //get random seed by accelerometer
  uint8_t res[6];
  softbeginTransmission(0x1F);
  softwrite(0x06);
  softendTransmission();
  softrequestFrom(0x1F , 6);
  res[0] = softread();
  res[1] = softread();
  int x = (int16_t)((res[1] << 8) | res[0]);
  randomSeed(x);
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
      //      case 2:
      //        if (millis() - displayRefreshTime > refreshRate) {
      //          displayRefreshTime = millis();
      //          displayMenu2();
      //        }
      //        break;
      case 3:
        if (millis() - displayRefreshTime > refreshRate) {
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
        case 0:
          menu = 3;
          break;
        //        case 1:
        //          menu = 2;
        //          break;
        //        case 2:
        //          menu = 3;
        //          break;
        case 3:
          //reset msg;
          msg = "";
          doneScrolling = true;
          menu = 4;
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
          }
          break;
        case 5:
          totalScore = 0;
          menu = 0;
          break;
        case 77:
          menu = 0;
          break;
        case 88:
          menu = 0;
          break;
        case 99:
          digitalWrite(25, LOW);
          menu = 0;
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
      case 77:
        if (millis() - sleepTime > 3000 ) powerDown();
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

void displayMenu0() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(bleSymbol);
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

//void displayMenu2() {
//  display.setRotation(0);
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
//
//  display.clearDisplay();
//  display.setCursor(0, 0);
//  display.println("Menue2 PushMSG:");
//  display.println(msgText);
//  display.print(x);
//  display.print(",");
//  display.print(y);
//  display.print(",");
//  display.println(z);
//  display.display();
//}


void displayMenu3() {
  if ((millis() - lastMS) > scrollWaitMS ) {
    //let's take care of scrolling here
    if (msg == "") {
      //we are just starting, so set the msg"
      doneScrolling = false;
      msg = getRandomQuote();
    }

    display.setRotation(0);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Message of the Moment");
    //let's determine if we will clip a character, or move the cursor;
    if (currentCharStep >= stepsPerChar) {
      //clip msg and reset
      msg = msg.substring(1);
      currentCharStep = 0;
    } else {
      currentCharStep += charScrollStep;
    }
    String thisLine;
    int width = 10;
    if (msg.length() < width) {
      width = msg.length();
      thisLine = msg;

    } else {
      thisLine = msg.substring(0, width);

    }
    if (msg.length() < 2) {
      doneScrolling = true;
      msg = "";
    }
    display.setCursor((currentCharStep * -1), 10);
    display.setTextSize(2);
    display.println(thisLine);
    display.display();
    display.setTextSize(1);
    //reset lastMS
    lastMS = millis();
  }

}


void displayMenu4() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Howdy From Arduino");
  display.println("  :)");
  display.println("Hold for Bootloader");
  display.display();
}

//shows the time when you shake the tracker
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
  if (month() < 10) display.print("0");
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


void displayMenu5() {
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


String getRandomQuote() {

  quoteIndex++;
  String thisMsg;

  //this seems a horrible way to iterate through strings.
  switch (quoteIndex) {
    case 0:
      thisMsg = "It is better to have loved and lost than just to have lost.";
      break;
    case 1:
      thisMsg = "It is bad luck to be superstitious.";
      break;
    case 2:
      thisMsg = "If it jams, force it. If it breaks, it needed replacement anyway.";
      break;
    case 3:
      thisMsg = "Always remember that you are unique. Just like everyone else.";
      break;
    case 4:
      thisMsg = "A bachelor is a guy who is footloose and fiancee free.";
      break;
    case 5:
      thisMsg = "If Yoda a great Jedi master he is, why not a good sentence construct can he?";
      break;
    case 6:
      thisMsg = "People will remember you better if you always wear the same outfit.";
      break;
    case 7:
      thisMsg = "All things are possible except for skiing through a revolving door.";
      break;
    case 8:
      thisMsg = "Only two of my personalities are schizophrenic, but one of them is paranoid and the other one is out to get him.";
      break;
    case 9:
      thisMsg = "If you love a thing of beauty, set it free. If it doesn't come back to you, hunt it down and kill it.";
      break;
    case 10:
      thisMsg = "I may be schizophrenic, but at least I'll always have each other.";
      break;
    case 11:
      thisMsg = "Give your child mental blocks for Christmas.";
      break;
    case 12:
      thisMsg = "I stayed up all night playing poker with tarot cards. I got a full house and four people died.";
      break;
    case 13:
      thisMsg = "It is impossible to make anything foolproof because fools are so ingenious.";
      break;
    case 14:
      thisMsg = "Those who can't write, write manuals.";
      break;
    case 15:
      thisMsg = "The brain is a wonderful organ:  it starts working the moment you get up in the morning, and does not stop until you get to school.";
      break;
    case 16:
      thisMsg = "You'd be paranoid too if everybody hated you.";
      break;
    case 17:
      thisMsg = "All generalities are false.";
      break;
    case 18:
      thisMsg = "I'd give my right arm to be ambidextrous.";
      break;
    case 19:
      thisMsg = "Duct tape is like the Force.  It has a light side, and a dark side, and it holds the universe together.";
      break;
    case 20:
      thisMsg = "I used to think I was indecisive, but now I'm not so sure.";
      break;
    case 21:
      thisMsg = "Cole's Law:  Thinly sliced cabbage.";
      break;
    case 22:
      thisMsg = "Things are more like they used to be than they are now.";
      break;
    case 23:
      thisMsg = "There is so much sand in Northern Africa that if it were spread out it would completely cover the Sahara Desert.";
      break;
    case 24:
      thisMsg = "It has been said that we only use 15% of our brain.  I wonder what we do with the other 75%?";
      break;
    case 25:
      thisMsg = "If you want your spouse to listen and pay strict attention to every word you say, talk in your sleep.";
      break;
    case 26:
      thisMsg = "If you have a difficult task, give it to someone lazy ... that person will find an easier way to do it.";
      break;
    case 27:
      thisMsg = "You know it's going to be a bad day when your car horn goes off accidentally and remains stuck as you follow a group of Hell's Angels on the freeway.";
      break;
    case 28:
      thisMsg = "The way to make a small fortune in the commodities market is to start with a large fortune.";
      break;
    case 29:
      thisMsg = "A closed mouth gathers no feet.";
      break;
    case 30:
      thisMsg = "A journey of a thousand miles begins with a cash advance.";
      break;
    case 31:
      thisMsg = "A king's castle is his home.";
      break;
    case 32:
      thisMsg = "A penny saved is ridiculous.";
      break;
    case 33:
      thisMsg = "All that glitters has a high refractive index.";
      break;
    case 34:
      thisMsg = "Ambition a poor excuse for not having enough sense to be lazy.";
      break;
    case 35:
      thisMsg = "Anarchy is better that no government at all.";
      break;
    case 36:
      thisMsg = "Any small object when dropped will hide under a larger object.";
      break;
    case 37:
      thisMsg = "Automobile - A mechanical device that runs up hills and down people.";
      break;
    case 38:
      thisMsg = "Be moderate where pleasure is concerned, avoid fatigue.";
      break;
    case 39:
      thisMsg = "Brain -- the apparatus with which we think that we think.";
      break;
    case 40:
      thisMsg = "BATCH - A group, kinda like a herd.";
      break;
    case 41:
      thisMsg = "omputer modelers simulate it first.";
      break;
    case 42:
      thisMsg = "Computer programmers don't byte, they nybble a bit.";
      break;
    case 43:
      thisMsg = "Computer programmers know how to use their hardware.";
      break;
    case 44:
      thisMsg = "Computers are not intelligent.  They only think they are.";
      break;
    case 45:
      thisMsg = "Courage is your greatest present need.";
      break;
    case 46:
      thisMsg = "Death is life's way of telling you you've been fired.";
      break;
    case 47:
      thisMsg = "Death is Nature's way of saying 'slow down'.";
      break;
    case 48:
      thisMsg = "Do something unusual today.  Accomplish work on the computer.";
      break;
    case 49:
      thisMsg = "Don't force it, get a larger hammer.";
      break;
    case 50:
      thisMsg = "Don't hate yourself in the morning -- sleep till noon.";
      break;
    case 51:
      thisMsg = "Drive defensively -- buy a tank.";
      break;
    case 52:
      thisMsg = "Earn cash in your spare time -- blackmail friends.";
      break;
    case 53:
      thisMsg = "Entropy isn't what it used to be.";
      break;
    case 54:
      thisMsg = "Fairy tales: horror stories for children to get them use to reality.";
      break;
    case 55:
      thisMsg = "Familiarity breeds children.";
      break;
    case 56:
      thisMsg = "God didn't create the world in 7 days.  He pulled an all-nighter on the 6th.";
      break;
    case 57:
      thisMsg = "Going the speed of light is bad for your age.";
      break;
    case 58:
      thisMsg = "He who hesitates is sometimes saved.";
      break;
    case 59:
      thisMsg = "Health is merely the slowest possible rate at which one can die.";
      break;
    case 60:
      thisMsg = "Help support helpless victims of computer error.";
      break;
    case 61:
      thisMsg = "Herblock's Law: if it is good, they will stop making it.";
      break;
    case 62:
      thisMsg = "History does not repeat itself, -- historians merely repeat each other.";
      break;
    case 63:
      thisMsg = "If you don't change your direction, you may end up where you were headed.";
      break;
    case 64:
      thisMsg = "If you're not part of the solution, be part of the problem!";
      break;
    case 65:
      thisMsg = "In the field of observation, chance favors only the prepared minds.";
      break;
    case 66:
      thisMsg = "It is a miracle that curiosity survives formal education.  Albert Einstein";
      break;
    case 67:
      thisMsg = "It works better if you plug it in.";
      break;
    case 68:
      thisMsg = "It's not hard to meet expenses, they're everywhere.";
      break;
    case 69:
      thisMsg = "Jury -- Twelve people who determine which client has the better lawyer.";
      break;
    case 70:
      thisMsg = "KODACLONE - duplicating film.";
      break;
    case 71:
      thisMsg = "Let not the sands of time get in your lunch.";
      break;
    case 72:
      thisMsg = "Life is what happens to you while you are planning to do something else.";
      break;
    case 73:
      thisMsg = "Lynch's Law: When the going gets tough, everyone leaves.";
      break;
    case 74:
      thisMsg = "Mediocrity thrives on standardization.";
      break;
    case 75:
      thisMsg = "MOP AND GLOW - Floor wax used by Three Mile Island cleanup team.";
      break;
    case 76:
      thisMsg = "Never lick a gift horse in the mouth.";
      break;
    case 77:
      thisMsg = "Old MacDonald had an agricultural real estate tax abatement.";
      break;
    case 78:
      thisMsg = "Quoting one is plagiarism.  Quoting many is research.";
      break;
    case 79:
      thisMsg = "Reality's the only obstacle to happiness.";
      break;
    case 80:
      thisMsg = "Screw up your life, you've screwed everything else up.";
      break;
    case 81:
      thisMsg = "Silver's law:   If Murphy's law can go wrong it will.";
      break;
    case 82:
      thisMsg = "Some grow with responsibility, others just swell.";
      break;
    case 83:
      thisMsg = "The attention span of a computer is as long as its electrical cord.";
      break;
    case 84:
      thisMsg = "The only difference between a rut and a grave is the depth.";
      break;
    case 85:
      thisMsg = "The road to to success is always under construction.";
      break;
    case 86:
      thisMsg = "Those who can't write, write help files.";
      break;
    case 87:
      thisMsg = "To be, or not to be, those are the parameters.";
      break;
    case 88:
      thisMsg = "To err is human, to really foul things up requires a computer.";
      break;
    case 89:
      thisMsg = "Today is the last day of your life so far.";
      break;
    case 90:
      thisMsg = "TRAPEZOID - A device for catching zoids.";
      break;
    case 91:
      thisMsg = "Wasting time is an important part of life.";
      break;
    case 92:
      thisMsg = "When all else fails, read the instructions.";
      break;
    case 93:
      thisMsg = "When in doubt, don't bother.";
      break;
    case 94:
      thisMsg  = "When in doubt, ignore it.";
      quoteIndex = -1;
      break;
  }


  return "  " +  thisMsg;

}

String dow() {
  long t = now();
  int dayNum = int( ((t / 86400) + 4) % 7);

  String result = "";
  switch (dayNum) {
    case 1:
      result = "MON";
      break;
    case 2:
      result = "TUES";
      break;
    case 3:
      result = "WED";
      break;
    case 4:
      result = "THUR";
      break;
    case 5:
      result = "FRI";
      break;
    case 6:
      result = "SAT";
      break;
    case 0:
      result = "SUN";
      break;
  }
  return result;
}
