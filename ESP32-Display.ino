#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include "myfont.h"

#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
// OR #include "SH1106Wire.h"   // legacy: #include "SH1106.h"

#define RX_PIN 16
#define TX_PIN 17
#define TIMEOUT 20
#define PWR_TIMEOUT 2000
#define PROTO_1_BYTES 11
#define PROTO_2_BYTES 8
#define LAST_PROTO 1

#define HALL_MIN 500
#define HALL_MAX 3500
#define ACC_PIN 34    // Пин подключения датчика холла акселератора
#define REG_PIN 35    // Пин подключения датчика холла тормоза
#define BAT_PIN 36    // Пин подключения резистивного делителя напряжения (200к/10к) АКБ
#define POWER_PIN 23  // Пин на блок питания (поддержка после отпуская конопки питания)
#define POWER_BT 19   // Пин подключения кнопки питания (через диод)
#define SPEED_BT 18   // Пин подключения кнопки скорости (нажата LOW)
#define WHEEL_D 7.8
#define MY_NAME "ESP32dsp"
#define PROTO_ERR 1
#define ACC_REG_ERR 5
#define MAX_SENSOR_VALUE 4095

#define PROG_VER 0.17


//bool start_p = false;
byte byte_p = 0;
byte uart_protocol = 0;
byte packet[30] = {0};
byte inByte, oldByte, cmdByte = 0;
uint8_t BTmac[6] = {0};
byte r_packet[] = {0x3e, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05};
byte start_packet[] = {0x3e, 0x2c, 0x0b, 0x00, 0x3c, 0x28, 0x1e, 0x32, 0x46, 0x64, 0x01, 0x00, 0x00, 0x00, 0x01, 0x96};

unsigned long loop_time;
unsigned long pwr_bt_time;

int16_t bat_current;
//byte accValue = 0;
//byte regValue = 0;
float battery_fvol = 0;
byte acc_vol, reg_vol = 0;
char myBTName[22] = {0};
byte Err_N = 0;

uint16_t min_acc_value = 800;
uint16_t max_acc_value = 3080;
uint16_t min_reg_value = 800;
uint16_t max_reg_value = 3080;

int16_t volt_correct = 130;   // корекция вольтметра 130 ~= 2 Вольт

byte max_speed;
unsigned int sensorValue = 0;
unsigned long pinValue = 0;

boolean is_start_p = true;
boolean is_error = false;
boolean mk_ok = false;
boolean pwr_bt = false;
boolean Power_Off = false;
boolean Power_On = true;

uint8_t power_bt = HIGH;

// boolean run_loop = true;

HardwareSerial mySerial(2);
BluetoothSerial SerialBT;

// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA, SCL);
// ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h



void setup() {
  uint16_t hallValue = 0;


  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(POWER_BT, INPUT_PULLUP);
  pinMode(SPEED_BT, INPUT_PULLUP);
  pinMode(BAT_PIN, INPUT);
  pinMode(ACC_PIN, INPUT);
  pinMode(REG_PIN, INPUT);

  digitalWrite(POWER_PIN,HIGH);

  esp_read_mac(BTmac, ESP_MAC_BT);
  sprintf(myBTName, "%s%02X%02X%02X%02X%02X%02X", MY_NAME, BTmac[0], BTmac[1], BTmac[2], BTmac[3], BTmac[4], BTmac[5]);


 // Initialising the UI will init the display too.
  display.init();

//  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 20, "VER: " + String(PROG_VER));
  display.display();

  // initialize both serial ports:
  Serial.begin(115200);
  Serial.println("Start ...");

  SerialBT.begin(myBTName); //Bluetooth device name
  SerialBT.println("Start ...");


//  Serial.println("Try to detect baudrate with inverted logic.");
//  SerialBT.println("Try to detect baudrate with inverted logic.");

//  mySerial.begin(0, SERIAL_8N1, -1, -1, true, 11000UL);  // Passing 0 for baudrate to detect it, the last parameter is a timeout in ms

/*  unsigned long detectedBaudRate = mySerial.baudRate();

     if(detectedBaudRate) {
       Serial.printf("Detected baudrate is %lu\n", detectedBaudRate);
       SerialBT.printf("Detected baudrate is %lu\n", detectedBaudRate);
     } else {
       Serial.println("No baudrate detected, trying to use normal logic.");
       SerialBT.println("No baudrate detected, trying to use normal logic.");

       mySerial.begin(0, SERIAL_8N1, -1, -1, false, 11000UL);  // Passing 0 for baudrate to detect it, the last parameter is a timeout in ms
       detectedBaudRate = mySerial.baudRate();

       if(detectedBaudRate) {
          Serial.printf("Detected baudrate is %lu\n", detectedBaudRate);
          SerialBT.printf("Detected baudrate is %lu\n", detectedBaudRate);
       } else {
          Serial.println("No baudrate detected, mySerial will not work!");
          SerialBT.println("No baudrate detected, mySerial will not work!");
       }
     }
*/
  
  mySerial.begin(9600,SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("Port 9600 8N1 RX pin 16 TX pin 17");
//  SerialBT.println("Port 9600 8N1 RX pin 16 TX pin 17");

  hallValue = analogRead(ACC_PIN);

  if(hallValue < HALL_MIN || hallValue > HALL_MAX) {
      Serial.printf("Значение акселератора: %d\n", hallValue);
      SerialBT.printf("ERR ACC: %d\n", hallValue);
      is_error = true;
      Err_N = ACC_REG_ERR; // TODO:
  }

  hallValue = analogRead(REG_PIN);

  if(hallValue < HALL_MIN || hallValue > HALL_MAX) {
      Serial.printf("Значение тормоза: %d\n", hallValue);
      SerialBT.printf("ERR REG: %d\n", hallValue);
//      is_error = true;
//      Err_N = ACC_REG_ERR; // TODO:
  }
  

  calc_start_CRC(LAST_PROTO);

  loop_time = millis() + TIMEOUT;
  
}

void loop() {

  if(Power_Off) {   // режим выключения, небольшой таймаут с инфомацией на экране о выключении
    displayPowerOff();
    digitalWrite(POWER_PIN, LOW);
    delay(500);
    while(true);  
  }

  tik();    // Отправка сообщения контроллеру MK если надо.

  check_battery();  // Считываем напряжение АКБ

  check_mySerial();

  check_serial();

  check_BTserial();

  check_pwr_bts();

}

void tik() {
  if(uart_protocol) {   // протокол обмена с контроллером МК уже известен
    if(millis() > loop_time) {
      sendPacket(uart_protocol);
      loop_time = millis() + TIMEOUT;
    }
  }
}


void check_mySerial() {

  if (mySerial.available()) {
    oldByte = inByte;
    inByte = mySerial.read();

    Serial.printf("%.2X ",inByte);

    if(uart_protocol) {
      packet[byte_p++] = inByte;
    } else {
      if(inByte == 0x07 && oldByte == 0x3c) {
        uart_protocol = 1;
        byte_p = 0;
        packet[byte_p++] = 0x3c;
        packet[byte_p++] = 0x07;
        Serial.println();
      } else // if (inByte == 0x28) TODO!!!
        packet[byte_p++] = inByte;
    }

    if(uart_protocol == 1 && byte_p == PROTO_1_BYTES) {
      if(packet[0] == 0x3c && packet[1] == 0x07) {
        printPacket();
        byte_p = 0;
        mk_ok = true;
      }
    } else if (uart_protocol == 2 && byte_p == PROTO_2_BYTES) {
      if(packet[0] == 0x28) {
        printPacket();
        byte_p = 0;
        mk_ok = true;
      }
    }

    if(byte_p >= 29) { // ошибка приема данных, не верный протокол?!
      uart_protocol = 0;
      mk_ok = false;
      protoError();
      byte_p = 0; 
    }
    
  }
}

void sendPacket(byte p_ver) {
  if(is_start_p&&mk_ok) {
    is_start_p = false;
    
    mySerial.write(start_packet,sizeof(start_packet));
  } else if(!is_error) {  // если ошибка не посылаем обратный пакет ???
    r_packet[4] = readACC();
    r_packet[5] = readREG();

    int16_t myCRC = r_packet[1] + r_packet[2] + r_packet[3] + r_packet[4] + r_packet[5];
    r_packet[6] = highByte(myCRC);
    r_packet[7] = lowByte(myCRC);

    mySerial.write(r_packet,sizeof(r_packet));
  }
}

void protoError() {
  Serial.println();
  Serial.println("Protocol unknown!!!");
  SerialBT.println("PROTO ERR");
  Err_N = PROTO_ERR;
  displayError();
  
}

void printPacket() {

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
  Serial.println();
//  SerialBT.println();
  Serial.print("RECV PACKET: ");
//  SerialBT.println("RECV PACKET: ");
  
  for ( byte element=0; element<=byte_p-1; element++) {  // printf(tmp, "0x%.2X",data[i]);
    Serial.printf("%.2X ",packet[element]);
    SerialBT.printf("%.2X ",packet[element]);
  }
//  Serial.println();

  if(checkCRC(uart_protocol)) {
    Serial.println("CRC OK");
    SerialBT.println("OK");
    if(is_error) displayError(); 
      else printPacketInfo(uart_protocol);
  }
  else { 
    Serial.println("CRC BAD!");
    SerialBT.println("!CRC");
  }

  Serial.printf("SEND PACKET: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n", r_packet[0], r_packet[1], r_packet[2], r_packet[3], r_packet[4], r_packet[5], r_packet[6], r_packet[7]);
}


void check_pwr_bts() {
  
  power_bt = digitalRead(POWER_BT);

  if(Power_On && power_bt == LOW) return; // всё еще включаемся

  if(Power_On && power_bt == HIGH) {   // включились
    Power_On = false;
    return;
  }

  if(power_bt == LOW) // кнопка питания нажата
    
    if(!pwr_bt) { // кнопку питания только нажали 
        pwr_bt = true;
        pwr_bt_time = millis() + PWR_TIMEOUT;
    } else { // кнопка питания уже была нажата
      if(millis() > pwr_bt_time) Power_Off = true; // прошло 2 секунды для выключения ?
    } else {
      if(pwr_bt) pwr_bt = false;
    }
  
}


void check_serial() {
  if (Serial.available()) {
    cmdByte = Serial.read();
    Serial.printf("\nGot: %.2X\n",cmdByte);

    if(cmdByte == 'v' || cmdByte == 'V') Serial.printf("Version: %.2f\n",PROG_VER);
    if(cmdByte == 'a' || cmdByte == 'A') Serial.printf("ACC: %d\n",readACC());
    if(cmdByte == 'r' || cmdByte == 'R') Serial.printf("REG: %d\n",readREG());
    
    if(cmdByte == 's' || cmdByte == 'S') Serial.printf("SPEED: %d\n",r_packet[2]);

  }
  
}


void check_BTserial() {
  if (SerialBT.available()) {
    cmdByte = SerialBT.read();
    SerialBT.printf("\nGot: %.2X\n",cmdByte);

    if(cmdByte == 'v' || cmdByte == 'V') SerialBT.printf("Version: %.2f\n",PROG_VER);
    if(cmdByte == 'a' || cmdByte == 'A') SerialBT.printf("ACC: %d\n",readACC());
    if(cmdByte == 'r' || cmdByte == 'R') SerialBT.printf("REG: %d\n",readREG());
    
    if(cmdByte == '1') r_packet[2] = 0x01;  // переключаем скорость на 1
    if(cmdByte == '2') r_packet[2] = 0x02;  // переключаем скорость на 2
    if(cmdByte == '3') r_packet[2] = 0x03;  // переключаем скорость на 3
    
    if(cmdByte == 's' || cmdByte == 'S') SerialBT.printf("SPEED: %d\n",r_packet[2]);
  }
  
}



boolean checkCRC(byte p_ver) {

  uint16_t pCRC, myCRC = 0;
  if(p_ver == 1) {
    pCRC = (packet[9]<<8)+packet[10];
    myCRC = packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8];
    if(myCRC == pCRC) 
      return true;    
  } else if (p_ver == 2) {
    pCRC = packet[7];
    myCRC = packet[0]^packet[1]^packet[2]^packet[3]^packet[4]^packet[5]^packet[6];
    if(myCRC == pCRC) 
      return true;
  }
  Serial.printf("CRC Packet=0x%.2X Our=0x%.2X PROTO: %d\n", pCRC, myCRC, p_ver);
  return false;
}

void printPacketInfo(byte p_ver) {
  byte mySpeed;
  uint16_t mk_speed;

  Serial.printf("Протокол: %d\n", p_ver);

  if(p_ver == 1) {
    mk_speed = ((packet[5]<<8)+packet[6]);
    mySpeed = ((((1000/mk_speed)*60) * ((WHEEL_D*3.14)/39)) * 60) / 1000;
    bat_current = packet[4];
    if(packet[2] == 0) Serial.println("Двигатель блокирован"); 
      else if(packet[2] == 1) Serial.println("Нормальная работа"); 
        else if(packet[2] == 3) Serial.println("Настройки приняты"); 
          else Serial.println("Неизвестное значение 3-го байта!");
    if(packet[3] & (1 << 0)) Serial.println("Ошибка двигателя (M)");
    if(packet[3] & (1 << 1)) Serial.println("Ошибка 'ECU'");
    if(packet[3] & (1 << 2)) Serial.println("Тормоз '!'");
    Serial.printf("Ток: %d Ампер\n", bat_current);
    Serial.printf("Скорость: %d км/ч\n", mySpeed);

  } else if (p_ver == 2) {
    mk_speed = (packet[5]<<8)+packet[6];
    mySpeed = ((((1000/mk_speed)*60) * ((WHEEL_D*3.14)/39)) * 60) / 1000;
    bat_current = (packet[3]<<8)+packet[4];
    if(packet[1] & (1 << 0)) Serial.println("Ошибка двигателя (M)");
    if(packet[1] & (1 << 1)) Serial.println("Круиз контроль ON");
    if(packet[1] & (1 << 3)) Serial.println("Ошибка 'ECU'");
    if(packet[1] & (1 << 6)) Serial.println("Ошибка!!!");
    Serial.printf("Ток: %d.%d Ампер\n", bat_current/10, bat_current%10);
    Serial.printf("Скорость: %d км/ч\n", mySpeed);

  }
  if(!Power_On) {
    displayUpInfo(r_packet[5],p_ver,r_packet[4]);
    displaySpeed(mySpeed);
    displayDownInfo(r_packet[2],bat_current);
  }
}

void check_battery(){
  uint16_t bat_val, sensorValue = 0;
  
  for(byte i=0; i<10; i++) sensorValue += (analogRead(BAT_PIN) + volt_correct);
    
  sensorValue = (sensorValue/10);

  if(sensorValue > MAX_SENSOR_VALUE) sensorValue = MAX_SENSOR_VALUE; // 4095 TODO: обработку ошибки

//  sensorValue = sensorValue * 2; 

  battery_fvol = sensorValue / 6;
  battery_fvol = battery_fvol / (float)10;

//  bat_val = map(sensorValue, 0, MAX_SENSOR_VALUE, 0, 33);
//  battery_fvol = bat_val * 20; // коррекция резистивного делителя
//  battery_fvol = battery_fvol/10;
  
//  acc_vol = map(sensorValue, min_acc_value, max_acc_value, 0, max_speed);
  
}

byte readACC() {

  max_speed = start_packet[r_packet[2] + 6] * 2;
  // read the input on analog pin ACC_PIN
//  sensorValue = 0;
  pinValue = 0;
  for(byte i=0; i<100; i++) pinValue += analogRead(ACC_PIN);
    
  sensorValue = (pinValue/100);

  if(sensorValue < min_acc_value) sensorValue = min_acc_value; // 800
  if(sensorValue > max_acc_value) { 
    if(sensorValue < HALL_MAX) max_acc_value = sensorValue;
  }
  

//  acc_vol = map(sensorValue, 800, 3070, 0, 205);
  acc_vol = map(sensorValue, min_acc_value, max_acc_value, 0, max_speed);
  if(acc_vol < 10) acc_vol = 0;

  //voltage = (sensorValue/100) * (3.3 / 4095.0);

  return acc_vol;
}

byte readREG() {
  
  // read the input on analog pin REG_PIN
  pinValue = 0;
  for(byte i=0; i<20; i++) pinValue += analogRead(REG_PIN);
    
  sensorValue = (pinValue/20);

    
  if(sensorValue < min_reg_value) sensorValue = min_reg_value;
  if(sensorValue > max_reg_value) {
    if(sensorValue < HALL_MAX) max_reg_value = sensorValue;
  }
  reg_vol = map(sensorValue, min_reg_value, max_reg_value, 0, 205);
  if(reg_vol < 10) reg_vol = 0;

  return reg_vol;
}


void calc_start_CRC(byte p_ver) {
  uint16_t myCRC;
  
  if(p_ver == 1) {
    myCRC = start_packet[1]+start_packet[2]+start_packet[3]+start_packet[4]+start_packet[5]+start_packet[6]+start_packet[7]+start_packet[8]+start_packet[9]+start_packet[10]+start_packet[11]+start_packet[12]+start_packet[13];
    start_packet[14] = highByte(myCRC);
    start_packet[15] = lowByte(myCRC);
  } else if (p_ver == 2) {
    myCRC = 0;
    // TODO:
  }
  
}


void displayUpInfo(byte reg_v, byte p_ver, byte acc_v) {
  // clear the display
  display.clear();

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, String(reg_v));
  if(reg_v) display.fillRect(0, 64 - (reg_v/4), 4, 64);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 0, String(p_ver));
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, String(acc_v));
  if(acc_v) display.fillRect(124, 64 - (acc_v/4), 128, 64);

//  display.drawVerticalLine(126, 64 - (acc_v/4), 64);
//  display.drawVerticalLine(127, 64 - (acc_v/4), 64);
//  display.drawVerticalLine(128, 64 - (acc_v/4), 64);
  
  display.display();
}

void displaySpeed(byte mk_speed) {
  display.setFont(Roboto_Bold_32);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 12, String(mk_speed));
  display.display();
}


void displayDownInfo(byte gear, byte current) {
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(22, 48, String(gear));
  display.drawString(64, 48, String(battery_fvol, 1));
  display.drawString(106, 48, String(current));
//  display.drawString(96, 48, String(power_bt));
  display.display();
}


void displayError() {
  // clear the display
  display.clear();
  
  display.setFont(Roboto_Bold_32);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 13, "Error " + String(Err_N));
  display.display();
}

void displayPowerOff() {
  // clear the display
  display.clear();
  
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 40, "OFF");
  display.display();
}

/*
void displayACC() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 22, String(accValue));
  display.display();
//  run_loop = false;
}
*/

