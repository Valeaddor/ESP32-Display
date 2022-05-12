#include <HardwareSerial.h>
#include "BluetoothSerial.h"

#define RX_PIN 16
#define TX_PIN 17
#define TIMEOUT 20
#define PROTO_1_BYTES 11
#define PROTO_2_BYTES 8
#define PROG_VER 0.11
#define ACC_PIN 34
#define MY_NAME "ESP32dsp"


//bool start_p = false;
byte byte_p = 0;
byte uart_protocol = 0;
byte packet[30] = {0};
byte inByte, oldByte, cmdByte = 0;
uint8_t BTmac[6] = {0};
byte r_packet[] = {0x3e, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05};
byte start_packet[] = {0x3e, 0x2c, 0x0b, 0x00, 0x3c, 0x28, 0x1e, 0x32, 0x46, 0x64, 0x01, 0x00, 0x00, 0x00, 0x01, 0x96};
unsigned long loop_time;
int16_t mk_speed, bat_current;
uint16_t accValue = 0;
char myBTName[22] = {0};
unsigned long sensorValue;
uint8_t Err_N = 0;

boolean is_start_p = true;
boolean is_error = false;

HardwareSerial mySerial(2);
BluetoothSerial SerialBT;

void setup() {

  esp_read_mac(BTmac, ESP_MAC_BT);
  sprintf(myBTName, "%s%02X%02X%02X%02X%02X%02X", MY_NAME, BTmac[0], BTmac[1], BTmac[2], BTmac[3], BTmac[4], BTmac[5]);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ACC_PIN, INPUT_PULLUP);

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
  SerialBT.println("Port 9600 8N1 RX pin 16 TX pin 17");

  accValue = readACC();
  if(accValue != 0) {
      Serial.printf("Значение акселератора: %d\n", accValue);
      Serial.println("Требуется калибровка!!!");
      is_error = true;
      Err_N = 5;
  }

  calc_start_CRC(1);

  loop_time = millis() + TIMEOUT;
  
}

void loop() {
  
  if (mySerial.available()) {
    oldByte = inByte;
    inByte = mySerial.read();

    Serial.printf("%.2X ",inByte);

    if(uart_protocol != 0) {
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
      }
    } else if (uart_protocol == 2 && byte_p == PROTO_2_BYTES) {
      if(packet[0] == 0x28) {
        printPacket();
        byte_p = 0;
      }
    }

    if(byte_p >= 29) {
      uart_protocol = 0;
      protoError();
      byte_p = 0; // переполнение буфера!
    }
    
  }

  tik();  

  check_serial();



}

void sendPacket() {
  if(is_start_p) {
    is_start_p = false;
    
    mySerial.write(start_packet,sizeof(start_packet));
  } else {
    r_packet[4] = readACC();

    int16_t myCRC = r_packet[1] + r_packet[2] + r_packet[3] + r_packet[4] + r_packet[5];
    r_packet[6] = highByte(myCRC);
    r_packet[7] = lowByte(myCRC);

    mySerial.write(r_packet,sizeof(r_packet));
  }
}

void protoError() {
  Serial.println();
  SerialBT.println();
  Serial.print("Protocol unknown!!!");
  SerialBT.println("Protocol unknown!!!");
  
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
    printPacketInfo(uart_protocol);
  }
  else { 
    Serial.println("CRC BAD!");
    SerialBT.println("!CRC");
  }

  Serial.printf("SEND PACKET: %.2X %.2X %.2X %.2X %.2X %.2X %.2X %.2X\n", r_packet[0], r_packet[1], r_packet[2], r_packet[3], r_packet[4], r_packet[5], r_packet[6], r_packet[7]);
}

void tik() {
  if(uart_protocol == 1) {
    if(millis() > loop_time) {
      sendPacket();
      loop_time = millis() + TIMEOUT;
    }
  }
}

void check_serial() {
  if (Serial.available()) {
    cmdByte = Serial.read();
    Serial.printf("\nGot: %.2X\n",cmdByte);

    if(cmdByte == 'v' || cmdByte == 'V') Serial.printf("Version: %.2f\n",PROG_VER);
    if(cmdByte == 'a' || cmdByte == 'A') Serial.printf("ACC: %d\n",readACC());
    if(cmdByte == '1') r_packet[2] = 0x01;  // переключаем скорость на 1
    if(cmdByte == '2') r_packet[2] = 0x02;  // переключаем скорость на 2
    if(cmdByte == '3') r_packet[2] = 0x03;  // переключаем скорость на 3

//    if(cmdByte == 'u' || cmdByte == 'U' && (r_packet[4] <= 254)) r_packet[4]++;
//    if(cmdByte == 'd' || cmdByte == 'D' && (r_packet[4] >= 1)) r_packet[4]--;
//    if(cmdByte == '0') r_packet[4] = 0x00;  // сбрасываем газ

    if(cmdByte == 'f' || cmdByte == 'F' && (r_packet[5] <= 200)) r_packet[5]++;
//    if(cmdByte == 'c' || cmdByte == 'C' && (r_packet[5] >= 1)) r_packet[5]--;
    if(cmdByte == '-') r_packet[5] = 0x00;  // сбрасываем тормоз

    
    int16_t myCRC = r_packet[1] + r_packet[2] + r_packet[3] + r_packet[4] + r_packet[5];
    r_packet[6] = highByte(myCRC);
    r_packet[7] = lowByte(myCRC);
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

  Serial.printf("Протокол: %d\n", p_ver);
  if(p_ver == 1) {
    mk_speed = ((packet[5]<<8)+packet[6]);
    if(packet[2] == 0) Serial.println("Двигатель блокирован"); 
      else if(packet[2] == 1) Serial.println("Нормальная работа"); 
        else if(packet[2] == 3) Serial.println("Настройки приняты"); 
          else Serial.println("Неизвестное значение 3-го байта!");
    if(packet[3] & (1 << 0)) Serial.println("Ошибка двигателя (M)");
    if(packet[3] & (1 << 1)) Serial.println("Ошибка 'ECU'");
    if(packet[3] & (1 << 2)) Serial.println("Тормоз '!'");
    Serial.printf("Ток: %d Ампер\n", packet[4]);
    Serial.printf("Скорость: %d км/ч\n", mk_speed);
  } else if (p_ver == 2) {
    mk_speed = (packet[5]<<8)+packet[6];
    bat_current = (packet[3]<<8)+packet[4];
    if(packet[1] & (1 << 0)) Serial.println("Ошибка двигателя (M)");
    if(packet[1] & (1 << 1)) Serial.println("Круиз контроль ON");
    if(packet[1] & (1 << 3)) Serial.println("Ошибка 'ECU'");
    if(packet[1] & (1 << 6)) Serial.println("Ошибка!!!");
    Serial.printf("Ток: %d.%d Ампер\n", bat_current/10, bat_current%10);
    Serial.printf("Скорость: %d км/ч\n", mk_speed);
    
  }
}

uint8_t readACC() {
  uint8_t i, acc_vol;
  
  // read the input on analog pin ACC_PIN
  sensorValue = 0;
  for(i=0;i<100;i++) sensorValue += analogRead(ACC_PIN);
    
  sensorValue = (sensorValue/100);
  if(sensorValue < 800) sensorValue = 800;
  if(sensorValue > 3050) sensorValue = 3050;
  acc_vol = map(sensorValue, 800, 3050, 0, 205);
  if(acc_vol < 10) acc_vol = 0;

  //voltage = (sensorValue/100) * (3.3 / 4095.0);

  return acc_vol;
}

void calc_start_CRC(uint8_t p_ver) {
  int16_t myCRC;
  
  if(p_ver == 1) {
    myCRC = start_packet[1]+start_packet[2]+start_packet[3]+start_packet[4]+start_packet[5]+start_packet[6]+start_packet[7]+start_packet[8]+start_packet[9]+start_packet[10]+start_packet[11]+start_packet[12]+start_packet[13];
    r_packet[14] = highByte(myCRC);
    r_packet[15] = lowByte(myCRC);
  } else if (p_ver == 2) {
    myCRC = 0;
    // TODO:
  }
  
}
