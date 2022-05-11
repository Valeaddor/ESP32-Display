#include <HardwareSerial.h>
#include "BluetoothSerial.h"

#define RX_PIN 16
#define TX_PIN 17
#define TIMEOUT 20
#define PROTO_1_BYTES 11
#define PROTO_2_BYTES 8


//bool start_p = false;
byte byte_p = 0;
byte uart_protocol = 0;
byte packet[30] = {0};
byte myCRC, inByte, oldByte, cmdByte = 0;
byte r_packet[] = {0x3e, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x05};
byte start_packet[] = {0x3e, 0x2c, 0x0b, 0x00, 0x3c, 0x28, 0x1e, 0x32, 0x46, 0x64, 0x01, 0x00, 0x00, 0x00, 0x01, 0x96};
unsigned long loop_time;
int16_t mk_speed, bat_current;

boolean is_start_p = true;

HardwareSerial mySerial(2);
BluetoothSerial SerialBT;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize both serial ports:
  Serial.begin(115200);
  Serial.println("Start ...");

  SerialBT.begin("ESP32Display"); //Bluetooth device name
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

  loop_time = millis() + TIMEOUT;
  
}

void loop() {
  
  if (mySerial.available()) {
    oldByte = inByte;
    inByte = mySerial.read();

    Serial.printf("%.2X ",inByte);
//    SerialBT.print(inByte, HEX);

    if(uart_protocol != 0) {
      packet[byte_p++] = inByte;
    } else {
      if(inByte == 0x07 && oldByte == 0x3c) {
        uart_protocol = 1;
        byte_p = 0;
        packet[byte_p++] = 0x3c;
        packet[byte_p++] = 0x07;
        Serial.println();
      } else packet[byte_p++] = inByte;
    }

    if(uart_protocol == 1 && byte_p == PROTO_1_BYTES) {
      if(packet[0] == 0x3c && packet[1] == 0x07) {
        printPacket();
        byte_p = 0;
      }
    } else if (uart_protocol == 2 && byte_p == PROTO_2_BYTES) {
      printPacket();
      byte_p = 0;
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
  } else mySerial.write(r_packet,sizeof(r_packet));
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
//  SerialBT.println();

  if(checkCRC(uart_protocol)) {
    Serial.println("CRC OK");
    printPacketInfo(uart_protocol);
  }
  else Serial.println("CRC BAD!");

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
//    Serial.print(cmdByte, HEX);
    if(cmdByte == 'l' || cmdByte == 'L') Serial.print("Check done!!!\n");
    if(cmdByte == '1') r_packet[2] = 0x01;  // переключаем скорость на 1
    if(cmdByte == '2') r_packet[2] = 0x02;  // переключаем скорость на 2
    if(cmdByte == '3') r_packet[2] = 0x03;  // переключаем скорость на 3

    if(cmdByte == 'u' || cmdByte == 'U' && (r_packet[4] <= 254)) r_packet[4]++;
    if(cmdByte == 'd' || cmdByte == 'D' && (r_packet[4] >= 1)) r_packet[4]--;
    if(cmdByte == '0') r_packet[4] = 0x00;  // сбрасываем газ

    if(cmdByte == 'a' || cmdByte == 'A' && (r_packet[5] <= 254)) r_packet[5]++;
    if(cmdByte == 'z' || cmdByte == 'Z' && (r_packet[5] >= 1)) r_packet[5]--;
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
