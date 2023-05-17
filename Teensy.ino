//  Pembacaan energy meter Schneider iEM2050 menggunakan RS485
//  Penting!!!
//  Agar coding bisa tercompile,1 folder SBMS ini harus terdapat coding ini, ModbusMaster.h, konversi.h, ModbusMaster.cpp, trust_anchors.h serta util.
//  Mikrokontroller : Teensy 4.0
//  Coding ini membaca 2 energy meter Schneider iEM2050 yang masing-masing terhubung dengan RS485 menuju Serial2 & Serial3 Teensy


#include "ModbusMaster.h" //menggunakan library ModbusMaster.h
#include "konversi.h"     //menggunakan konversi union 
#include <SPI.h>
#include <EthernetWebServer_SSL.h>

#include "trust_anchors.h" //library agar dapat mengakses web SSL

#define NUMBER_OF_MAC 20
#define METHOD "POST"

byte mac[][NUMBER_OF_MAC] =
{
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x02 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x04 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x06 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x07 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x08 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x09 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0A },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0B },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0C },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0D },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0E },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0F },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x10 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x11 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x12 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x13 },
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x14 },
};

// Static IP Settings
IPAddress ip(192, 168, 0, 93);
IPAddress myDns(192, 168, 0, 254);
 
const uint16_t server_port = 443; //443 adalah server https
const char server_host[] = "iotlab-uns.com";

EthernetClient client;
EthernetSSLClient sslClient(client, TAs, (size_t)TAs_NUM);

#define PIN_DE  20     //menghubungkan pin DE --> pin teensy 19 SERIAL 2
#define PIN_RE  19     //menghubungkan pin RE --> pin teensy 18 SERIAL 2

//Serial2 RX->7 to R0 // TX->8
//Serial3 RX->15 to R0 // TX->14

#define PIN_DE1  18     //menghubungkan pin DE --> pin teensy 18 SERIAL 3
#define PIN_RE1  17     //menghubungkan pin RE --> pin teensy 17 SERIAL 3

ModbusMaster node;      //node terhubung pada Slave 1 & SERIAL2
ModbusMaster node1;     //node terhubung pada Slave 2 & SERIAL3

uint8_t result;
uint16_t data[10];

//Define di bawah untuk hasil pembacaan modbus dan kirim ke web
float reactivepower, reactivepower1;
float current, current1;
float volt, volt1;
float freq, freq1;
float activepower, activepower1;
float Apparentpower, Apparentpower1;

//unsigned : tidak ada penanda
// long = 32bit
unsigned long beginMicros, endMicros;
unsigned long byteCount = 0;
unsigned long x = millis(); //declare variable x dengan value millis

bool initEth = true;
bool f = true; //penanda f
bool ethernet = true; //boolean flag untuk memulai ethernet
bool modbus1 = true; //boolean flag untuk memulai pembacaan modbus

String queryString ; //String global agar queryString didefinisikan 1x
int KWH = 1; //int id KWH dari 1

void startTrans() {             //Memulai transmisi pengiriman data, 1 artinya mulai dan 0 artinya selesai
  digitalWrite(PIN_RE, 1);
  digitalWrite(PIN_DE, 1);
  digitalWrite(PIN_RE1, 1);
  digitalWrite(PIN_DE1, 1);
}

void endTrans() {
  digitalWrite(PIN_RE, 0);
  digitalWrite(PIN_DE, 0);
  digitalWrite(PIN_RE1, 0);
  digitalWrite(PIN_DE1, 0);
}

void setup() {

  delay(500);
  Serial.begin(115200);
  Serial.println("Ethernet Init...");
  Ethernet.init(10);
  uint16_t index = millis() % NUMBER_OF_MAC;
  Ethernet.begin(mac[index]);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  Serial.print("Connecting to: ");
  Serial.print(server_host);
  Serial.print(", port: ");
  Serial.println(server_port);
  
  pinMode(PIN_RE, OUTPUT);
  pinMode(PIN_DE, OUTPUT);
  pinMode(PIN_RE1, OUTPUT);
  pinMode(PIN_DE1, OUTPUT);
  digitalWrite(PIN_RE, 0);
  digitalWrite(PIN_DE, 0);
  digitalWrite(PIN_RE1, 0);
  digitalWrite(PIN_DE1, 0);


  Serial2.begin(9600, SERIAL_8N1);      //Serial 2 begin dengan baudrate 9600 (menyesuaikan energy meter), serta 8N1 (8 data bits, None Parity & 1 stop bits)
  Serial3.begin(9600, SERIAL_8N1);      //Serial 3 begin dengan baudrate 9600 (menyesuaikan energy meter), serta 8N1 (8 data bits, None Parity & 1 stop bits)

  node.begin(2, Serial2); //(slave id : 1)  //setting agar node pembacaan membaca slave ID 1 menggunakan Serial2
  node1.begin(1, Serial3);                  //setting agar node pembacaan membaca slave ID 2 menggunakan Serial3

  Serial.println("Mulai:");                 //Memulai pengiriman data melalui modbus
  node.preTransmission(startTrans);
  node1.preTransmission(startTrans);
  node.postTransmission(endTrans);
  node1.postTransmission(endTrans);


//  delay(100);
}



void start_conn() {

  auto start = millis();
  if (sslClient.connect(server_host, server_port)) {
    auto time = millis() - start;
    Serial.print("connected to: ");
    Serial.println(client.remoteIP());
    Serial.print("Took: ");
    Serial.println(time);

    if (METHOD == "GET") {
      Serial.println("METHOD: GET");
      sslClient.println("GET /smart-bms/public/api/ApiEnergy HTTP/1.1");
      sslClient.println("User-Agent: SSLClientOverEthernet");
      sslClient.print("Host: ");
      sslClient.println(server_host);
      sslClient.println("Connection: close");
      sslClient.println();
    }
        else if (METHOD == "POST") {
          Serial.println("METHOD: POST");
          if (KWH > 2) { //Artinya apabila id KWH lebih dari 2 maka akan kembali membaca id KWH 1
           KWH = 1;
          }
          Post_data(KWH); //Melakukan fungsi POST pembacaan KWH ke database
          KWH++; //agar pembacaan KWH selalu bertambah 1 (id KWH + 1 )
      
        }
        else {
          Serial.println("UNKNOWN HTTP METHOD");
        }
  }
        else {
          Serial.println("Connection Failed");
        }
  beginMicros = micros();
  
  
}

void freqs1() {
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0C26, 2); //memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" Freq: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    freq = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.println(result, HEX);
  }
}

void freqs2() {
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0C26, 2); //memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Freq: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    freq1 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.println(result, HEX);
  }
}

void volts() {
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BD4,2); //memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" Tegangan: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    volt = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void volts1() {
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BD4,2); //memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Tegangan1: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    volt1 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void currents() {
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BB8,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" Arus: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    current = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void currents1() {
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BB8,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Arus1: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    current1 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void reactivepowers() {
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BFC,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" reactive power: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    reactivepower = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void reactivepowers1() {
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BFC,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" reactive power1: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    reactivepower1 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void activepowers() {
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BEE,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" activepower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    activepower = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void activepowers1() {
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BEE,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" activepower1: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    activepower1 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void Apparentpowers() {
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0C04,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" Apparentpower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    Apparentpower = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void Apparentpowers1() {
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0C04,2); //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Apparentpower1: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    Apparentpower1 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void Post_data(int KWH) { //void untuk POST ke database
      if (KWH == 1) { //Apabila ID KWH =1, maka mengirimkan queryString pembacaan id 1
      queryString = "id_kwh=1&frekuensi=" + String(freq) + "&arus=" + String(current) + "&tegangan=" + String(volt) + "&active_power=" + String(activepower)+ "&reactive_power=" + String(reactivepower) + "&apparent_power=" + String(Apparentpower)+"&submit=enter";

      
}
      else if (KWH == 2) { //Apabila ID KWH =1 sudah terbaca, maka akan lanjut membaca id KWH 2 dan mengirimkan queryString id KWH 2
      queryString = "id_kwh=2&frekuensi=" + String(freq1) + "&arus=" + String(current1) + "&tegangan=" + String(volt1) + "&active_power=" + String(activepower1)+ "&reactive_power=" + String(reactivepower1) + "&apparent_power=" + String(Apparentpower1)+"&submit=enter";

      }
      Serial.print("Query: ");
      Serial.println(queryString);
      sslClient.println("POST /smart-bms/public/api/ApiEnergy HTTP/1.1");
      sslClient.println("User-Agent: SSLClientOverEthernet");
      sslClient.print("Host: ");
      sslClient.println(server_host);
      sslClient.println("Connection: close");
      sslClient.print("Content-Length: ");
      sslClient.println(queryString.length());
      sslClient.println("Content-Type: application/x-www-form-urlencoded; charset=UTF-8");
      sslClient.println();
      sslClient.println(queryString);

      
}

void modbus_jalan() { 
  //Pembacaan dari energy meter dengan delay <100
      
          freqs1();
          freqs2(); 
          delay(25);
          volts();
          volts1();
          delay(25);
          currents();
          currents1();
          delay(25);
          reactivepowers();
          reactivepowers1();
          delay(25);
          activepowers();
          activepowers1();
          delay(25);
          Apparentpowers();
          Apparentpowers1();
          delay(25);
          modbus1 == false; //pembacaan modbus selesai
          if (ethernet == true) { 
            start_conn();
            ethernet == false;
            //modbus mulai
          }
  }
      
void loop() {
  
  if (initEth == true) { //Apabila koneksi ethernet selesai maka akan start koneksi 1x saja dan berhenti
    Serial.println("Starting Connection...");
    start_conn(); //memulai fungsi start_conn 1x saja karena setelah itu inithEth =false, alias 1x saja start_conn
    initEth = false;
  }
  int len = sslClient.available();
  if (len > 0) {
    byte buffer[80];
    len = (len > 80) ? 80 : len;
    sslClient.read(buffer, len);
    Serial.write(buffer, len);
    byteCount = byteCount + len;
  }
  if (!sslClient.connected()) {
    endMicros = micros();

    Serial.println();
    Serial.println("Disconnecting.");
    sslClient.stop();

    Serial.print("Received ");
    Serial.print(byteCount);
    Serial.print(" bytes in ");
    float seconds = (float)(endMicros - beginMicros) / 1000000.0;
    Serial.print(seconds, 4);
    float rate = (float)byteCount / seconds / 1000.0;
    Serial.print(" s, rate = ");
    Serial.print(rate);
    Serial.print(" kbytes/second");
    Serial.println();
    Serial.println("=============================================================");
 
        if (ethernet == true) { //Apabila koneksi ethernet selesai maka akan start koneksi 1x saja dan berhenti
            start_conn();
            ethernet = false;
            modbus1 = true; //maka modbus 1 akan memulai
            
          }

        if (modbus1 ==true) { //Apabila modbus 1 mulai maka akan trigger void modbus_jalan (pembacaan modbus) 1x saja
            modbus_jalan();
            modbus1 = false;
            ethernet = true; //setelah 1x modbus jalan, maka ethernet akan memulai kembali
          }
    

}
}
