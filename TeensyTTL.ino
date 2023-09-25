#include "ModbusMaster.h"  //menggunakan library ModbusMaster.h
#include "konversi.h"      //menggunakan konversi union
#include <SPI.h>
#include <EthernetWebServer_SSL.h>

#include "trust_anchors.h"  //library agar dapat mengakses web SSL

#define NUMBER_OF_MAC 20
#define METHOD "POST"

byte mac[][NUMBER_OF_MAC] = {
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
IPAddress myDns(192, 168, 0, 1); 

const uint16_t server_port = 443;  //443 adalah server https
const char server_host[] = "iotlab-uns.com";

EthernetClient client;
EthernetSSLClient sslClient(client, TAs, (size_t)TAs_NUM);

#define PIN_DE_1 2   // PIN DE 2 SERIAL 1
#define PIN_RE_1 3   //PIN RE 3 SERIAL 1
#define PIN_DE_2 5   // PIN DE 2 SERIAL 2
#define PIN_RE_2 6   //PIN RE 3 SERIAL 2
#define PIN_DE_3 18  // PIN DE 2 SERIAL 4
#define PIN_RE_3 19  //PIN RE 3 SERIAL 4
#define PIN_DE_4 22  // PIN DE 2 SERIAL 5
#define PIN_RE_4 23  //PIN RE 3 SERIAL 5

ModbusMaster node;   //NODE TERHUBUNG PADA SLAVE 1 SERIAL 1
ModbusMaster node1;  //NODE TERHUBUNG PADA SLAVE 2 SERIAL 2
ModbusMaster node2;  //NODE TERHUBUNG PADA SLAVE 3 SERIAL 4
ModbusMaster node3;  //NODE TERHUBUNG PADA SLAVE 4 SERIAL 5

uint8_t result;     //GLOBAL DECLARATION
uint16_t data[10];  //GLOBAL DECLARATION

//Define di bawah untuk hasil pembacaan modbus dan kirim ke web
float reactivepower, reactivepower2, reactivepower3, reactivepower4;
float current, current2, current3, current4;
float volt, volt2, volt3, volt4;
float freq, freq2, freq3, freq4;
float activepower, activepower2, activepower3, activepower4;
float Apparentpower, Apparentpower2, Apparentpower3, Apparentpower4;
float totalactive, totalactive2, totalactive3, totalactive4;

unsigned long beginMicros, endMicros;
unsigned long byteCount = 0;
unsigned long x = millis();   //declare variable x dengan value millis
unsigned long oldMillis = 0;  //declare variable previous millis

bool initEth = true;
bool f = true;         //penanda f
bool ethernet = false;  //boolean flag untuk memulai ethernet
bool modbus1 = true;  //boolean flag untuk memulai pembacaan modbus

String queryString;  //String global agar queryString didefinisikan 1x
int KWH = 1;         //int id KWH dari 1

void startTrans() {  //Memulai transmisi pengiriman data, 1 artinya mulai dan 0 artinya selesai
  digitalWrite(PIN_RE_1, 1);
  digitalWrite(PIN_DE_1, 1);
  digitalWrite(PIN_RE_2, 1);
  digitalWrite(PIN_DE_2, 1);
  digitalWrite(PIN_RE_3, 1);
  digitalWrite(PIN_DE_3, 1);
  digitalWrite(PIN_RE_4, 1);
  digitalWrite(PIN_DE_4, 1);
}

void endTrans() {
  digitalWrite(PIN_RE_1, 0);
  digitalWrite(PIN_DE_1, 0);
  digitalWrite(PIN_RE_2, 0);
  digitalWrite(PIN_DE_2, 0);
  digitalWrite(PIN_RE_3, 0);
  digitalWrite(PIN_DE_3, 0);
  digitalWrite(PIN_RE_4, 0);
  digitalWrite(PIN_DE_4, 0);
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
      delay(1);  // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    while (true) {
      delay(1);  // do nothing, no point running without Ethernet hardware
    }
  }
  Serial.print("Connecting to: ");
  Serial.print(server_host);
  Serial.print(", port: ");
  Serial.println(server_port);

  pinMode(PIN_RE_1, OUTPUT);
  pinMode(PIN_DE_1, OUTPUT);
  pinMode(PIN_RE_2, OUTPUT);
  pinMode(PIN_DE_2, OUTPUT);
  pinMode(PIN_RE_3, OUTPUT);
  pinMode(PIN_DE_3, OUTPUT);
  pinMode(PIN_RE_4, OUTPUT);
  pinMode(PIN_DE_4, OUTPUT);

  digitalWrite(PIN_RE_1, 0);
  digitalWrite(PIN_DE_1, 0);
  digitalWrite(PIN_RE_2, 0);
  digitalWrite(PIN_DE_2, 0);
  digitalWrite(PIN_RE_3, 0);
  digitalWrite(PIN_DE_3, 0);
  digitalWrite(PIN_RE_4, 0);
  digitalWrite(PIN_DE_4, 0);

  Serial1.begin(9600, SERIAL_8E1);
  Serial2.begin(9600, SERIAL_8E1);
  Serial4.begin(9600, SERIAL_8E1);
  Serial5.begin(9600, SERIAL_8E1);
  //  Serial3.begin(9600, SERIAL_8N1);

  node.begin(1, Serial1);  //(slave id : 1)
  node1.begin(2, Serial2); //(slave id : 2)
  node2.begin(3, Serial4); //(slave id : 3)
  node3.begin(4, Serial5); //(slave id : 4)
  //node1.begin(2, Serial2);

  Serial.println("BEGIN:");
  node.preTransmission(startTrans);
  node1.preTransmission(startTrans);
  node2.preTransmission(startTrans);
  node3.preTransmission(startTrans);
  //  node1.preTransmission(startTrans);

  node.postTransmission(endTrans);
  node1.postTransmission(endTrans);
  node2.postTransmission(endTrans);
  node3.postTransmission(endTrans);
  //  node1.postTransmission(endTrans);
}

void loop() {
  // if (initEth == true) {  //Apabila koneksi ethernet selesai maka akan start koneksi 1x saja dan berhenti
  //   Serial.println("Starting Connection...");
  //   start_conn();
  //   Serial.println("Disconnecting.");
  //   sslClient.stop();
  //   start_conn();
  //   Serial.println("Disconnecting.");
  //   sslClient.stop();
  //   start_conn();
  //   Serial.println("Disconnecting.");
  //   sslClient.stop();
  //   start_conn();
  //   ethernet = true;
  //   initEth = false;
  // }

  if (millis() - oldMillis >= 60000 && modbus1 == false) {
    start_conn();
    Serial.println("Disconnecting.");
    sslClient.stop();
    start_conn();
    Serial.println("Disconnecting.");
    sslClient.stop();
    start_conn();
    Serial.println("Disconnecting.");
    sslClient.stop();
    start_conn();
    ethernet = true;
    oldMillis = millis();
  }

  if (ethernet == true) {
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
      ethernet = false;
      modbus1 = true;
    }
  }
  if (modbus1 == true) {
    modbus_jalan();
    modbus1 = false;
  }
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
      sslClient.println("GET /smart-bms-kestl/api/energyMonitor HTTP/1.1");
      sslClient.println("User-Agent: SSLClientOverEthernet");
      sslClient.print("Host: ");
      sslClient.println(server_host);
      sslClient.println("Connection: close");
      sslClient.println();
    } 
    else if (METHOD == "POST") {
      Serial.println("METHOD: POST");
      if (KWH > 4) {  //Artinya apabila id KWH lebih dari 2 maka akan kembali membaca id KWH 1
        KWH = 1;
      }
      Serial.println("KWH ke-" + String(KWH));
      Post_data(KWH);  //Melakukan fungsi POST pembacaan KWH ke database
      KWH++;           //agar pembacaan KWH selalu bertambah 1 (id KWH + 1 )
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

void freqs() {  //freq OUTLET
  Serial.println("Node 1");
  // Freq
  result = node.readHoldingRegisters(0x07E1, 1);  //memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" Frekuensi Nominal:");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    freq = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void freqs2() {  //freq Lamp
  Serial.println("Node 2");
  // Freq
  result = node1.readHoldingRegisters(0x07E1, 1);  //memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Frekuensi Nominal:");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    freq2 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void freqs3() {  //freq AC1
  Serial.println("Node 3");
  result = node2.readHoldingRegisters(0x07E1, 1);  //memanggil fungsi read pada register (alamat, besar data)
  if (result == node2.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node2.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Frekuensi Nominal:");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    freq3 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void freqs4() {  //freq AC2
  Serial.println("Node 4");
  result = node3.readHoldingRegisters(0x07E1, 1);  //memanggil fungsi read pada register (alamat, besar data)
  if (result == node3.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node3.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Frekuensi Nominal:");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    freq4 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void volts() {  //volt outlet
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BD4, 2);  //memanggil fungsi read pada register (alamat, besar data)
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

void volts2() {  //volt lamp
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BD4, 2);  //memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Tegangan: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    volt2 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void volts3() {  //volt ac1
  Serial.println("Node 3");
  result = node2.readHoldingRegisters(0x0BD4, 2);  //memanggil fungsi read pada register (alamat, besar data)
  if (result == node2.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node2.getResponseBuffer(0);
    data[1] = node2.getResponseBuffer(1);
    Serial.print(" Tegangan: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    volt3 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void volts4() {  //volt ac2
  Serial.println("Node 4");
  result = node3.readHoldingRegisters(0x0BD4, 2);  //memanggil fungsi read pada register (alamat, besar data)
  if (result == node3.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node3.getResponseBuffer(0);
    data[1] = node3.getResponseBuffer(1);
    Serial.print(" Tegangan: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    volt4 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void currents() {  //current outlet
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BB8, 2);  //Memanggil fungsi read pada register (alamat, besar data)
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

void currents2() {  //current lamp
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BB8, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Arus: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    current2 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void currents3() {  //current ac1
  Serial.println("Node 3");
  result = node2.readHoldingRegisters(0x0BB8, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node2.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node2.getResponseBuffer(0);
    data[1] = node2.getResponseBuffer(1);
    Serial.print(" Arus: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    current3 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void currents4() {  //current ac2
  Serial.println("Node 4");
  result = node3.readHoldingRegisters(0x0BB8, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node3.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node3.getResponseBuffer(0);
    data[1] = node3.getResponseBuffer(1);
    Serial.print(" Arus: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    current4 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void reactivepowers() {  //reactiv outlet
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BFC, 2);  //Memanggil fungsi read pada register (alamat, besar data)
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

void reactivepowers2() {  //reactive lamp
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BFC, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" reactive power: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    reactivepower2 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void reactivepowers3() {  //reactive ac1
  Serial.println("Node 3");
  result = node2.readHoldingRegisters(0x0BFC, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node2.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node2.getResponseBuffer(0);
    data[1] = node2.getResponseBuffer(1);
    Serial.print(" reactive power: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    reactivepower3 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void reactivepowers4() {  //reactive ac2
  Serial.println("Node 4");
  result = node3.readHoldingRegisters(0x0BFC, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node3.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node3.getResponseBuffer(0);
    data[1] = node3.getResponseBuffer(1);
    Serial.print(" reactive power: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    reactivepower4 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void activepowers() {  //active outlet
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0BEE, 2);  //Memanggil fungsi read pada register (alamat, besar data)
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

void activepowers2() {  //active lamp
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0BEE, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" activepower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    activepower2 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void activepowers3() {  //active ac1
  Serial.println("Node 3");
  result = node2.readHoldingRegisters(0x0BEE, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node2.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node2.getResponseBuffer(0);
    data[1] = node2.getResponseBuffer(1);
    Serial.print(" activepower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    activepower3 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void activepowers4() {  //active ac2
  Serial.println("Node 4");
  result = node3.readHoldingRegisters(0x0BEE, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node3.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node3.getResponseBuffer(0);
    data[1] = node3.getResponseBuffer(1);
    Serial.print(" activepower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    activepower4 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void Apparentpowers() {  //apparent outlet
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0C04, 2);  //Memanggil fungsi read pada register (alamat, besar data)
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

void Apparentpowers2() {  //apparent lamp
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0C04, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node1.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" Apparentpower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    Apparentpower2 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void Apparentpowers3() {  //apparent ac1
  Serial.println("Node 3");
  result = node2.readHoldingRegisters(0x0C04, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node2.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node2.getResponseBuffer(0);
    data[1] = node2.getResponseBuffer(1);
    Serial.print(" Apparentpower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    Apparentpower3 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void Apparentpowers4() {  //apparent ac2
  Serial.println("Node 4");
  result = node3.readHoldingRegisters(0x0C04, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node3.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node3.getResponseBuffer(0);
    data[1] = node3.getResponseBuffer(1);
    Serial.print(" Apparentpower: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    Apparentpower4 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void totalactives() {  //totalactive mod_1
  Serial.println("Node 1");
  result = node.readHoldingRegisters(0x0C8C, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node.getResponseBuffer(0);
    data[1] = node.getResponseBuffer(1);
    Serial.print(" TotalActive: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    totalactive = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void totalactives2() {  //totalactive mod_2
  Serial.println("Node 2");
  result = node1.readHoldingRegisters(0x0C8C, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node1.getResponseBuffer(0);
    data[1] = node1.getResponseBuffer(1);
    Serial.print(" TotalActive: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    totalactive2 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void totalactives3() {  //totalactive mod_3
  Serial.println("Node 3");
  result = node2.readHoldingRegisters(0x0C8C, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node2.getResponseBuffer(0);
    data[1] = node2.getResponseBuffer(1);
    Serial.print(" TotalActive: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    totalactive3 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void totalactives4() {  //totalactive mod_1
  Serial.println("Node 4");
  result = node3.readHoldingRegisters(0x0C8C, 2);  //Memanggil fungsi read pada register (alamat, besar data)
  if (result == node.ku8MBSuccess) {
    Serial.print("Baca data: ");
    data[0] = node3.getResponseBuffer(0);
    data[1] = node3.getResponseBuffer(1);
    Serial.print(" TotalActive: ");
    konversi.dataInt[0] = data[1];
    konversi.dataInt[1] = data[0];
    Serial.println(konversi.dataFloat);
    totalactive4 = konversi.dataFloat;
  } else {
    Serial.print("No data: ");
    Serial.print(result, HEX);
  }
}

void Post_data(int KWH) {  //void untuk POST ke database
  if (KWH == 1) {          //Apabila ID KWH =1, maka mengirimkan queryString pembacaan id 1
    queryString = "id_kwh=1&frekuensi=" + String(freq) + "&arus=" + String(current) + "&tegangan=" + String(volt) + "&active_power=" + String(activepower) + "&reactive_power=" + String(reactivepower) + "&apparent_power=" + String(Apparentpower) + "&energy=" + String(totalactive) + "&submit=enter";
  } else if (KWH == 2) {  //Apabila ID KWH =2 sudah terbaca, maka akan lanjut membaca id KWH 2 dan mengirimkan queryString id KWH 2
    queryString = "id_kwh=2&frekuensi=" + String(freq2) + "&arus=" + String(current2) + "&tegangan=" + String(volt2) + "&active_power=" + String(activepower2) + "&reactive_power=" + String(reactivepower2) + "&apparent_power=" + String(Apparentpower2) + "&energy=" + String(totalactive2) + "&submit=enter";
  } else if (KWH == 3) {  //Apabila ID KWH =3 sudah terbaca, maka akan lanjut membaca id KWH 2 dan mengirimkan queryString id KWH 2
    queryString = "id_kwh=3&frekuensi=" + String(freq3) + "&arus=" + String(current3) + "&tegangan=" + String(volt3) + "&active_power=" + String(activepower3) + "&reactive_power=" + String(reactivepower3) + "&apparent_power=" + String(Apparentpower3) + "&energy=" + String(totalactive3) + "&submit=enter";
  } else if (KWH == 4) {  //Apabila ID KWH =4 sudah terbaca, maka akan lanjut membaca id KWH 2 dan mengirimkan queryString id KWH 2
    queryString = "id_kwh=4&frekuensi=" + String(freq4) + "&arus=" + String(current4) + "&tegangan=" + String(volt4) + "&active_power=" + String(activepower4) + "&reactive_power=" + String(reactivepower4) + "&apparent_power=" + String(Apparentpower4) + "&energy=" + String(totalactive4) + "&submit=enter";
  }

  Serial.print("Query: ");
  Serial.println(queryString);
  sslClient.println("POST /smart-bms-kestl/api/energyMonitor HTTP/1.1");
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
  freqs();
  freqs2();
  freqs3();
  freqs4();
  delay(10);
  volts();
  volts2();
  volts3();
  volts4();
  delay(10);
  currents();
  currents2();
  currents3();
  currents4();
  delay(10);
  reactivepowers();
  reactivepowers2();
  reactivepowers3();
  reactivepowers4();
  delay(10);
  activepowers();
  activepowers2();
  activepowers3();
  activepowers4();
  delay(10);
  Apparentpowers();
  Apparentpowers2();
  Apparentpowers3();
  Apparentpowers4();
  delay(10);
  totalactives();
  totalactives2();
  totalactives3();
  totalactives4();
  delay(10);
  // start_conn();
}
