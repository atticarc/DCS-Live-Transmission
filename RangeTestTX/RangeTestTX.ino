#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// Radio Addresses
#define TRANSMITTER_ADDR 1
#define RECEIVER_ADDR 2

// Radio Pin Definitions
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4

// Frequency Select (MHz)
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(rf95, TRANSMITTER_ADDR);

unsigned long txStart;
unsigned long txEnd;
unsigned long txTime;
unsigned long rxStart;
unsigned long rxEnd;
unsigned long rxTime;

uint8_t fullPacket[251];
int startTime;
int endTime;
int numPacketsSent = 0;
int rssi_data[500];
int bitrate_data[500];
int txtime_data[500];
int rxtime_data[500];
int snr_data[500];

String _inputBuffer = "";

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

state current_state {idle};
bool contestFlag = false;

uint8_t rec[] = "Recieved";

enum state {
  idle,
  contest,
  txtest,
  repdata
};

bool setup_radio(){

  // Initialize the radio
  if(manager.init()){

    // Manually reset the radio
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    /* 
    Transmitter Power: 
    - +23dBm (legal max).
    - Set to 20dBm in software, which is measured as +23dBm 
    - according to RadioHead docs. Maximizes range.
    */
    rf95.setTxPower(20, false);

    /* 
    Spreading Factor:
    - Number of LoRa chirps per bit.
    - Higher value gives longer range and lower bitrate.
    - Lower value gives shorter range, higher bitrate.
    - Ranges from 6-12.
    */
    rf95.setSpreadingFactor(7);

    /*
    Coding Rate:
    - Error correction ratio (useful data:total data).
    - 4/5 means 4 bits of useful data for every 5 bits sent.
    - Range: 4/5 (default) to 4/8; higher numbers give more error correction.
    */
    rf95.setCodingRate4(5);

    /*
    Signal Bandwidth:
    - Range of frequencies used.
    - Higher bandwidth = higher data rate and power use.
    - Lower bandwidth = lower sensitivity, shorter range.
    - Using highest possible (500kHz) for max data rate.
    */
    rf95.setSignalBandwidth(250001);

    // Set radio frequency
    if (!rf95.setFrequency(RF95_FREQ)) {
      Serial.println("setFrequency failed");
      return false;
    }
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

    return true;

  } else {
    Serial.println("INITIALIZATION FAILED");
    return false;
  }


}


// Function for transmitting a reliable, acknowledged packet with timing info
void transmit(uint8_t packet[], uint8_t length){

  txStart = millis();

  if (manager.sendtoWait(packet, length, RECEIVER_ADDR)) {
    txEnd = millis();
    txTime = txEnd - txStart;

    Serial.print("TX time (ms): ");
    Serial.println(txTime);

    uint8_t len = sizeof(buf);
    uint8_t from;

    rxStart = millis();
    if (manager.recvfromAckTimeout(buf, &len, 500, &from)){
      rxEnd = millis();
      rxTime = rxEnd - rxStart;

      Serial.print("RX time (ms): ");
      Serial.println(rxTime);

      Serial.print("Got reply from ");
      Serial.print(from);
      Serial.print(": ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");   
      Serial.println(rf95.lastRssi());
    } else {
      rxEnd = millis();
      rxTime = rxEnd - rxStart;

      Serial.print("RX time (timeout, ms): ");
      Serial.println(rxTime);

      Serial.println("No reply, is the receiver running?");
    }

  } else {
    unsigned long txFail = millis() - txStart;
    Serial.print("Send failed, TX time (ms): ");
    Serial.println(txFail);
  }
}

void recieve(){
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);

      // Send a reply back to the originator client
      if (!manager.sendtoWait(rec, sizeof(rec), from))
        Serial.println("sendtoWait failed");
    }
  }
}


void getSerialInput() {
  int i = 0;  // prevent infinite loops - unlikely
  while (Serial.available() > 0 && i < 100) {
    i++;
    char incomingByte = Serial.read();

    // Waiting for return key to be hit before handling command
    if (incomingByte == '\r' || incomingByte == '\n') {
      if (_inputBuffer.length() == 0) {
        return;
      }
      Serial.println();
      command_parser(_inputBuffer);
      _inputBuffer = "";
    } else {
      _inputBuffer += incomingByte;
      Serial.print(incomingByte);
    }
  }
}

/*
Commands:
contest:
  Only passable in the idle state.
  This tests the radio connection to make sure that there is a link before testing.
txtest:
  Only passable after a good connection test is established.
  This sends full test packets over a time period to gather performance metrics
*/
void command_parser(String& input){

  if (input == "contest"){
    if (current_state == idle){
      current_state = contest;
    } else {
      Serial.println("Connection test can only be performed once in idle state");
    }
    return;
  }

  if (input == "txtest"){
    if (current_state == contest && contestFlag){
      current_state = txtest;
    } else if (current_state == contest && !contestFlag){
      Serial.println("Connection must be good before running transmission test!");
    } else {
      Serial.println("Connection must be tested before running transmission test!");
    }
    return;
  }
  
  Serial.println("unknown");
  delay(100);
}


void setup() {

  Serial.begin(19600);
  while (!Serial) {
    delay(10);
  }

  if (!setup_radio()){
    Serial.println("Radio initialization failure. Please fix the issue and retry.");
    while (1);
  }
  Serial.println("Radio setup successful");


  for (int i = 0; i < 251; i++){
    fullPacket[i] = 0xFF;
  }

}


bool connectionTest(){
  transmit();
  String recieved = (char*)buf;
  if (recieved == "Recieved") {
    contestFlag = true;
  } else {
    contestFlag = false;
  }
}


void txtest() {
  startTime = millis();
  endTime = millis() + 30000;
  int i = 0;

  while (millis() < endTime){
    transmit(fullPacket, sizeof(fullPacket));
    rssi_data[i] = rf95.lastRssi();
    bitrate_data[i] = 2016 / (txTime + rxTime);
    txtime_data[i] = txTime;
    rxtime_data[i] = rxTime;
    snr_data[i] = rf95.lastSNR();
    i++;
  }

  current_state = repdata_state;

}

void repdata() {

}

void contest_state() {
  contest();
}

void txtest_state() {
  txtest();
}

void repdata_state() {
  repdata();
}

// Idling
void idle_state(){

  Serial.println("Idling...");
  delay(1000);

}

void loop() {
  getSerialInput();
  switch(current_state){
    case idle:
      idle_state();
      break;
    case contest:
      contest_state();
      break;
    case txtest:
      txtest_state();
      break;
    case repdata:
      repdata_state();
      break;
  }
}









