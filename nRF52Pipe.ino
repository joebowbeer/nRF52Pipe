/*  OpenPipe Breakout with BLE MIDI for Adafruit and SparkFun nRF52 boards.
 *  
 *  Connect OpenPipe Breakout to Adafruit Bluefruit nRF52 Feather:
 *  
 *  RED -> 27
 *  BLACK -> 30
 *  WHITE-> 25 (SDA)
 *  GREEN-> 26 (SCL)
 *  
 *  NOTE: SDA and SCL require external pull-up resistors (4k7)
 *  
 *  Connect OpenPipe Breakout to SparkFun nRF52 Breakout:
 *  
 *  RED -> 22
 *  BLACK -> 23
 *  WHITE-> 24 (SDA)
 *  GREEN-> 25 (SCL)
 *  
 *  NOTE: SDA and SCL must be redefined in SparkFun variants.h
 *  
 *  If attaching OpenPipe with a long cable, connect RED to 3V3 pin.
 *  
 *  www.openpipe.cc
 */

#include <OpenPipe.h>
#include <BLEPeripheral.h>

// Select fingering here
#define FINGERING FINGERING_GAITA_GALEGA
//#define FINGERING FINGERING_GAITA_ASTURIANA
//#define FINGERING FINGERING_GREAT_HIGHLAND_BAGPIPE
//#define FINGERING FINGERING_UILLEANN_PIPE
//#define FINGERING FINGERING_SACKPIPA

// MIDI channels
#define CHANNEL 0

// MIDI messages
#define NOTEOFF 0x80
#define NOTEON 0x90

#if defined(ARDUINO_FEATHER52)

#define VCC_PIN 27
#define GND_PIN 30

// Blue LED
#define LED_PIN LED_BLUE
#define LED_ACTIVE LED_STATE_ON

#elif defined(ARDUINO_NRF52_DK)

#define VCC_PIN 22
#define GND_PIN 23

// LED on pin 7 is active low
#define LED_PIN 7
#define LED_ACTIVE LOW

#else
#error "Unsupported platform." 
#endif

#define BLE_PACKET_SIZE 20

// BLE MIDI
BLEPeripheral BLE;
BLEService midiSvc("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
BLECharacteristic midiChar("7772E5DB-3868-4112-A1A9-F2669D106BF3",
    BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead, BLE_PACKET_SIZE);

unsigned long previousFingers;
uint8_t previousNote;
boolean playing;
boolean connected;

uint8_t midiData[BLE_PACKET_SIZE];
int byteOffset = 0;
uint8_t lastStatus;
uint32_t lastTime;

void setup() {
  previousFingers = 0xFF;
  previousNote = 0;
  playing = false;
  connected = false;

  pinMode(LED_PIN, OUTPUT);
  displayConnectionState();

  setupOpenPipe();
  setupBle();
}

void loop() {
  BLE.poll();
  connected = !!BLE.central();
  displayConnectionState();
  if (connected) {
    readFingers();
  }
}

//void testNotes() {
//  loadNoteOn(CHANNEL, 60, 127);
//  sendMessages();
//  delay(200);
//  loadNoteOff(CHANNEL, 60);
//  sendMessages();
//  delay(400);
//}

void readFingers() {
  unsigned long fingers = OpenPipe.readFingers();

  // If fingers have changed...
  if (fingers != previousFingers) {
    previousFingers = fingers;

    // Check the low right thumb sensor
    if (OpenPipe.isON()) {
      playing = true;

      // If note changed...
      if (OpenPipe.note != previousNote && OpenPipe.note != 0xFF) {
        // Stop previous note and start current
        loadNoteOff(CHANNEL, previousNote);
        loadNoteOn(CHANNEL, OpenPipe.note, 127);
        sendMessages();
        previousNote = OpenPipe.note;
      }
    } else {
      if (playing) {
        loadNoteOff(CHANNEL, previousNote); // Stop the note
        sendMessages();
        playing = false;
      }
    }
  }
}

void displayConnectionState() {
  // LED is lit until we're connected
  digitalWrite(LED_PIN, connected ? !LED_ACTIVE : LED_ACTIVE);
}

void setupOpenPipe() {
  OpenPipe.power(VCC_PIN, GND_PIN);
  OpenPipe.config();
  OpenPipe.setFingering(FINGERING);
}

void setupBle() {
  BLE.setConnectionInterval(6, 12); // 7.5 to 15 millis

  // set the local name peripheral advertises
  BLE.setLocalName("nRFPipe");

  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedServiceUuid(midiSvc.uuid());

  // add service and characteristic
  BLE.addAttribute(midiSvc);
  BLE.addAttribute(midiChar);

  // set an initial value for the characteristic
  sendMessage(0, 0, 0);

  BLE.begin();
}

boolean loadNoteOn(uint8_t channel, uint8_t note, uint8_t volume) {
  return loadMessage(NOTEON | channel, note & 0x7F, volume & 0x7F);
}

boolean loadNoteOff(uint8_t channel, uint8_t note) {
  return loadMessage(NOTEOFF | channel, note & 0x7F, 0);
}

boolean loadMessage(uint8_t status, uint8_t byte1, uint8_t byte2) {
  // Assert BLE_PACKET_SIZE > 4
  if (byteOffset > BLE_PACKET_SIZE - 4) return false;
  uint32_t timestamp = millis();
  boolean empty = byteOffset == 0;
  if (empty) {
    uint8_t headTs = timestamp >> 7;
    headTs |= 1 << 7;  // set the 7th bit
    headTs &= ~(1 << 6);  // clear the 6th bit
    midiData[byteOffset++] = headTs;
  }
  if (empty || lastStatus != status || lastTime != timestamp) {
    uint8_t msgTs = timestamp;
    msgTs |= 1 << 7;  // set the 7th bit
    midiData[byteOffset++] = msgTs;
    midiData[byteOffset++] = status;
    midiData[byteOffset++] = byte1;
    midiData[byteOffset++] = byte2;
    lastStatus = status;
    lastTime = timestamp;
  } else {
    midiData[byteOffset++] = byte1;
    midiData[byteOffset++] = byte2;
  }
  return true;
}

boolean sendMessage(uint8_t status, uint8_t byte1, uint8_t byte2) {
  return loadMessage(status, byte1, byte2) && sendMessages();
}

boolean sendMessages() {
  if (byteOffset != 0) {
    midiChar.setValue(midiData, byteOffset);
    byteOffset = 0;
    return true;
  }
  return false;
}

