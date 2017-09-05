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
#include "BleMidiEncoder.h"

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

// BLE MIDI
BLEPeripheral BLE;
BLEService midiSvc("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
BLECharacteristic midiChar("7772E5DB-3868-4112-A1A9-F2669D106BF3",
    BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead,
    BLE_MIDI_PACKET_SIZE);

class NordicBleMidiEncoder: public BleMidiEncoder {
  boolean setValue(const unsigned char value[], unsigned char length) {
    return midiChar.setValue(value, length);
  }
};

NordicBleMidiEncoder encoder;
unsigned long previousFingers;
uint8_t previousNote;
boolean playing;
boolean connected;

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
        encodeNoteOff(CHANNEL, previousNote);
        encodeNoteOn(CHANNEL, OpenPipe.note, 127);
        encoder.sendMessages();
        previousNote = OpenPipe.note;
      }
    } else {
      if (playing) {
        encodeNoteOff(CHANNEL, previousNote); // Stop the note
        encoder.sendMessages();
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
  encoder.sendMessage(0, 0, 0);

  BLE.begin();
}

boolean encodeNoteOn(uint8_t channel, uint8_t note, uint8_t volume) {
  return encoder.appendMessage(NOTEON | channel, note & 0x7F, volume & 0x7F);
}

boolean encodeNoteOff(uint8_t channel, uint8_t note) {
  return encoder.appendMessage(NOTEOFF | channel, note & 0x7F, 0);
}

