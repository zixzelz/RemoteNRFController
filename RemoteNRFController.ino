/*
 * RemoteNRFController Rev 4
 * Description: Remote controller code with 21 buttons, deep sleep mode,
 *              and power-efficient handling of the nRF24L01+ and ATmega328P.
 * Date: 2025-01-23
 */

// #define DEBUG

#ifdef DEBUG
  #pragma message "Debugging enabled"
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINT_NUM(n, base) Serial.print(n, base)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT_NUM(n, base)
#endif

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <PinChangeInterrupt.h>  // For pin change interrupts on all buttons

#define CE_PIN 10
#define CSN_PIN 9
#define LED_PIN A5

const uint8_t buttonPinBus_0 = A0;
const uint8_t buttonPinBus_1 = A1;
const uint8_t buttonPinBus_2 = A2;

const uint8_t buttonBusPins[3] = {buttonPinBus_0, buttonPinBus_1, buttonPinBus_2};
const int buttonBusPinsCount = 3;
const uint8_t buttonLinePins[10] = {3, 2, 4, 5, 7, 6, 8};
const int buttonLinePinsCount = 7;

volatile bool wakeUpFlag = false;  // Flag to indicate wake-u

RF24 radio(CE_PIN, CSN_PIN);  // CE, CSN pins
const byte address[6] = "30012";  // Address for communication

// Define a unique ID for this remote controller in HEX
const unsigned long remoteID = 0xA1B2C3D4;  // Example: Remote ID in hexadecimal

void setup() {
  if (F_CPU == 8000000) clock_prescale_set(clock_div_2);
  if (F_CPU == 4000000) clock_prescale_set(clock_div_4);
  if (F_CPU == 2000000) clock_prescale_set(clock_div_8);
  if (F_CPU == 1000000) clock_prescale_set(clock_div_16);

  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  DEBUG_PRINTLN("Setup");

  for (int i = 0; i < 20; i++) {
    pinMode(i, INPUT_PULLUP);  // For unused pins
  }

  pinMode(LED_PIN, OUTPUT);  // For LED_PIN, for my case 700 uA
  turnOffLed();

  for (int i = 0; i < 3; i++) {
    pinMode(buttonBusPins[i], OUTPUT);
    digitalWrite(buttonBusPins[i], LOW);
  }

  // Initialize buttons as input with pull-up resistors
  for (int i = 0; i < buttonLinePinsCount; i++) {
    pinMode(buttonLinePins[i], INPUT_PULLUP);
    attachPinChangeInterrupt(digitalPinToPCINT(buttonLinePins[i]), wakeUp, FALLING);
  }

  precomputeBusStates();

  // Initialize the radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);  // Low power for battery saving
  radio.stopListening();  // Set as a transmitter

  sleepMode();
}

void loop() {
  static unsigned long sentMs;
  static int line;

  int pressedButton = -1;
  if (wakeUpFlag) {
    DEBUG_PRINTLN("wakeUp");

    pressedButton = readSelectedButton();

    if (pressedButton != -1) {
      sendButtonPress(pressedButton);

      #ifdef DEBUG
      Serial.flush();  // Wait until all data is sent
      #endif
      debounceWithSleep(128);  // Enter sleep mode, but in real delay 300 ... 
    }

    enabeAllBus();
  }

  if (pressedButton == -1) {
    wakeUpFlag = false;
    sleepMode();
  }
}

void sendButtonPress(int buttonIndex) {
  // Create a message structure
  struct Message {
    unsigned long remoteID;  // Remote ID in HEX
    int buttonIndex;         // Button index
  };

  Message msg;
  msg.remoteID = remoteID;  // Assign Remote ID
  msg.buttonIndex = buttonIndex;

  // Power up the radio, send the message, and power down
  radio.powerUp();
  bool success = radio.write(&msg, sizeof(msg));
  radio.powerDown();

  // Debugging via Serial Monitor
  DEBUG_PRINT("Remote ID: 0x");
  DEBUG_PRINT_NUM(remoteID, HEX);  // Print ID in HEX format
  DEBUG_PRINT(" - Button ");
  DEBUG_PRINT(buttonIndex);
  DEBUG_PRINTLN(success ? " Sent Successfully" : " Failed to Send");
}

void turnOnLed() {
  digitalWrite(LED_PIN, HIGH);
}

void turnOffLed() {
  digitalWrite(LED_PIN, LOW);
}

//

void setupWatchdogTimer(uint16_t millis) {
  cli();
  wdt_reset();
  uint8_t wdtPrescaler = 0;
  if (millis <= 16) wdtPrescaler = (1 << WDP0);
  else if (millis <= 32) wdtPrescaler = (1 << WDP1);
  else if (millis <= 64) wdtPrescaler = (1 << WDP1) | (1 << WDP0);
  else if (millis <= 128) wdtPrescaler = (1 << WDP2);
  else if (millis <= 256) wdtPrescaler = (1 << WDP2) | (1 << WDP0);
  else if (millis <= 512) wdtPrescaler = (1 << WDP2) | (1 << WDP1);
  else if (millis <= 1024) wdtPrescaler = (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  else if (millis <= 2048) wdtPrescaler = (1 << WDP3);
  else if (millis <= 4096) wdtPrescaler = (1 << WDP3) | (1 << WDP0);
  else wdtPrescaler = (1 << WDP3) | (1 << WDP1);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = wdtPrescaler;
  WDTCSR |= (1 << WDIE);
  sei();
}

ISR(WDT_vect) {
  wdt_disable();
}

void sleepWithWatchdog(uint16_t millis) {
  // detachPinChangeInterrupt
  for (int i = 0; i < buttonLinePinsCount; i++) {
    detachPinChangeInterrupt(digitalPinToPCINT(buttonLinePins[i]));
  }

  //Disable ADC
  ADCSRA &= ~(1<<ADEN); 

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  setupWatchdogTimer(millis);
  sleep_mode();
  sleep_disable();

  // Re-enable ADC
  ADCSRA |= (1 << ADEN);

  // attachPinChangeInterrupt
  for (int i = 0; i < buttonLinePinsCount; i++) {
    attachPinChangeInterrupt(digitalPinToPCINT(buttonLinePins[i]), wakeUp, FALLING);
  }
}

void debounceWithSleep(uint16_t millis) {
  sleepWithWatchdog(millis);
}

//

void wakeUp() {
  // Set the wake-up flag
  wakeUpFlag = true;
}

void sleepMode() {
  DEBUG_PRINTLN("enter sleep");
  #ifdef DEBUG
  Serial.flush();
  #endif

  // Power down the nRF24L01+
  radio.powerDown();

  ADCSRA &= ~(1<<ADEN); //Disable ADC
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); //Disable digital input buffer on AIN1/0
  
  power_all_disable();
  power_adc_disable();
  power_twi_disable();
  power_spi_disable();
  power_usart0_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();

  // Set sleep mode to power-down
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Disable Brown-Out Detector (BOD)
  sleep_bod_disable();

  // Enter sleep mode
  sleep_enable();
  sleep_mode();  // The MCU sleeps here until an interrupt occurs

  // Wake up here
  sleep_disable();  // Disable sleep mode after waking up
  power_all_enable();

  // Re-enable ADC
  ADCSRA |= (1 << ADEN);
}

// Utils

//Line: 128-320 us
//Bus: 192 - 640 us
int readSelectedButton() {

  #ifdef DEBUG
  static unsigned long start;
  static unsigned long selectedLineMs;
  static unsigned long selectedBusMs;

  start = micros();
  #endif

  int selectedLinePin = readSelectedLinePin();
  int selectedBus = -1; // Initialize selectedBus variable

  #ifdef DEBUG
  selectedLineMs = micros() - start;
  start = micros();
  #endif

  if (selectedLinePin != -1) {
    turnOnLed(); // not the best place, but I do not want to turn on led during transmission to decrease peak consumption

    selectedBus = readSelectedBusWithPin(selectedLinePin);  // Create a mask for buttonBusPins (assuming they are on PORTC)

    turnOffLed();

    #ifdef DEBUG
    selectedBusMs = micros() - start;

    DEBUG_PRINT("Selected line pin: ");
    DEBUG_PRINT(selectedLinePin);  // Print ID in HEX format
    DEBUG_PRINT("; selected bus: ");
    DEBUG_PRINTLN(selectedBus);

    DEBUG_PRINT("Selected line us: ");
    DEBUG_PRINT(selectedLineMs);
    DEBUG_PRINT("; Selected Bus us: ");
    DEBUG_PRINTLN(selectedBusMs);  // Print ID in HEX format
    #endif

    if (selectedBus != -1) {
      return (selectedBus * 10) + selectedLinePin;
    }
  }
  
  return -1;
}

static uint8_t busStateMask[buttonBusPinsCount];  // Store (bitMask[i1] | bitMask[i2])

void precomputeBusStates() {
  for (uint8_t i = 0; i < buttonBusPinsCount; i++) {
    uint8_t i1 = (i + 1) % buttonBusPinsCount;
    uint8_t i2 = (i + 2) % buttonBusPinsCount;
    busStateMask[i] = digitalPinToBitMask(buttonBusPins[i1]) | digitalPinToBitMask(buttonBusPins[i2]);  
  }
}

int readSelectedBusWithPin(int selectedPin) {
  int selectedBus = -1;  // Initialize selectedBus variable
  static uint8_t portMask = buttonBusPinsMask();  // Create a mask for buttonBusPins (assuming they are on PORTC)

  for (uint8_t i = 0; i < buttonBusPinsCount; i++) {
    // Set the bus state in one operation
    PORTC = (PORTC & ~portMask) | busStateMask[i];

    int newSelectedPin = readSelectedLinePin();
    if (selectedPin == newSelectedPin) {
      selectedBus = i;
      break;
    }
  }
  return selectedBus;
}

int readSelectedLinePin() {
  int selectedLinePin = -1;

  // Mask for pins to check on PORTD (pins 0-7 on ATmega328P)
  static uint8_t pinPortDMask = buttonLinePinsMaskPortD(); // 0 - 7
  static uint8_t pinPortBMask = buttonLinePinsMaskPortB(); // 8 - 13

  // Read the entire PORTD and PORTB
  uint8_t pinPortDStates = PIND | ~pinPortDMask; // mark all bits that not in mask as HIGH

  selectedLinePin = getLowPinFromPort(pinPortDStates);
  if (selectedLinePin == -1) {
    uint8_t pinPortBStates = PINB | ~pinPortBMask; // mark all bits that not in mask as HIGH
    selectedLinePin = getLowPinFromPort(pinPortBStates);
    if (selectedLinePin >= 0) {
      selectedLinePin += 8;
    }
  }

  return selectedLinePin;
}

int getLowPinFromPort(uint8_t portStates) {
  // Check which pin is LOW in the given port state.
  // Only one pin is assumed to be LOW at a time.

  if (portStates == 0b11111110) return 0; // 254 Pin 0 is LOW
  else if (portStates == 0b11111101) return 1; // 253 Pin 1 is LOW
  else if (portStates == 0b11111011) return 2; // 251 Pin 2 is LOW
  else if (portStates == 0b11110111) return 3; // 247 Pin 3 is LOW
  else if (portStates == 0b11101111) return 4; // 239 Pin 4 is LOW
  else if (portStates == 0b11011111) return 5; // 223 Pin 5 is LOW
  else if (portStates == 0b10111111) return 6; // 191 Pin 6 is LOW
  else if (portStates == 0b01111111) return 7; // 127 Pin 7 is LOW
  else return -1; // No LOW pin detected
}

void enabeAllBus() {
  static uint8_t portMask = buttonBusPinsMask();
  PORTC &= ~portMask;  // Set all bus pins to LOW
}

uint8_t buttonLinePinsMaskPortD() {
  uint8_t pinMask = 0;
  for (int i = 0; i < buttonLinePinsCount; i++) {
    if (buttonLinePins[i] >= 2 && buttonLinePins[i] <= 7) {
      pinMask |= digitalPinToBitMask(buttonLinePins[i]);  // Create a mask for the pins
    }
  }
  return pinMask;
}

uint8_t buttonLinePinsMaskPortB() {
  uint8_t pinMask = 0;
  for (int i = 0; i < buttonLinePinsCount; i++) {
    if (buttonLinePins[i] >= 8 && buttonLinePins[i] <= 13) {
      pinMask |= digitalPinToBitMask(buttonLinePins[i]);  // Create a mask for the pins
    }
  }
  return pinMask;
}

uint8_t buttonBusPinsMask() {
  uint8_t pinMask = 0;
  for (int i = 0; i < buttonBusPinsCount; i++) {
    pinMask |= digitalPinToBitMask(buttonBusPins[i]);  // Create a mask for the pins
  }
  return pinMask;
}

//Line: 384; then 128-192 us
//Bus: 3000 us
int readSelectedButton1() {
  static unsigned long start;
  static unsigned long selectedLineMs;
  static unsigned long selectedBusMs;

  start = micros();

  unsigned long selectedLine = -1;
  for (int i = 0; i < buttonLinePinsCount; i++) {
    if (digitalRead(buttonLinePins[i]) == LOW) {
      selectedLine = i;
      break;
    }
  }

  selectedLineMs = micros() - start;
  start = micros();

  if (selectedLine != -1) {
    int selectedBus = -1;
    for (int i = 0; i < buttonBusPinsCount; i++) {
      int i1 = (i + 1) % buttonBusPinsCount;
      int i2 = (i + 2) % buttonBusPinsCount;
      digitalWrite(buttonBusPins[i], LOW);
      digitalWrite(buttonBusPins[i1], HIGH);
      digitalWrite(buttonBusPins[i2], HIGH);

      if (digitalRead(buttonLinePins[selectedLine]) == LOW) {
        selectedBus = i;
        break;
      }
    }

    selectedBusMs = micros() - start;
    
    DEBUG_PRINT("Selected line: ");
    DEBUG_PRINT(selectedLine);  // Print ID in HEX format
    DEBUG_PRINT("; selected bus: ");
    DEBUG_PRINTLN(selectedBus);

    DEBUG_PRINT("Selected line us: ");
    DEBUG_PRINT(selectedLineMs);
    DEBUG_PRINT("; Selected Bus us: ");
    DEBUG_PRINTLN(selectedBusMs);  // Print ID in HEX format

    if (selectedBus != -1) {
      return (selectedBus * 10) + selectedLine;
    }
  }
  return -1;
}

// void test() {
  // Return to sleep mode
  // radio.powerDown();  // Ensure the radio is in power-down mode
  // Serial.println("radio.powerDown");

  // sleepMode();

  // Serial.println("powerUp");
  // radio.powerUp();
  // delay(8000);

  // Serial.println("startListening 1");
  // radio.startListening();
  // delay(8000); // 21 mA

  // Serial.println("stopListening");
  // radio.stopListening();  // Set as a transmitter
  // delay(8000); // 7.7mA

  // Serial.println("powerDown");
  // radio.powerDown();
  // digitalWrite(9, LOW);  // CE pin
  // digitalWrite(10, LOW); // CSN pin
  // delay(8000); // 7.7 mA

  // sendButtonPress(1);
  // delay(2000);
// }