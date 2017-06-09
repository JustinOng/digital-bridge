#include <Arduino.h>
#include <LiquidCrystal.h>

// pin for debug software serial TX
#define DEBUG_TX 10

// pin for determining if module is tx or rx
#define CONFIG_PIN 11

LiquidCrystal lcd(14, 15, 16, 17, 18, 19);

uint8_t AM_TX = 0;
void setup() {
  pinMode(CONFIG_PIN, INPUT_PULLUP);

  if (digitalRead(CONFIG_PIN) == 0) {
    AM_TX = 1;
  }

  for(uint8_t pin = 2; pin <= 9; pin++) {
    if (AM_TX) {
      pinMode(pin, INPUT_PULLUP);
    }
    else {
      pinMode(pin, OUTPUT);
    }
  }

  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  if (AM_TX) {
    lcd.print("Master (TX)");
  }
  else {
    lcd.print("Slave (RX)");
  }
}

void tx_loop() {
  static uint8_t pData = 0;

  // read input pins and compile into a single byte
  uint8_t data = (PIND >> 2) | (PINB << 6);
  Serial.write(data);

  if (data != pData) {
    lcd.setCursor(0, 1);
    lcd.print(data, BIN);
  }

  pData = data;
}

void rx_loop() {
  static uint8_t pData = 0;

  if (Serial.available()) {
    uint8_t data = Serial.read();

    PORTD = (data << 2) | (PORTD & 0x03);
    PORTB = (data >> 6) | (PORTB & 0xFC);

    if (data != pData) {
      lcd.setCursor(0, 1);
      lcd.print(data, BIN);
    }

    pData = data;
  }
}

void loop() {
  if (AM_TX) {
    tx_loop();
  }
  else {
    rx_loop();
  }
}
