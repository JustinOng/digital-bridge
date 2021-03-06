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

void lcd_print_byte(uint8_t &data, uint8_t invert = 0) {
  if (!invert) {
    for(int8_t i = 7; i >= 0; i--) {
      if (data & (1<<i)) {
        lcd.print(1);
      }
      else {
        lcd.print(0);
      }
    }
  }
  else {
    for(uint8_t i = 0; i < 8; i++) {
      if (data & (1<<i)) {
        lcd.print(1);
      }
      else {
        lcd.print(0);
      }
    }
  }
}

uint32_t last_message = 0;

void tx_loop() {
  static uint8_t pData = 0;

  // read input pins and compile into a single byte
  uint8_t data = (PIND >> 2) | (PINB << 6);
  Serial.write(data);

  if (data != pData) {
    lcd.setCursor(0, 1);
    lcd_print_byte(data, 1);
  }

  if (Serial.available()) {
    // rx will reply with 0xA3 upon receiving data
    if (Serial.read() == 0xA3) {
      last_message = millis();
    }
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
      lcd_print_byte(data, 1);
    }

    // "ack" to tx
    Serial.write(0xA3);

    last_message = millis();

    pData = data;
  }
}

void loop() {
  static uint32_t last_tx = millis();

  // throttle tx
  if (AM_TX) {
    if ((millis() - last_tx) > 50) {
      tx_loop();

      last_tx = millis();
    }
  }
  else {
    rx_loop();
  }

  if ((millis() - last_message) < 500) {
    lcd.setCursor(8, 1);
    lcd.print("      Ok");
  }
  else {
    lcd.setCursor(8, 1);
    lcd.print("No data!");
  }
}
