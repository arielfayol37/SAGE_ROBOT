// Pin definitions
const int LED_CATHODE = 7;
const int LED_GREEN   = 9;
const int LED_RED     = 5;
const int MOSFET_EN   = 2;
const int VOLT_PIN    = A0;

// ADC / voltage settings
const float ADC_REF = 5.0;
const int ADC_MAX = 1023;

// Voltage thresholds (at A0 pin)
const float ENABLE_VOLTAGE = 2.0;
const float RED_VOLTAGE    = 4.0; // Threshold for changing LED colors

// LED brightness (small duty cycle)
const int LED_BRIGHTNESS = 80;  // ~15% duty

// 30-minute timer for software reset (30 mins * 60 secs * 1000 ms)
const unsigned long RESET_TIME_MS = 1800000;

// Standard Arduino trick to force a software reset
void (*resetFunc)(void) = 0;

void setup() {
  pinMode(LED_CATHODE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(MOSFET_EN, OUTPUT);

  // Safe startup state (Charger OFF)
  digitalWrite(MOSFET_EN, LOW);
  digitalWrite(LED_CATHODE, LOW);   // Sink current
  analogWrite(LED_GREEN, 0);
  analogWrite(LED_RED, 0);
}

float readVoltage() {
  int raw = analogRead(VOLT_PIN);
  return (raw * ADC_REF) / ADC_MAX;
}

void loop() {
  int validReads = 0;

  // 1. The Waiting Phase: Require 5 valid reads, 1 second apart
  while (validReads < 5) {
    // Check if 30 minutes have passed (safety timeout)
    if (millis() >= RESET_TIME_MS) resetFunc();

    float voltage = readVoltage();

    if (voltage > ENABLE_VOLTAGE) {
      validReads++;
    } else {
      validReads = 0;  // Reset the 5-second timer if voltage dips
    }

    delay(1000);
  }

  // 2. The Turn-On Phase: Enable MOSFET
  digitalWrite(MOSFET_EN, HIGH);

  // 3. The Charging & LED Loop
  while (true) {
    // Check if 30 minutes have passed (safety timeout triggers charger OFF via reboot)
    if (millis() >= RESET_TIME_MS) resetFunc();

    float voltage = readVoltage();

    // Normal charging LED indication
    if (voltage > RED_VOLTAGE) {
      analogWrite(LED_GREEN, 0);
      analogWrite(LED_RED, LED_BRIGHTNESS);
    } else {
      analogWrite(LED_RED, 0);
      analogWrite(LED_GREEN, LED_BRIGHTNESS);
    }

    delay(200);
  }
}