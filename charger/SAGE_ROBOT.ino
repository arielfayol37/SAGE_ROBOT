// -------- PIN DEFINITIONS --------
const int LED_CATHODE = 7;
const int LED_GREEN   = 9;
const int LED_RED     = 5;
const int MOSFET_EN   = 2;
const int VOLT_PIN    = A0;

// -------- ADC SETTINGS --------
const float ADC_REF = 5.0;
const int ADC_MAX = 1023;

// Divider ratio
const float DIV_RATIO = 2.2 / 12.2;

// -------- BATTERY THRESHOLDS --------
const float RISE_THRESHOLD = 0.5;    // 0.5V battery rise per window

// -------- LED --------
const int LED_BRIGHTNESS = 80;       // ~15% duty cycle
const float RED_VOLTAGE  = 14.0;     // Battery voltage threshold for red LED

// -------- TIMING --------
const unsigned long STARTUP_OFF_TIME = 60000UL;
const unsigned long SAMPLE_INTERVAL  = 100;
const unsigned long WINDOW_TIME      = 5000UL;
const unsigned long RESET_INTERVAL   = 600000UL;  // 10 minutes

unsigned long lastSample  = 0;
unsigned long windowStart = 0;
unsigned long resetTimer  = 0;

float sumVoltage = 0;
int sampleCount  = 0;
float lastWindowBattery = 0;

bool chargerEnabled = false;

// -------- FUNCTIONS --------

float readBatteryVoltage() {
  int raw = analogRead(VOLT_PIN);
  float a0 = (raw * ADC_REF) / ADC_MAX;
  return a0 / DIV_RATIO;
}

void updateLEDs(float battery) {
  if (battery > RED_VOLTAGE) {
    analogWrite(LED_GREEN, 0);
    analogWrite(LED_RED, LED_BRIGHTNESS);
  } else {
    analogWrite(LED_RED, 0);
    analogWrite(LED_GREEN, LED_BRIGHTNESS);
  }
}

void softwareReset() {
  digitalWrite(MOSFET_EN, LOW);
  analogWrite(LED_GREEN, 0);
  analogWrite(LED_RED, 0);
  delay(100);
  asm volatile ("jmp 0");
}

void setup() {
  pinMode(LED_CATHODE, OUTPUT);
  pinMode(LED_GREEN,   OUTPUT);
  pinMode(LED_RED,     OUTPUT);
  pinMode(MOSFET_EN,   OUTPUT);

  digitalWrite(MOSFET_EN,   LOW);
  digitalWrite(LED_CATHODE, LOW);   // Sink current
  analogWrite(LED_GREEN, 0);
  analogWrite(LED_RED,   0);

  analogReference(DEFAULT);

  // Flash both LEDs at startup to signal boot
  analogWrite(LED_GREEN, LED_BRIGHTNESS);
  analogWrite(LED_RED,   LED_BRIGHTNESS);
  delay(500);
  analogWrite(LED_GREEN, 0);
  analogWrite(LED_RED,   0);

  delay(STARTUP_OFF_TIME);          // 30s discharge window

  windowStart = millis();
  resetTimer  = millis();
}

void loop() {

  // -------- PERIODIC 10-MIN RESET --------
  if (millis() - resetTimer >= RESET_INTERVAL) {
    softwareReset();
  }

  // -------- SAMPLE --------
  if (millis() - lastSample >= SAMPLE_INTERVAL) {
    lastSample = millis();

    float battery = readBatteryVoltage();
    sumVoltage += battery;
    sampleCount++;
  }

  // -------- WINDOW EVALUATION --------
  if (millis() - windowStart >= WINDOW_TIME) {

    float avgBattery = sumVoltage / sampleCount;

    // ----- STARTUP ENABLE -----
    if (!chargerEnabled) {
      if (avgBattery >= 11.5) {
        digitalWrite(MOSFET_EN, HIGH);
        chargerEnabled = true;
      }
    }
    else {
      // ----- UPDATE LEDs WHILE CHARGING -----
      updateLEDs(avgBattery);

      // ----- DISCONNECT DETECTION -----
      float rise = avgBattery - lastWindowBattery;

      if (rise > RISE_THRESHOLD) {
        softwareReset();
      }

      if (avgBattery >= 16.5) {
        softwareReset();
      }
    }

    // Prepare next window
    lastWindowBattery = avgBattery;
    sumVoltage  = 0;
    sampleCount = 0;
    windowStart = millis();
  }
}