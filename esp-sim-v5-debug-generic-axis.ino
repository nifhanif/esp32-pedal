#include <BleGamepad.h>

// =========================
// PIN PEDAL
// =========================

#define PIN_ACCELERATOR 34
#define PIN_BRAKE 35
#define PIN_CLUTCH 32
#define PIN_BTN 33  // DIPINDAH dari 32 ke 33 agar tidak bentrok dengan clutch

// =========================
// BLE GAMEPAD
// =========================

BleGamepad bleGamepad(
  "v5 Driving",
  "ESP32",
  100
);

// =========================
// ADC CONFIGURATION
// =========================

const int ADC_RESOLUTION = 12;
const int ADC_MIN_VALUE = 0;
const int ADC_MAX_VALUE = 4095;

// =========================
// KALIBRASI PEDAL
// =========================

const int ACCELERATOR_MIN = 1700;
const int ACCELERATOR_MAX = 2350;

const int BRAKE_MIN = 1500;
const int BRAKE_MAX = 2120;

const int CLUTCH_MIN = 2000;
const int CLUTCH_MAX = 2700;

// =========================
// ARAH PEDAL
// =========================

const bool INVERT_ACCELERATOR = false;
const bool INVERT_BRAKE = false;
const bool INVERT_CLUTCH = false;

// =========================
// FILTER ADC & DEAD ZONE
// =========================

const int ADC_SAMPLES = 16;           // Naikkan dari 8 ke 16 untuk smoothing lebih baik
const int DEAD_ZONE = 25;             // Threshold noise: perubahan <25 unit ADC diabaikan
const float SMOOTHING_ALPHA = 0.3;    // Exponential smoothing factor (0.1-0.4 recommended)

// =========================
// VARIABEL GLOBAL UNTUK SMOOTHING
// =========================

float smoothedAcc = 0;
float smoothedBrake = 0;
float smoothedClutch = 0;

int lastAcc = 0;
int lastBrake = 0;
int lastClutch = 0;

// =========================
// BACA ADC RATA-RATA
// =========================

int readAnalogAverage(int pin) {
  long total = 0;

  for (int i = 0; i < ADC_SAMPLES; i++) {
    total += analogRead(pin);
    delayMicroseconds(500);
  }

  return total / ADC_SAMPLES;
}

// =========================
// APPLY SMOOTHING & DEAD ZONE
// =========================

int applySmoothingAndDeadZone(int rawValue, float &smoothedValue, int lastValue) {
  // Exponential smoothing
  smoothedValue = (SMOOTHING_ALPHA * rawValue) + ((1.0 - SMOOTHING_ALPHA) * smoothedValue);
  int smoothedInt = (int)smoothedValue;

  // Dead zone: kalau perubahan dari lastValue kecil, kembalikan lastValue
  if (abs(smoothedInt - lastValue) < DEAD_ZONE) {
    return lastValue;
  }

  return smoothedInt;
}

// =========================
// MAP PEDAL KE AXIS
// =========================

int16_t mapPedalToAxis(
  int rawValue,
  int rawMin,
  int rawMax,
  bool invert
) {
  rawValue = constrain(rawValue, rawMin, rawMax);

  long axisValue = map(
    rawValue,
    rawMin,
    rawMax,
    0,
    32767
  );

  if (invert) {
    axisValue = -axisValue;
  }

  return constrain(axisValue, 0, 32767);
}

// =========================
// SETUP
// =========================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting BLE Driving Controller (v5 Fixed)...");

  analogReadResolution(ADC_RESOLUTION);

  analogSetPinAttenuation(
    PIN_ACCELERATOR,
    ADC_11db
  );

  analogSetPinAttenuation(
    PIN_BRAKE,
    ADC_11db
  );

  analogSetPinAttenuation(
    PIN_CLUTCH,
    ADC_11db
  );

  BleGamepadConfiguration bleGamepadConfig;

  bleGamepadConfig.setAutoReport(false);

  bleGamepadConfig.setControllerType(
    CONTROLLER_TYPE_GAMEPAD
  );

  bleGamepadConfig.setButtonCount(0);
  bleGamepadConfig.setHatSwitchCount(0);

  bleGamepadConfig.setWhichAxes(
    true, // X
    true, // Y
    true,  // Z
    false, // RX
    false,  // RY
    false,  // RZ
    false, // Slider 1
    false  // Slider 2
  );

  bleGamepadConfig.setWhichSimulationControls(
    false, // Rudder
    false, // Throttle
    false, // Accelerator
    false, // Brake
    false  // Steering
  );

  bleGamepadConfig.setSimulationMin(0);
  bleGamepadConfig.setSimulationMax(32767);

  bleGamepad.begin(&bleGamepadConfig);

  bleGamepad.sendReport();

  Serial.println("BLE Gamepad started");
  Serial.println("Axis 0 / Z = Accelerator");
  Serial.println("Axis 1 / RZ = Brake");
  Serial.println("Axis 2 / RY = Clutch");
  Serial.println();
  Serial.println("Dead zone: " + String(DEAD_ZONE) + " ADC units");
  Serial.println("Smoothing alpha: " + String(SMOOTHING_ALPHA));
  Serial.println("ADC samples: " + String(ADC_SAMPLES));
}

// =========================
// LOOP
// =========================

void loop() {
  if (!bleGamepad.isConnected()) {
    delay(100);
    return;
  }

  // Baca ADC raw
  int rawAcc = readAnalogAverage(PIN_ACCELERATOR);
  int rawBrake = readAnalogAverage(PIN_BRAKE);
  int rawClutch = readAnalogAverage(PIN_CLUTCH);

  // Apply smoothing & dead zone
  int filteredAcc = applySmoothingAndDeadZone(rawAcc, smoothedAcc, lastAcc);
  int filteredBrake = applySmoothingAndDeadZone(rawBrake, smoothedBrake, lastBrake);
  int filteredClutch = applySmoothingAndDeadZone(rawClutch, smoothedClutch, lastClutch);

  // Update last values hanya kalau berubah
  bool changed = false;
  if (filteredAcc != lastAcc) {
    lastAcc = filteredAcc;
    changed = true;
  }
  if (filteredBrake != lastBrake) {
    lastBrake = filteredBrake;
    changed = true;
  }
  if (filteredClutch != lastClutch) {
    lastClutch = filteredClutch;
    changed = true;
  }

  // Konversi ke axis
  int16_t acceleratorAxis = mapPedalToAxis(filteredAcc, ACCELERATOR_MIN, ACCELERATOR_MAX, INVERT_ACCELERATOR);
  int16_t brakeAxis = mapPedalToAxis(filteredBrake, BRAKE_MIN, BRAKE_MAX, INVERT_BRAKE);
  int16_t clutchAxis = mapPedalToAxis(filteredClutch, CLUTCH_MIN, CLUTCH_MAX, INVERT_CLUTCH);

  // Kirim report hanya kalau ada perubahan signifikan
  if (changed) {
    bleGamepad.setX(acceleratorAxis);
    bleGamepad.setY(brakeAxis);
    bleGamepad.setZ(clutchAxis);
    bleGamepad.sendReport();
  }

  // Debug Serial Monitor
  Serial.print("ACC: ");
  Serial.print(rawAcc);
  Serial.print(" -> ");
  Serial.print(filteredAcc);
  Serial.print(" (");
  Serial.print(acceleratorAxis);
  Serial.print(") | BRAKE: ");
  Serial.print(rawBrake);
  Serial.print(" -> ");
  Serial.print(filteredBrake);
  Serial.print(" (");
  Serial.print(brakeAxis);
  Serial.print(") | CLUTCH: ");
  Serial.print(rawClutch);
  Serial.print(" -> ");
  Serial.print(filteredClutch);
  Serial.print(" (");
  Serial.print(clutchAxis);
  Serial.println(")");

  delay(10);
}
