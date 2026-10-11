#include <BleGamepad.h>

// =========================
// PIN PEDAL
// =========================

#define PIN_ACCELERATOR 34
#define PIN_BRAKE 35
#define PIN_CLUTCH 32
#define PIN_STEERING 33  // Kaki tengah potensiometer steering

// =========================
// BLE GAMEPAD
// =========================

BleGamepad bleGamepad(
  "v6 Driving",
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

// Kalibrasi steering dari pembacaan potensiometer Anda
const int STEERING_MIN = 250;
const int STEERING_MAX = 3600;

// =========================
// ARAH PEDAL
// =========================

const bool INVERT_ACCELERATOR = false;
const bool INVERT_BRAKE = false;
const bool INVERT_CLUTCH = false;
const bool INVERT_STEERING = false;

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
float smoothedSteering = 0;

int lastAcc = 0;
int lastBrake = 0;
int lastClutch = 0;
int lastSteering = 0;

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
// MAP STEERING KE AXIS TENGAH
// =========================

int16_t mapSteeringToAxis(int rawValue) {
  rawValue = constrain(rawValue, STEERING_MIN, STEERING_MAX);

  long axisValue = map(
    rawValue,
    STEERING_MIN,
    STEERING_MAX,
    -32767,
    32767
  );

  if (INVERT_STEERING) {
    axisValue = -axisValue;
  }

  return constrain(axisValue, -32767, 32767);
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

  analogSetPinAttenuation(
    PIN_STEERING,
    ADC_11db
  );

  BleGamepadConfiguration bleGamepadConfig;

  bleGamepadConfig.setAutoReport(false);

  bleGamepadConfig.setControllerType(
    CONTROLLER_TYPE_GAMEPAD
  );

  // macOS/browser Gamepad API lebih konsisten mengenali gamepad
  // yang memiliki minimal satu tombol pada HID descriptor.
  // Tombol ini hanya melengkapi identitas gamepad; belum ada tombol fisik.
  bleGamepadConfig.setButtonCount(0);
  bleGamepadConfig.setHatSwitchCount(0);

  // Hanya tiga sumbu untuk kompatibilitas game yang hanya membaca X, Y, Z.
  bleGamepadConfig.setWhichAxes(
    true,  // X: clutch (-) dan brake (+)
    true,  // Y: accelerator
    true,  // Z: steering
    false, // RX
    false, // RY
    false, // RZ
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
  Serial.println("Axis X = Clutch (-) / Brake (+)");
  Serial.println("Axis Y = Accelerator");
  Serial.println("Axis Z = Steering");
  Serial.println();
  Serial.println("Steering calibration: " + String(STEERING_MIN) + " to " + String(STEERING_MAX));
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
  int rawSteering = readAnalogAverage(PIN_STEERING);

  // Apply smoothing & dead zone
  int filteredAcc = applySmoothingAndDeadZone(rawAcc, smoothedAcc, lastAcc);
  int filteredBrake = applySmoothingAndDeadZone(rawBrake, smoothedBrake, lastBrake);
  int filteredClutch = applySmoothingAndDeadZone(rawClutch, smoothedClutch, lastClutch);
  int filteredSteering = applySmoothingAndDeadZone(rawSteering, smoothedSteering, lastSteering);

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
  if (filteredSteering != lastSteering) {
    lastSteering = filteredSteering;
    changed = true;
  }

  // Konversi ke axis
  int16_t acceleratorAxis = mapPedalToAxis(filteredAcc, ACCELERATOR_MIN, ACCELERATOR_MAX, INVERT_ACCELERATOR);
  int16_t brakeAxis = mapPedalToAxis(filteredBrake, BRAKE_MIN, BRAKE_MAX, INVERT_BRAKE);
  int16_t clutchAxis = mapPedalToAxis(filteredClutch, CLUTCH_MIN, CLUTCH_MAX, INVERT_CLUTCH);
  int16_t steeringAxis = mapSteeringToAxis(filteredSteering);

  // X memakai titik tengah sebagai netral: clutch bergerak ke negatif,
  // brake bergerak ke positif. Jika keduanya diinjak bersamaan, nilainya saling mengurangi.
  int16_t combinedPedalAxis = constrain((int32_t)brakeAxis - (int32_t)clutchAxis, -32767, 32767);

  // Kirim report hanya kalau ada perubahan signifikan
  if (changed) {
    bleGamepad.setX(combinedPedalAxis);
    bleGamepad.setY(acceleratorAxis);
    bleGamepad.setZ(steeringAxis);
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
  Serial.print(") | X COMBINED: ");
  Serial.print(combinedPedalAxis);
  Serial.print(" | STEERING: ");
  Serial.print(rawSteering);
  Serial.print(" -> ");
  Serial.print(filteredSteering);
  Serial.print(" (");
  Serial.print(steeringAxis);
  Serial.println(")");

  delay(10);
}
