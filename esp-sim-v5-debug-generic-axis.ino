#include <BleGamepad.h>

// =========================
// PIN PEDAL
// =========================

#define PIN_ACCELERATOR 34
// Port brake dan clutch ditukar: brake sekarang membaca GPIO32,
// sedangkan clutch membaca GPIO35. Tidak perlu memindahkan kabel.
#define PIN_BRAKE 32
#define PIN_CLUTCH 35
#define PIN_STEERING 33  // Kaki tengah potensiometer steering

// =========================
// BLE GAMEPAD
// =========================

BleGamepad bleGamepad(
  "v10 Driving",
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

// Nilai kalibrasi mengikuti potensiometer pada port yang baru.
const int BRAKE_MIN = 2000;
const int BRAKE_MAX = 2600;

const int CLUTCH_MIN = 1500;
const int CLUTCH_MAX = 2120;

// Kalibrasi steering dari pembacaan potensiometer Anda.
// Steering lurus berada di tengah rentang: (250 + 3600) / 2 = 1925.
const int STEERING_MIN = 250;
const int STEERING_MAX = 3600;

// Nilai tengah steering diambil saat ESP32 dinyalakan.
// Nyalakan board ketika roda berada pada posisi lurus.
int steeringCenter = 1925;

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

const int ADC_SAMPLES = 8;           // Percepat pembacaan ADC
const int DEAD_ZONE = 25;             // Threshold noise pedal: perubahan <25 unit ADC diabaikan
const float SMOOTHING_ALPHA = 0.3;    // Smoothing pedal

// Steering membutuhkan gerakan halus. Ambang kecil mencegah nilai lompat,
// sementara rata-rata 16 sampel di atas sudah cukup meredam noise.
const int STEERING_DEAD_ZONE = 8;            // naikkan jika masih bergerak sendiri
const float STEERING_ALPHA_IDLE = 0.15;      // sangat halus saat diam
const float STEERING_ALPHA_MOVE = 0.85;      // responsif saat diputar
const int STEERING_MOVE_THRESHOLD = 40;      // selisih ADC yang dianggap "sedang diputar"

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

int readAnalogAverage(int pin, int samples = ADC_SAMPLES) {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
  }
  return total / samples;
}

// =========================
// APPLY SMOOTHING & DEAD ZONE
// =========================

int applySmoothingAndDeadZone(int rawValue, float &smoothedValue, int lastValue) {
  // Exponential smoothing untuk pedal
  smoothedValue = (SMOOTHING_ALPHA * rawValue) + ((1.0 - SMOOTHING_ALPHA) * smoothedValue);
  int smoothedInt = (int)smoothedValue;

  if (abs(smoothedInt - lastValue) < DEAD_ZONE) {
    return lastValue;
  }

  return smoothedInt;
}

int applySteeringSmoothing(int rawValue) {
  float diff = abs(rawValue - smoothedSteering);
  bool moving = diff > STEERING_MOVE_THRESHOLD;

  float alpha = moving ? STEERING_ALPHA_MOVE : STEERING_ALPHA_IDLE;
  smoothedSteering = alpha * rawValue + (1.0 - alpha) * smoothedSteering;
  int smoothedInt = (int)smoothedSteering;

  // Saat roda diputar cepat, dead zone dilewati agar tidak ada lag.
  // Saat diam, perubahan kecil akibat noise diabaikan.
  if (!moving && abs(smoothedInt - lastSteering) < STEERING_DEAD_ZONE) {
    return lastSteering;
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

  const int AXIS_CENTER = 16383;
  long axisValue;

  if (rawValue <= steeringCenter) {
    axisValue = map(rawValue, STEERING_MIN, steeringCenter, 0, AXIS_CENTER);
  } else {
    axisValue = map(rawValue, steeringCenter, STEERING_MAX, AXIS_CENTER, 32767);
  }

  if (INVERT_STEERING) {
    axisValue = 32767 - axisValue;   // bukan -axisValue
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

  analogSetPinAttenuation(
    PIN_STEERING,
    ADC_11db
  );

  // Mulai dari posisi steering yang sebenarnya agar tidak bergerak dari nol
  // saat board baru dinyalakan atau Bluetooth baru tersambung.
  int initialSteering = readAnalogAverage(PIN_STEERING);
  steeringCenter = constrain(initialSteering, STEERING_MIN, STEERING_MAX);
  smoothedSteering = steeringCenter;
  lastSteering = steeringCenter;

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

  // Empat sumbu HID terpisah. Steering adalah sumbu bipolar Z:
  // kiri = -32767, lurus = 0, kanan = +32767.
  bleGamepadConfig.setWhichAxes(
    true,  // X: brake
    true,  // Y: accelerator
    true,  // Z: steering
    true,  // RX: clutch
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
  Serial.println("Axis X = Brake");
  Serial.println("Axis Y = Accelerator");
  Serial.println("Axis Z = Steering (- left, 0 center, + right)");
  Serial.println("Axis RX = Clutch");
  Serial.println();
  Serial.println("Steering calibration: " + String(STEERING_MIN) + " to " + String(STEERING_MAX));
  Serial.println("Steering center at startup: " + String(steeringCenter));
  Serial.println("Steering dead zone: " + String(STEERING_DEAD_ZONE) + " ADC units");
  Serial.println("Steering smoothing alpha idle/move: " + String(STEERING_ALPHA_IDLE) + " / " + String(STEERING_ALPHA_MOVE));
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
  int filteredSteering = applySteeringSmoothing(rawSteering);

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

  // Kirim report hanya kalau ada perubahan. Brake dan clutch adalah
  // sumbu independen; keduanya tidak lagi digabung.
  if (changed) {
    bleGamepad.setX(brakeAxis);
    bleGamepad.setY(acceleratorAxis);
    bleGamepad.setZ(steeringAxis);
    bleGamepad.setRX(clutchAxis);
    bleGamepad.sendReport();
  }

 static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

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
    Serial.print(") | STEERING Z: ");
    Serial.print(rawSteering);
    Serial.print(" -> ");
    Serial.print(filteredSteering);
    Serial.print(" (");
    Serial.print(steeringAxis);
    Serial.println(")");
  }

  delay(10);
}
