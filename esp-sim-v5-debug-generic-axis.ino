#include <BleGamepad.h>

// =========================
// PIN PEDAL & STEERING
// =========================

#define PIN_ACCELERATOR 35
// Pin sudah di-remap: accelerator GPIO35, brake GPIO32, clutch GPIO34.
#define PIN_BRAKE 32
#define PIN_CLUTCH 34
#define PIN_STEERING 33  // Kaki tengah potensiometer steering

// =========================
// PIN SHIFTER (2 potensiometer silang)
// =========================
// Dibaca sebagai input ADC biasa, BUKAN dikirim sebagai sumbu HID.
// Hasilnya diubah menjadi gear 1-6 lalu dikirim sebagai hat/button.

#define PIN_SHIFT_X 36  // VP
#define PIN_SHIFT_Y 39  // VN

// =========================
// BUTTON TAMBAHAN
// =========================
// Wiring: satu kaki button ke GPIO, kaki lainnya ke GND (pull-up internal).

const int EXTRA_BUTTON_COUNT = 2;
const uint8_t EXTRA_BUTTON_PINS[EXTRA_BUTTON_COUNT] = { 25, 26 };
// Urutan button HID: BUTTON_1 = A, BUTTON_2 = B, BUTTON_3 = X, BUTTON_4 = Y.
// GPIO25 = Button A, GPIO26 = Button B.
const uint8_t EXTRA_BUTTON_HID[EXTRA_BUTTON_COUNT] = { BUTTON_1, BUTTON_2 };
const unsigned long BUTTON_DEBOUNCE_MS = 20;

bool extraStable[EXTRA_BUTTON_COUNT] = { false, false };
bool extraLastRead[EXTRA_BUTTON_COUNT] = { false, false };
unsigned long extraLastChange[EXTRA_BUTTON_COUNT] = { 0, 0 };

// =========================
// BLE GAMEPAD
// =========================
// Nama dinaikkan ke v11 karena HID descriptor berubah (ada hat + button),
// supaya macOS tidak memakai cache descriptor lama.

BleGamepad bleGamepad(
  "v11 Driving",
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
// KALIBRASI SHIFTER
// =========================
// Hasil pengukuran shifter (raw ADC):
//   Gear 1: X 2524, Y 3096    Gear 2: X 2460, Y 503
//   Gear 3: X 1465, Y 3461    Gear 4: X 1484, Y 438
//   Gear 5: X 361,  Y 3193    Gear 6: X 350,  Y 456
// X: tinggi = gear 1/2, tengah = gear 3/4, rendah = gear 5/6.
// Y: tinggi = gear 1/3/5, rendah = gear 2/4/6.
// Batas diberi margin di sekitar nilai terukur.

const int SHIFT_X_MIN = 300;
const int SHIFT_X_MAX = 2600;
const int SHIFT_Y_MIN = 400;
const int SHIFT_Y_MAX = 3500;

// Y aktif saat melewati batas ini (persen dari rentang),
// lepas saat kembali ke batas dalam (hysteresis).
const int Y_ENGAGE_LOW = 25;
const int Y_ENGAGE_HIGH = 75;
const int Y_RELEASE_LOW = 40;
const int Y_RELEASE_HIGH = 60;

// [kolom X: 0 = X rendah, 1 = tengah, 2 = X tinggi][Y: 0 = Y rendah, 1 = Y tinggi]
// Sesuai hasil pengukuran di atas.
const int GEAR_TABLE[3][2] = {
  { 6, 5 },  // X rendah (~355):  Y rendah = 6, Y tinggi = 5
  { 4, 3 },  // X tengah (~1470): Y rendah = 4, Y tinggi = 3
  { 2, 1 }   // X tinggi (~2500): Y rendah = 2, Y tinggi = 1
};

int lastGear = 0;

// Nilai min/max yang pernah terbaca (untuk kalibrasi). Ketik 'r' untuk reset.
int obsXMin = 4095, obsXMax = 0;
int obsYMin = 4095, obsYMax = 0;

// =========================
// ARAH PEDAL
// =========================

const bool INVERT_ACCELERATOR = false;
const bool INVERT_BRAKE = false;
const bool INVERT_CLUTCH = false;
const bool INVERT_STEERING = false;

// =========================
// DEBUG
// =========================

const bool DEBUG_PEDALS = true;       // log pedal & steering
const bool DEBUG_SHIFTER = true;      // log posisi shifter + min/max
const bool DEBUG_WITHOUT_BLE = false; // true = loop jalan walau BLE belum tersambung (untuk kalibrasi)

// =========================
// FILTER ADC & DEAD ZONE
// =========================

const int ADC_SAMPLES = 8;           // Percepat pembacaan ADC
const int DEAD_ZONE = 25;            // Threshold noise pedal: perubahan <25 unit ADC diabaikan
const float SMOOTHING_ALPHA = 0.3;   // Smoothing pedal

// Steering membutuhkan gerakan halus. Saat diam filter sangat halus,
// saat diputar filter hampir tidak menahan agar tidak lag.
const int STEERING_DEAD_ZONE = 8;        // naikkan jika masih bergerak sendiri
const float STEERING_ALPHA_IDLE = 0.15;  // sangat halus saat diam
const float STEERING_ALPHA_MOVE = 0.85;  // responsif saat diputar
const int STEERING_MOVE_THRESHOLD = 40;  // selisih ADC yang dianggap "sedang diputar"

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
  bool invert) {
  rawValue = constrain(rawValue, rawMin, rawMax);

  long axisValue = map(
    rawValue,
    rawMin,
    rawMax,
    0,
    32767);

  if (invert) {
    axisValue = 32767 - axisValue;  // bukan -axisValue
  }

  return constrain(axisValue, 0, 32767);
}

// =========================
// MAP STEERING KE AXIS TENGAH
// =========================
// Rentang 0 - 32767 (unsigned): kiri = 0, tengah = 16383, kanan = 32767.

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
    axisValue = 32767 - axisValue;  // bukan -axisValue
  }

  return constrain(axisValue, 0, 32767);
}

// =========================
// SHIFTER: DETEKSI GEAR
// =========================
// Return 0 = netral, 1-6 = gear.

int detectGear(int rawX, int rawY) {
  static int yState = 0;  // -1 = Y min, 0 = netral, 1 = Y max

  int pctX = map(constrain(rawX, SHIFT_X_MIN, SHIFT_X_MAX), SHIFT_X_MIN, SHIFT_X_MAX, 0, 100);
  int pctY = map(constrain(rawY, SHIFT_Y_MIN, SHIFT_Y_MAX), SHIFT_Y_MIN, SHIFT_Y_MAX, 0, 100);

  // Hysteresis agar tidak bolak-balik di batas
  if (yState == 0) {
    if (pctY < Y_ENGAGE_LOW) yState = -1;
    else if (pctY > Y_ENGAGE_HIGH) yState = 1;
  } else if (yState == -1 && pctY > Y_RELEASE_LOW) {
    yState = 0;
  } else if (yState == 1 && pctY < Y_RELEASE_HIGH) {
    yState = 0;
  }

  if (yState == 0) return 0;

  int column = (pctX < 33) ? 0 : (pctX < 67) ? 1 : 2;
  return GEAR_TABLE[column][yState == -1 ? 0 : 1];
}

// =========================
// SHIFTER: KIRIM GEAR SEBAGAI HAT / BUTTON
// =========================
// Gear 1 = Kanan, 2 = Kiri, 3 = Atas, 4 = Bawah (D-pad), 5 = Button X, 6 = Button Y.
// Urutan button: BUTTON_1 = A, BUTTON_2 = B (button tambahan), BUTTON_3 = X, BUTTON_4 = Y.

void applyGear(int gear) {
  bleGamepad.setHat1(HAT_CENTERED);
  bleGamepad.release(BUTTON_3);  // X
  bleGamepad.release(BUTTON_4);  // Y

  switch (gear) {
    case 1: bleGamepad.setHat1(HAT_RIGHT); break;
    case 2: bleGamepad.setHat1(HAT_LEFT); break;
    case 3: bleGamepad.setHat1(HAT_UP); break;
    case 4: bleGamepad.setHat1(HAT_DOWN); break;
    case 5: bleGamepad.press(BUTTON_3); break;  // X
    case 6: bleGamepad.press(BUTTON_4); break;  // Y
  }
}

// =========================
// SHIFTER: LOG POSISI & MIN/MAX
// =========================

void logShifter(int rawX, int rawY, int gear) {
  // Min/max diperbarui di setiap putaran loop agar puncak tidak terlewat
  if (rawX < obsXMin) obsXMin = rawX;
  if (rawX > obsXMax) obsXMax = rawX;
  if (rawY < obsYMin) obsYMin = rawY;
  if (rawY > obsYMax) obsYMax = rawY;

  // Reset dengan mengetik 'r' di Serial Monitor
  if (Serial.available() && Serial.read() == 'r') {
    obsXMin = obsYMin = 4095;
    obsXMax = obsYMax = 0;
    Serial.println(">> Min/max shifter direset");
  }

  // Cetak 5 kali per detik
  static unsigned long lastLog = 0;
  if (millis() - lastLog < 200) return;
  lastLog = millis();

  int pctX = map(constrain(rawX, SHIFT_X_MIN, SHIFT_X_MAX), SHIFT_X_MIN, SHIFT_X_MAX, 0, 100);
  int pctY = map(constrain(rawY, SHIFT_Y_MIN, SHIFT_Y_MAX), SHIFT_Y_MIN, SHIFT_Y_MAX, 0, 100);

  Serial.print("SHIFT X: ");
  Serial.print(rawX);
  Serial.print(" (");
  Serial.print(pctX);
  Serial.print("%) [min ");
  Serial.print(obsXMin);
  Serial.print(" max ");
  Serial.print(obsXMax);
  Serial.print("] | Y: ");
  Serial.print(rawY);
  Serial.print(" (");
  Serial.print(pctY);
  Serial.print("%) [min ");
  Serial.print(obsYMin);
  Serial.print(" max ");
  Serial.print(obsYMax);
  Serial.print("] | GEAR: ");
  Serial.println(gear);
}

// =========================
// BUTTON TAMBAHAN (DEBOUNCE)
// =========================
// Return true kalau ada button yang berubah status.

bool updateExtraButtons() {
  bool anyChanged = false;

  for (int i = 0; i < EXTRA_BUTTON_COUNT; i++) {
    bool pressed = (digitalRead(EXTRA_BUTTON_PINS[i]) == LOW);

    if (pressed != extraLastRead[i]) {
      extraLastRead[i] = pressed;
      extraLastChange[i] = millis();
    }

    // Status dianggap valid setelah stabil selama debounce
    if ((millis() - extraLastChange[i]) >= BUTTON_DEBOUNCE_MS && pressed != extraStable[i]) {
      extraStable[i] = pressed;
      if (pressed) bleGamepad.press(EXTRA_BUTTON_HID[i]);
      else bleGamepad.release(EXTRA_BUTTON_HID[i]);
      anyChanged = true;
    }
  }

  return anyChanged;
}

// =========================
// SETUP
// =========================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting BLE Driving Controller (v11)...");

  analogReadResolution(ADC_RESOLUTION);

  analogSetPinAttenuation(PIN_ACCELERATOR, ADC_11db);
  analogSetPinAttenuation(PIN_BRAKE, ADC_11db);
  analogSetPinAttenuation(PIN_CLUTCH, ADC_11db);
  analogSetPinAttenuation(PIN_STEERING, ADC_11db);
  analogSetPinAttenuation(PIN_SHIFT_X, ADC_11db);
  analogSetPinAttenuation(PIN_SHIFT_Y, ADC_11db);

  for (int i = 0; i < EXTRA_BUTTON_COUNT; i++) {
    pinMode(EXTRA_BUTTON_PINS[i], INPUT_PULLUP);
  }

  // Mulai dari posisi steering yang sebenarnya agar tidak bergerak dari nol
  // saat board baru dinyalakan atau Bluetooth baru tersambung.
  int initialSteering = readAnalogAverage(PIN_STEERING);
  steeringCenter = constrain(initialSteering, STEERING_MIN, STEERING_MAX);
  smoothedSteering = steeringCenter;
  lastSteering = steeringCenter;

  BleGamepadConfiguration bleGamepadConfig;

  bleGamepadConfig.setAutoReport(false);

  bleGamepadConfig.setControllerType(
    CONTROLLER_TYPE_GAMEPAD);

  // 4 button: BUTTON_1 = A (GPIO25), BUTTON_2 = B (GPIO26),
  // BUTTON_3 = X (gear 5), BUTTON_4 = Y (gear 6).
  // 1 hat switch: D-pad untuk gear 1-4.
  bleGamepadConfig.setButtonCount(4);
  bleGamepadConfig.setHatSwitchCount(1);

  // Empat sumbu HID terpisah. Semua sumbu memakai rentang 0 - 32767.
  // Steering (Z): kiri = 0, lurus = 16383, kanan = 32767.
  bleGamepadConfig.setWhichAxes(
    true,   // X: brake
    true,   // Y: accelerator
    true,   // Z: steering
    true,   // RX: clutch
    false,  // RY
    false,  // RZ
    false,  // Slider 1
    false   // Slider 2
  );

  bleGamepadConfig.setWhichSimulationControls(
    false,  // Rudder
    false,  // Throttle
    false,  // Accelerator
    false,  // Brake
    false   // Steering
  );

  bleGamepadConfig.setSimulationMin(0);
  bleGamepadConfig.setSimulationMax(32767);

  bleGamepad.begin(&bleGamepadConfig);

  bleGamepad.sendReport();

  Serial.println("BLE Gamepad started");
  Serial.println("Axis X = Brake");
  Serial.println("Axis Y = Accelerator");
  Serial.println("Axis Z = Steering (0 left, 16383 center, 32767 right)");
  Serial.println("Axis RX = Clutch");
  Serial.println("Shifter: 1=Right 2=Left 3=Up 4=Down 5=Button X 6=Button Y");
  Serial.println("Extra buttons: A (GPIO25), B (GPIO26)");
  Serial.println();
  Serial.println("Steering calibration: " + String(STEERING_MIN) + " to " + String(STEERING_MAX));
  Serial.println("Steering center at startup: " + String(steeringCenter));
  Serial.println("Steering dead zone: " + String(STEERING_DEAD_ZONE) + " ADC units");
  Serial.println("Steering smoothing alpha idle/move: " + String(STEERING_ALPHA_IDLE) + " / " + String(STEERING_ALPHA_MOVE));
  Serial.println("ADC samples: " + String(ADC_SAMPLES));
  Serial.println("Ketik 'r' di Serial Monitor untuk reset min/max shifter");
}

// =========================
// LOOP
// =========================

void loop() {
  if (!bleGamepad.isConnected() && !DEBUG_WITHOUT_BLE) {
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

  // Shifter: baca dua potensiometer, tentukan gear
  int rawShiftX = readAnalogAverage(PIN_SHIFT_X);
  int rawShiftY = readAnalogAverage(PIN_SHIFT_Y);
  int gear = detectGear(rawShiftX, rawShiftY);

  if (gear != lastGear) {
    lastGear = gear;
    applyGear(gear);
    changed = true;
  }

  // Button tambahan
  if (updateExtraButtons()) {
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

  // Debug Serial Monitor (maksimal 5 kali per detik)
  if (DEBUG_PEDALS) {
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
  }

  if (DEBUG_SHIFTER) {
    logShifter(rawShiftX, rawShiftY, gear);
  }

  delay(10);
}
