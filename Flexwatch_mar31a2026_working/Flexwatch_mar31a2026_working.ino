#include <Wire.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <LSM6DS3.h>
#include <RV3028C7.h>

// =====================================================
// Pin mapping for Seeed Studio XIAO nRF52840 Sense
// =====================================================
#define EPD_CS   D1
#define EPD_DC   D3
#define EPD_RST  D0
#define EPD_BUSY D2

// Battery measurement pins
#define BAT_PIN PIN_VBAT
#define BAT_PIN_ENABLE 14

// ADC conversion constants for battery voltage
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V reference, 12 bit ADC
#define VBAT_DIVIDER      (0.66225165F)   // Voltage divider ratio
#define VBAT_DIVIDER_COMP (0.001F)        // Compensation factor

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// Battery state of charge lookup table (rough approximation)
const uint8_t socTable[11] = {
  0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};

// Approximate LiPo voltage curve for 0 to 100 percent
const float vTable[11] = {
  3.20, 3.35, 3.50, 3.60, 3.68, 3.73, 3.77, 3.80, 3.85, 3.95, 4.20
};

// ADC reference values
const float ADC_REF = 3.0;
const int ADC_MAX = 4095;
const float DIVIDER_RATIO = 3.0;

// =====================================================
// E paper display instance (2.9 inch Waveshare panel)
// =====================================================
GxEPD2_BW<GxEPD2_290_I6FD, GxEPD2_290_I6FD::HEIGHT> display(
  GxEPD2_290_I6FD(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY)
);

// RTC and IMU instances
RV3028C7 rtc;
LSM6DS3 imu(I2C_MODE, 0x6A);

// Time tracking
int lastMinute = -1;
unsigned long lastMotionMillis = 0;
bool screenCleared = false;

// =====================================================
// Convert compile time into ISO 8601 format
// Used to set RTC on first flash
// =====================================================
String compileTimeISO8601() {
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char monthStr[4];
  int day, year;

  // Parse __DATE__
  sscanf(__DATE__, "%3s %d %d", monthStr, &day, &year);
  int month = (strstr(months, monthStr) - months) / 3 + 1;

  // Parse __TIME__
  int hour, minute, second;
  sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);

  // Build ISO string
  char isoStr[25];
  snprintf(isoStr, sizeof(isoStr), "%04d-%02d-%02dT%02d:%02d:%02d",
           year, month, day, hour, minute, second);

  return String(isoStr);
}

// =====================================================
// Read battery voltage and return "(XX%)" string
// =====================================================
String getBatteryVoltage() {
  // Use 3.0V internal reference
  analogReference(AR_INTERNAL_3_0);
  analogReadResolution(12);
  delay(1);

  int raw = analogRead(BAT_PIN);

  // Restore defaults
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  float adcVoltage = raw * REAL_VBAT_MV_PER_LSB;
  float batteryVoltage = adcVoltage * DIVIDER_RATIO;
  int percent = batteryPercent(batteryVoltage);

  // Build percentage string
  String result = "(";
  result += String(percent);
  result += "%)";

  Serial.printf("[BAT] ADCVolt=%.3f batVolt=%.3f percent=%.3f\n",
                adcVoltage, batteryVoltage, percent);

  return result;
}

// =====================================================
// Convert battery voltage to approximate percentage
// Using linear interpolation between vTable entries
// =====================================================
int batteryPercent(float v) {
  if (v <= vTable[0]) return socTable[0];
  if (v >= vTable[10]) return socTable[10];

  int i;
  for (i = 0; i < 10; i++) {
    if (v < vTable[i + 1]) break;
  }

  float v0 = vTable[i];
  float v1 = vTable[i + 1];
  int s0 = socTable[i];
  int s1 = socTable[i + 1];

  float t = (v - v0) / (v1 - v0);
  float soc = s0 + t * (s1 - s0);

  if (soc < 0.0f) soc = 0.0f;
  if (soc > 100.0f) soc = 100.0f;

  return (int)(soc + 0.5f);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("=== E Paper Watch (Compile Time RTC Set) ===");

  Wire.begin();

  // Battery measurement setup
  pinMode(BAT_PIN_ENABLE, OUTPUT);
  digitalWrite(BAT_PIN_ENABLE, LOW);
  pinMode(BAT_PIN, INPUT);

  // Init e paper display
  Serial.println("[INIT] E paper...");
  display.init(4000000);
  display.setRotation(3);
  display.setFullWindow();
  Serial.println("[OK] E paper ready");

  // Init RTC
  Serial.println("[INIT] RTC...");
  if (!rtc.begin()) {
    Serial.println("[ERROR] Could not find RV3028C7 RTC");
    while (1);
  }
  Serial.println("[OK] RTC ready");

  // Set RTC to compile time (only needed once)
  String dateTime = compileTimeISO8601();
  Serial.print("[SET] Setting RTC to compile time: ");
  Serial.println(dateTime);
  rtc.setDateTimeFromISO8601(dateTime);
  rtc.synchronize();
  // COMMENT OUT ABOVE AFTER FIRST FLASH

  // Clear ghosting on e paper
  display.fillScreen(GxEPD_BLACK);
  display.fillScreen(GxEPD_BLACK);
  display.fillScreen(GxEPD_WHITE);
  display.fillScreen(GxEPD_BLACK);

  // Init IMU
  Serial.println("[INIT] IMU...");
  if (imu.begin() != 0) {
    Serial.println("[ERROR] Could not find LSM6DS3 IMU");
    while (1);
  }
  Serial.println("[OK] IMU ready");

  updateDisplay();
  lastMotionMillis = millis();
}

// =====================================================
// Main loop
// =====================================================
void loop() {
  // Read accelerometer
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  // Read time from RTC
  String nowstr = rtc.getCurrentDateTime();
  int minuteNow = nowstr.substring(14, 16).toInt();

  // Detect wrist movement
  if (magnitude > 1.2) {
    Serial.printf("[IMU] X=%.3f Y=%.3f Z=%.3f Mag=%.3f\n",
                  ax, ay, az, magnitude);
    lastMotionMillis = millis();
    screenCleared = false;
    digitalWrite(LED_PWR, LOW);
  }

  // Update display once per minute
  if (minuteNow != lastMinute && !screenCleared) {
    Serial.println("[TRIGGER] Updating display...");
    lastMinute = minuteNow;
    updateDisplay();
  } else {
    // Clear screen after 5 minutes of no movement
    if (!screenCleared && (millis() - lastMotionMillis > 300000)) {
      Serial.println("[TIMEOUT] No motion for 300s, clearing screen...");
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);
      } while (display.nextPage());
      screenCleared = true;
    }
  }

  delay(500);
}

// =====================================================
// Draw time, date, and battery on e paper
// =====================================================
void updateDisplay() {
  String nowstr = rtc.getCurrentDateTime();

  int year    = nowstr.substring(0, 4).toInt();
  int month   = nowstr.substring(5, 7).toInt();
  int day     = nowstr.substring(8, 10).toInt();
  int hours   = nowstr.substring(11, 13).toInt();
  int minutes = nowstr.substring(14, 16).toInt();

  // Convert to 12 hour format
  bool isPM = false;
  if (hours == 0) hours = 12;
  else if (hours == 12) isPM = true;
  else if (hours > 12) {
    hours -= 12;
    isPM = true;
  }

  char timeStr[12];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d %s",
           hours, minutes, isPM ? "PM" : "AM");

  char dateStr[16];
  snprintf(dateStr, sizeof(dateStr), "%02d/%02d/%04d",
           day, month, year);

  String batStr = getBatteryVoltage();

  // Draw to e paper
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    display.setFont(&FreeMonoBold18pt7b);
    display.setCursor(40, 60);
    display.print(timeStr);

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(40, 100);
    display.print(dateStr);

    display.setCursor(180, 100);
    display.print(batStr);

  } while (display.nextPage());

  Serial.println("[OK] Display updated");
}
