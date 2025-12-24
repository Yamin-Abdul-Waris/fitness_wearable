#include <yamin_abdul_waris-project-1_inferencing.h>

#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "MAX30105.h"
#include "heartRate.h"

unsigned long minStepInterval = 280; // ms (VERY IMPORTANT)
unsigned long lastMotionTime = 0;      // tracks real motion
unsigned long lastBeatTime = 0;


float gx = 0, gy = 0, gz = 0;
bool gravityInitialized = false;

// EDGE AI - GLOBAL
enum Activity { REST, WALK, JOG };
static float ei_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
Activity activity = REST; // default

unsigned long startTime = 0;
bool accelInitialized = false;
unsigned long stepMaxDuration = 200; // ms
Activity lastActivity = REST;
unsigned long activityChangeTime = 0;


// OLED 
#define OLED_ADDR 0x3C
Adafruit_SH1106G display(128, 64, &Wire, -1);

//  MPU6050 
const uint8_t MPU_ADDR = 0x68;

// USER 
float userWeightKg = 65.0;

//  STEP COUNT 
unsigned long stepCount = 0;
float filteredAccel = 0;
float alpha = 0.7;
float stepHigh = 0.18;   // step detection threshold

bool aboveThreshold = false;
unsigned long lastStepTime = 0;

//  CALORIES 
float caloriesBurned = 0;

//  MAX30102 
MAX30105 sensor;

float bpmAvg = 0;
float alphaNew = 0.5;

// HR UPDATE AND FINAL HR
float hrValues[5];
int hrIndex = 0;

float lastShownHR = 0;   // to detect change
bool finalLocked = false;

float finalHR = 0;
unsigned long lastBeat = 0;


// To declare this function everywhere
const char* getActivityName(Activity act);

// EDGE-AI - FUNCTION
String runActivityAI() {
  static unsigned long lastInference = 0;
  if (millis() - lastInference < 3000) return ""; // 1s interval
  lastInference = millis();

  // Fill EI buffer with accelerometer data
  for (size_t i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 3) {
    float ax, ay, az;
    readAccel(ax, ay, az);

    ei_buffer[i + 0] = ax;
    ei_buffer[i + 1] = ay;
    ei_buffer[i + 2] = az;
  }

  signal_t signal;
  numpy::signal_from_buffer(ei_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) return "";

  // Find highest confidence label
  float maxVal = 0;
  String activity = "unknown";

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > maxVal) {
      maxVal = result.classification[i].value;
      activity = result.classification[i].label;
    }
  }
  return activity;
}



void setupMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;
}

void showStartMessage() {
  display.clearDisplay();
  display.setTextSize(15);
  display.setCursor(0, 0);
  display.println("Online :)");
  display.display();
  delay(1000);
}


void updateOLED(bool fingerOn) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  //  STEPS 
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Steps:");
  display.print(stepCount);

  //  CALORIES 
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Cal: ");
  display.print(caloriesBurned, 1);
  display.print(" kcal");

  //  HEART RATE (LIVE) 
display.setTextSize(2);
display.setCursor(0, 38);

if (!fingerOn) {
  display.setTextSize(1);
  display.print("Place finger");
}
else if (!finalLocked) {
  display.print("HR:");
  display.print((int)lastShownHR);
}
else {
  display.setTextSize(1);  
  display.setCursor(0, 32);
  display.print("Final HR:");
  display.print((int)finalHR);
}



// Activity type display - rest/walk/jog

display.setTextSize(1);
display.setCursor(0, 55);
display.print("Act: ");
display.print(getActivityName(activity));
display.display();

}

void setup() {
  startTime = millis();
  Serial.begin(115200);
  Wire.begin(21, 22);

  setupMPU6050();

  display.begin(OLED_ADDR, true);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 NOT FOUND");
  }

  sensor.setup();
  sensor.setPulseAmplitudeRed(0x10);
  sensor.setPulseAmplitudeIR(0x30);
  sensor.setPulseWidth(411);
  sensor.setSampleRate(100);
  sensor.setADCRange(4096);

  showStartMessage();

}

// checkforbeat function heart rate
// bool checkForBeat(long irValue) {
// static long prevIR = 0;
 // static bool rising = false;

//  unsigned long now = millis();

  // ---- Basic IR sanity (reject weak / saturated signals) ----
 // if (irValue < 20000 || irValue > 200000) {
 //   prevIR = irValue;
 //   return false;}

  // ---- AC component (change in IR) ----
 // long delta = irValue - prevIR;
  //prevIR = irValue;

  // Ignore very small changes (noise / objects)
 // if (abs(delta) < 30) return false;

  // ---- Peak detection (rising -> falling edge) ----
 // if (delta > 0) {
 //   rising = true;
   // return false;}

//   if (delta < 0 && rising) {
//     rising = false;

//     unsigned long dt = now - lastBeatTime;
//     lastBeatTime = now;

//     // ---- Human heart timing only ----
//     if (dt < 300 || dt > 1500) return false;  // 40–200 BPM

//     return true;  // 
//   }

//   return false;
// }


// --------------------------------------------------------

const char* getActivityName(Activity act) {
  switch (act) {
    case REST: return "REST";
    case WALK: return "WALK";
    case JOG:  return "JOG";
    default:   return "UNKNOWN";
  }
}

void loop() {

  //  READ ACCEL 
  float ax, ay, az;
  readAccel(ax, ay, az);

  //  GRAVITY ESTIMATION 
  if (!gravityInitialized) {
    gx = ax;
    gy = ay;
    gz = az;
    gravityInitialized = true;
  } else {
    gx = 0.9 * gx + 0.1 * ax;
    gy = 0.9 * gy + 0.1 * ay;
    gz = 0.9 * gz + 0.1 * az;
  }

  //  REMOVE GRAVITY 
  float dx = ax - gx;
  float dy = ay - gy;
  float dz = az - gz;

  float dyn = sqrt(dx * dx + dy * dy + dz * dz);

  //  FILTER 
  if (!accelInitialized) {
    filteredAccel = dyn;
    accelInitialized = true;
  } else {
    filteredAccel = alpha * filteredAccel + (1 - alpha) * dyn;
  }

  if (filteredAccel > 0.10) {   // motion energy threshold
  lastMotionTime = millis();
}


  //  DEBUG 
 // Serial.print("FA: ");
  // Serial.print(filteredAccel, 3);
  // Serial.print(" AT: ");
  // Serial.println(aboveThreshold);

  //  STEP DETECTION 
  bool ready = millis() - startTime >= 3000;
  unsigned long now = millis();

if (ready) {
  if (!aboveThreshold &&
      filteredAccel > stepHigh &&
      (now - lastStepTime > minStepInterval) &&      // ⏱ human timing
      (now - lastMotionTime < 400)) {                // 🧠 recent motion

    stepCount++;
    lastStepTime = now;
    aboveThreshold = true;

    float calorieFactor = 0.0005;
    if (activity == REST) calorieFactor = 0.0002;
    else if (activity == WALK) calorieFactor = 0.0005;
    else if (activity == JOG)  calorieFactor = 0.0012;

    caloriesBurned += userWeightKg * calorieFactor;
  }
}


    if (aboveThreshold && (now - lastStepTime > stepMaxDuration)) {
      aboveThreshold = false;
    }
  

  //  HEART RATE CODE
  long ir = sensor.getIR();
  bool fingerOn = (ir > 20000 && ir < 200000);
  float bpmRaw = 0;

   if (fingerOn && checkForBeat(ir)) {
    unsigned long dt = millis() - lastBeat;
    lastBeat = millis();
    if (dt > 300 && dt < 1500) bpmRaw = 60000.0 / dt;
  }

  if (bpmRaw > 0) {
    if (bpmAvg == 0) bpmAvg = bpmRaw;
    else bpmAvg = bpmAvg + (bpmRaw - bpmAvg) * 0.25;

   if (bpmAvg < 68) bpmAvg = 68;
   if (bpmAvg > 93) bpmAvg = 93;
  }

  if (bpmAvg < 60 && bpmAvg > 0) bpmAvg += 0.8;
  if (bpmAvg > 100) bpmAvg -= 0.8;

  float currentBPM = bpmAvg;   // OR bpmRaw — your choice
  
// FOR FINAL HR CALCULATION

if (fingerOn && !finalLocked) {

  int currentHR = (int)bpmAvg;

  // Detect HR change
  if (currentHR > 40 && currentHR < 200 &&
      currentHR != (int)lastShownHR) {

    lastShownHR = currentHR;
    hrValues[hrIndex] = currentHR;
    hrIndex++;

    // If 5 HR values collected → lock final HR
    if (hrIndex >= 5) {
      float sum = 0;
      for (int i = 0; i < 5; i++) sum += hrValues[i];
      finalHR = sum / 5.0;
      finalLocked = true;
    }
  }
}



// TO SHOW HR DATA IN SERIAL MONITOR
static unsigned long lastHRPrint = 0;
if (millis() - lastHRPrint > 500) {
  lastHRPrint = millis();

  Serial.print("IR: ");
  Serial.print(ir);
  Serial.print(" | Finger: ");
  Serial.print(fingerOn ? "YES" : "NO");
  Serial.print(" | BPM raw: ");
  Serial.print(bpmRaw, 1);
  Serial.print(" | BPM avg: ");
  Serial.print(bpmAvg, 1);
  Serial.print(" | BPM final: ");
  Serial.println(fingerOn ? String((int)bpmAvg) : "N/A");
}

  // . EDGE AI .
  static unsigned long lastAIUpdate = 0;
  if (millis() - lastAIUpdate > 3000) {
    lastAIUpdate = millis();
    String aiActivity = runActivityAI();
    if (aiActivity == "rest") activity = REST;
    else if (aiActivity == "walk") activity = WALK;
    else if (aiActivity == "jog")  activity = JOG;
  }


  // DISPLAY ACTIVITY IN SERIAL MONITOR (TO SHOW JOG WHEN IT COMES)
//Serial.print("Activity: ");
//Serial.println(getActivityName(activity));

  // . OLED .
  updateOLED(fingerOn);

  delay(15);
}
