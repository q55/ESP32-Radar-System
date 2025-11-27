#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <vl53l4cx_class.h>

/* ========= Pin Map ========= */
#define SERVO_PIN    40
#define SDA_PIN      41
#define SCL_PIN      42
#define BUZZER_PIN   39
#define LED_R_PIN     2
#define LED_G_PIN     4
#define LED_B_PIN     5
#define BUTTON_PIN   37

/* ========= Devices ========= */
Servo myservo;
VL53L4CX tof(&Wire, -1);

/* ========= Shared Variables ========= */
volatile bool sweeping        = false;   // controlled by button
volatile bool systemReady     = false;   // becomes true after startup sequence
volatile bool objectDetected  = false;   // instantaneous: true iff current reading ≤ threshold
volatile int  distanceMM      = 0;

/* Alarm is only valid after user arms via button; hold guarantees min 3s alarm */
volatile bool     alarmArmed        = false;
volatile uint32_t alarmHoldUntilMs  = 0;   // alarm active while (millis() < this)

/* ========= Sync ========= */
SemaphoreHandle_t distanceMutex;   // mutex for distance/object flags
SemaphoreHandle_t buttonSem;       // binary semaphore for button ISR

/* ========= Buzzer LEDC ========= */
#define BUZZER_CH 5  // dedicated channel; ESP32Servo uses another LEDC timer internally

/* ========= Direction helper =========
   If your servo moves opposite to what you expect, flip this to true/false.
   Based on your report ("does the opposite"), we default to true here.       */
static const bool SERVO_FLIP = true;  // set to false if your servo already moves correctly
inline void servoWriteDeg(int deg){
  deg = constrain(deg, 0, 180);
  myservo.write(SERVO_FLIP ? (180 - deg) : deg);
}

/* ========= LED helper (digital on/off; set invert=true for common-anode) ========= */
static const bool LED_INVERT = true; // true: common-anode (ON=LOW). false: common-cathode (ON=HIGH)
inline void ledWrite(uint8_t pin, bool on){
  digitalWrite(pin, LED_INVERT ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}
inline void changeLED(uint8_t r, uint8_t g, uint8_t b){ // r/g/b treated as on/off
  ledWrite(LED_R_PIN, r > 0);
  ledWrite(LED_G_PIN, g > 0);
  ledWrite(LED_B_PIN, b > 0);
}

/* ========= ISR (button) ========= */
volatile unsigned long lastPress = 0;
const unsigned long debounceMs = 250;
void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastPress > debounceMs) {
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(buttonSem, &hpw);
    portYIELD_FROM_ISR(hpw);
    lastPress = now;
  }
}

/* ========= Task Prototypes ========= */
void StartupTask(void *pvParameters);
void ServoTask(void *pvParameters);
void ToFTask(void *pvParameters);
void BuzzerTask(void *pvParameters);
void LEDTask(void *pvParameters);
void ButtonTask(void *pvParameters);

/* ========= Setup ========= */
void setup() {
  Serial.begin(115200);
  Serial.println("=== FreeRTOS Radar (VL53L4CX + RGB pins 2/4/5) ===");

  // RGB pins (digital on/off)
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  changeLED(0,0,0);

  // Buzzer (dedicated LEDC channel)
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);
  ledcWriteTone(BUZZER_CH, 0);

  // Button (GPIO 37 may need external 10k pull-up to 3V3)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  // Servo (50 Hz, separate LEDC timer internally)
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN, 500, 2400);
  servoWriteDeg(90); // start centered
  Serial.println("Servo ready.");

  // I2C / ToF init
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  tof.begin();
  if (tof.InitSensor(0x29) != 0) {
    Serial.println("ToF init failed!");
  } else if (tof.VL53L4CX_StartMeasurement() != 0) {
    Serial.println("ToF start failed!");
  } else {
    Serial.println("ToF sensor ready.");
  }

  // Sync primitives
  distanceMutex = xSemaphoreCreateMutex();
  buttonSem     = xSemaphoreCreateBinary();

  // Tasks
  xTaskCreate(StartupTask, "Startup Task", 4096, NULL, 3, NULL);
  xTaskCreate(ServoTask,   "Servo Task",   4096, NULL, 2, NULL);
  xTaskCreate(ToFTask,     "ToF Task",     4096, NULL, 2, NULL);
  xTaskCreate(BuzzerTask,  "Buzzer Task",  2048, NULL, 1, NULL);
  xTaskCreate(LEDTask,     "LED Task",     2048, NULL, 1, NULL);
  xTaskCreate(ButtonTask,  "Button Task",  2048, NULL, 2, NULL);
}

void loop() {
  // handled by FreeRTOS
}

/* ========= TASK DEFINITIONS ========= */

// ----- Startup Task -----
// EXACT ORDER: 90° -> 180° -> 0° -> 90° (at 100°/s).
// LED alternates red/green; buzzer ON during sequence. Then solid green, buzzer off, READY.
void StartupTask(void *pvParameters) {
  Serial.println("Startup: servo+LED+buzzer concurrently...");
  const TickType_t step10ms = pdMS_TO_TICKS(10);   // 1° every 10 ms => 100°/s
  const TickType_t led300ms = pdMS_TO_TICKS(300);  // alternate red/green every 300 ms

  TickType_t lastStep = xTaskGetTickCount();
  TickType_t lastLED  = lastStep;

  bool redPhase = true;
  changeLED(255, 0, 0);           // start red
  ledcWriteTone(BUZZER_CH, 2000); // buzzer ON during startup

  int angle = 90;
  servoWriteDeg(angle);

  // IMPORTANT: order is 90 -> 180 -> 0 -> 90
  const int targets[] = {180, 0, 90};
  int i = 0;

  while (i < 3) {
    TickType_t now = xTaskGetTickCount();

    // LED alternate (red/green)
    if (now - lastLED >= led300ms) {
      redPhase = !redPhase;
      changeLED(redPhase ? 255 : 0, redPhase ? 0 : 255, 0);
      lastLED = now;
    }

    // Servo step 1° at 100°/s
    if (now - lastStep >= step10ms) {
      int tgt = targets[i];
      if (angle < tgt) angle++;
      else if (angle > tgt) angle--;
      servoWriteDeg(angle);
      lastStep = now;
      if (angle == tgt) i++;
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // allow other tasks
  }

  // Ready: buzzer off, solid green
  ledcWriteTone(BUZZER_CH, 0);
  changeLED(0, 255, 0);
  systemReady = true;
  Serial.println("System READY. Press button to toggle sweep.");
  vTaskDelete(NULL);
}

// ----- Servo Task -----
// Sweep 0↔180 at 100°/s when sweeping==true.
// Freeze servo while alarm active (object present OR 3s hold not expired).
// Park at 90° when not sweeping.
void ServoTask(void *pvParameters) {
  const TickType_t step10ms = pdMS_TO_TICKS(10);
  int angle = 90, dir = +1;
  servoWriteDeg(angle);

  while (true) {
    if (!systemReady) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    bool activeAlarm = (alarmArmed && (objectDetected || (millis() < alarmHoldUntilMs)));

    if (sweeping) {
      if (activeAlarm) {
        // hold current angle
        vTaskDelay(step10ms);
      } else {
        angle += dir;
        if (angle >= 180) { angle = 180; dir = -1; }
        else if (angle <= 0) { angle = 0; dir = +1; }
        servoWriteDeg(angle);
        vTaskDelay(step10ms);
      }
    } else {
      if (angle != 90) { angle = 90; servoWriteDeg(90); }
      vTaskDelay(step10ms);
    }
  }
}

// ----- ToF Task -----
// If armed and ≤ 1 m: set objectDetected and extend hold to now+3s.
// Alarm clears only when (object gone AND 3s hold expired). Also prints distance.
void ToFTask(void *pvParameters) {
  const int THRESH_MM = 1000;      // 1 m
  const uint32_t HOLD_MS = 3000;   // minimum alarm duration

  while (true) {
    if (!systemReady) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    uint8_t ready = 0;
    if (tof.VL53L4CX_GetMeasurementDataReady(&ready) == 0 && ready) {
      VL53L4CX_MultiRangingData_t d;
      if (tof.VL53L4CX_GetMultiRangingData(&d) == 0 && d.NumberOfObjectsFound > 0) {
        auto &r = d.RangeData[0];
        if (r.RangeStatus == 0) {
          int mm = (int)r.RangeMilliMeter;

          if (xSemaphoreTake(distanceMutex, portMAX_DELAY) == pdTRUE) {
            distanceMM = mm;
            if (alarmArmed && distanceMM <= THRESH_MM) {
              objectDetected   = true;
              alarmHoldUntilMs = millis() + HOLD_MS;   // extend 3s from latest detection
            } else {
              objectDetected = false;                   // may still be under hold by time
            }
            xSemaphoreGive(distanceMutex);
          }

          Serial.print("Distance: ");
          Serial.print(mm);
          Serial.println(" mm");
        }
      }
      tof.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
  }
}

// ----- Buzzer Task -----
// ON during active alarm (object present OR hold active), OFF otherwise.
void BuzzerTask(void *pvParameters) {
  while (true) {
    if (!systemReady) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    bool activeAlarm = (alarmArmed && (objectDetected || (millis() < alarmHoldUntilMs)));
    ledcWriteTone(BUZZER_CH, activeAlarm ? 2200 : 0);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ----- LED Task -----
// READY: solid green; SWEEPING: slow green blink; ALARM: fast red blink during active alarm.
void LEDTask(void *pvParameters) {
  bool on = false;
  TickType_t last = xTaskGetTickCount();

  while (true) {
    if (!systemReady) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    bool activeAlarm = (alarmArmed && (objectDetected || (millis() < alarmHoldUntilMs)));

    if (activeAlarm) {
      if (xTaskGetTickCount() - last >= pdMS_TO_TICKS(100)) {
        on = !on; changeLED(on ? 255 : 0, 0, 0); last = xTaskGetTickCount();  // fast red
      }
    } else if (sweeping) {
      if (xTaskGetTickCount() - last >= pdMS_TO_TICKS(500)) {
        on = !on; changeLED(0, on ? 255 : 0, 0); last = xTaskGetTickCount();  // slow green
      }
    } else {
      changeLED(0, 255, 0);                 // solid green (ready)
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ----- Button Task -----
// Toggle sweeping; arm/disarm alarm; clear hold/flags when stopping.
void ButtonTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(buttonSem, portMAX_DELAY) == pdTRUE) {
      if (!systemReady) continue;

      sweeping   = !sweeping;
      alarmArmed =  sweeping;           // arm alarm when user enables sweep

      if (!sweeping) {                  // disarm: clear detection and hold
        if (xSemaphoreTake(distanceMutex, portMAX_DELAY) == pdTRUE) {
          objectDetected     = false;
          alarmHoldUntilMs   = 0;
          xSemaphoreGive(distanceMutex);
        }
      }

      Serial.print("Sweeping: ");
      Serial.println(sweeping ? "ON (armed, 3s hold)" : "OFF (disarmed)");
    }
  }
}
