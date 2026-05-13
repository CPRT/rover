#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "DHT.h"

#pragma pack(push, 1)
struct PwmCommand {
  uint8_t  pin;
  uint16_t duty_cycle;
  uint16_t duration;
  uint16_t frequency;
  uint16_t ramp;
};

struct SensorReadings {
  uint16_t methane;
  uint16_t co2;
  uint16_t polarimeter;
  float    temperature;
  float    moisture;
};
#pragma pack(pop)

static const uint8_t  PWM_RES_BITS   = 10;           
static const uint16_t PWM_MAX_DUTY   = (1u << PWM_RES_BITS) - 1;
static const uint8_t  PWM_MAX_ACTIVE = 8;            
static const uint32_t PWM_TICK_MS    = 10;            

static const uint32_t SENSOR_PERIOD_MS = 100;

static const int METHANE_PIN     = 12;
static const int CO2_PIN         = 33;
static const int POLARIMETER_PIN = 4;
static const int DHT_PIN = 14;

#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

static QueueHandle_t g_pwmCmdQueue = nullptr;

struct ActivePwm {
  bool     active;
  uint8_t  pin;
  uint16_t duty_cycle;
  uint16_t frequency;
  uint32_t start_ms;
  uint32_t ramp_end_ms;
  uint32_t run_end_ms;
};

static ActivePwm active_pins[PWM_MAX_ACTIVE];

static int findPWMIndex(uint8_t pin) {
  for (int i = 0; i < PWM_MAX_ACTIVE; ++i) {
    if (active_pins[i].active && active_pins[i].pin == pin) return i;
  }
  return -1;
}

static int findFreeIndex() {
  for (int i = 0; i < PWM_MAX_ACTIVE; ++i) {
    if (!active_pins[i].active) return i;
  }
  return -1;
}

static void releaseIndex(int i) {
  if (i < 0 || i >= PWM_MAX_ACTIVE) return;
  if (!active_pins[i].active) return;
  ledcWrite(active_pins[i].pin, 0);
  ledcDetach(active_pins[i].pin);
  active_pins[i].active = false;
}

static void runCommand(const PwmCommand& cmd) {
  uint16_t duty = cmd.duty_cycle;
  if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;

  int i = findPWMIndex(cmd.pin);
  if (i < 0) i = findFreeIndex();
  if (i < 0) return;
  
  if (!active_pins[i].active) {
    if (!ledcAttach(cmd.pin, cmd.frequency, PWM_RES_BITS)) {
      return;
    }
  } else if(cmd.frequency != active_pins[i].frequency) return;

  uint32_t now    = millis();
  uint32_t rampMs = (uint32_t)cmd.ramp     * 100u;
  uint32_t holdMs = (uint32_t)cmd.duration * 100u;

  active_pins[i].active      = true;
  active_pins[i].pin         = cmd.pin;
  active_pins[i].duty_cycle = duty;
  active_pins[i].start_ms    = now;
  active_pins[i].ramp_end_ms = now + rampMs;
  active_pins[i].run_end_ms  = active_pins[i].ramp_end_ms + holdMs;
  active_pins[i].frequency = cmd.frequency;

  if (rampMs == 0) {
    ledcWrite(cmd.pin, duty);
  } else {
    ledcWrite(cmd.pin, 0);
  }
}

static void updateActivePwm() {
  uint32_t now = millis();
  for (int i = 0; i < PWM_MAX_ACTIVE; ++i) {
    if (!active_pins[i].active) continue;
    ActivePwm& a = active_pins[i];

    if ((int32_t)(now - a.run_end_ms) >= 0) {
      releaseIndex(i);
      continue;
    }

    if ((int32_t)(now - a.ramp_end_ms) >= 0) {
      ledcWrite(a.pin, a.duty_cycle);
      continue;
    }

    uint32_t elapsed  = now - a.start_ms;
    uint32_t rampSpan = a.ramp_end_ms - a.start_ms;
    uint32_t scaled   = (uint32_t)a.duty_cycle * elapsed / rampSpan;
    if (scaled > a.duty_cycle) scaled = a.duty_cycle;
    ledcWrite(a.pin, scaled);
  }
}

static void pwmTask(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    PwmCommand cmd;
    while (xQueueReceive(g_pwmCmdQueue, &cmd, 0) == pdTRUE) {
      runCommand(cmd);
    }
    updateActivePwm();
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PWM_TICK_MS));
  }
}

static uint16_t readMethane()      { return (uint16_t)analogRead(METHANE_PIN); }
static uint16_t readCo2()          { return (uint16_t)analogRead(CO2_PIN); }
static uint16_t readPolarimeter()  { return (uint16_t)analogRead(POLARIMETER_PIN); }
static float    readTemperatureC() { return dht.readTemperature(); }
static float    readMoisture()     { return dht.readHumidity(); }

static void sensorTask(void* /*arg*/) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    SensorReadings pkt;
    pkt.methane     = readMethane();
    pkt.co2         = readCo2();
    pkt.polarimeter = readPolarimeter();
    pkt.temperature = readTemperatureC();
    pkt.moisture    = readMoisture();

    Serial.write(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  for (int i = 0; i < PWM_MAX_ACTIVE; ++i) active_pins[i].active = false;

  g_pwmCmdQueue = xQueueCreate(8, sizeof(PwmCommand));

  xTaskCreatePinnedToCore(pwmTask,    "pwmTask",    4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, nullptr, 1, nullptr, 1);
}

void loop() {
  if (Serial.available() >= (int)sizeof(PwmCommand)) {
    PwmCommand cmd;
    Serial.readBytes(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
    xQueueSend(g_pwmCmdQueue, &cmd, 0);
  }
  vTaskDelay(pdMS_TO_TICKS(1));
}
