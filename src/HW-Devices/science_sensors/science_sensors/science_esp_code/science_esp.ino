#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <DHTesp.h>

#define TYPE_DC 0x00
#define TYPE_SERVO 0x01
#define TYPE_POLAR 0x02

#pragma pack(push, 1)
struct PwmCommand {
  uint8_t  pin;
  uint8_t  type;
  uint16_t duty_cycle;
  uint16_t duration;
  uint16_t frequency;
  uint16_t ramp;
};

struct SensorReadings {
  uint16_t methane;
  uint16_t co2;
  float    temperature;
  float    moisture;
};
#pragma pack(pop)

static const uint8_t MAGIC0 = 0xAA;
static const uint8_t MAGIC1 = 0x55;

static const uint8_t  PWM_RES_BITS   = 10;
static const uint16_t PWM_MAX_DUTY   = (1u << PWM_RES_BITS) - 1;
static const uint8_t  PWM_MAX_ACTIVE = 8;
static const uint32_t PWM_TICK_MS    = 10;

static const uint32_t SENSOR_PERIOD_MS = 100;
static const uint32_t DHT_PERIOD_MS = 2000;

static const int METHANE_PIN     = 12;
static const int CO2_PIN         = 33;
static const int DHT_PIN         = 14;

DHTesp dht;

static QueueHandle_t g_pwmCmdQueue = nullptr;

struct ActivePwm {
  bool     active;
  uint8_t  pin;
  uint8_t  type;
  uint16_t duty_cycle;
  uint16_t frequency;
  uint32_t start_ms;
  uint32_t ramp_end_ms;
  uint32_t run_end_ms;
  uint32_t start_duty;
};

static ActivePwm active_pins[PWM_MAX_ACTIVE];

static uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
  }
  return c;
}

static bool readFramedCommand(PwmCommand& cmd) {
  static uint8_t state = 0;
  static uint8_t payload[sizeof(PwmCommand)];
  static size_t payload_index = 0;

  while (Serial.available() > 0) {
    uint8_t b = Serial.read();

    switch (state) {
      case 0:
        if (b == MAGIC0) {
          state = 1;
        }
        break;

      case 1:
        if (b == MAGIC1) {
          payload_index = 0;
          state = 2;
        } else if (b == MAGIC0) {
          state = 1;
        } else {
          state = 0;
        }
        break;

      case 2:
        payload[payload_index++] = b;
        if (payload_index >= sizeof(PwmCommand)) {
          state = 3;
        }
        break;

      case 3: {
        uint8_t expected = calcChecksum(payload, sizeof(PwmCommand));
        uint8_t received = b;

        state = 0;
        payload_index = 0;

        if (received == expected) {
          memcpy(&cmd, payload, sizeof(PwmCommand));
          return true;
        }

        break;
      }

      default:
        state = 0;
        payload_index = 0;
        break;
    }
  }

  return false;
}

#define TYPE_SENSORS 0x01
#define TYPE_POLAR 0x02

static void writeFramedSensorReadings(const uint8_t* payload, size_t len, uint8_t type) {
  uint8_t checksum = calcChecksum(payload, len);

  Serial.write(MAGIC0);
  Serial.write(MAGIC1);
  Serial.write(type);
  Serial.write(payload, len);
  Serial.write(checksum);
}

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
  } else if (cmd.frequency != active_pins[i].frequency) {
    releaseIndex(i);

    if (!ledcAttach(cmd.pin, cmd.frequency, PWM_RES_BITS)) {
      return;
    }
  }

  if (cmd.type == TYPE_SERVO) {
    active_pins[i].active     = true;
    active_pins[i].type       = TYPE_SERVO;
    active_pins[i].pin        = cmd.pin;
    active_pins[i].duty_cycle = duty;
    active_pins[i].frequency  = cmd.frequency;
    ledcWrite(cmd.pin, duty);
  } else if (cmd.type == TYPE_DC) {
    uint32_t now    = millis();
    uint32_t rampMs = (uint32_t)cmd.ramp     * 100u;
    uint32_t holdMs = (uint32_t)cmd.duration * 100u;

    active_pins[i].active      = true;
    active_pins[i].type        = TYPE_DC;
    active_pins[i].start_duty  = active_pins[i].duty_cycle;
    active_pins[i].pin         = cmd.pin;
    active_pins[i].duty_cycle  = duty;
    active_pins[i].start_ms    = now;
    active_pins[i].ramp_end_ms = now + rampMs;
    active_pins[i].run_end_ms  = active_pins[i].ramp_end_ms + holdMs;
    active_pins[i].frequency   = cmd.frequency;

    if (rampMs == 0) {
      ledcWrite(cmd.pin, duty);
    }
  } else if (cmd.type == TYPE_POLAR) {
    active_pins[i].active      = true;
    active_pins[i].type        = TYPE_POLAR;
    active_pins[i].pin         = cmd.pin;
    active_pins[i].duty_cycle  = duty;
    active_pins[i].frequency   = cmd.frequency;
    runPolarimeter(cmd.pin);
  }
}

static void updateActivePwm() {
  uint32_t now = millis();
  for (int i = 0; i < PWM_MAX_ACTIVE; ++i) {
    if (!active_pins[i].active) continue;
    if (active_pins[i].type == TYPE_SERVO) continue;
    if (active_pins[i].type == TYPE_POLAR) continue;
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
    uint32_t rampDelta = (uint32_t)a.duty_cycle - a.start_duty;
    uint32_t scaled = rampDelta * elapsed / rampSpan + a.start_duty;
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

static uint16_t readMethane()     { return (uint16_t)analogRead(METHANE_PIN); }
static uint16_t readCo2()         { return (uint16_t)analogRead(CO2_PIN); }

static void sensorTask(void* /*arg*/) {
  static const int DHT_INTERVAL = DHT_PERIOD_MS / SENSOR_PERIOD_MS;
  static int dht_counter = 0;
  static float lastTemperature = 0;
  static float lastMoisture = 0;
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    SensorReadings pkt;
    pkt.methane     = readMethane();
    pkt.co2         = readCo2();
    if (++dht_counter >= DHT_INTERVAL) {
      dht_counter = 0;
      TempAndHumidity data = dht.getTempAndHumidity();
      lastTemperature = data.temperature;
      lastMoisture    = data.humidity;
    }
    pkt.temperature = lastTemperature;
    pkt.moisture = lastMoisture;
    writeFramedSensorReadings(reinterpret_cast<const uint8_t*>(&pkt), sizeof(SensorReadings), TYPE_SENSORS);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

int pwmAngle = 0;
const int POLAR_MIN_MS = 615;
const int POLAR_MAX_MS = 2495;
const int POLAR_MAX_ANGLE = 180;
const int POLAR_SENSOR_PIN1 = 4;
const int POLAR_SENSOR_PIN2 = 5;

static void runPolarimeter(uint8_t pin) {
  int sensor_readings[POLAR_MAX_ANGLE + 1] = {0};
  while (pwmAngle <= POLAR_MAX_ANGLE) {
    int ms = map(pwmAngle, 0, POLAR_MAX_ANGLE, POLAR_MIN_MS, POLAR_MAX_MS);
    ledcWrite(pin, map(ms, 0, 20000, 0, PWM_MAX_DUTY));
    delay(200);
    pwmAngle++;
    int sensor1 = analogRead(POLAR_SENSOR_PIN1);
    int sensor2 = analogRead(POLAR_SENSOR_PIN2);
    int diff = abs(sensor1 - sensor2);
    sensor_readings[pwmAngle] = diff;
  }
  writeFramedSensorReadings(reinterpret_cast<const uint8_t*>(sensor_readings), (POLAR_MAX_ANGLE + 1) * sizeof(int), TYPE_POLAR);
}

void setup() {
  Serial.begin(115200);
  dht.setup(DHT_PIN, DHTesp::DHT22);

  for (int i = 0; i < PWM_MAX_ACTIVE; ++i) {
    active_pins[i].active = false;
  }

  g_pwmCmdQueue = xQueueCreate(8, sizeof(PwmCommand));

  xTaskCreatePinnedToCore(pwmTask,    "pwmTask",    4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
  PwmCommand cmd;

  while (readFramedCommand(cmd)) {
    xQueueSend(g_pwmCmdQueue, &cmd, pdMS_TO_TICKS(20));
  }
  vTaskDelay(pdMS_TO_TICKS(1));
}
