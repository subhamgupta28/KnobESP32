#include <Arduino.h>
#include "Automata.h"
#include "ArduinoJson.h"
#include <WiFi.h>
#include <Arduino.h>
#include "bidi_switch_knob.h"

const char *HOST = "raspberry.local";
int PORT = 8010;
Preferences preferences;
Automata automata("KNOB_NOM", HOST, PORT);
long d = 8000;
long st = millis();
unsigned long startMillis;
int ch = millis();
long start = millis();
JsonDocument doc;

// Knob
#define EXAMPLE_ENCODER_ECA_PIN 19
#define EXAMPLE_ENCODER_ECB_PIN 22

#define SET_BIT(reg, bit) (reg |= ((uint32_t)0x01 << bit))
#define CLEAR_BIT(reg, bit) (reg &= (~((uint32_t)0x01 << bit)))
#define READ_BIT(reg, bit) (((uint32_t)reg >> bit) & 0x01)
#define BIT_EVEN_ALL (0x00ffffff)
EventGroupHandle_t knob_even_ = NULL;
static knob_handle_t s_knob = 0;
int encPos = 0;
SemaphoreHandle_t mutex;

void action(const Action action)
{
  String jsonString;
  serializeJson(action.data, jsonString);
  Serial.println(jsonString);
}
void sendData()
{
  automata.sendData(doc);
}
static void _knob_left_cb(void *arg, void *data)
{
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_, 0);
  xEventGroupSetBits(knob_even_, eventBits_);
}
static void _knob_right_cb(void *arg, void *data)
{
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_, 1);
  xEventGroupSetBits(knob_even_, eventBits_);
}

bool shoot = false;
int pos = -96;
bool changeDetected = false;

static void user_encoder_loop_task(void *arg)
{

  for (;;)
  {
    EventBits_t even = xEventGroupWaitBits(knob_even_, BIT_EVEN_ALL, pdTRUE, pdFALSE, pdMS_TO_TICKS(50));
    if (READ_BIT(even, 0))
    {
      if (xSemaphoreTake(mutex, portMAX_DELAY))
      {
        if (encPos > 0)
          encPos = encPos - 2;
        Serial.println("even");
        changeDetected = true;
        xSemaphoreGive(mutex);
      }
      // vibrateStrong2Sec();
    }
    if (READ_BIT(even, 1))
    {
      if (xSemaphoreTake(mutex, portMAX_DELAY))
      {
        encPos = encPos + 2;
        if (encPos > 100)
          encPos = 100;
        Serial.println("odd");
        changeDetected = true;
        xSemaphoreGive(mutex);
      }
      // vibrateStrong2Sec();
    }
    vTaskDelay(100);
  }
}
void initKnob()
{
  knob_even_ = xEventGroupCreate();
  // create knob
  knob_config_t cfg =
      {
          .gpio_encoder_a = EXAMPLE_ENCODER_ECA_PIN,
          .gpio_encoder_b = EXAMPLE_ENCODER_ECB_PIN,
      };
  s_knob = iot_knob_create(&cfg);

  iot_knob_register_cb(s_knob, KNOB_LEFT, _knob_left_cb, NULL);
  iot_knob_register_cb(s_knob, KNOB_RIGHT, _knob_right_cb, NULL);
  xTaskCreate(user_encoder_loop_task, "user_encoder_loop_task", 3000, NULL, 2, NULL);
}

void setup()
{
  mutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  delay(1000);
  initKnob();
  preferences.begin("bat", false);
  automata.begin();
  automata.addAttribute("encoder", "Encoder", "", "DATA|MAIN");
  // automata.addAttribute("battery_volt", "Battery", "", "DATA|MAIN");
  // automata.addAttribute("upTime", "Up Time", "Hours", "DATA|MAIN");

  automata.registerDevice();
  automata.onActionReceived(action);
  automata.delayedUpdate(sendData);
}

void loop()
{
  // Serial.println("loop");
  doc["encoder"] = encPos;

  // Serial.println("loop");
  automata.loop();

  if (changeDetected || (millis() - start) > 10000)
  {
    automata.sendLive(doc);
    start = millis();
    // Start vibration
    changeDetected = false;
  }

  // delay(200);
}
