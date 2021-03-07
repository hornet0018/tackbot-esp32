//Motor ID: 0x02

#include <CAN.h>
#include <FastLED.h>
#include <Arduino.h>
#include <ArduinoJson.h>

#define NUM_LEDS 25
#define DATA_PIN 27
CRGB leds[NUM_LEDS];

#define M1A 22
#define M1B 19
#define M2A 23
#define M2B 33

#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8
#define M1A_PWM_CHANNEL 0
#define M1B_PWM_CHANNEL 1
#define M2A_PWM_CHANNEL 2
#define M2B_PWM_CHANNEL 3

#define MAX_SPEED 255

void commandProcess();

int16_t speedR_data = 0;
int16_t speedL_data = 0;
int32_t clientSendTimeDiff = 0; // クライアントとの通信間隔
int32_t prevClientSendTime = 0; // 前回クライアントと通信した時の時間
int32_t receivedTime = 0;       // こっちが受信したときの内部時間
int32_t prevSendTime = 0;       // こっちが受信したときの内部時間

float t = 0;
float a1 = 0;
float a2 = 0;

void task0(void *arg)
{
  while (1)
  {
    DynamicJsonDocument root(200);
    int packetSize = CAN.parsePacket();
    while (CAN.available())
    {
      if (CAN.packetId() == 0x01)
      {
        uint8_t dataHH = (uint8_t)CAN.read();
        uint8_t dataHL = (uint8_t)CAN.read();
        uint8_t dataLH = (uint8_t)CAN.read();
        uint8_t dataLL = (uint8_t)CAN.read();
        char soc = (uint8_t)CAN.read();

        int32_t data = (int32_t)(
            (((int32_t)dataHH << 24) & 0xFF000000) | (((int32_t)dataHL << 16) & 0x00FF0000) | (((int32_t)dataLH << 8) & 0x0000FF00) | (((int32_t)dataLL << 0) & 0x000000FF));
        root["batt"] = data;
        root["soc"] = soc;
        serializeJson(root, Serial);
        Serial.println("");
      }
      else
      {
        break;
      }
    }
    delay(1);
  }
}

void robotMove(int speedLeft, int speedRight)
{
  if (speedLeft > 0)
  {
    int speedL = map(speedLeft, 0, 255, 255, 0);

    ledcWrite(M1A_PWM_CHANNEL, 255);
    ledcWrite(M1B_PWM_CHANNEL, speedL);
  }
  else
  {
    int speedL = map(speedLeft, -255, 0, 0, 255);
    ledcWrite(M1A_PWM_CHANNEL, speedL);
    ledcWrite(M1B_PWM_CHANNEL, 255);
  }

  if (speedRight > 0)
  {
    int speedR = map(speedRight, 0, 255, 255, 0);
    ledcWrite(M2A_PWM_CHANNEL, 255);
    ledcWrite(M2B_PWM_CHANNEL, speedR);
  }
  else
  {
    int speedR = map(speedRight, -255, 0, 0, 255);
    ledcWrite(M2A_PWM_CHANNEL, speedR);
    ledcWrite(M2B_PWM_CHANNEL, 255);
  }
}

void setup()
{
  ledcSetup(M1A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M1B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2A_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(M2B_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(M1A, M1A_PWM_CHANNEL);
  ledcAttachPin(M1B, M1B_PWM_CHANNEL);
  ledcAttachPin(M2A, M2A_PWM_CHANNEL);
  ledcAttachPin(M2B, M2B_PWM_CHANNEL);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 26, 32);
  // create tasks
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(20);

  CAN.setPins(25, 21);

  for (int k = 0; k < NUM_LEDS; k++)
  {
    leds[k] = CRGB::Blue;
  }
  FastLED.show();

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
  }
}

void loop()
{
  while (Serial.available())
  {
    DynamicJsonDocument root(1024);
    deserializeJson(root, Serial);
    a1 = root["axes1"];
    a2 = root["axes2"];
    t = root["time"];
    commandProcess();
  }
  int32_t clientSendTime = t;
  clientSendTimeDiff = millis() - clientSendTime;
  if (millis() - prevSendTime > 200)
  {
    prevSendTime = millis();
    Serial1.println(clientSendTimeDiff);
  }
  delay(1);
}

void commandProcess(void)
{
  if (abs(a1) > 0)
  {
    robotMove(a1 * -255, a1 * 255);
  }
  else if (abs(a2) > 0)
  {
    robotMove(a2 * 255, a2 * 255);
  }
  else
  {
    robotMove(0, 0);
  }
}