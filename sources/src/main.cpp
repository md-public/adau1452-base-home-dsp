#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <EncButton.h>
#include <EEPROM.h>
#include <TimerMs.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include "SigmaStudioFW.h"
#include "USER_SETTINGS.h"
#include "adau_IC_1.h"
#include "adau_IC_1_PARAM.h"

#define DSP_TYPE DSP_TYPE_SIGMA300_350

#define VOL_MAIN_ADDR MOD_VOL_MAIN_ALG0_TARGET_ADDR
#define VOL_SUB_ADDR MOD_VOL_SUB_ALG0_TARGET_ADDR
#define SWITCH_SOURCE_ADDR MOD_SWITCH_SOURCE_STEREOMUXSIGMA300NS1INDEX_ADDR

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr uint8_t SCREEN_ADDRESS = 0x3C;
constexpr uint32_t SCREEN_REFRESH_TIMEOUT = 50;

constexpr uint32_t SYNC_DELAY = 2000;
constexpr uint32_t SLEEP_HOLD_TIMEOUT = 3000;

constexpr uint8_t ROTARY_ENCODER_A_PIN = 4;
constexpr uint8_t ROTARY_ENCODER_B_PIN = 14;
constexpr uint8_t ROTARY_ENCODER_BUTTON_PIN = 27;
constexpr int ROTARY_ACCELERATION = 4;
constexpr uint8_t MAX_VOL = 30;
constexpr uint8_t I2C_SDA_PIN = SDA;
constexpr uint8_t I2C_SCL_PIN = SCL;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
EncButton enc(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN);
TimerMs rerenderDisplayTimer(SCREEN_REFRESH_TIMEOUT, 1, 1);
TimerMs syncStateTimer(SYNC_DELAY, 1, 1);

// PERSISTABLE STATE
struct State
{
  uint8_t volume;
  uint8_t subVolume;
  uint8_t source;
};

constexpr size_t EEPROM_SIZE = sizeof(State);

const char *const sources[] = {
    "OPT",
    "AUX",
    // " BT",
};
constexpr size_t SOURCES_COUNT = sizeof(sources) / sizeof(sources[0]);

State activeState;
bool sleepMode = false;

void rerenderDisplay();
void updateVolume(bool isLeft, bool isFast);
void updateSubVolume(bool isLeft, bool isFast);
void toggleSleepMode();
bool releaseI2CBus();
bool restoreI2CBus();
void changeSource();
void saveStateToEEPROM();
void syncVolumeWithDSP();
void syncSubVolumeWithDSP();
void syncSourceWithDSP();
void writeToEEPROM();

void setupTimers()
{
  rerenderDisplayTimer.setPeriodMode();
  syncStateTimer.setTimerMode();
}

void timersLoop()
{
  if (rerenderDisplayTimer.tick())
    rerenderDisplay();

  if (syncStateTimer.tick())
    writeToEEPROM();
}

void setupDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.display();
}

void setSmallFont()
{
  display.setFont(&FreeSansBold9pt7b);
}

void setLargeFont()
{
  display.setFont(&FreeSansBold18pt7b);
}

void rerenderDisplay()
{
  if (sleepMode)
    return;

  display.clearDisplay();
  setSmallFont();

  display.setCursor(0, 12);
  display.println(F("Sub: "));
  display.setCursor(60, 12);
  if (activeState.subVolume >= 10)
    display.setCursor(50, 12);
  if (activeState.subVolume >= 100)
    display.setCursor(40, 12);
  display.println(activeState.subVolume);

  display.setCursor(86, 12);
  display.println(F(sources[activeState.source]));

  display.drawFastHLine(0, 16, SCREEN_WIDTH, SSD1306_WHITE);
  display.drawFastVLine(80, 0, 16, SSD1306_WHITE);

  display.setCursor(0, 45);
  display.println(F("Volume:"));
  setLargeFont();
  display.setCursor(90, 51);
  if (activeState.volume >= 10)
    display.setCursor(80, 51);
  if (activeState.volume >= 100)
    display.setCursor(70, 51);

  display.println(activeState.volume);

  display.display();
}

void handleSleepButton()
{
  static uint32_t sleepPressStartedAt = 0;
  static bool sleepHoldHandled = false;
  static bool sleepPressTracked = false;

  if (enc.pressing())
  {
    if (!sleepPressTracked)
    {
      sleepPressStartedAt = millis();
      sleepHoldHandled = false;
      sleepPressTracked = true;
    }

    if (!sleepHoldHandled &&
        (uint32_t)(millis() - sleepPressStartedAt) >= SLEEP_HOLD_TIMEOUT)
    {
      toggleSleepMode();
      sleepHoldHandled = true;
    }
  }
  else
  {
    sleepPressTracked = false;
  }
}

void rotaryLoop()
{
  const bool wasSleeping = sleepMode;
  enc.tick();
  handleSleepButton();

  if (wasSleeping || sleepMode)
    return;

  if (enc.turnH())
  {
    updateSubVolume(enc.leftH(), enc.fast());
  }
  else if (enc.turn())
  {
    updateVolume(enc.left(), enc.fast());
  }
  if (enc.click())
  {
    changeSource();
  }
}

void adjustVolume(uint8_t &volume, bool isLeft, bool isFast)
{
  int value = volume;
  const int step = isFast ? ROTARY_ACCELERATION : 1;
  value += isLeft ? -step : step;
  value = constrain(value, 0, MAX_VOL);
  volume = static_cast<uint8_t>(value);
  saveStateToEEPROM();
}

void updateVolume(bool isLeft, bool isFast)
{
  adjustVolume(activeState.volume, isLeft, isFast);
  syncVolumeWithDSP();
}

void updateSubVolume(bool isLeft, bool isFast)
{
  adjustVolume(activeState.subVolume, isLeft, isFast);
  syncSubVolumeWithDSP();
}

void toggleSleepMode()
{
  if (!sleepMode)
  {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    if (!releaseI2CBus())
    {
      Serial.println("Failed to release I2C bus; sleep mode not entered");
      display.ssd1306_command(SSD1306_DISPLAYON);
      return;
    }

    sleepMode = true;
  }
  else
  {
    if (!restoreI2CBus())
    {
      Serial.println("Failed to restore I2C bus; device remains asleep");
      return;
    }

    display.ssd1306_command(SSD1306_DISPLAYON);
    sleepMode = false;
  }
}

void changeSource()
{
  activeState.source++;
  if (activeState.source >= SOURCES_COUNT)
    activeState.source = 0;

  saveStateToEEPROM();
  syncSourceWithDSP();
}

void setupState()
{
  EEPROM.get(0, activeState);

  if (activeState.volume > MAX_VOL)
    activeState.volume = 0;
  if (activeState.subVolume > MAX_VOL)
    activeState.subVolume = 0;

  if (activeState.source >= SOURCES_COUNT)
    activeState.source = 0;

  syncVolumeWithDSP();
  syncSubVolumeWithDSP();
  syncSourceWithDSP();
}

void saveStateToEEPROM()
{
  syncStateTimer.start();
}

void writeToEEPROM()
{
  EEPROM.put(0, activeState);
  EEPROM.commit();
}

double volumeToGain(uint8_t volume)
{
  return volume == 0 ? 0.0 : pow(10.0, (static_cast<double>(volume) - MAX_VOL) / 20.0);
}

void syncVolumeWithDSP()
{
  if (sleepMode)
    return;
  SIGMA_WRITE_REGISTER_FLOAT(VOL_MAIN_ADDR, volumeToGain(activeState.volume));
  Serial.print("Volume: ");
  Serial.println(activeState.volume);
  Serial.println(volumeToGain(activeState.volume));
}

void syncSubVolumeWithDSP()
{
  if (sleepMode)
    return;
  SIGMA_WRITE_REGISTER_FLOAT(VOL_SUB_ADDR, volumeToGain(activeState.subVolume));
}

void syncSourceWithDSP()
{
  if (sleepMode)
    return;
  const int sourceValue = activeState.source * 2;
  SIGMA_WRITE_REGISTER_INTEGER(SWITCH_SOURCE_ADDR, sourceValue);
}

bool releaseI2CBus()
{
  if (!Wire.end())
    return false;

  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);

  return true;
}

bool restoreI2CBus()
{
  return Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_SPEED);
}

void setup()
{
  Serial.begin(115200);
  if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_SPEED))
  {
    Serial.println("Failed to initialize I2C bus");
    for (;;)
      ;
  }
  default_download_IC_1();

  EEPROM.begin(EEPROM_SIZE);
  setupDisplay();
  setupTimers();
  setupState();
}

void loop()
{
  rotaryLoop();
  timersLoop();
}