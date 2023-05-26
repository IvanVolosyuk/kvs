#include <Arduino.h>
#include "storage.hpp"
#include <SPI.h>
#include <SD.h>

FlashStorage storage;

#include <SPI.h>
#include <SD.h>

File myFile;


#include <driver/dac.h>
 
// Timer0 Configuration Pointer (Handle)
hw_timer_t *Timer0_Cfg = NULL;
 
static const int BUF_SIZE = 65536;
uint8_t ringBuffer[BUF_SIZE];
volatile uint32_t SampleIdx = 0;
uint32_t ReadIdx = 0;
 
// The Timer0 ISR Function (Executes Every Timer0 Interrupt Interval)
void IRAM_ATTR Timer0_ISR()
{
  // Send SineTable Values To DAC One By One
  dac_output_voltage(DAC_CHANNEL_1, ringBuffer[SampleIdx++]);
  if(SampleIdx == BUF_SIZE)
  {
    SampleIdx = 0;
  }
}

void fillRing(uint32_t start, uint32_t size) {
  while (size > 0) {
    if (!myFile.available()) {
      myFile.close();
      myFile = SD.open("/audio.bin");
      if (!myFile) {
        Serial.println("Failed to reopen file");
        while (true) sleep(1);
      }
    }
    int len = myFile.readBytes((char*)(ringBuffer + start), size);
    if (len <= 0) {
        Serial.println("Failed to read file");
        while (true) sleep(1);
    }
    start += len;
    size -= len;
  }
}

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Initialized.");
  storage.Init();
  auto value = storage.Get("foo");
  Serial.println(value.c_str());

  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);

  const int kCS = 5;

  // pinMode(MOSI, INPUT);
  // pinMode(MISO, INPUT);
  // pinMode(SCK, INPUT);
  // pinMode(SS, INPUT_PULLDOWN);
  // pinMode(kCS, INPUT_PULLDOWN);

  Serial.println("Initializing SD card...");
  if (!SD.begin(kCS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("/audio.bin");
  if (!myFile) {
    Serial.println("Failed to open /audio.bin");
    while (true) sleep(1);
  }

  fillRing(0, BUF_SIZE);

  // Configure Timer0 Interrupt
  Timer0_Cfg = timerBegin(0, 181, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 10, true);
  timerAlarmEnable(Timer0_Cfg);
  // Enable DAC1 Channel's Output
  dac_output_enable(DAC_CHANNEL_1);
}

static const int DAC_CH1 = 25;

void loop() {
  delay(100);
  Serial.println(SampleIdx);
  uint16_t sample = SampleIdx;
  if (sample > ReadIdx) {
    fillRing(ReadIdx, sample - ReadIdx);
  } else {
    fillRing(ReadIdx, BUF_SIZE - ReadIdx);
    fillRing(0, sample);

  }
  ReadIdx = sample;
}
