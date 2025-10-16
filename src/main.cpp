#include <Arduino.h>
int adcPin = 0;
int adcResolution = 12;
float vRef = 3.3;
int sampleRateHz = 4000;
float dcTau = 0.15;
float rmsTau = 0.05;
unsigned long sampleIntervalUs;
float adcScale;
float dcEstimate = 0.0;
float sqEstimate = 0.0;
float alpha;
float beta;
unsigned long nextSampleUs = 0;
unsigned long lastPrintMs = 0;
int printIntervalMs = 100;

void setup() {
  Serial.begin(115200);
  analogReadResolution(adcResolution);
  analogSetPinAttenuation(adcPin, ADC_11db);
  adcScale = vRef / (float)((1 << adcResolution) - 1);
  sampleIntervalUs = 1000000UL / sampleRateHz;
  alpha = 1.0f - expf(-1.0f / (sampleRateHz * dcTau));
  beta = 1.0f - expf(-1.0f / (sampleRateHz * rmsTau));
  nextSampleUs = micros();
}

void loop() {
  while ((long)(micros() - nextSampleUs) >= 0) {
    int raw = analogRead(adcPin);
    float v = raw * adcScale;
    dcEstimate += alpha * (v - dcEstimate);
    float ac = v - dcEstimate;
    sqEstimate += beta * (ac * ac - sqEstimate);
    nextSampleUs += sampleIntervalUs;
  }

  if (millis() - lastPrintMs >= (unsigned long)printIntervalMs) {
    float rms = sqrtf(sqEstimate);
    int raw = analogRead(adcPin);
    float v = raw * adcScale;
    Serial.print("raw=");
    Serial.print(raw);
    Serial.print(" bias=");
    Serial.print(dcEstimate,4);
    Serial.print("V rms=");
    Serial.print(rms,4);
    Serial.println("V");
    lastPrintMs = millis();
  }
}

