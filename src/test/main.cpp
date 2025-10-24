#include <Arduino.h>

int adcPin = 0;
int adcResolution = 12;
float vRef = 3.3f;
int sampleRateHz = 4000;
float dcTau = 0.15f;
float rmsTau = 0.04f;
unsigned long sampleIntervalUs;
float adcScale;
float dcEstimate = 0.0f;
float sqEstimate = 0.0f;
float alpha;
float beta;
unsigned long nextSampleUs = 0;
unsigned long lastPrintMs = 0;
int printIntervalMs = 60;
int led_pin = 8;
const int rms_buffer_size = 6;
float rms_buffer[10];
int rms_buffer_idx;
bool blinking;
unsigned long blink_start;
unsigned long blink_duration = 200UL;
const int history_size = 100;
float rms_history[100];
int history_idx;
int history_count;
float silence_percentile = 0.06f;
float noise_percentile = 0.20f;
float baseline_silence = 0.0f;
float baseline_noise = 0.0f;
float detection_sigma_multiplier = 3.0f;
float reset_sigma_multiplier = 1.0f;
unsigned long last_trigger_ms = 0;
unsigned long refractory_ms = 150UL;
int consecutive_required = 1;
int consecutive_count = 0;
float prev_rms = 0.0f;
float rms_derivative = 0.0f;
float derivative_alpha = 0.6f;
float min_derivative_threshold = 0.006f;
const int LED_ON = LOW;
const int LED_OFF = HIGH;

float get_median() {
    float temp[10];
    for (int k = 0; k < rms_buffer_size; k++) temp[k] = rms_buffer[k];
    for (int i = 0; i < rms_buffer_size - 1; i++) {
        for (int j = i + 1; j < rms_buffer_size; j++) {
            if (temp[i] > temp[j]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    return temp[rms_buffer_size / 2];
}

float get_history_percentile(float p) {
    if (history_count == 0) return 0.0f;
    float temp[100];
    int n = history_count < history_size ? history_count : history_size;
    for (int k = 0; k < n; k++) temp[k] = rms_history[k];
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (temp[i] > temp[j]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    int idx = (int)(p * (n - 1));
    if (idx < 0) idx = 0;
    if (idx >= n) idx = n - 1;
    return temp[idx];
}

float get_history_std() {
    if (history_count < 2) return 0.0f;
    int n = history_count < history_size ? history_count : history_size;
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += rms_history[i];
    float mean = sum / (float)n;
    float sq_sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float dev = rms_history[i] - mean;
        sq_sum += dev * dev;
    }
    return sqrtf(sq_sum / (float)n);
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(adcResolution);
    analogSetPinAttenuation(adcPin, ADC_11db);
    adcScale = vRef / (float)((1 << adcResolution) - 1);
    sampleIntervalUs = 1000000UL / sampleRateHz;
    alpha = 1.0f - expf(-1.0f / (sampleRateHz * dcTau));
    beta = 1.0f - expf(-1.0f / (sampleRateHz * rmsTau));
    nextSampleUs = micros();
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LED_OFF);
    blinking = false;
    blink_start = 0;
    rms_buffer_idx = 0;
    for (int i = 0; i < rms_buffer_size; i++) rms_buffer[i] = 0.0f;
    history_idx = 0;
    history_count = 0;
    for (int i = 0; i < history_size; i++) rms_history[i] = 0.0f;
    baseline_silence = 0.0f;
    baseline_noise = 0.0f;
    prev_rms = 0.0f;
    rms_derivative = 0.0f;
    consecutive_count = 0;
    last_trigger_ms = 0;
    Serial.println("Continuous adaptive bang detector started");
}

void loop() {
    static int lastRaw = 0;
    unsigned long nowMs = millis();
    if (blinking && millis() - blink_start >= blink_duration) {
        digitalWrite(led_pin, LED_OFF);
        blinking = false;
    }
    while ((long)(micros() - nextSampleUs) >= 0) {
        int raw = analogRead(adcPin);
        lastRaw = raw;
        float v = raw * adcScale;
        dcEstimate += alpha * (v - dcEstimate);
        float ac = v - dcEstimate;
        sqEstimate += beta * (ac * ac - sqEstimate);
        nextSampleUs += sampleIntervalUs;
    }
    if (nowMs - lastPrintMs >= (unsigned long)printIntervalMs) {
        float rms = sqrtf(sqEstimate);
        rms_buffer[rms_buffer_idx] = rms;
        rms_buffer_idx = (rms_buffer_idx + 1) % rms_buffer_size;
        float filtered_rms = get_median();
        rms_history[history_idx] = filtered_rms;
        history_idx = (history_idx + 1) % history_size;
        if (history_count < history_size) history_count++;
        rms_derivative = derivative_alpha * (filtered_rms - prev_rms) + (1.0f - derivative_alpha) * rms_derivative;
        prev_rms = filtered_rms;
        if (history_count >= 10) {
            baseline_silence = get_history_percentile(silence_percentile);
            baseline_noise = get_history_percentile(noise_percentile);
            float noise_range = baseline_noise - baseline_silence;
            if (noise_range < 0.001f) noise_range = 0.001f;
            float dynamic_threshold = baseline_noise + detection_sigma_multiplier * noise_range;
            float reset_threshold = baseline_noise + reset_sigma_multiplier * noise_range;
            float derivative_threshold = min_derivative_threshold;
            if (noise_range > min_derivative_threshold) {
                derivative_threshold = noise_range / (printIntervalMs * 0.001f);
            }
            bool amplitude_exceeded = filtered_rms >= dynamic_threshold;
            bool transient_detected = rms_derivative > derivative_threshold;
            bool event_detected = amplitude_exceeded && transient_detected;
            if (event_detected) {
                consecutive_count++;
                if (consecutive_count >= consecutive_required && (nowMs - last_trigger_ms >= refractory_ms)) {
                    if (!blinking) {
                        digitalWrite(led_pin, LED_ON);
                        blinking = true;
                        blink_start = millis();
                        last_trigger_ms = nowMs;
                        Serial.println(">>> BANG DETECTED <<<");
                    }
                }
            } else if (filtered_rms < reset_threshold) {
                consecutive_count = 0;
            }
            int raw_sample = lastRaw;
            Serial.print("raw=");
            Serial.print(raw_sample);
            Serial.print(" rms=");
            Serial.print(rms, 4);
            Serial.print("V filt=");
            Serial.print(filtered_rms, 6);
            Serial.print("V sil=");
            Serial.print(baseline_silence, 6);
            Serial.print("V nse=");
            Serial.print(baseline_noise, 6);
            Serial.print("V thr=");
            Serial.print(dynamic_threshold, 6);
            Serial.print("V drv=");
            Serial.print(rms_derivative, 6);
            Serial.print(" cnt=");
            Serial.println(consecutive_count);
        } else {
            int raw_sample = lastRaw;
            Serial.print("raw=");
            Serial.print(raw_sample);
            Serial.print(" rms=");
            Serial.print(rms, 4);
            Serial.print("V filt=");
            Serial.print(filtered_rms, 6);
            Serial.print("V [learning: ");
            Serial.print(history_count);
            Serial.print("/");
            Serial.print(history_size);
            Serial.println("]");
        }
        lastPrintMs = nowMs;
    }
}

