// =============================================================================
//  Single-Phase Smart Power Meter | Dual-Core FreeRTOS Edition
//  Core 1 (APP_CPU): DMA ADC Sampling & 64-bit DSP Math
//  Core 0 (PRO_CPU): Blynk IoT, Wi-Fi & Cloud-Persisted Energy Accumulation
// =============================================================================

// --- Blynk Credentials ---
#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "YOUR_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN    "YOUR_AUTH_TOKEN"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "esp_adc/adc_continuous.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// --- Wi-Fi Credentials ---
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";

// =============================================================================
//  CALIBRATION CONSTANTS (Tuned for 100W Bulb @ ~235V)
// =============================================================================
float VCAL = 0.8269;   
float ICAL = 0.0177;   
int SAMPLE_SHIFT = -14; // Coarse Tune: The Macro Step (Array Shift)
float PHASECAL = 1.20;  // Fine Tune: The Micro Step (Interpolation)

// =============================================================================
//  ALERT CONFIGURATION
// =============================================================================
#define POWER_LIMIT_W           80.0    // Trigger alert if power exceeds 150 Watts
#define ALERT_COOLDOWN_MS       60000UL  // Wait 60 seconds before sending another power alert
#define ENERGY_LIMIT_WH         200000.0 // Trigger alert if energy exceeds 200 kWh (200,000 Wh)

// --- DMA & Sampling Configuration ---
#define SAMPLE_FREQ_HZ          20000   // ESP32 Hardware Minimum is 20kHz (10k per channel)
#define NUM_CHANNELS            2
#define READ_LEN                256     
#define SAMPLES_PER_CALC        2000    // 10,000 SPS * 0.2 seconds (200ms) = 2000

// 200ms calculation window in hours (used for Wh accumulation)
#define CALC_INTERVAL_HOURS     (200.0f / 1000.0f / 3600.0f)

// --- Blynk Send Rate Control ---
#define TELEMETRY_INTERVAL_MS   2000UL  // UI update every 2 seconds
#define ENERGY_SAVE_INTERVAL_MS 30000UL // Cloud save every 30 seconds

// ADC1 channel map 
adc_channel_t channels[NUM_CHANNELS] = {
    ADC_CHANNEL_4,   // Voltage (GPIO32 - ZMPT101B)
    ADC_CHANNEL_5,   // Current (GPIO33 - ACS712)
};
adc_continuous_handle_t handle = NULL;

// --- DC Offset ---
int adcCenter[NUM_CHANNELS]   = {2048, 2048};
const int calPins[NUM_CHANNELS] = {32, 33};

// --- Waveform buffers (accessed ONLY by Core 1 / MathTask) ---
int16_t vRaw[SAMPLES_PER_CALC];
int16_t iRaw[SAMPLES_PER_CALC];
int sample_index = 0;

// =============================================================================
//  ENERGY ACCUMULATOR
// =============================================================================
volatile double totalEnergyWh   = 0.0;
bool energyRestoredFromCloud   = false;

// =============================================================================
//  FREERTOS IPC STRUCTURES
// =============================================================================
typedef struct {
    float vRms;
    float iRms;
    float realP;
    float pf;
} PowerData_t;

PowerData_t latestData = {};
bool        newDataAvailable = false;

QueueHandle_t powerDataQueue;
TaskHandle_t TaskMath_Handle;
TaskHandle_t TaskIoT_Handle;

void core1MathTask(void * pvParameters);
void core0IoTTask(void * pvParameters);

// =============================================================================
//  BLYNK CLOUD ENERGY PERSISTENCE CALLBACKS
// =============================================================================
BLYNK_CONNECTED() {
    Serial.println("[Blynk] Connected. Requesting saved energy value from cloud...");
    Blynk.syncVirtual(V5); // Energy is mapped to V5
}

BLYNK_WRITE(V5) {
    float restoredKwh = param.asFloat();
    if (restoredKwh >= 0.0f && restoredKwh < 1000000.0f) {
        totalEnergyWh = restoredKwh * 1000.0;
        energyRestoredFromCloud = true;
        Serial.printf("[Blynk] Energy counter restored: %.3f Wh (%.5f kWh)\n", totalEnergyWh, restoredKwh);
    } else {
        Serial.printf("[Blynk] WARN: Rejected implausible V5 value: %.2f kWh\n", restoredKwh);
        energyRestoredFromCloud = true;
    }
}

// =============================================================================
//  HELPER: AUTO DC-OFFSET CALIBRATION
// =============================================================================
void calibrateOffsets() {
    Serial.println("\n--- DC Offset Auto-Calibration ---");
    Serial.println("Ensure NO AC signals are connected. Settling for 1s...");
    delay(1000);
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        long sum = 0;
        const int samples = 500;
        for (int i = 0; i < samples; i++) {
            sum += analogRead(calPins[ch]);
            delayMicroseconds(200);
        }
        adcCenter[ch] = (int)(sum / samples);
        Serial.printf("  Channel %d (GPIO%d): offset = %d\n", ch, calPins[ch], adcCenter[ch]);
    }
    Serial.println("Calibration complete.\n");
}

int channelToIndex(uint32_t ch_num) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if ((uint32_t)channels[i] == ch_num) return i;
    }
    return -1;
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== Single-Phase Smart Meter Booting ===");

    calibrateOffsets();
    
    // Lower Wi-Fi TX Power to reduce heat
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    powerDataQueue = xQueueCreate(5, sizeof(PowerData_t));
    if (powerDataQueue == NULL) {
        Serial.println("FATAL: Failed to create power data queue. Halting.");
        while (1);
    }

    // Configure DMA ADC
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size    = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_digi_pattern_config_t adc_pattern[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        adc_pattern[i].atten     = ADC_ATTEN_DB_11;
        adc_pattern[i].channel   = channels[i];
        adc_pattern[i].unit      = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }

    adc_continuous_config_t dig_cfg = {
        .pattern_num    = NUM_CHANNELS,
        .adc_pattern    = adc_pattern,
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    xTaskCreatePinnedToCore(core1MathTask, "MathTask", 8192, NULL, 2, &TaskMath_Handle, 1);
    xTaskCreatePinnedToCore(core0IoTTask,  "IoTTask",  8192, NULL, 1, &TaskIoT_Handle,  0);

    Serial.println("Dual-Core RTOS Started.\n");
}

void loop() {
    vTaskDelete(NULL);
}

// =============================================================================
//  CORE 1 TASK: DMA SAMPLING + DSP MATH
// =============================================================================
void core1MathTask(void * pvParameters) {
    uint8_t  result[READ_LEN];
    uint32_t ret_num = 0;

    for (;;) {
        memset(result, 0, sizeof(result));
        esp_err_t ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);

        if (ret == ESP_OK) {
            int samples_in_frame = (int)(ret_num / SOC_ADC_DIGI_RESULT_BYTES);

            for (int i = 0; i < samples_in_frame; i++) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i * SOC_ADC_DIGI_RESULT_BYTES];
                uint32_t channel_num = p->type1.channel;
                int ch_idx = channelToIndex(channel_num);
                if (ch_idx < 0) continue;

                int16_t raw_val   = (int16_t)p->type1.data - adcCenter[ch_idx];
                int     write_idx = sample_index + (i / NUM_CHANNELS);

                if (write_idx < SAMPLES_PER_CALC) {
                    if (channel_num == ADC_CHANNEL_4) vRaw[write_idx] = raw_val;
                    else if (channel_num == ADC_CHANNEL_5) iRaw[write_idx] = raw_val;
                }
            }

            sample_index += samples_in_frame / NUM_CHANNELS;

            if (sample_index >= SAMPLES_PER_CALC) {
                int64_t sumSqV = 0, sumSqI = 0;
                float sumP = 0.0;
                int16_t lastV = 0;

                // Prevent reading outside the array bounds when shifting
                int start_idx = (SAMPLE_SHIFT > 0) ? SAMPLE_SHIFT : 0;
                int end_idx   = (SAMPLE_SHIFT < 0) ? SAMPLES_PER_CALC + SAMPLE_SHIFT : SAMPLES_PER_CALC;
                int valid_samples = end_idx - start_idx;

                for (int i = start_idx; i < end_idx; i++) {
                    int vIdx = i;
                    int iIdx = i - SAMPLE_SHIFT;

                    sumSqV += (int64_t)vRaw[vIdx] * vRaw[vIdx];
                    sumSqI += (int64_t)iRaw[iIdx] * iRaw[iIdx];
                    
                    // The Hybrid Phase Correction
                    // 1. We take the already macro-shifted voltage wave (vRaw[vIdx])
                    // 2. We apply fractional interpolation to nudge it into the perfect spot
                    float fineShiftedV = lastV + PHASECAL * (vRaw[vIdx] - lastV);
                    sumP += fineShiftedV * iRaw[iIdx];
                    lastV = vRaw[vIdx];
                }

                PowerData_t data;
                data.vRms  = sqrtf((float)sumSqV / valid_samples) * VCAL;
                data.iRms  = sqrtf((float)sumSqI / valid_samples) * ICAL;
                
                // NOISE GATES: Force phantom readings to zero when power is off
                if (data.vRms < 50.0) data.vRms = 0.0;
                if (data.iRms < 0.25) data.iRms = 0.0;

                // Only calculate power if voltage and current exist
                if (data.vRms > 0.0 && data.iRms > 0.0) {
                    data.realP = (sumP / valid_samples) * (VCAL * ICAL);
                    float appP = data.vRms * data.iRms;
                    data.pf    = (appP > 0.0f) ? (data.realP / appP) : 0.0f;
                    
                    // Account for natural grid harmonics clipping the wave top
                    if (data.pf > 0.97) data.pf = 1.0; 
                } else {
                    data.realP = 0.0;
                    data.pf = 0.0;
                }

                if (xQueueSend(powerDataQueue, &data, 0) != pdTRUE) {
                    Serial.println("WARN: Queue full — frame dropped.");
                }

                sample_index = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================================
//  CORE 0 TASK: IOT, BLYNK & CLOUD ENERGY PERSISTENCE
// =============================================================================
void core0IoTTask(void * pvParameters) {
    uint32_t lastTelemetrySendMs = 0;
    uint32_t lastEnergySaveMs    = 0;
    uint32_t lastAlertTime       = 0; 
    bool     energyAlertSent     = false; // Lockout flag for total energy

    for (;;) {
        Blynk.run();

        PowerData_t incoming;
        while (xQueueReceive(powerDataQueue, &incoming, 0) == pdTRUE) {
            latestData       = incoming;
            newDataAvailable = true;

            if (energyRestoredFromCloud) {
                totalEnergyWh += (double)incoming.realP * CALC_INTERVAL_HOURS;
            }
        }

        uint32_t now = millis();

        // --- DEADLOCK FAILSAFE ---
        if (!energyRestoredFromCloud && now > 15000) {
            Serial.println("[Blynk] No historical data found on server. Starting fresh at 0 kWh.");
            energyRestoredFromCloud = true;
        }

        // --- REAL POWER PUSH NOTIFICATION ---
        // Fires if power is too high AND (it is the first alert OR 60 seconds have passed)
        if (latestData.realP > POWER_LIMIT_W && (lastAlertTime == 0 || now - lastAlertTime >= ALERT_COOLDOWN_MS)) {
            String alertMsg = String("Warning! Power consumption reached ") + String(latestData.realP, 1) + " W";
            Blynk.logEvent("high_power", alertMsg);
            Serial.println("\n*** PUSH NOTIFICATION SENT: " + alertMsg + " ***\n");
            lastAlertTime = now;
        }

        // --- TOTAL ENERGY PUSH NOTIFICATION ---
        if (totalEnergyWh > ENERGY_LIMIT_WH && !energyAlertSent) {
            String energyMsg = String("Notice! Total energy has reached ") + String(totalEnergyWh / 1000.0, 2) + " kWh";
            Blynk.logEvent("high_energy", energyMsg);
            Serial.println("\n*** PUSH NOTIFICATION SENT: " + energyMsg + " ***\n");
            energyAlertSent = true; // Lock the flag so it only fires once!
        }

        // --- TELEMETRY SEND LOGIC ---
        if (newDataAvailable && (now - lastTelemetrySendMs >= TELEMETRY_INTERVAL_MS)) {
            Serial.println("-------------------------------");
            Serial.printf("Voltage : %6.2f V\n", latestData.vRms);
            Serial.printf("Current : %6.3f A\n", latestData.iRms);
            Serial.printf("Power   : %6.2f W\n", latestData.realP);
            Serial.printf("PF      : %6.3f\n", latestData.pf);
            Serial.printf("Energy  : %7.3f Wh (%.5f kWh)%s\n",
                          totalEnergyWh, totalEnergyWh / 1000.0,
                          energyRestoredFromCloud ? "" : "  [waiting for sync]");
            Serial.println("-------------------------------\n");

            Blynk.beginGroup();
                Blynk.virtualWrite(V1, latestData.vRms);
                Blynk.virtualWrite(V2, latestData.iRms);
                Blynk.virtualWrite(V3, latestData.realP);
                Blynk.virtualWrite(V4, latestData.pf);
            Blynk.endGroup();

            lastTelemetrySendMs = now;
            newDataAvailable    = false;
        }

        // --- ENERGY CLOUD SAVE LOGIC ---
        if (energyRestoredFromCloud && (now - lastEnergySaveMs >= ENERGY_SAVE_INTERVAL_MS)) {
            Blynk.virtualWrite(V5, totalEnergyWh / 1000.0); // Save as kWh
            lastEnergySaveMs = now;
            Serial.printf("[Blynk] Energy saved to cloud: %.5f kWh\n", totalEnergyWh / 1000.0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}