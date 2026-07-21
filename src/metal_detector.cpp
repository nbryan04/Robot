#include "metal_detector.h"

// Legacy PCNT counter limit. The hardware auto-resets the count to 0 when it
// reaches this, and fires the H_LIM event we tally as an overflow.
static const int16_t PCNT_HIGH_LIMIT = 30000;

MetalDetector::MetalDetector(int pin, unsigned long interval, float thresholdHz,
                             pcnt_unit_t unit)
    : sensorPin(pin), pcntUnit(unit), updateInterval(interval), threshold(thresholdHz) {
    // Initialize the filter buffer array with zeros
    for(int i = 0; i < FILTER_SIZE; i++) {
        freqBuffer[i] = 0.0;
    }
}

// ISR that fires silently in the background whenever the counter hits the high
// limit (and the hardware auto-resets it to 0). File-static so its signature
// stays out of the header.
static void IRAM_ATTR pcnt_overflow_isr(void* arg) {
    std::atomic<int>* overflows = static_cast<std::atomic<int>*>(arg);
    overflows->fetch_add(1, std::memory_order_relaxed);
}

void MetalDetector::begin() {
    Serial.println("Initializing Hardware Pulse Counter (PCNT)...");

    // 1. Configure the PCNT unit + channel: count UP on rising edges only.
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = sensorPin;
    cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    cfg.channel        = PCNT_CHANNEL_0;
    cfg.unit           = pcntUnit;
    cfg.pos_mode       = PCNT_COUNT_INC;   // rising edge -> increment
    cfg.neg_mode       = PCNT_COUNT_DIS;   // falling edge -> ignore
    cfg.lctrl_mode     = PCNT_MODE_KEEP;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.counter_h_lim  = PCNT_HIGH_LIMIT;
    cfg.counter_l_lim  = -1;               // never reached (count-up only)
    pcnt_unit_config(&cfg);

    // 2. Fire an event (and auto-reset) when we reach the high limit.
    pcnt_event_enable(pcntUnit, PCNT_EVT_H_LIM);

    // 3. Zero and pause while we wire up the ISR.
    pcnt_counter_pause(pcntUnit);
    pcnt_counter_clear(pcntUnit);

    // 4. Attach the overflow ISR. The service may already be installed by
    //    ESP32Encoder; a second install just returns an error we can ignore.
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(pcntUnit, pcnt_overflow_isr, &overflow_count);

    // 5. Start counting.
    pcnt_counter_resume(pcntUnit);

    // --- Calibration Phase ---
    Serial.println("Calibrating metal detector... Keep metal away!");

    // Take a deep 2-second sample on startup
    recalibrate(2000);

    Serial.print("Base Frequency established: ");
    Serial.print(baseFrequency);
    Serial.println(" Hz");
}

void MetalDetector::update() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastUpdateTime >= updateInterval) {

        // 1. Read hardware counter and clear it immediately
        int16_t hardwareCount = 0;
        pcnt_get_counter_value(pcntUnit, &hardwareCount);
        pcnt_counter_clear(pcntUnit);

        // 2. Fetch and reset overflows
        int overflows = overflow_count.exchange(0, std::memory_order_relaxed);
        unsigned long totalPulses = (overflows * PCNT_HIGH_LIMIT) + hardwareCount;

        // 3. Calculate raw live frequency for this interval
        float rawFrequency = totalPulses * (1000.0 / updateInterval);

        // --- 4. Apply Moving Average Filter ---
        freqSum -= freqBuffer[bufferIndex];        // Subtract the oldest reading
        freqBuffer[bufferIndex] = rawFrequency;    // Insert the newest reading
        freqSum += rawFrequency;                   // Add the newest reading

        bufferIndex = (bufferIndex + 1) % FILTER_SIZE; // Wrap index back to 0 if it hits max

        currentFrequency = freqSum / FILTER_SIZE;  // Calculate the smoothed average
        // --------------------------------------

        // 5. Calculate shift from baseline
        frequencyShift = currentFrequency - baseFrequency;

        lastUpdateTime = currentMillis;
    }
}

// Instantly zeroes out the detector using the current smoothed background data
void MetalDetector::tare() {
    baseFrequency = currentFrequency;
    frequencyShift = 0.0;
}

// Forces a fresh hardware sample (Blocking)
void MetalDetector::recalibrate(unsigned long sampleTimeMs) {
    // 1. Clear the hardware counter and overflow tracker
    pcnt_counter_clear(pcntUnit);
    overflow_count.store(0, std::memory_order_relaxed);

    // 2. Wait for the sample duration
    delay(sampleTimeMs);

    // 3. Read the hardware
    int16_t hardwareCount = 0;
    pcnt_get_counter_value(pcntUnit, &hardwareCount);
    int overflows = overflow_count.exchange(0, std::memory_order_relaxed);

    unsigned long totalPulses = (overflows * PCNT_HIGH_LIMIT) + hardwareCount;

    // 4. Resume normal counting
    pcnt_counter_clear(pcntUnit);

    // 5. Calculate new baseline and reset the math
    baseFrequency = totalPulses * (1000.0 / sampleTimeMs);

    // Flush the moving average buffer so old data doesn't drag the new baseline down
    for(int i = 0; i < FILTER_SIZE; i++) {
        freqBuffer[i] = baseFrequency;
    }
    freqSum = baseFrequency * FILTER_SIZE;
    bufferIndex = 0;

    frequencyShift = 0.0;
    lastUpdateTime = millis();
}

float MetalDetector::getShift() {
    return frequencyShift;
}

float MetalDetector::getBaseFrequency() {
    return baseFrequency;
}

float MetalDetector::getCurrentFrequency() {
    return currentFrequency;
}

bool MetalDetector::isMetalDetected() {
    // Returns true if the shift exceeds your threshold (checks both positive and negative shifts)
    return abs(frequencyShift) >= threshold;
}
