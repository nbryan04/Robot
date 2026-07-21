#include "metal_detector.h"

MetalDetector::MetalDetector(int pin, unsigned long interval, float thresholdHz)
    : sensorPin(pin), updateInterval(interval), threshold(thresholdHz) {
    // Initialize the filter buffer array with zeros
    for(int i = 0; i < FILTER_SIZE; i++) {
        freqBuffer[i] = 0.0;
    }
}

// ISR that fires silently in the background whenever the counter hits 30,000
bool IRAM_ATTR MetalDetector::pcnt_overflow_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    std::atomic<int>* overflows = static_cast<std::atomic<int>*>(user_ctx);
    overflows->fetch_add(1, std::memory_order_relaxed);
    return false; 
}

void MetalDetector::begin() {
    Serial.println("Initializing Hardware Pulse Counter (PCNT)...");

    // 1. Configure the PCNT Unit limits
    pcnt_unit_config_t unit_config = {};
    unit_config.high_limit = 30000;  // Trigger overflow event here
    unit_config.low_limit = -1;
    pcnt_new_unit(&unit_config, &pcnt_unit);

    // 2. Assign your sensor pin to this hardware unit
    pcnt_chan_config_t chan_config = {}; // FIXED: Abbreviated to chan_config_t
    chan_config.edge_gpio_num = sensorPin;
    chan_config.level_gpio_num = -1; // -1 means unused
    pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);

    // 3. Count UP on rising edges, ignore falling edges
    pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);

    // 4. Attach the overflow callback
    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_overflow_callback };
    pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, &overflow_count);
    pcnt_unit_add_watch_point(pcnt_unit, 30000);

    // 5. Power up the hardware counter
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit); // FIXED
    pcnt_unit_start(pcnt_unit);

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
        int hardwareCount = 0;
        pcnt_unit_get_count(pcnt_unit, &hardwareCount);
        pcnt_unit_clear_count(pcnt_unit); // FIXED
        
        // 2. Fetch and reset overflows
        int overflows = overflow_count.exchange(0, std::memory_order_relaxed);
        unsigned long totalPulses = (overflows * 30000) + hardwareCount;
        
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
    pcnt_unit_clear_count(pcnt_unit); // FIXED
    overflow_count.store(0, std::memory_order_relaxed);
    
    // 2. Wait for the sample duration
    delay(sampleTimeMs); 
    
    // 3. Read the hardware
    int hardwareCount = 0;
    pcnt_unit_get_count(pcnt_unit, &hardwareCount);
    int overflows = overflow_count.exchange(0, std::memory_order_relaxed);
    
    unsigned long totalPulses = (overflows * 30000) + hardwareCount;
    
    // 4. Resume normal counting
    pcnt_unit_clear_count(pcnt_unit); // FIXED
    
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