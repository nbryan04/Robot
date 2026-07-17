#include "ir_sensor.h"
#include <math.h>

IR_Sensor::IR_Sensor(int adcPin, int selectPin)
    : _adcPin(adcPin), _selectPin(selectPin) {}

void IR_Sensor::begin() {
    pinMode(_selectPin, INPUT);  // add a pull if the beacon board doesn't drive it hard

    // Map the GPIO to its ADC1 channel. Pin must be an ADC1 input (GPIO1-10 on the S3).
    adc_unit_t unit;
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(_adcPin, &unit, &_channel));

    // Background DMA ring buffer: 8 frames of 64 samples each (4 bytes/sample).
    adc_continuous_handle_cfg_t handleCfg = {};
    handleCfg.max_store_buf_size = 2048;
    handleCfg.conv_frame_size = 256;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handleCfg, &_handle));

    adc_digi_pattern_config_t pattern = {};
    pattern.atten = ADC_ATTEN_DB_12;      // full 0-3.3V range
    pattern.channel = _channel;
    pattern.unit = ADC_UNIT_1;
    pattern.bit_width = ADC_BITWIDTH_12;  // raw 0-4095, midpoint 2048

    adc_continuous_config_t cfg = {};
    cfg.pattern_num = 1;
    cfg.adc_pattern = &pattern;
    cfg.sample_freq_hz = SAMPLE_RATE;
    cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    ESP_ERROR_CHECK(adc_continuous_config(_handle, &cfg));

    // Precompute Goertzel coefficients: k = round(N * f / Fs), coeff = 2*cos(2*pi*k/N).
    int kLow  = lroundf((float)WINDOW_N * FREQ_LOW  / SAMPLE_RATE);
    int kHigh = lroundf((float)WINDOW_N * FREQ_HIGH / SAMPLE_RATE);
    _coeffLow  = 2.0f * cosf(2.0f * PI * kLow  / WINDOW_N);
    _coeffHigh = 2.0f * cosf(2.0f * PI * kHigh / WINDOW_N);
}

void IR_Sensor::startSearch() {
    if (digitalRead(_selectPin) == HIGH) {  // HIGH = 1kHz
        _coeff = _coeffLow;
        _threshold = THRESHOLD_1K;
        _targetFreq = FREQ_LOW;
    } else {                                 // LOW = 10kHz
        _coeff = _coeffHigh;
        _threshold = THRESHOLD_10K;
        _targetFreq = FREQ_HIGH;
    }

    resetWindow();
    _hits = 0;
    _lastMag = 0.0f;

    if (_state == State::IDLE) {
        ESP_ERROR_CHECK(adc_continuous_start(_handle));
    }
    _state = State::SEARCHING;
}

void IR_Sensor::stop() {
    if (_state != State::IDLE) {
        ESP_ERROR_CHECK(adc_continuous_stop(_handle));
        _state = State::IDLE;
    }
}

void IR_Sensor::update() {
    if (_state == State::IDLE) return;

    uint8_t buf[256];
    uint32_t outLen = 0;

    // Drain everything currently in the DMA buffer (timeout 0 = never blocks).
    while (adc_continuous_read(_handle, buf, sizeof(buf), &outLen, 0) == ESP_OK) {
        // Once detected we keep draining to avoid buffer overflow, but skip the math.
        if (_state != State::SEARCHING) continue;

        int count = outLen / SOC_ADC_DIGI_RESULT_BYTES;
        for (int i = 0; i < count; i++) {
            adc_digi_output_data_t *p =
                (adc_digi_output_data_t *)&buf[i * SOC_ADC_DIGI_RESULT_BYTES];
            if (p->type2.channel != _channel) continue;  // ignore stray channels

            // Center on the ~1.65V DC bias and normalize to roughly [-1, 1].
            float x = ((float)p->type2.data - 2048.0f) / 2048.0f;
            processSample(x);
        }
    }
}

void IR_Sensor::processSample(float x) {
    float s = x + _coeff * _s1 - _s2;
    _s2 = _s1;
    _s1 = s;

    if (++_n >= WINDOW_N) {
        evaluateWindow();
    }
}

void IR_Sensor::evaluateWindow() {
    // Goertzel magnitude, converted to an AC amplitude estimate in ~[0, 1].
    float magSq = _s1 * _s1 + _s2 * _s2 - _coeff * _s1 * _s2;
    _lastMag = 2.0f * sqrtf(fmaxf(magSq, 0.0f)) / WINDOW_N;

    if (_lastMag >= _threshold) {
        // Require several windows in a row so a burst of noise can't misfire.
        if (++_hits >= CONFIRM_COUNT) {
            _state = State::DETECTED;
        }
    } else {
        _hits = 0;  // streak broken
    }

    resetWindow();
}
