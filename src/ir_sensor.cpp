#include "ir_sensor.h"
#include <math.h>

IR_Sensor::IR_Sensor(int adcPin, int selectPin, int lfLeftPin, int lfMidPin, int lfRightPin)
    : _adcPin(adcPin), _selectPin(selectPin), _lfPin{lfLeftPin, lfMidPin, lfRightPin} {}

void IR_Sensor::begin() {
    pinMode(_selectPin, INPUT);  // add a pull if the beacon board doesn't drive it hard

    // Map every GPIO to its ADC1 channel. All must be ADC1 inputs (GPIO1-10 on S3).
    adc_unit_t unit;
    ESP_ERROR_CHECK(adc_continuous_io_to_channel(_adcPin, &unit, &_irChannel));
    for (int i = 0; i < 3; i++) {
        adc_unit_t u;
        ESP_ERROR_CHECK(adc_continuous_io_to_channel(_lfPin[i], &u, &_lfChannel[i]));
    }

    // Background DMA ring buffer (~12.8ms at the total sample rate).
    adc_continuous_handle_cfg_t handleCfg = {};
    handleCfg.max_store_buf_size = 4096;
    handleCfg.conv_frame_size = 256;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handleCfg, &_handle));

    // Scan pattern: IR on every other slot (so its samples stay uniform at half
    // the total rate = IR_SAMPLE_RATE), with the three LF channels sharing the rest.
    adc_channel_t order[PATTERN_LEN] = {
        _irChannel, _lfChannel[0], _irChannel, _lfChannel[1], _irChannel, _lfChannel[2]
    };
    adc_digi_pattern_config_t pattern[PATTERN_LEN] = {};
    for (int i = 0; i < PATTERN_LEN; i++) {
        pattern[i].atten     = ADC_ATTEN_DB_12;   // full 0-3.3V range
        pattern[i].channel   = order[i];
        pattern[i].unit      = ADC_UNIT_1;
        pattern[i].bit_width = ADC_BITWIDTH_12;    // raw 0-4095, midpoint 2048
    }

    adc_continuous_config_t cfg = {};
    cfg.pattern_num = PATTERN_LEN;
    cfg.adc_pattern = pattern;
    cfg.sample_freq_hz = ADC_TOTAL_RATE;
    cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    ESP_ERROR_CHECK(adc_continuous_config(_handle, &cfg));

    // Precompute Goertzel coefficients: k = round(N * f / Fs), coeff = 2*cos(2*pi*k/N).
    // Fs is the IR's effective rate (half the total), so the coefficients are unchanged.
    int kLow  = lroundf((float)WINDOW_N * FREQ_LOW  / IR_SAMPLE_RATE);
    int kHigh = lroundf((float)WINDOW_N * FREQ_HIGH / IR_SAMPLE_RATE);
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
        int count = outLen / SOC_ADC_DIGI_RESULT_BYTES;
        for (int i = 0; i < count; i++) {
            adc_digi_output_data_t *p =
                (adc_digi_output_data_t *)&buf[i * SOC_ADC_DIGI_RESULT_BYTES];
            uint32_t ch  = p->type2.channel;
            uint32_t raw = p->type2.data;

            if (ch == (uint32_t)_irChannel) {
                // IR channel: stream through Goertzel (only while still searching).
                if (_state == State::SEARCHING) {
                    // Center on the ~1.65V DC bias and normalize to roughly [-1, 1].
                    float x = ((float)raw - 2048.0f) / 2048.0f;
                    processSample(x);
                }
            } else if (ch == (uint32_t)_lfChannel[0]) {
                _lfRaw[0] = (int)raw;   // line-follower left: latch latest raw value
            } else if (ch == (uint32_t)_lfChannel[1]) {
                _lfRaw[1] = (int)raw;   // line-follower middle
            } else if (ch == (uint32_t)_lfChannel[2]) {
                _lfRaw[2] = (int)raw;   // line-follower right
            }
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
