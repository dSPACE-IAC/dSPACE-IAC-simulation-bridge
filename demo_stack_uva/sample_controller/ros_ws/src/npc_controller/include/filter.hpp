#ifndef BUTTERWORTH_LOW_PASS_FILTER_H
#define BUTTERWORTH_LOW_PASS_FILTER_H

#include <cmath> // for M_PI and std::tan

class ButterworthLowPassFilter {
public:
    // Constructor
    ButterworthLowPassFilter(float sampleRate, float cutoffFrequency)
        : m_sampleRate(sampleRate), m_cutoffFrequency(cutoffFrequency) {
        calculateCoefficients();
    }

    // Process a single sample and return the filtered result
    float processSample(float input) {
        float output = (m_b0 * input + m_b1 * m_inputHistory - m_a1 * m_outputHistory);
        m_inputHistory = input; // Update history
        m_outputHistory = output;
        return output;
    }

    // Reset filter state
    void reset() {
        m_inputHistory = 0.0f;
        m_outputHistory = 0.0f;
    }

    // Set a new cutoff frequency and recalculate coefficients
    void setCutoffFrequency(float cutoffFrequency) {
        m_cutoffFrequency = cutoffFrequency;
        calculateCoefficients();
    }

private:
    float m_sampleRate;
    float m_cutoffFrequency;
    float m_inputHistory = 0.0f;
    float m_outputHistory = 0.0f;
    float m_b0, m_b1, m_a1; // Filter coefficients

    // Calculate the filter coefficients based on current parameters
    void calculateCoefficients() {
        float theta = 2 * M_PI * m_cutoffFrequency / m_sampleRate;
        float d = 1 / std::tan(theta / 2.0f);
        float beta = (1 - d) / (1 + d);
        m_b0 = 1 / (1 + d);
        m_b1 = m_b0;
        m_a1 = -beta;
    }
};

#endif // BUTTERWORTH_LOW_PASS_FILTER_H
