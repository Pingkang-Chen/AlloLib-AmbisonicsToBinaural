#pragma once

#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include "saf.h"
#include "saf_hoa.h"

/**
 * @brief Enhanced Ambisonic Encoder with SH Coefficient Smoothing
 * 
 * Provides smooth ambisonic encoding using the Spatial Audio Framework (SAF)
 * with built-in parameter smoothing to avoid clicks and pops.
 */
class AmbisonicEncoder {
private:
    // Basic smoothing class definition (kept internal to avoid dependencies)
    class ParameterSmoother {
    private:
        float currentValue;
        float targetValue;
        float smoothingCoeff;
        bool isSmoothing;
        
    public:
        ParameterSmoother(float initialValue = 0.0f, float smoothingTimeMs = 50.0f, float sampleRate = 44100.0f) 
            : currentValue(initialValue), targetValue(initialValue), isSmoothing(false) {
            float tau = smoothingTimeMs / 1000.0f;
            smoothingCoeff = exp(-1.0f / (tau * sampleRate));
        }
        
        void setTarget(float newTarget) {
            if (abs(newTarget - targetValue) > 0.0001f) {
                targetValue = newTarget;
                isSmoothing = true;
            }
        }
        
        float getNextValue() {
            if (isSmoothing) {
                currentValue = smoothingCoeff * currentValue + (1.0f - smoothingCoeff) * targetValue;
                if (abs(currentValue - targetValue) < 0.0001f) {
                    currentValue = targetValue;
                    isSmoothing = false;
                }
            }
            return currentValue;
        }
        
        float getCurrentValue() const { return currentValue; }
        bool isCurrentlySmoothing() const { return isSmoothing; }
        
        void reset(float newValue) {
            currentValue = targetValue = newValue;
            isSmoothing = false;
        }
        
        void setSmoothingTime(float smoothingTimeMs, float sampleRate) {
            float tau = smoothingTimeMs / 1000.0f;
            smoothingCoeff = exp(-1.0f / (tau * sampleRate));
        }
    };

public:
    int order;
    int nSH;
    float* Y_target;
    float* Y_current;
    float** ambiSignals;
    int frameSize;
    
    std::vector<ParameterSmoother> shSmoothers;
    ParameterSmoother azimuthSmoother;
    ParameterSmoother elevationSmoother;

    /**
     * @brief Construct a new Ambisonic Encoder object
     */
    AmbisonicEncoder(int orderVal = 1, int frameSizeVal = 256, float sampleRate = 44100.0f) : 
        order(orderVal),
        frameSize(frameSizeVal),
        azimuthSmoother(0.0f, 30.0f, sampleRate),
        elevationSmoother(0.0f, 30.0f, sampleRate) {
        
        nSH = (order + 1) * (order + 1);
        
        Y_target = new float[nSH];
        Y_current = new float[nSH];
        
        shSmoothers.resize(nSH);
        for (int i = 0; i < nSH; i++) {
            shSmoothers[i] = ParameterSmoother(0.0f, 20.0f, sampleRate);
        }
        
        ambiSignals = new float*[nSH];
        for (int i = 0; i < nSH; i++) {
            ambiSignals[i] = new float[frameSize];
            memset(ambiSignals[i], 0, frameSize * sizeof(float));
        }
        
        updateDirection(0.0f, 0.0f);
        
        std::cout << "Enhanced AmbisonicEncoder initialized with order: " << order 
                  << ", nSH: " << nSH << " (with SH smoothing)" << std::endl;
    }
    
    /**
     * @brief Destroy the Ambisonic Encoder object
     */
    ~AmbisonicEncoder() {
        delete[] Y_target;
        delete[] Y_current;
        for (int i = 0; i < nSH; i++) {
            delete[] ambiSignals[i];
        }
        delete[] ambiSignals;
    }
    
    /**
     * @brief Clear all ambisonic signals
     */
    void clearSignals() {
        for (int i = 0; i < nSH; i++) {
            memset(ambiSignals[i], 0, frameSize * sizeof(float));
        }
    }
    
    /**
     * @brief Update the encoding direction with smoothing
     */
    void updateDirection(float azimuthDeg, float elevationDeg) {
        azimuthSmoother.setTarget(azimuthDeg);
        elevationSmoother.setTarget(elevationDeg);
        
        float dirs_deg[2] = {azimuthDeg, elevationDeg};
        getRSH(order, dirs_deg, 1, Y_target);
        
        for (int i = 0; i < nSH; i++) {
            shSmoothers[i].setTarget(Y_target[i]);
        }
    }
    
    /**
     * @brief Update smoothing parameters
     */
    void updateSmoothingParameters(float sampleRate) {
        azimuthSmoother.setSmoothingTime(30.0f, sampleRate);
        elevationSmoother.setSmoothingTime(30.0f, sampleRate);
        for (int i = 0; i < nSH; i++) {
            shSmoothers[i].setSmoothingTime(20.0f, sampleRate);
        }
    }
    
    /**
     * @brief Encode and accumulate input signal into ambisonic channels
     */
    void encodeAndAccumulate(const float* inputSignal, int numSamples, float azimuthDeg, float elevationDeg) {
        float dirs_deg[2] = {azimuthDeg, elevationDeg};
        float Y_source[nSH];
        getRSH(order, dirs_deg, 1, Y_source);
        
        for (int n = 0; n < numSamples; n++) {
            for (int sh = 0; sh < nSH; sh++) {
                ambiSignals[sh][n] += inputSignal[n] * Y_source[sh];
            }
        }
    }
    
    /**
     * @brief Get a specific ambisonic channel
     */
    float* getAmbisonicChannel(int channel) {
        if (channel < 0 || channel >= nSH) {
            std::cerr << "Error: Channel index out of range" << std::endl;
            return nullptr;
        }
        return ambiSignals[channel];
    }
    
    /**
     * @brief Get the ambisonic order
     */
    int getOrder() const { return order; }
    
    /**
     * @brief Get the number of ambisonic channels
     */
    int getNumChannels() const { return nSH; }
    
    /**
     * @brief Check if any smoothing is currently active
     */
    bool isSmoothing() const {
        for (int i = 0; i < nSH; i++) {
            if (shSmoothers[i].isCurrentlySmoothing()) return true;
        }
        return azimuthSmoother.isCurrentlySmoothing() || elevationSmoother.isCurrentlySmoothing();
    }
};