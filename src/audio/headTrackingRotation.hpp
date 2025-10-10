#pragma once

#include "al/io/al_MIDI.hpp"
#include "al/protocol/al_OSC.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>

/**
 * @brief Head Tracking and Ambisonic Scene Rotation Module
 * 
 * Receives OSC messages containing head orientation (yaw, pitch, roll) and applies
 * rotation to ambisonic B-format signals using the Ivanic-Ruedenberg recursive method.
 * Also provides orientation data for visual head model rendering.
 * 
 * Based on:
 * - Ivanic, J., Ruedenberg, K. (1996). Rotation Matrices for Real Spherical Harmonics.
 */
class HeadTrackingRotation : public al::osc::PacketHandler {
public:
    /**
     * @brief Construct a new Head Tracking Rotation object
     */
    HeadTrackingRotation(int maxOrder = 7, int oscPort = 9000) 
        : maxAmbisonicOrder(maxOrder), oscPortNumber(oscPort) {
        
        // Initialize rotation matrices for each order (0 to maxOrder)
        for (int l = 0; l <= maxAmbisonicOrder; ++l) {
            int nCh = (2 * l + 1);
            orderMatrices.push_back(std::vector<float>(nCh * nCh, 0.0f));
            orderMatricesCopy.push_back(std::vector<float>(nCh * nCh, 0.0f));
            
            // Initialize to identity matrix
            for (int i = 0; i < nCh; ++i) {
                orderMatrices[l][i * nCh + i] = 1.0f;
                orderMatricesCopy[l][i * nCh + i] = 1.0f;
            }
        }
        
        std::cout << "HeadTrackingRotation initialized for max order " << maxOrder << std::endl;
    }
    
    /**
     * @brief Initialize OSC receiver
     */
    bool initOSC() {
        if (oscRecv.open(oscPortNumber, "", 0.05)) {
            oscRecv.handler(*this);
            oscRecv.start();
            oscConnected = true;
            std::cout << "OSC receiver started on port " << oscPortNumber << std::endl;
            return true;
        }
        std::cout << "Failed to open OSC port " << oscPortNumber << std::endl;
        oscConnected = false;
        return false;
    }
    
    /**
     * @brief Stop OSC receiver
     */
    void stopOSC() {
        oscRecv.stop();
        oscConnected = false;
        std::cout << "OSC receiver stopped" << std::endl;
    }
    
    /**
     * @brief OSC message handler
     */
    void onMessage(al::osc::Message& m) {
        if (!enableHeadTracking) return;

        oscReceivedMessages = true; 
        
        // Handle /ypr message (primary format)
        if (m.addressPattern() == "/ypr" && m.typeTags() == "fff") {
            float y, p, r;
            m >> y >> p >> r;
            
            if (flipYaw) y = -y;
            if (flipPitch) p = -p;
            if (flipRoll) r = -r;
            
            setOrientation(y, p, r);
            
        }
        // Handle individual messages
        else if (m.addressPattern() == "/yaw" && m.typeTags() == "f") {
            float y;
            m >> y;
            if (flipYaw) y = -y;
            yaw = y;
            rotationMatrixNeedsUpdate = true;
        }
        else if (m.addressPattern() == "/pitch" && m.typeTags() == "f") {
            float p;
            m >> p;
            if (flipPitch) p = -p;
            pitch = p;
            rotationMatrixNeedsUpdate = true;
        }
        else if (m.addressPattern() == "/roll" && m.typeTags() == "f") {
            float r;
            m >> r;
            if (flipRoll) r = -r;
            roll = r;
            rotationMatrixNeedsUpdate = true;
        }
    }
    
    /**
     * @brief Set head orientation
     */
    void setOrientation(float yawDeg, float pitchDeg, float rollDeg) {
        yaw = yawDeg;
        pitch = pitchDeg;
        roll = rollDeg;
        rotationMatrixNeedsUpdate = true;
    }
    
    /**
     * @brief Get current orientation
     */
    void getOrientation(float& yawDeg, float& pitchDeg, float& rollDeg) const {
        yawDeg = yaw;
        pitchDeg = pitch;
        rollDeg = roll;
    }
    
    /**
     * @brief Apply rotation to ambisonic B-format signals
     * 
     * @param ambiSignals Array of pointers to ambisonic channel buffers [nSH][numSamples]
     * @param numSamples Number of samples in each buffer
     * @param order Ambisonic order to process
     */
    void rotateAmbisonics(float** ambiSignals, int numSamples, int order) {
        if (!enableHeadTracking || order > maxAmbisonicOrder) return;
        
        // Update rotation matrices if parameters changed
        if (rotationMatrixNeedsUpdate) {
            computeRotationMatrices(order);
            rotationMatrixNeedsUpdate = false;
        }
        
        int nSH = (order + 1) * (order + 1);
        
        // Create temporary buffer to store rotated signals
        std::vector<std::vector<float>> tempSignals(nSH, std::vector<float>(numSamples, 0.0f));
        
        // Apply rotation order by order (skip order 0 - W channel unchanged)
        for (int l = 1; l <= order; ++l) {
            int offset = l * l;
            int nCh = 2 * l + 1;
            const auto& R = orderMatrices[l];
            
            // Multiply: tempSignals = R * ambiSignals
            for (int o = 0; o < nCh; ++o) {
                int chOut = offset + o;
                for (int p = 0; p < nCh; ++p) {
                    int chIn = offset + p;
                    float gain = R[o * nCh + p];
                    
                    for (int n = 0; n < numSamples; ++n) {
                        tempSignals[chOut][n] += gain * ambiSignals[chIn][n];
                    }
                }
            }
        }
        
        // Copy rotated signals back (skip W channel at index 0)
        for (int ch = 1; ch < nSH; ++ch) {
            for (int n = 0; n < numSamples; ++n) {
                ambiSignals[ch][n] = tempSignals[ch][n];
            }
        }
    }
    
    // Getters and setters
    bool isOSCConnected() const { return oscConnected; }
    bool hasReceivedMessages() const { return oscReceivedMessages; }
    bool isEnabled() const { return enableHeadTracking; }
    void setEnabled(bool enabled) { enableHeadTracking = enabled; }
    
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }
    
    void setFlipYaw(bool flip) { flipYaw = flip; }
    void setFlipPitch(bool flip) { flipPitch = flip; }
    void setFlipRoll(bool flip) { flipRoll = flip; }
    
    bool getFlipYaw() const { return flipYaw; }
    bool getFlipPitch() const { return flipPitch; }
    bool getFlipRoll() const { return flipRoll; }
    
    void setOSCPort(int port) { 
        if (port != oscPortNumber) {
            stopOSC();
            oscPortNumber = port;
            initOSC();
        }
    }
    
    int getOSCPort() const { return oscPortNumber; }

private:
    // OSC
    al::osc::Recv oscRecv;
    int oscPortNumber;
    bool oscConnected = false;
    bool oscReceivedMessages = false; 
    
    // Head tracking state
    bool enableHeadTracking = false;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    
    // Flip flags
    bool flipYaw = false;
    bool flipPitch = false;
    bool flipRoll = false;
    
    // Rotation matrices
    int maxAmbisonicOrder;
    std::vector<std::vector<float>> orderMatrices;      // Current rotation matrices
    std::vector<std::vector<float>> orderMatricesCopy;  // Previous (for crossfading)
    bool rotationMatrixNeedsUpdate = true;
    
    /**
     * @brief Compute rotation matrices using Ivanic-Ruedenberg method
     * 
     * Based on the recursive algorithm from:
     * Ivanic, J., Ruedenberg, K. (1996). Rotation Matrices for Real Spherical Harmonics.
     */
    void computeRotationMatrices(int order) {
        // Convert degrees to radians
        float yawRad = yaw * M_PI / 180.0f;
        float pitchRad = pitch * M_PI / 180.0f;
        float rollRad = roll * M_PI / 180.0f;
        
        // Compute trigonometric values
        float ca = std::cos(yawRad);
        float sa = std::sin(yawRad);
        float cb = std::cos(pitchRad);
        float sb = std::sin(pitchRad);
        float cy = std::cos(rollRad);
        float sy = std::sin(rollRad);
        
        // Construct 3x3 rotation matrix (Yaw -> Pitch -> Roll extrinsic rotations)
        float R3x3[3][3];
        
        R3x3[0][0] = ca * cb;
        R3x3[0][1] = -sa * cb;
        R3x3[0][2] = sb;
        
        R3x3[1][0] = sa * cy + ca * sb * sy;
        R3x3[1][1] = ca * cy - sa * sb * sy;
        R3x3[1][2] = -cb * sy;
        
        R3x3[2][0] = sa * sy - ca * sb * cy;
        R3x3[2][1] = ca * sy + sa * sb * cy;
        R3x3[2][2] = cb * cy;
        
        // Order 1: Map 3x3 rotation to first-order ambisonics (Y, Z, X ordering)
        auto& R1 = orderMatrices[1];
        R1[0 * 3 + 0] = R3x3[1][1];  // Y -> Y
        R1[0 * 3 + 1] = R3x3[1][2];  // Y -> Z
        R1[0 * 3 + 2] = R3x3[1][0];  // Y -> X
        
        R1[1 * 3 + 0] = R3x3[2][1];  // Z -> Y
        R1[1 * 3 + 1] = R3x3[2][2];  // Z -> Z
        R1[1 * 3 + 2] = R3x3[2][0];  // Z -> X
        
        R1[2 * 3 + 0] = R3x3[0][1];  // X -> Y
        R1[2 * 3 + 1] = R3x3[0][2];  // X -> Z
        R1[2 * 3 + 2] = R3x3[0][0];  // X -> X
        
        // Recursively compute higher-order matrices
        for (int l = 2; l <= order; ++l) {
            computeOrderMatrix(l);
        }
    }
    
    /**
     * @brief Compute rotation matrix for a specific order using recursion
     */
    void computeOrderMatrix(int l) {
        const auto& R1 = orderMatrices[1];
        const auto& Rlm1 = orderMatrices[l - 1];
        auto& Rl = orderMatrices[l];
        
        int nCh = 2 * l + 1;
        int nChPrev = 2 * (l - 1) + 1;
        
        for (int m = -l; m <= l; ++m) {
            for (int n = -l; n <= l; ++n) {
                float u = 0.0f, v = 0.0f, w = 0.0f;
                
                // Compute coefficients
                int d = (m == 0) ? 1 : 0;
                float denom;
                
                if (std::abs(n) == l) {
                    denom = (2 * l) * (2 * l - 1);
                } else {
                    denom = l * l - n * n;
                }
                
                float u_coeff = std::sqrt((l * l - m * m) / denom);
                float v_coeff = std::sqrt((1 + d) * (l + std::abs(m) - 1) * (l + std::abs(m)) / denom) 
                               * (1 - 2 * d) * 0.5f;
                float w_coeff = std::sqrt((l - std::abs(m) - 1) * (l - std::abs(m)) / denom) 
                               * (1 - d) * (-0.5f);
                
                // Compute U, V, W terms
                if (std::abs(u_coeff) > 1e-6f) {
                    u = u_coeff * P(0, l, m, n, R1, Rlm1, nChPrev);
                }
                
                if (std::abs(v_coeff) > 1e-6f) {
                    v = v_coeff * V(l, m, n, R1, Rlm1, nChPrev);
                }
                
                if (std::abs(w_coeff) > 1e-6f) {
                    w = w_coeff * W(l, m, n, R1, Rlm1, nChPrev);
                }
                
                Rl[(m + l) * nCh + (n + l)] = u + v + w;
            }
        }
    }
    
    // Helper functions for Ivanic-Ruedenberg recursion
    float P(int i, int l, int a, int b, const std::vector<float>& R1, 
            const std::vector<float>& Rlm1, int nChPrev) {
        
        float ri1 = R1[(i + 1) * 3 + 2];  // R1(i+1, 2)
        float rim1 = R1[(i + 1) * 3 + 0]; // R1(i+1, 0)
        float ri0 = R1[(i + 1) * 3 + 1];  // R1(i+1, 1)
        
        if (b == -l) {
            return ri1 * Rlm1[(a + l - 1) * nChPrev + 0] + 
                   rim1 * Rlm1[(a + l - 1) * nChPrev + (2 * l - 2)];
        } else if (b == l) {
            return ri1 * Rlm1[(a + l - 1) * nChPrev + (2 * l - 2)] - 
                   rim1 * Rlm1[(a + l - 1) * nChPrev + 0];
        } else {
            return ri0 * Rlm1[(a + l - 1) * nChPrev + (b + l - 1)];
        }
    }
    
    float U(int l, int m, int n, const std::vector<float>& R1, 
            const std::vector<float>& Rlm1, int nChPrev) {
        return P(0, l, m, n, R1, Rlm1, nChPrev);
    }
    
    float V(int l, int m, int n, const std::vector<float>& R1, 
            const std::vector<float>& Rlm1, int nChPrev) {
        
        if (m == 0) {
            float p0 = P(1, l, 1, n, R1, Rlm1, nChPrev);
            float p1 = P(-1, l, -1, n, R1, Rlm1, nChPrev);
            return p0 + p1;
        } else if (m > 0) {
            float p0 = P(1, l, m - 1, n, R1, Rlm1, nChPrev);
            if (m == 1) {
                return p0 * std::sqrt(2.0f);
            } else {
                return p0 - P(-1, l, 1 - m, n, R1, Rlm1, nChPrev);
            }
        } else {
            float p1 = P(-1, l, -m - 1, n, R1, Rlm1, nChPrev);
            if (m == -1) {
                return p1 * std::sqrt(2.0f);
            } else {
                return p1 + P(1, l, m + 1, n, R1, Rlm1, nChPrev);
            }
        }
    }
    
    float W(int l, int m, int n, const std::vector<float>& R1, 
            const std::vector<float>& Rlm1, int nChPrev) {
        
        if (m > 0) {
            float p0 = P(1, l, m + 1, n, R1, Rlm1, nChPrev);
            float p1 = P(-1, l, -m - 1, n, R1, Rlm1, nChPrev);
            return p0 + p1;
        } else if (m < 0) {
            float p0 = P(1, l, m - 1, n, R1, Rlm1, nChPrev);
            float p1 = P(-1, l, 1 - m, n, R1, Rlm1, nChPrev);
            return p0 - p1;
        }
        
        return 0.0f;
    }
};