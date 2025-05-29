#include <iostream>
#include <cmath>
#include <chrono>
#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Spherical.hpp"
#include "al/scene/al_DynamicScene.hpp"
#include "al/ui/al_ControlGUI.hpp"
#include "al/ui/al_PickableManager.hpp"
#include "al/io/al_File.hpp"
#include "saf.h"
#include "saf_hrir.h"
#include "saf_hoa.h"
#include "saf_vbap.h"
#include "saf_utilities.h"

// JUCE includes - ONLY the modules we need, no GUI components
#include "juce_core/juce_core.h"
#include "juce_audio_basics/juce_audio_basics.h"
#include "juce_dsp/juce_dsp.h"

using namespace al;

// ===================
// Parameter Smoother Class for Click-Free Changes
// ===================
class ParameterSmoother {
private:
    float currentValue;
    float targetValue;
    float smoothingCoeff;
    bool isSmoothing;
    
public:
    ParameterSmoother(float initialValue = 0.0f, float smoothingTimeMs = 50.0f, float sampleRate = 44100.0f) 
        : currentValue(initialValue), targetValue(initialValue), isSmoothing(false) {
        // Calculate smoothing coefficient for exponential smoothing
        // Time constant: tau = smoothingTimeMs / 1000.0f
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
            
            // Stop smoothing when close enough to target
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

// ===================
// Helper: Convert Spherical to Cartesian - Traditional Globe Convention
// Azimuth: 0° = front (+Z), 90° = right (+X), 180° = back (-Z), 270° = left (-X)
// Elevation: 0° = horizon, +90° = up (+Y), -90° = down (-Y)
// ===================
Vec3f sphericalToCartesian(float azimuthDeg, float elevationDeg, float radius) {
  float azimuth_rad = azimuthDeg * M_PI / 180.0f;
  float elevation_rad = elevationDeg * M_PI / 180.0f;
  
  // Standard spherical coordinates with Y-up (like a globe)
  float x = radius * cos(elevation_rad) * sin(azimuth_rad);
  float y = radius * sin(elevation_rad);  // Y is up/down (elevation)
  float z = radius * cos(elevation_rad) * cos(azimuth_rad);  // Z is front/back
  
  return Vec3f(x, y, z);
}

// Convert Cartesian to Spherical coordinates
void cartesianToSpherical(const Vec3f& cartesian, float& azimuthDeg, float& elevationDeg, float& radius) {
  float x = cartesian.x;
  float y = cartesian.y;
  float z = cartesian.z;
  
  radius = sqrt(x*x + y*y + z*z);
  
  if (radius < 0.0001f) {
    azimuthDeg = 0.0f;
    elevationDeg = 0.0f;
    return;
  }
  
  elevationDeg = asin(y / radius) * 180.0f / M_PI;  // Y is elevation
  azimuthDeg = atan2(x, z) * 180.0f / M_PI;  // Standard atan2(x, z) for azimuth
}

// ===================
// Enhanced Ambisonic Encoder with SH Coefficient Smoothing
// ===================
class AmbisonicEncoder {
public:
    int order;
    int nSH;
    float* Y_target;        // Target SH coefficients
    float* Y_current;       // Current smoothed SH coefficients  
    float** ambiSignals;
    int frameSize;
    
    // SH coefficient smoothers
    std::vector<ParameterSmoother> shSmoothers;
    
    // Direction smoothers
    ParameterSmoother azimuthSmoother;
    ParameterSmoother elevationSmoother;

    AmbisonicEncoder(int orderVal = 1, int frameSizeVal = 256, float sampleRate = 44100.0f) : 
        order(orderVal),
        frameSize(frameSizeVal),
        azimuthSmoother(0.0f, 30.0f, sampleRate),  // 30ms smoothing for direction changes
        elevationSmoother(0.0f, 30.0f, sampleRate) {
        
        nSH = (order + 1) * (order + 1);
        
        Y_target = new float[nSH];
        Y_current = new float[nSH];
        
        // Initialize SH coefficient smoothers
        shSmoothers.resize(nSH);
        for (int i = 0; i < nSH; i++) {
            shSmoothers[i] = ParameterSmoother(0.0f, 20.0f, sampleRate);  // 20ms smoothing for SH coefficients
        }
        
        ambiSignals = new float*[nSH];
        for (int i = 0; i < nSH; i++) {
            ambiSignals[i] = new float[frameSize];
            memset(ambiSignals[i], 0, frameSize * sizeof(float));
        }
        
        // Initialize with default direction (0, 0)
        updateDirection(0.0f, 0.0f);
        
        std::cout << "Enhanced AmbisonicEncoder initialized with order: " << order 
                  << ", nSH: " << nSH << " (with SH smoothing)" << std::endl;
    }
    
    ~AmbisonicEncoder() {
        delete[] Y_target;
        delete[] Y_current;
        for (int i = 0; i < nSH; i++) {
            delete[] ambiSignals[i];
        }
        delete[] ambiSignals;
    }
    
    void updateDirection(float azimuthDeg, float elevationDeg) {
        // Set target directions for smoothing
        azimuthSmoother.setTarget(azimuthDeg);
        elevationSmoother.setTarget(elevationDeg);
        
        // Calculate target SH coefficients
        float dirs_deg[2] = {azimuthDeg, elevationDeg};
        getRSH(order, dirs_deg, 1, Y_target);
        
        // Set targets for SH coefficient smoothers
        for (int i = 0; i < nSH; i++) {
            shSmoothers[i].setTarget(Y_target[i]);
        }
        
        std::cout << "Target SH coefficients for direction: (" 
                  << azimuthDeg << ", " << elevationDeg << ")" << std::endl;
    }
    
    void updateSmoothingParameters(float sampleRate) {
        azimuthSmoother.setSmoothingTime(30.0f, sampleRate);
        elevationSmoother.setSmoothingTime(30.0f, sampleRate);
        for (int i = 0; i < nSH; i++) {
            shSmoothers[i].setSmoothingTime(20.0f, sampleRate);
        }
    }
    
    void encode(const float* inputSignal, int numSamples) {
        // Clear previous content
        for (int i = 0; i < nSH; i++) {
            memset(ambiSignals[i], 0, frameSize * sizeof(float));
        }
        
        // Encode sample by sample to allow for smooth SH coefficient changes
        for (int n = 0; n < numSamples; n++) {
            // Update smoothed SH coefficients for this sample
            for (int sh = 0; sh < nSH; sh++) {
                Y_current[sh] = shSmoothers[sh].getNextValue();
            }
            
            // Apply current smoothed SH coefficients
            for (int sh = 0; sh < nSH; sh++) {
                ambiSignals[sh][n] = inputSignal[n] * Y_current[sh];
            }
        }
    }
    
    float* getAmbisonicChannel(int channel) {
        if (channel < 0 || channel >= nSH) {
            std::cerr << "Error: Channel index out of range" << std::endl;
            return nullptr;
        }
        return ambiSignals[channel];
    }
    
    int getOrder() const { return order; }
    int getNumChannels() const { return nSH; }
    
    // Check if any smoothing is currently active
    bool isSmoothing() const {
        for (int i = 0; i < nSH; i++) {
            if (shSmoothers[i].isCurrentlySmoothing()) return true;
        }
        return azimuthSmoother.isCurrentlySmoothing() || elevationSmoother.isCurrentlySmoothing();
    }
};

// ===================
// Enhanced Ambisonic Binaural Decoder with Virtual Speaker Signal Smoothing
// ===================
class AmbisonicBinauralDecoder {
public:
    // SAF HRIR data
    const float* hrirs;
    const float* hrir_dirs_deg;
    int N_hrir_dirs;
    int hrir_len;
    int fs;
    
    // Ambisonic parameters
    int order;
    int nSH;
    
    // Virtual loudspeaker setup
    float* vls_dirs_deg;
    int nVirtualSpeakers;
    
    // Decoding matrices
    float* dec_mat;
    float* vls_gains;
    
    // JUCE Convolution processors
    std::vector<std::unique_ptr<juce::dsp::Convolution>> convolutionProcessorsL;
    std::vector<std::unique_ptr<juce::dsp::Convolution>> convolutionProcessorsR;
    
    // Audio buffers
    std::vector<juce::AudioBuffer<float>> virtualSpeakerBuffers;
    juce::AudioBuffer<float> tempBufferL;
    juce::AudioBuffer<float> tempBufferR;
    
    // Virtual speaker signal smoothers
    std::vector<std::vector<ParameterSmoother>> virtualSpeakerSmoothers;  // [virtualSpeaker][shChannel]
    
    double sampleRate;
    int maxBlockSize;
    
    // Pointer to current T-design selection
    int* currentTDesignPtr;
    
    AmbisonicBinauralDecoder(int orderVal = 1, int sampleRateVal = 44100, int maxBlockSizeVal = 256, int* tdesignPtr = nullptr) : 
        order(orderVal), sampleRate(sampleRateVal), maxBlockSize(maxBlockSizeVal), currentTDesignPtr(tdesignPtr) {
        
        nSH = (order + 1) * (order + 1);
        
        // Get SAF HRIR data
        hrirs = &__default_hrirs[0][0][0];
        hrir_dirs_deg = &__default_hrir_dirs_deg[0][0];
        N_hrir_dirs = __default_N_hrir_dirs;
        hrir_len = __default_hrir_len;
        fs = __default_hrir_fs;
        
        setupVirtualLoudspeakers();
        createDecodingMatrix();
        setupVirtualSpeakerSmoothers();
        setupJUCEConvolution();
        
        std::cout << "Enhanced AmbisonicBinauralDecoder with Virtual Speaker Smoothing:" << std::endl;
        std::cout << "  Order: " << order << ", nSH: " << nSH << std::endl;
        std::cout << "  Virtual speakers: " << nVirtualSpeakers << std::endl;
        std::cout << "  Using JUCE partitioned convolution + signal smoothing" << std::endl;
    }
    
    ~AmbisonicBinauralDecoder() {
        delete[] vls_dirs_deg;
        delete[] dec_mat;
        delete[] vls_gains;
    }
    
    void setupVirtualLoudspeakers() {
        // Get the count from the current T-design selection
        if (currentTDesignPtr) {
            const int tdesignCounts[] = {4, 12, 24, 36, 48};
            nVirtualSpeakers = tdesignCounts[*currentTDesignPtr];
        } else {
            nVirtualSpeakers = 4; // Default fallback
        }
        
        vls_dirs_deg = new float[nVirtualSpeakers * 2];
        
        // Copy coordinates from T-design data
        switch (nVirtualSpeakers) {
            case 4: // T-design (4)
                {
                    float speakers[4][2] = {
                        {45.000f, 35.264f}, {-45.000f, -35.264f},
                        {135.000f, -35.264f}, {-135.000f, 35.264f}
                    };
                    for (int i = 0; i < 4; i++) {
                        vls_dirs_deg[i*2] = speakers[i][0];
                        vls_dirs_deg[i*2+1] = speakers[i][1];
                    }
                }
                break;
                
            case 12: // T-design (12)
                {
                    float speakers[12][2] = {
                        {0.0f, -31.717f}, {-58.283f, 0.0f}, {-90.0f, 58.283f},
                        {0.0f, 31.717f}, {-121.717f, 0.0f}, {90.0f, -58.283f},
                        {180.0f, -31.717f}, {121.717f, 0.0f}, {90.0f, 58.283f},
                        {180.0f, 31.717f}, {58.283f, 0.0f}, {-90.0f, -58.283f}
                    };
                    for (int i = 0; i < 12; i++) {
                        vls_dirs_deg[i*2] = speakers[i][0];
                        vls_dirs_deg[i*2+1] = speakers[i][1];
                    }
                }
                break;
                
            case 24: // T-design (24)
                {
                    float speakers[24][2] = {
                        {26.001f, 15.464f}, {-26.001f, -15.464f}, {17.109f, -24.994f}, {-17.109f, 24.994f},
                        {153.999f, -15.464f}, {-153.999f, 15.464f}, {162.891f, 24.994f}, {-162.891f, -24.994f},
                        {72.891f, 24.994f}, {107.109f, -24.994f}, {116.001f, 15.464f}, {63.999f, -15.464f},
                        {-107.109f, 24.994f}, {-72.891f, -24.994f}, {-63.999f, 15.464f}, {-116.001f, -15.464f},
                        {32.254f, 60.025f}, {-147.746f, 60.025f}, {-57.746f, 60.025f}, {122.254f, 60.025f},
                        {-32.254f, -60.025f}, {147.746f, -60.025f}, {57.746f, -60.025f}, {-122.254f, -60.025f}
                    };
                    for (int i = 0; i < 24; i++) {
                        vls_dirs_deg[i*2] = speakers[i][0];
                        vls_dirs_deg[i*2+1] = speakers[i][1];
                    }
                }
                break;
                
            case 36: // T-design (36)
                {
                    float speakers[36][2] = {
                        {-31.106f, 53.651f}, {110.815f, 30.496f}, {148.894f, 53.651f}, {32.214f, -17.830f},
                        {69.185f, -30.496f}, {-32.214f, 17.830f}, {-69.185f, 30.496f}, {-147.786f, -17.830f},
                        {-110.815f, -30.496f}, {147.786f, 17.830f}, {31.106f, -53.651f}, {-148.894f, -53.651f},
                        {-21.246f, -47.775f}, {-108.204f, 38.782f}, {158.754f, -47.775f}, {139.774f, -14.095f},
                        {-71.796f, -38.782f}, {-139.774f, 14.095f}, {71.796f, 38.782f}, {-40.226f, -14.095f},
                        {108.204f, -38.782f}, {40.226f, 14.095f}, {21.246f, 47.775f}, {-158.754f, 47.775f},
                        {106.650f, -2.552f}, {-2.663f, -16.634f}, {-73.350f, -2.552f}, {-98.841f, 73.161f},
                        {-177.337f, 16.634f}, {98.841f, -73.161f}, {177.337f, -16.634f}, {81.159f, 73.161f},
                        {2.663f, 16.634f}, {-81.159f, -73.161f}, {-106.650f, 2.552f}, {73.350f, 2.552f}
                    };
                    for (int i = 0; i < 36; i++) {
                        vls_dirs_deg[i*2] = speakers[i][0];
                        vls_dirs_deg[i*2+1] = speakers[i][1];
                    }
                }
                break;
                
            case 48: // T-design (48)
                {
                    float speakers[48][2] = {
                        {20.746f, -3.552f}, {-20.746f, 3.552f}, {-3.798f, -20.704f}, {3.798f, 20.704f},
                        {159.254f, 3.552f}, {-159.254f, -3.552f}, {-176.202f, 20.704f}, {176.202f, -20.704f},
                        {93.798f, 20.704f}, {86.202f, -20.704f}, {110.746f, -3.552f}, {69.254f, 3.552f},
                        {-86.202f, 20.704f}, {-93.798f, -20.704f}, {-69.254f, -3.552f}, {-110.746f, 3.552f},
                        {-9.939f, 68.966f}, {170.061f, 68.966f}, {-99.939f, 68.966f}, {80.061f, 68.966f},
                        {9.939f, -68.966f}, {-170.061f, -68.966f}, {99.939f, -68.966f}, {-80.061f, -68.966f},
                        {42.147f, 17.568f}, {-42.147f, -17.568f}, {23.124f, -39.772f}, {-23.124f, 39.772f},
                        {137.853f, -17.568f}, {-137.853f, 17.568f}, {156.876f, 39.772f}, {-156.876f, -39.772f},
                        {66.876f, 39.772f}, {113.124f, -39.772f}, {132.147f, 17.568f}, {47.853f, -17.568f},
                        {-113.124f, 39.772f}, {-66.876f, -39.772f}, {-47.853f, 17.568f}, {-132.147f, -17.568f},
                        {25.259f, 44.979f}, {-154.741f, 44.979f}, {-64.741f, 44.979f}, {115.259f, 44.979f},
                        {-25.259f, -44.979f}, {154.741f, -44.979f}, {64.741f, -44.979f}, {-115.259f, -44.979f}
                    };
                    for (int i = 0; i < 48; i++) {
                        vls_dirs_deg[i*2] = speakers[i][0];
                        vls_dirs_deg[i*2+1] = speakers[i][1];
                    }
                }
                break;
                
            default:
                // Fallback to T-design (4)
                std::cout << "Warning: Unsupported T-design count " << nVirtualSpeakers << ", falling back to T-design (4)" << std::endl;
                nVirtualSpeakers = 4;
                float speakers[4][2] = {
                    {45.000f, 35.264f}, {-45.000f, -35.264f},
                    {135.000f, -35.264f}, {-135.000f, 35.264f}
                };
                for (int i = 0; i < 4; i++) {
                    vls_dirs_deg[i*2] = speakers[i][0];
                    vls_dirs_deg[i*2+1] = speakers[i][1];
                }
                break;
        }
        
        std::cout << "Setup virtual loudspeakers: " << nVirtualSpeakers << " speakers" << std::endl;
    }
    
    void createDecodingMatrix() {
        dec_mat = new float[nVirtualSpeakers * nSH];
        float* Y_ls = new float[nSH];
        
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            float ls_dir[2] = {vls_dirs_deg[ls*2], vls_dirs_deg[ls*2+1]};
            getRSH(order, ls_dir, 1, Y_ls);
            
            for (int n = 0; n < nSH; n++) {
                dec_mat[ls*nSH + n] = Y_ls[n];
            }
        }
        
        delete[] Y_ls;
        
        vls_gains = new float[nVirtualSpeakers];
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            vls_gains[ls] = 1.0f / sqrt((float)nVirtualSpeakers);
        }
    }
    
    void setupVirtualSpeakerSmoothers() {
        // Create smoothers for each virtual speaker's contribution from each SH channel
        virtualSpeakerSmoothers.resize(nVirtualSpeakers);
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            virtualSpeakerSmoothers[ls].resize(nSH);
            for (int sh = 0; sh < nSH; sh++) {
                // Initialize with current decoding matrix values
                float initialGain = dec_mat[ls*nSH + sh] * vls_gains[ls];
                virtualSpeakerSmoothers[ls][sh] = ParameterSmoother(initialGain, 15.0f, sampleRate);  // 15ms smoothing
            }
        }
    }
    
    void setupJUCEConvolution() {
        convolutionProcessorsL.resize(nVirtualSpeakers);
        convolutionProcessorsR.resize(nVirtualSpeakers);
        
        juce::dsp::ProcessSpec spec;
        spec.sampleRate = sampleRate;
        spec.maximumBlockSize = static_cast<juce::uint32>(maxBlockSize);
        spec.numChannels = 1;
        
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            convolutionProcessorsL[ls] = std::make_unique<juce::dsp::Convolution>();
            convolutionProcessorsR[ls] = std::make_unique<juce::dsp::Convolution>();
            
            convolutionProcessorsL[ls]->prepare(spec);
            convolutionProcessorsR[ls]->prepare(spec);
            
            int nearest_idx = findNearestHRIR(vls_dirs_deg[ls*2], vls_dirs_deg[ls*2+1]);
            
            const float* hrir_left = &hrirs[nearest_idx*2*hrir_len];
            const float* hrir_right = &hrirs[nearest_idx*2*hrir_len + hrir_len];
            
            juce::AudioBuffer<float> leftBuffer(1, hrir_len);
            juce::AudioBuffer<float> rightBuffer(1, hrir_len);
            
            for (int i = 0; i < hrir_len; i++) {
                leftBuffer.setSample(0, i, hrir_left[i]);
                rightBuffer.setSample(0, i, hrir_right[i]);
            }
            
            convolutionProcessorsL[ls]->loadImpulseResponse(
                std::move(leftBuffer),
                sampleRate,
                juce::dsp::Convolution::Stereo::no,
                juce::dsp::Convolution::Trim::no,
                juce::dsp::Convolution::Normalise::no
            );
            
            convolutionProcessorsR[ls]->loadImpulseResponse(
                std::move(rightBuffer),
                sampleRate,
                juce::dsp::Convolution::Stereo::no,
                juce::dsp::Convolution::Trim::no,
                juce::dsp::Convolution::Normalise::no
            );
        }
        
        virtualSpeakerBuffers.resize(nVirtualSpeakers);
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            virtualSpeakerBuffers[ls].setSize(1, maxBlockSize);
        }
        
        tempBufferL.setSize(1, maxBlockSize);
        tempBufferR.setSize(1, maxBlockSize);
        
        std::cout << "JUCE convolution setup complete for " << nVirtualSpeakers << " virtual speakers" << std::endl;
    }
    
    int findNearestHRIR(float azimuth_deg, float elevation_deg) {
        float lookup_azi = azimuth_deg;
        if (lookup_azi < 0.0f) lookup_azi += 360.0f;
        float lookup_elev = elevation_deg;
        
        float min_dist = 1000000.0f;
        int nearest_idx = 0;
        
        for (int i = 0; i < N_hrir_dirs; i++) {
            float dir_azi = hrir_dirs_deg[i*2];
            float dir_elev = hrir_dirs_deg[i*2 + 1];
            
            float azi_diff = fabs(lookup_azi - dir_azi);
            if (azi_diff > 180.0f) azi_diff = 360.0f - azi_diff;
            float elev_diff = fabs(lookup_elev - dir_elev);
            
            float dist = sqrt(azi_diff*azi_diff + elev_diff*elev_diff);
            
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
    }
    
    void updateSmoothingParameters(float newSampleRate) {
        sampleRate = newSampleRate;
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            for (int sh = 0; sh < nSH; sh++) {
                virtualSpeakerSmoothers[ls][sh].setSmoothingTime(15.0f, sampleRate);
            }
        }
    }
    
    void decode(float** ambiSignals, float* leftOut, float* rightOut, int numSamples) {
        if (numSamples > maxBlockSize) {
            std::cerr << "Error: numSamples exceeds maxBlockSize" << std::endl;
            return;
        }
        
        memset(leftOut, 0, numSamples * sizeof(float));
        memset(rightOut, 0, numSamples * sizeof(float));
        
        // Decode to virtual loudspeakers with sample-by-sample smoothing
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            virtualSpeakerBuffers[ls].clear();
            
            // Process sample by sample for smooth gain changes
            for (int n = 0; n < numSamples; n++) {
                float sample = 0.0f;
                
                for (int sh = 0; sh < nSH; sh++) {
                    // Get smoothed gain for this virtual speaker from this SH channel
                    float smoothedGain = virtualSpeakerSmoothers[ls][sh].getNextValue();
                    sample += smoothedGain * ambiSignals[sh][n];
                }
                
                virtualSpeakerBuffers[ls].setSample(0, n, sample);
            }
        }
        
        // Apply JUCE convolution to smoothed virtual speaker signals
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            tempBufferL.clear();
            tempBufferR.clear();
            tempBufferL.copyFrom(0, 0, virtualSpeakerBuffers[ls], 0, 0, numSamples);
           tempBufferR.copyFrom(0, 0, virtualSpeakerBuffers[ls], 0, 0, numSamples);
           
           juce::dsp::AudioBlock<float> leftBlock(tempBufferL.getArrayOfWritePointers(), 1, 0, static_cast<size_t>(numSamples));
           juce::dsp::AudioBlock<float> rightBlock(tempBufferR.getArrayOfWritePointers(), 1, 0, static_cast<size_t>(numSamples));
           
           convolutionProcessorsL[ls]->process(juce::dsp::ProcessContextReplacing<float>(leftBlock));
           convolutionProcessorsR[ls]->process(juce::dsp::ProcessContextReplacing<float>(rightBlock));
           
           for (int n = 0; n < numSamples; n++) {
               leftOut[n] += tempBufferL.getSample(0, n);
               rightOut[n] += tempBufferR.getSample(0, n);
           }
       }
   }
   
   // Check if any virtual speaker smoothing is currently active
   bool isSmoothing() const {
       for (int ls = 0; ls < nVirtualSpeakers; ls++) {
           for (int sh = 0; sh < nSH; sh++) {
               if (virtualSpeakerSmoothers[ls][sh].isCurrentlySmoothing()) return true;
           }
       }
       return false;
   }
};

// ===================
// Enhanced Sine Agent with Parameter Smoothing
// ===================
class SineAgent : public PositionedVoice {
public:
   float phase = 0.0f;
   
   // Smoothed parameters
   ParameterSmoother freqSmoother;
   ParameterSmoother gainSmoother;
   
   float amplitude = 0.2f;
   float azimuth = 0.0f;
   float elevation = 30.0f;
   
   SineAgent() : freqSmoother(440.0f, 100.0f), gainSmoother(1.0f, 50.0f) {}
   
   void onProcess(AudioIOData &io) override {
       int numFrames = io.framesPerBuffer();
       
       for (int i = 0; i < numFrames; i++) {
           // Get smoothed parameters for this sample
           float smoothedFreq = freqSmoother.getNextValue();
           float smoothedGain = gainSmoother.getNextValue();
           
           float phaseInc = 2.0f * M_PI * smoothedFreq / io.framesPerSecond();
           
           io.out(0, i) += amplitude * smoothedGain * sin(phase);
           io.out(1, i) += amplitude * smoothedGain * sin(phase);
           phase += phaseInc;
           if (phase > 2.0f * M_PI) 
               phase -= 2.0f * M_PI;
       }
   }
   
   void set(float azimuthDeg, float elevationDeg, float frequency, float gainVal) {
       Vec3f position = sphericalToCartesian(azimuthDeg, elevationDeg, 1.0f);
       setPose(Pose(position));
       
       azimuth = azimuthDeg;
       elevation = elevationDeg;
       
       // Set smoothed targets instead of instant changes
       freqSmoother.setTarget(frequency);
       gainSmoother.setTarget(gainVal);
   }
   
   void onTriggerOn() override {
       phase = 0.0f;
   }
   
   void updateSmoothingParameters(float sampleRate) {
       freqSmoother.setSmoothingTime(100.0f, sampleRate);
       gainSmoother.setSmoothingTime(50.0f, sampleRate);
   }
};

class SelectablePickable : public PickableBB {
public:
   bool selected = false;

   bool onEvent(PickEvent e, Hit h) override {
       bool handled = PickableBB::onEvent(e, h);
       if (e.type == Pick && h.hit) {
           selected = true;
       }
       return handled;
   }
};

// ===================
// Enhanced Main App with T-design Layout Selection
// ===================
struct MyApp : public App {
   DynamicScene scene;
   PickableManager pickableManager;
   ControlGUI gui;
   
   Parameter azimuthParam{"Azimuth", "", 0.0, "", -180.0, 180.0};
   Parameter elevationParam{"Elevation", "", 30.0, "", -90.0, 90.0};
   Parameter gainParam{"Gain", "", 1.0, "", 0.0, 2.0};
   Parameter freqParam{"Frequency", "Hz", 440.0, "", 20.0, 2000.0};
   
   // T-design layout selection
   int currentTDesign = 0;  // Index: 0=T(4), 1=T(12), 2=T(24), 3=T(36), 4=T(48)
   const char* tdesignOptions[5] = {"T-design (4)", "T-design (12)", "T-design (24)", "T-design (36)", "T-design (48)"};
   const int tdesignCounts[5] = {4, 12, 24, 36, 48};
   const int tdesignOrders[5] = {1, 3, 5, 6, 7};  // Corresponding ambisonic orders
   
   VAOMesh sourceMesh;
   VAOMesh sphereMesh;
   VAOMesh coneMesh;  // Added for virtual speaker cones
   std::vector<Vec3f> virtualSpeakerPositions;  // Added for cone positions
   
   SineAgent* sineAgent = nullptr;
   bool pickablesUpdatingParameters = false;
   Pose fixedListenerPose{Vec3f(0, 0, 0)};
   float visualRadius = 3.0f;
   SelectablePickable* pickable = nullptr;
   
   AmbisonicEncoder* ambiEncoder = nullptr;
   AmbisonicBinauralDecoder* ambiDecoder = nullptr;
   
   float* inputBuffer = nullptr;
   float* leftOutputBuffer = nullptr;
   float* rightOutputBuffer = nullptr;
   int bufferSize = 0;
   
   // Rate limiting for parameter changes
   double lastParameterUpdateTime = 0.0;
   const double parameterUpdateInterval = 0.01; // 10ms minimum between updates
   
   // Helper function to get current time
   double getCurrentTime() {
       auto now = std::chrono::steady_clock::now();
       auto duration = now.time_since_epoch();
       return std::chrono::duration<double>(duration).count();
   }

   // Create smaller cone mesh with apex pointing toward origin
   void createCone(Mesh& mesh, float baseRadius, float height, int numSegments = 8) {
       mesh.reset();
       mesh.primitive(Mesh::TRIANGLES);
       
       // Apex at origin (0, 0, 0) - this points toward sphere center
       Vec3f apex(0, 0, 0);
       
       // Base center moved away from origin along positive Z axis
       Vec3f baseCenter(0, 0, height);
       
       // Create base vertices in a circle around the base center
       std::vector<Vec3f> baseVertices;
       for (int i = 0; i < numSegments; i++) {
           float angle = 2.0f * M_PI * i / numSegments;
           float x = baseRadius * cos(angle);
           float y = baseRadius * sin(angle);
           baseVertices.push_back(Vec3f(x, y, height));
       }
       
       // Create triangular faces from apex to base edge
       for (int i = 0; i < numSegments; i++) {
           int next = (i + 1) % numSegments;
           
           // Triangle: apex -> base[next] -> base[i] (reversed winding for correct normals)
           mesh.vertex(apex);
           mesh.vertex(baseVertices[next]);
           mesh.vertex(baseVertices[i]);
           
           // Calculate normal for this triangle
           Vec3f edge1 = baseVertices[next] - apex;
           Vec3f edge2 = baseVertices[i] - apex;
           Vec3f normal = cross(edge1, edge2).normalize();
           mesh.normal(normal);
           mesh.normal(normal);
           mesh.normal(normal);
       }
       
       // Create base (circular face)
       for (int i = 1; i < numSegments - 1; i++) {
           mesh.vertex(baseCenter);
           mesh.vertex(baseVertices[i]);
           mesh.vertex(baseVertices[i + 1]);
           
           // Normal pointing in +Z direction (away from apex)
           Vec3f baseNormal(0, 0, 1);
           mesh.normal(baseNormal);
           mesh.normal(baseNormal);
           mesh.normal(baseNormal);
       }
   }

   void createWireframeSphere(Mesh& mesh, float radius, int numLongitudes = 12, int numLatitudes = 8) {
       mesh.reset();
       mesh.primitive(Mesh::LINES);

       // Longitude lines (vertical lines, from north to south)
       for (int i = 0; i < numLongitudes; ++i) {
           float theta = 2.0f * M_PI * i / numLongitudes; // longitude angle

           for (int j = 0; j < numLatitudes; ++j) {
               float phi1 = M_PI * (j    ) / numLatitudes; // latitude angle
               float phi2 = M_PI * (j + 1) / numLatitudes;

               float x1 = radius * sin(phi1) * cos(theta);
               float y1 = radius * cos(phi1);
               float z1 = radius * sin(phi1) * sin(theta);

               float x2 = radius * sin(phi2) * cos(theta);
               float y2 = radius * cos(phi2);
               float z2 = radius * sin(phi2) * sin(theta);

               mesh.vertex(x1, y1, z1);
               mesh.vertex(x2, y2, z2);
           }
       }

       // Latitude lines (horizontal rings, from south to north)
       for (int j = 1; j < numLatitudes; ++j) { // skip poles for latitude lines
           float phi = M_PI * j / numLatitudes; // latitude angle
           float y = radius * cos(phi);
           float r = radius * sin(phi);

           for (int i = 0; i < numLongitudes; ++i) {
               float theta1 = 2.0f * M_PI * i / numLongitudes;
               float theta2 = 2.0f * M_PI * (i + 1) / numLongitudes;

               float x1 = r * cos(theta1);
               float z1 = r * sin(theta1);

               float x2 = r * cos(theta2);
               float z2 = r * sin(theta2);

               mesh.vertex(x1, y, z1);
               mesh.vertex(x2, y, z2);
           }
       }
   }

   void setupVirtualSpeakerPositions() {
       virtualSpeakerPositions.clear();
       
       int speakerCount = tdesignCounts[currentTDesign];
       
       switch (speakerCount) {
           case 4: // T-design (4) - 1st order
               {
                   float speakers[4][2] = {
                       {45.000f, 35.264f},    {-45.000f, -35.264f},
                       {135.000f, -35.264f},  {-135.000f, 35.264f}
                   };
                   for (int i = 0; i < 4; i++) {
                       virtualSpeakerPositions.push_back(sphericalToCartesian(speakers[i][0], speakers[i][1], visualRadius));
                   }
               }
               break;
               
           case 12: // T-design (12) - 3rd order
               {
                   float speakers[12][2] = {
                       {0.0f, -31.717f}, {-58.283f, 0.0f}, {-90.0f, 58.283f},
                       {0.0f, 31.717f}, {-121.717f, 0.0f}, {90.0f, -58.283f},
                       {180.0f, -31.717f}, {121.717f, 0.0f}, {90.0f, 58.283f},
                       {180.0f, 31.717f}, {58.283f, 0.0f}, {-90.0f, -58.283f}
                   };
                   for (int i = 0; i < 12; i++) {
                       virtualSpeakerPositions.push_back(sphericalToCartesian(speakers[i][0], speakers[i][1], visualRadius));
                   }
               }
               break;
               
           case 24: // T-design (24) - 5th order
               {
                   float speakers[24][2] = {
                       {26.001f, 15.464f}, {-26.001f, -15.464f}, {17.109f, -24.994f}, {-17.109f, 24.994f},
                       {153.999f, -15.464f}, {-153.999f, 15.464f}, {162.891f, 24.994f}, {-162.891f, -24.994f},
                       {72.891f, 24.994f}, {107.109f, -24.994f}, {116.001f, 15.464f}, {63.999f, -15.464f},
                       {-107.109f, 24.994f}, {-72.891f, -24.994f}, {-63.999f, 15.464f}, {-116.001f, -15.464f},
                       {32.254f, 60.025f}, {-147.746f, 60.025f}, {-57.746f, 60.025f}, {122.254f, 60.025f},
                       {-32.254f, -60.025f}, {147.746f, -60.025f}, {57.746f, -60.025f}, {-122.254f, -60.025f}
                   };
                   for (int i = 0; i < 24; i++) {
                       virtualSpeakerPositions.push_back(sphericalToCartesian(speakers[i][0], speakers[i][1], visualRadius));
                   }
               }
               break;
               
           case 36: // T-design (36) - 6th order
               {
                   float speakers[36][2] = {
                       {-31.106f, 53.651f}, {110.815f, 30.496f}, {148.894f, 53.651f}, {32.214f, -17.830f},
                       {69.185f, -30.496f}, {-32.214f, 17.830f}, {-69.185f, 30.496f}, {-147.786f, -17.830f},
                       {-110.815f, -30.496f}, {147.786f, 17.830f}, {31.106f, -53.651f}, {-148.894f, -53.651f},
                       {-21.246f, -47.775f}, {-108.204f, 38.782f}, {158.754f, -47.775f}, {139.774f, -14.095f},
                       {-71.796f, -38.782f}, {-139.774f, 14.095f}, {71.796f, 38.782f}, {-40.226f, -14.095f},
                       {108.204f, -38.782f}, {40.226f, 14.095f}, {21.246f, 47.775f}, {-158.754f, 47.775f},
                       {106.650f, -2.552f}, {-2.663f, -16.634f}, {-73.350f, -2.552f}, {-98.841f, 73.161f},
                       {-177.337f, 16.634f}, {98.841f, -73.161f}, {177.337f, -16.634f}, {81.159f, 73.161f},
                       {2.663f, 16.634f}, {-81.159f, -73.161f}, {-106.650f, 2.552f}, {73.350f, 2.552f}
                   };
                   for (int i = 0; i < 36; i++) {
                       virtualSpeakerPositions.push_back(sphericalToCartesian(speakers[i][0], speakers[i][1], visualRadius));
                   }
               }
               break;
               
           case 48: // T-design (48) - 7th order
               {
                   float speakers[48][2] = {
                       {20.746f, -3.552f}, {-20.746f, 3.552f}, {-3.798f, -20.704f}, {3.798f, 20.704f},
                       {159.254f, 3.552f}, {-159.254f, -3.552f}, {-176.202f, 20.704f}, {176.202f, -20.704f},
                       {93.798f, 20.704f}, {86.202f, -20.704f}, {110.746f, -3.552f}, {69.254f, 3.552f},
                       {-86.202f, 20.704f}, {-93.798f, -20.704f}, {-69.254f, -3.552f}, {-110.746f, 3.552f},
                       {-9.939f, 68.966f}, {170.061f, 68.966f}, {-99.939f, 68.966f}, {80.061f, 68.966f},
                       {9.939f, -68.966f}, {-170.061f, -68.966f}, {99.939f, -68.966f}, {-80.061f, -68.966f},
                       {42.147f, 17.568f}, {-42.147f, -17.568f}, {23.124f, -39.772f}, {-23.124f, 39.772f},
                       {137.853f, -17.568f}, {-137.853f, 17.568f}, {156.876f, 39.772f}, {-156.876f, -39.772f},
                       {66.876f, 39.772f}, {113.124f, -39.772f}, {132.147f, 17.568f}, {47.853f, -17.568f},
                       {-113.124f, 39.772f}, {-66.876f, -39.772f}, {-47.853f, 17.568f}, {-132.147f, -17.568f},
                       {25.259f, 44.979f}, {-154.741f, 44.979f}, {-64.741f, 44.979f}, {115.259f, 44.979f},
                       {-25.259f, -44.979f}, {154.741f, -44.979f}, {64.741f, -44.979f}, {-115.259f, -44.979f}
                   };
                   for (int i = 0; i < 48; i++) {
                       virtualSpeakerPositions.push_back(sphericalToCartesian(speakers[i][0], speakers[i][1], visualRadius));
                   }
               }
               break;
               
           default:
               // Fallback to T-design (4)
               currentTDesign = 0;
               setupVirtualSpeakerPositions();
               return;
       }
       
       std::cout << "Virtual speakers setup: " << tdesignOptions[currentTDesign] 
                 << " with " << virtualSpeakerPositions.size() << " speakers" << std::endl;
   }

   void onCreate() override {
       double sampleRate = audioIO().framesPerSecond();
       int frameSize = audioIO().framesPerBuffer();
       
       ambiEncoder = new AmbisonicEncoder(1, frameSize, sampleRate);
       ambiDecoder = new AmbisonicBinauralDecoder(1, sampleRate, frameSize, &currentTDesign);
       
       addSphere(sourceMesh, 0.3);
       sourceMesh.primitive(Mesh::LINE_STRIP);
       sourceMesh.update();
       
       createWireframeSphere(sphereMesh, visualRadius, 24, 16);
       sphereMesh.primitive(Mesh::LINES);
       sphereMesh.update();
       
       // Create smaller cone mesh for virtual speakers
       createCone(coneMesh, 0.12f, 0.35f, 8);  // Smaller: baseRadius=0.12f, height=0.35f, 8 segments
       coneMesh.update();
       
       // Setup virtual speaker positions using T-design coordinates
       setupVirtualSpeakerPositions();
       
       imguiInit();
       
       gui.init(5, 5, false);
       gui.setTitle("Ambisonics Binaural Control");
       
       auto updateCallback = [this](float value) {
           if (!pickablesUpdatingParameters) {
               // Rate limit parameter updates
               double currentTime = getCurrentTime();
               if (currentTime - lastParameterUpdateTime > parameterUpdateInterval) {
                   updateSourcePosition();
                   lastParameterUpdateTime = currentTime;
               }
           }
       };
       
       azimuthParam.registerChangeCallback(updateCallback);
       elevationParam.registerChangeCallback(updateCallback);
       gainParam.registerChangeCallback(updateCallback);
       freqParam.registerChangeCallback(updateCallback);
       
       pickable = new SelectablePickable();
       pickable->set(sourceMesh);
       updateSourcePosition();
       pickableManager << pickable;
       
       sineAgent = scene.getVoice<SineAgent>();
       if (sineAgent) {
           sineAgent->updateSmoothingParameters(sampleRate);
           sineAgent->set(azimuthParam.get(), elevationParam.get(), freqParam.get(), gainParam.get());
           scene.triggerOn(sineAgent);
       }
       
       bufferSize = audioIO().framesPerBuffer();
       inputBuffer = new float[bufferSize];
       leftOutputBuffer = new float[bufferSize];
       rightOutputBuffer = new float[bufferSize];
       
       scene.prepare(audioIO());
       
       // Set camera to look at the sphere from the equator level (horizontal view)
       // Position camera in front of the sphere, looking toward the center
       nav().pos(0, 0, 8); // Move camera to equator, out in +Z
       nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0)); // Look at center, Y is up
       nav().quat(Quatd()); // Reset any roll/pitch (optional but recommended)

       
       std::cout << "Ambisonics-to-Binaural Renderer with T-design Virtual Speaker Visualization:" << std::endl;
       std::cout << "  1. Click and drag the sound source to move it on the sphere" << std::endl;
       std::cout << "  2. Use the GUI to adjust parameters and T-design layout" << std::endl;
       std::cout << "  3. Press SPACE to reset to default position" << std::endl;
       std::cout << "  4. Using JUCE partitioned convolution with parameter smoothing" << std::endl;
       std::cout << "  5. Cone-shaped virtual speakers represent selected T-design layout" << std::endl;
   }
   
   // Safe ambisonic order update that preserves convolution processors when possible
void updateAmbisonicOrderSafely(int newOrder) {
    // Step 1: Stop audio to prevent callback race
    audioIO().stop();

    if (newOrder < 1 || newOrder > 7) {
        std::cerr << "Invalid order: " << newOrder << ". Must be between 1 and 7." << std::endl;
        audioIO().start();
        return;
    }

    if (ambiEncoder && ambiEncoder->getOrder() == newOrder) {
        audioIO().start();
        return; // No change needed
    }

    std::cout << "Safely updating ambisonic order to: " << newOrder << std::endl;

    double sampleRate = audioIO().framesPerSecond();
    int frameSize = audioIO().framesPerBuffer();

    try {
        // Delete old objects BEFORE creating new to avoid memory leaks
        delete ambiEncoder;
        delete ambiDecoder;

        ambiEncoder = new AmbisonicEncoder(newOrder, frameSize, sampleRate);
        ambiDecoder = new AmbisonicBinauralDecoder(newOrder, sampleRate, frameSize, &currentTDesign);

        if (sineAgent) {
            sineAgent->updateSmoothingParameters(sampleRate);
        }
        setupVirtualSpeakerPositions();
        updateSourcePosition();
        std::cout << "Successfully updated to order " << newOrder << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error updating ambisonic order: " << e.what() << std::endl;
        currentTDesign = 0;
        // Optionally fall back to default here
    } catch (...) {
        std::cerr << "Unknown error updating ambisonic order" << std::endl;
        currentTDesign = 0;
    }

    // Step 2: Restart audio
    audioIO().start();
}

   
   void updateSourcePosition() {
       if (sineAgent) {
           sineAgent->set(azimuthParam.get(), elevationParam.get(), freqParam.get(), gainParam.get());
       }
       
       if (pickable) {
           Vec3f position = sphericalToCartesian(azimuthParam.get(), elevationParam.get(), visualRadius);
           pickable->pose = Pose(position);
       }
       
       if (ambiEncoder) {
           ambiEncoder->updateDirection(azimuthParam.get(), elevationParam.get());
       }
   }
   
   Vec3f constrainToSphere(const Vec3f& point) {
       float radius = point.mag();
       if (radius < 0.0001f)
           return Vec3f(0, 0, -visualRadius);
       return point.normalized() * visualRadius;
   }
   
   void onAnimate(double dt) override {
       navControl().active(!gui.usingInput());
       
       if (pickable && !gui.usingInput()) {
           pickablesUpdatingParameters = true;
           
           Vec3f pos = pickable->pose.get().pos();
           Vec3f constrained = constrainToSphere(pos);
           pickable->pose.set(Pose(constrained));
           
           float az, el, radius;
           cartesianToSpherical(constrained, az, el, radius);
           
           // Rate limit parameter changes from mouse interaction too
           double currentTime = getCurrentTime();
           if (currentTime - lastParameterUpdateTime > parameterUpdateInterval) {
               azimuthParam.set(az);
               elevationParam.set(el);
               
               if (sineAgent) {
                   sineAgent->set(az, el, freqParam.get(), gainParam.get());
               }
               
               if (ambiEncoder) {
                   ambiEncoder->updateDirection(az, el);
               }
               
               lastParameterUpdateTime = currentTime;
           }
           
           pickablesUpdatingParameters = false;
       }
   }

   void onDraw(Graphics &g) override {
       g.clear(0);
       gl::depthTesting(true);
       
       // Draw coordinate reference axes
       g.lineWidth(2.0);
       Mesh axes;
       axes.primitive(Mesh::LINES);
       
       // X axis (red)
       g.color(1, 0, 0);
       axes.vertex(0, 0, 0);
       axes.vertex(1, 0, 0);
       
       // Y axis (green)
       g.color(0, 1, 0);
       axes.vertex(0, 0, 0);
       axes.vertex(0, 1, 0);
       
       // Z axis (blue)
       g.color(0, 0, 1);
       axes.vertex(0, 0, 0);
       axes.vertex(0, 0, 1);
       
       g.draw(axes);
       
       // Draw reference sphere
       g.color(0.3, 0.3, 0.3, 0.4);
       g.polygonMode(GL_LINE);
       g.draw(sphereMesh);
       
       // Draw virtual speaker cones at T-design positions
       g.polygonMode(GL_FILL);
       for (int i = 0; i < virtualSpeakerPositions.size(); i++) {
           Vec3f speakerPos = virtualSpeakerPositions[i];
           
           g.pushMatrix();
           
           // Move to speaker position
           g.translate(speakerPos);
           
           // Point the cone toward the origin using lookAt
           Vec3f eye = speakerPos;
           Vec3f center = Vec3f(0, 0, 0);  // Origin
           Vec3f up = Vec3f(0, 1, 0);
           
           // Calculate forward, right, and up vectors
           Vec3f forward = (center - eye).normalize();
           Vec3f right = cross(forward, up).normalize();
           up = cross(right, forward).normalize();
           
           // Create and apply the transformation matrix
           float matrix[16] = {
               right.x,   right.y,   right.z,   0,
               up.x,      up.y,      up.z,      0,
               -forward.x, -forward.y, -forward.z, 0,
               0,         0,         0,         1
           };
           
           g.multModelMatrix(Matrix4f(matrix));
           
           // Color all cones the same (orange)
           g.color(1.0, 0.6, 0.2, 0.8);  // Orange color for all cones
           
           g.draw(coneMesh);
           g.popMatrix();
       }
       
       // Draw sound source
       if (pickable) {
           // Simple green color for the sound source
           g.color(0.2, 0.8, 0.4);
           g.polygonMode(GL_FILL);
           
           pickable->draw(g, [&](Pickable &p) {
               auto &b = dynamic_cast<PickableBB &>(p);
               b.drawMesh(g);
           });
           // Get spherical coordinates for visualization helpers
           float az, el, radius;
           cartesianToSpherical(pickable->pose.get().pos(), az, el, radius);
       }
       
       // Draw GUI
       imguiBeginFrame();
       gui.draw(g);
       
       // Custom ImGui controls
       ImGui::Begin("Source Controls");
       
       // T-design layout dropdown
       if (ImGui::Combo("Speaker Layout", &currentTDesign, tdesignOptions, 5)) {
           int newOrder = tdesignOrders[currentTDesign];
           updateAmbisonicOrderSafely(newOrder);
       }
       
       float azValue = azimuthParam.get();
       if (ImGui::SliderFloat("Azimuth", &azValue, -180.0f, 180.0f)) {
           azimuthParam.set(azValue);
           updateSourcePosition();
       }
       
       float elValue = elevationParam.get();
       if (ImGui::SliderFloat("Elevation", &elValue, -90.0f, 90.0f)) {
           elevationParam.set(elValue);
           updateSourcePosition();
       }
       
       float gainValue = gainParam.get();
       if (ImGui::SliderFloat("Gain", &gainValue, 0.0f, 2.0f)) {
           gainParam.set(gainValue);
           updateSourcePosition();
       }
       
       float freqValue = freqParam.get();
       if (ImGui::SliderFloat("Frequency", &freqValue, 20.0f, 2000.0f)) {
           freqParam.set(freqValue);
           updateSourcePosition();
       }
       
       // Display current T-design info
       ImGui::Separator();
       ImGui::Text("Current Layout: %s", tdesignOptions[currentTDesign]);
       ImGui::Text("Virtual Speakers: %d", tdesignCounts[currentTDesign]);
       ImGui::Text("Ambisonic Order: %d", tdesignOrders[currentTDesign]);
       
       ImGui::End();
       imguiEndFrame();
       imguiDraw();
   }

   void onSound(AudioIOData &io) override {
       Pose& listenerPose = scene.listenerPose();
       listenerPose = fixedListenerPose;
       
       int nFrames = io.framesPerBuffer();
       
       if (nFrames > bufferSize) {
           delete[] inputBuffer;
           delete[] leftOutputBuffer;
           delete[] rightOutputBuffer;
           
           bufferSize = nFrames;
           inputBuffer = new float[bufferSize];
           leftOutputBuffer = new float[bufferSize];
           rightOutputBuffer = new float[bufferSize];
       }
       
       memset(leftOutputBuffer, 0, nFrames * sizeof(float));
       memset(rightOutputBuffer, 0, nFrames * sizeof(float));
       
       // Generate smoothed sine wave input
       if (sineAgent) {
           float phase = sineAgent->phase;
           
           for (int i = 0; i < nFrames; i++) {
               // Get smoothed parameters for each sample
               float smoothedFreq = sineAgent->freqSmoother.getNextValue();
               float smoothedGain = sineAgent->gainSmoother.getNextValue();
               
               float phaseInc = 2.0f * M_PI * smoothedFreq / io.framesPerSecond();
               float amp = sineAgent->amplitude * smoothedGain;
               
               inputBuffer[i] = amp * sin(phase);
               phase += phaseInc;
               if (phase > 2.0f * M_PI) 
                   phase -= 2.0f * M_PI;
           }
           
           sineAgent->phase = phase;
       }
       
       // Process through smoothed Ambisonics pipeline
       if (ambiEncoder && ambiDecoder) {
           // Encode with SH coefficient smoothing
           ambiEncoder->encode(inputBuffer, nFrames);
           
           // Decode with virtual speaker signal smoothing
           ambiDecoder->decode(ambiEncoder->ambiSignals, leftOutputBuffer, rightOutputBuffer, nFrames);
       }
       
       // Copy to output
       for (int i = 0; i < nFrames; i++) {
           io.out(0, i) = leftOutputBuffer[i];
           io.out(1, i) = rightOutputBuffer[i];
       }
   }

   void onExit() override {
       delete ambiEncoder;
       delete ambiDecoder;
       delete[] inputBuffer;
       delete[] leftOutputBuffer;
       delete[] rightOutputBuffer;
   }

   bool onKeyDown(const Keyboard &k) override {
       if (k.key() == ' ') {
           azimuthParam.set(0.0);
           elevationParam.set(30.0);
           gainParam.set(1.0);
           freqParam.set(440.0);
           updateSourcePosition();
           return true;
       }
       return false;
   }
   
   // Mouse event handling
   bool onMouseMove(const Mouse &m) override {
       if (gui.usingInput()) return true;
       pickableManager.onMouseMove(graphics(), m, width(), height());
       return true;
   }
   
   bool onMouseDown(const Mouse &m) override {
       if (gui.usingInput()) return true;
       pickableManager.onMouseDown(graphics(), m, width(), height());
       return true;
   }
   
   bool onMouseDrag(const Mouse &m) override {
       if (gui.usingInput()) return true;
       pickableManager.onMouseDrag(graphics(), m, width(), height());
       return true;
   }
   
   bool onMouseUp(const Mouse &m) override {
       if (gui.usingInput()) return true;
       pickableManager.onMouseUp(graphics(), m, width(), height());
       return true;
   }
};

int main() {
   MyApp app;
   app.dimensions(800, 600);
   app.title("AlloLib Ambisonics-to-Binaural Renderer with T-design Virtual Speakers");
   app.configureAudio(44100, 256, 2, 0);  
   app.start();
   return 0;
}