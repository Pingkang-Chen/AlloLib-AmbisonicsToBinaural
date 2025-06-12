#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <cstring>
#include <cmath>
#include <algorithm>

// JUCE includes
#include "juce_core/juce_core.h"
#include "juce_audio_basics/juce_audio_basics.h"
#include "juce_dsp/juce_dsp.h"

// SAF includes
#include "saf.h"
#include "saf_hrir.h"
#include "saf_hoa.h"
#include "saf_vbap.h"  // Added for VBAP interpolation

// Local includes
#include "speakerLayout.hpp"
#include "AmbisonicReverbProcessor.hpp"

/**
 * @brief Enhanced Ambisonic Binaural Decoder with VBAP-based HRTF Interpolation
 * 
 * Provides high-quality binaural decoding of ambisonic signals using JUCE convolution
 * and virtual speaker layouts with VBAP-interpolated HRTFs for accurate spatial positioning.
 */
class AmbisonicBinauralDecoder {
private:
    // Internal parameter smoother
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
        
        bool isCurrentlySmoothing() const { return isSmoothing; }
        
        void setSmoothingTime(float smoothingTimeMs, float sampleRate) {
            float tau = smoothingTimeMs / 1000.0f;
            smoothingCoeff = exp(-1.0f / (tau * sampleRate));
        }
    };

    // Private helper method for energy compensation
float getVirtualSpeakerGain() const {
    // Base energy compensation
    float baseCompensation = 0.7f / sqrt(static_cast<float>(nVirtualSpeakers));
    
    // Volume normalization to maintain consistent loudness across T-designs
    float volumeNormalization;
    switch (nVirtualSpeakers) {
        case 4:  volumeNormalization = 1.0f;   break;
        case 12: volumeNormalization = 2.0f;   break;
        case 24: volumeNormalization = 3.5f;   break;
        case 36: volumeNormalization = 5.0f;   break;
        case 48: volumeNormalization = 7.0f;   break;
        default: volumeNormalization = 1.0f;   break;
    }
    
    return baseCompensation * volumeNormalization;
}

public:
    const float* hrirs;
    const float* hrir_dirs_deg;
    int N_hrir_dirs;
    int hrir_len;
    int fs;
    
    int order;
    int nSH;
    
    float* vls_dirs_deg;
    int nVirtualSpeakers;
    
    float* dec_mat;
    float* vls_gains;
    
    // VBAP interpolation data
    float* vbapGainTable;
    int vbapTableSize;
    bool vbapInitialized;
    
    std::vector<std::unique_ptr<juce::dsp::Convolution>> convolutionProcessorsL;
    std::vector<std::unique_ptr<juce::dsp::Convolution>> convolutionProcessorsR;
    
    std::vector<juce::AudioBuffer<float>> virtualSpeakerBuffers;
    juce::AudioBuffer<float> tempBufferL;
    juce::AudioBuffer<float> tempBufferR;
    
    // Interpolated HRIR buffers
    juce::AudioBuffer<float> interpolatedHRIR_L;
    juce::AudioBuffer<float> interpolatedHRIR_R;
    
    std::vector<std::vector<ParameterSmoother>> virtualSpeakerSmoothers;
    
    double sampleRate;
    int maxBlockSize;
    
    SpeakerLayout::TDesignType currentTDesign;
    
    /**
     * @brief Construct a new Ambisonic Binaural Decoder object with VBAP interpolation
     */
    AmbisonicBinauralDecoder(int orderVal = 1, int sampleRateVal = 44100, int maxBlockSizeVal = 256, 
                            SpeakerLayout::TDesignType tdesign = SpeakerLayout::T4) : 
        order(orderVal), sampleRate(sampleRateVal), maxBlockSize(maxBlockSizeVal), 
        currentTDesign(tdesign), vbapGainTable(nullptr), vbapTableSize(0), vbapInitialized(false) {
        
        nSH = (order + 1) * (order + 1);
        
        // Initialize HRIR data from SAF
        hrirs = &__default_hrirs[0][0][0];
        hrir_dirs_deg = &__default_hrir_dirs_deg[0][0];
        N_hrir_dirs = __default_N_hrir_dirs;
        hrir_len = __default_hrir_len;
        fs = __default_hrir_fs;
        
        setupVirtualLoudspeakers();
        createDecodingMatrix();
        setupVBAPInterpolation();
        setupJUCEConvolution();
        
        std::cout << "Enhanced AmbisonicBinauralDecoder with VBAP HRTF Interpolation:" << std::endl;
        std::cout << "  Order: " << order << ", nSH: " << nSH << std::endl;
        std::cout << "  Virtual speakers: " << nVirtualSpeakers << std::endl;
        std::cout << "  HRIR measurement points: " << N_hrir_dirs << std::endl;
        std::cout << "  Using VBAP interpolation for accurate HRTF positioning" << std::endl;
    }
    
    /**
     * @brief Destroy the Ambisonic Binaural Decoder object
     */
    ~AmbisonicBinauralDecoder() {
        delete[] vls_dirs_deg;
        delete[] dec_mat;
        delete[] vls_gains;
        if (vbapGainTable) {
            free(vbapGainTable);
        }
    }
    
    /**
     * @brief Setup virtual loudspeaker layout
     */
    void setupVirtualLoudspeakers() {
        nVirtualSpeakers = SpeakerLayout::getSpeakerCount(currentTDesign);
        
        vls_dirs_deg = new float[nVirtualSpeakers * 2];
        
        auto coords = SpeakerLayout::getTDesignSphericalCoords(currentTDesign);
        for (int i = 0; i < nVirtualSpeakers; i++) {
            vls_dirs_deg[i*2] = coords[i].first;      // azimuth
            vls_dirs_deg[i*2+1] = coords[i].second;   // elevation
        }
        
        std::cout << "Setup virtual loudspeakers: " << nVirtualSpeakers << " speakers" << std::endl;
    }
    
    /**
     * @brief Create the ambisonic decoding matrix
     */
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
    
    /**
     * @brief Setup VBAP interpolation for HRTF positioning
     */
    void setupVBAPInterpolation() {
        // Create a copy of HRIR directions for VBAP (need non-const)
        float* hrir_dirs_copy = new float[N_hrir_dirs * 2];
        for (int i = 0; i < N_hrir_dirs * 2; i++) {
            hrir_dirs_copy[i] = hrir_dirs_deg[i];
        }
        
        // Generate VBAP gain table for virtual speaker directions
        int nTriangles;
        generateVBAPgainTable3D_srcs(vls_dirs_deg,          // Source directions (virtual speakers)
                                   nVirtualSpeakers,        // Number of sources  
                                   hrir_dirs_copy,          // HRIR measurement directions (loudspeakers)
                                   N_hrir_dirs,             // Number of HRIR directions
                                   1,                       // Omit large triangles
                                   1,                       // Enable dummy speakers
                                   0.0f,                    // No spreading (pure VBAP)
                                   &vbapGainTable,          // Output gain table
                                   &vbapTableSize,          // Output table size
                                   &nTriangles);            // Output triangle count
        
        delete[] hrir_dirs_copy;
        
        if (vbapGainTable != nullptr) {
            vbapInitialized = true;
            std::cout << "VBAP interpolation initialized: " << vbapTableSize 
                      << " entries, " << nTriangles << " triangles" << std::endl;
        } else {
            vbapInitialized = false;
            std::cerr << "Error: VBAP interpolation setup failed - cannot proceed without HRTF interpolation" << std::endl;
            throw std::runtime_error("VBAP HRTF interpolation setup failed");
        }
    }
    
    /**
     * @brief Get interpolated HRIR using VBAP for a specific virtual speaker index
     */
    void getInterpolatedHRIR(int vls_idx, float* hrir_left_out, float* hrir_right_out) {
        
        if (!vbapInitialized || !vbapGainTable || vls_idx >= nVirtualSpeakers) {
            std::cerr << "Error: VBAP not initialized or invalid virtual speaker index" << std::endl;
            memset(hrir_left_out, 0, hrir_len * sizeof(float));
            memset(hrir_right_out, 0, hrir_len * sizeof(float));
            return;
        }
        
        // Use VBAP gains for this virtual speaker to interpolate HRIRs
        const float* gains = &vbapGainTable[vls_idx * N_hrir_dirs];
        
        // Clear output buffers
        memset(hrir_left_out, 0, hrir_len * sizeof(float));
        memset(hrir_right_out, 0, hrir_len * sizeof(float));
        
        // Interpolate HRIRs using VBAP gains
        for (int hrir_idx = 0; hrir_idx < N_hrir_dirs; hrir_idx++) {
            float gain = gains[hrir_idx];
            
            if (gain > 0.001f) {  // Only process significant gains for efficiency
                const float* src_left = &hrirs[hrir_idx*2*hrir_len];
                const float* src_right = &hrirs[hrir_idx*2*hrir_len + hrir_len];
                
                for (int i = 0; i < hrir_len; i++) {
                    hrir_left_out[i] += gain * src_left[i];
                    hrir_right_out[i] += gain * src_right[i];
                }
            }
        }
    }
    
    /**
     * @brief Setup JUCE convolution processors with VBAP-interpolated HRIRs
     */
    void setupJUCEConvolution() {
        convolutionProcessorsL.resize(nVirtualSpeakers);
        convolutionProcessorsR.resize(nVirtualSpeakers);
        
        juce::dsp::ProcessSpec spec;
        spec.sampleRate = sampleRate;
        spec.maximumBlockSize = static_cast<juce::uint32>(maxBlockSize);
        spec.numChannels = 1;
        
        // Buffers for interpolated HRIRs
        std::vector<float> hrir_left(hrir_len);
        std::vector<float> hrir_right(hrir_len);
        
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            convolutionProcessorsL[ls] = std::make_unique<juce::dsp::Convolution>();
            convolutionProcessorsR[ls] = std::make_unique<juce::dsp::Convolution>();
            
            convolutionProcessorsL[ls]->prepare(spec);
            convolutionProcessorsR[ls]->prepare(spec);
            
            // Get VBAP-interpolated HRIR for this virtual speaker
            getInterpolatedHRIR(ls, hrir_left.data(), hrir_right.data());
            
            // Create JUCE audio buffers
            juce::AudioBuffer<float> leftBuffer(1, hrir_len);
            juce::AudioBuffer<float> rightBuffer(1, hrir_len);
            
            for (int i = 0; i < hrir_len; i++) {
                leftBuffer.setSample(0, i, hrir_left[i]);
                rightBuffer.setSample(0, i, hrir_right[i]);
            }
            
            // Load interpolated HRIRs into convolution processors
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
        
        std::cout << "JUCE convolution setup complete with VBAP-interpolated HRIRs for " 
                  << nVirtualSpeakers << " virtual speakers" << std::endl;
    }
    
    /**
     * @brief Update smoothing parameters
     */
    void updateSmoothingParameters(float newSampleRate) {
        sampleRate = newSampleRate;
        // Note: No smoothing used for immediate spatial response
    }
    
    /**
     * @brief Decode ambisonic signals to binaural output using VBAP-interpolated HRTFs
     */
    void decode(float** ambiSignals, float* leftOut, float* rightOut, int numSamples) {
        if (numSamples > maxBlockSize) {
            std::cerr << "Error: numSamples exceeds maxBlockSize" << std::endl;
            return;
        }
        
        memset(leftOut, 0, numSamples * sizeof(float));
        memset(rightOut, 0, numSamples * sizeof(float));
        
        // Get energy compensation factor
        float compensationGain = getVirtualSpeakerGain();
        
        // Decode to virtual loudspeakers WITH energy compensation
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            virtualSpeakerBuffers[ls].clear();
            
            for (int n = 0; n < numSamples; n++) {
                float sample = 0.0f;
                
                for (int sh = 0; sh < nSH; sh++) {
                    float gain = dec_mat[ls*nSH + sh] * vls_gains[ls] * compensationGain;
                    sample += gain * ambiSignals[sh][n];
                }
                
                virtualSpeakerBuffers[ls].setSample(0, n, sample);
            }
        }
        
        // Apply JUCE convolution with VBAP-interpolated HRTFs
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
    
    /**
     * @brief Decode ambisonic signals to binaural output with reverb processing
     */
    void decodeWithReverb(float** ambiSignals, float* leftOut, float* rightOut, int numSamples, 
                         AmbisonicReverbProcessor* reverbProcessor) {
        if (numSamples > maxBlockSize) {
            std::cerr << "Error: numSamples exceeds maxBlockSize" << std::endl;
            return;
        }
        
        memset(leftOut, 0, numSamples * sizeof(float));
        memset(rightOut, 0, numSamples * sizeof(float));
        
        // Get energy compensation factor
        float compensationGain = getVirtualSpeakerGain();
        
        // Decode to virtual loudspeakers WITH energy compensation
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            virtualSpeakerBuffers[ls].clear();
            
            for (int n = 0; n < numSamples; n++) {
                float sample = 0.0f;
                
                for (int sh = 0; sh < nSH; sh++) {
                    float gain = dec_mat[ls*nSH + sh] * vls_gains[ls] * compensationGain;
                    sample += gain * ambiSignals[sh][n];
                }
                
                virtualSpeakerBuffers[ls].setSample(0, n, sample);
            }
        }
        
        // Apply reverb processing if not free-field
        if (reverbProcessor && !reverbProcessor->isCurrentlyFreeField()) {
            reverbProcessor->processReverbConvolution(virtualSpeakerBuffers, numSamples);
        }
        
        // Apply JUCE convolution with HRTFs
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
    
    /**
     * @brief Check if any smoothing is currently active (removed for immediate response)
     */
    bool isSmoothing() const {
        return false;  // No smoothing for immediate spatial response
    }
    
    /**
     * @brief Get current T-design type
     */
    SpeakerLayout::TDesignType getCurrentTDesign() const {
        return currentTDesign;
    }
    
    /**
     * @brief Update T-design layout (requires recreation)
     */
    void updateTDesign(SpeakerLayout::TDesignType newTDesign) {
        if (newTDesign != currentTDesign) {
            currentTDesign = newTDesign;
            
            // Clean up old data
            delete[] vls_dirs_deg;
            delete[] dec_mat;
            delete[] vls_gains;
            if (vbapGainTable) {
                free(vbapGainTable);
                vbapGainTable = nullptr;
            }
            
            // Recreate with new layout
            setupVirtualLoudspeakers();
            createDecodingMatrix();
            setupVBAPInterpolation();
            setupJUCEConvolution();
        }
    }
};