#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <map>

// JUCE includes
#include "juce_core/juce_core.h"
#include "juce_audio_basics/juce_audio_basics.h"
#include "juce_dsp/juce_dsp.h"
#include "juce_audio_formats/juce_audio_formats.h"

// SAF includes
#include "saf.h"
#include "saf_hoa.h"

// Local includes
#include "speakerLayout.hpp"

/**
 * @brief Ambisonic Reverb Processor using JUCE's Partitioned Convolution
 * 
 * Loads B-format ambisonic impulse responses, decodes them to virtual speaker IRs,
 * and uses JUCE's efficient partitioned convolution for real-time reverb processing.
 */
class AmbisonicReverbProcessor {
public:
    enum ReverbEnvironment {
        FREE_FIELD = 0,
        CATHEDRAL = 1,
        CHURCH = 2,
        FOREST = 3,
        CHAMBER = 4
    };

private:
    int order;
    int nSH;
    double sampleRate;
    int maxBlockSize;
    
    SpeakerLayout::TDesignType currentTDesign;
    int nVirtualSpeakers;
    float* vls_dirs_deg;
    float* dec_mat;
    
    // B-format IR storage
    std::map<ReverbEnvironment, juce::AudioBuffer<float>> bFormatIRs;
    
    // Decoded virtual speaker IRs for each environment
    std::map<ReverbEnvironment, std::vector<juce::AudioBuffer<float>>> virtualSpeakerIRs;
    
    // JUCE Convolution processors for reverb - one per virtual speaker for current environment
    std::vector<std::unique_ptr<juce::dsp::Convolution>> currentReverbProcessors;
    
    // Temporary buffers for reverb processing
    std::vector<juce::AudioBuffer<float>> reverbTempBuffers;
    
    ReverbEnvironment currentEnvironment;
    bool isInitialized;
    bool convolutionSetup;
    
    juce::AudioFormatManager audioFormatManager;
    
    // Environment names and file paths
    std::map<ReverbEnvironment, std::string> environmentNames;
    std::map<ReverbEnvironment, std::string> environmentFilePaths;

    // Helper methods
    int getMaxIRLength() const {
        // Even more conservative limits for higher T-designs
        switch (nVirtualSpeakers) {
            case 4:  return static_cast<int>(sampleRate * 8.0);   // 8 seconds for T4
            case 12: return static_cast<int>(sampleRate * 6.0);   // 6 seconds for T12  
            case 24: return static_cast<int>(sampleRate * 3.0);   // 3 seconds for T24
            case 36: return static_cast<int>(sampleRate * 1.0);   // 1 second for T36
            case 48: return static_cast<int>(sampleRate * 0.5);   // 0.5 seconds for T48
            case 54: return static_cast<int>(sampleRate * 0.4);   // 0.4 seconds for Allosphere
            default: return static_cast<int>(sampleRate * 1.0);
        }
    }

    float getEnergyCompensationGain() const {
        // Much more aggressive compensation for higher T-designs
        float baseGain;
        switch (currentTDesign) {
            case SpeakerLayout::T4:  baseGain = 1.0f;    break;
            case SpeakerLayout::T12: baseGain = 0.5f;    break;
            case SpeakerLayout::T24: baseGain = 0.3f;    break;
            case SpeakerLayout::T36: baseGain = 0.15f;   break;
            case SpeakerLayout::T48: baseGain = 0.1f;    break;
            case SpeakerLayout::ALLOSPHERE: baseGain = 0.09f; break;
            default: baseGain = 1.0f;
        }
        
        // Additional compensation for longer reverbs
        float lengthCompensation = 1.0f;
        if (currentEnvironment == CATHEDRAL || currentEnvironment == FOREST || currentEnvironment == CHAMBER) {
            lengthCompensation = 0.6f;
        }
        
        return baseGain * lengthCompensation;
    }

    float getVolumeNormalizationGain() const {
        // Normalize volume across all T-designs to maintain consistent perceived loudness
        switch (currentTDesign) {
            case SpeakerLayout::T4:  return 1.0f;     // Reference level
            case SpeakerLayout::T12: return 2.0f;     // Compensate for 0.5x energy reduction
            case SpeakerLayout::T24: return 3.3f;     // Compensate for 0.3x energy reduction
            case SpeakerLayout::T36: return 6.7f;     // Compensate for 0.15x energy reduction
            case SpeakerLayout::T48: return 10.0f;    // Compensate for 0.1x energy reduction
            case SpeakerLayout::ALLOSPHERE: return 11.0f;
            default: return 1.0f;
        }
    }

public:
    AmbisonicReverbProcessor(int orderVal = 1, double sampleRateVal = 44100, int maxBlockSizeVal = 256,
                            SpeakerLayout::TDesignType tdesign = SpeakerLayout::T4) :
        order(orderVal), sampleRate(sampleRateVal), maxBlockSize(maxBlockSizeVal),
        currentTDesign(tdesign), currentEnvironment(FREE_FIELD), isInitialized(false), convolutionSetup(false) {
        
        nSH = (order + 1) * (order + 1);
        
        audioFormatManager.registerBasicFormats();
        
        setupEnvironmentMappings();
        setupVirtualLoudspeakers();
        createDecodingMatrix();
        loadAllAmbisonicIRs();
        setupReverbConvolution();
        
        std::cout << "AmbisonicReverbProcessor initialized with partitioned convolution" << std::endl;
    }
    
    ~AmbisonicReverbProcessor() {
        delete[] vls_dirs_deg;
        delete[] dec_mat;
    }
    
    void setupEnvironmentMappings() {
        environmentNames[FREE_FIELD] = "Free-field (No Reverb)";
        environmentNames[CATHEDRAL] = "Cathedral";
        environmentNames[CHURCH] = "Church";
        environmentNames[FOREST] = "Forest";
        environmentNames[CHAMBER] = "Chamber";
        
        // Set file paths relative to your assets folder
        environmentFilePaths[CATHEDRAL] = "../assets/ambisonic_IRs/cathedral.wav";
        environmentFilePaths[CHURCH] = "../assets/ambisonic_IRs/church.wav";
        environmentFilePaths[FOREST] = "../assets/ambisonic_IRs/forest.wav";
        environmentFilePaths[CHAMBER] = "../assets/ambisonic_IRs/chamber.wav";
    }
    
    void setupVirtualLoudspeakers() {
        nVirtualSpeakers = SpeakerLayout::getSpeakerCount(currentTDesign);
        vls_dirs_deg = new float[nVirtualSpeakers * 2];
        
        auto coords = SpeakerLayout::getTDesignSphericalCoords(currentTDesign);
        for (int i = 0; i < nVirtualSpeakers; i++) {
            vls_dirs_deg[i*2] = coords[i].first;      // azimuth
            vls_dirs_deg[i*2+1] = coords[i].second;   // elevation
        }
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
    }
    
    bool loadBFormatIR(ReverbEnvironment env, const std::string& filepath) {
        juce::File audioFile(filepath);
        if (!audioFile.exists()) {
            std::cerr << "Ambisonic IR file does not exist: " << filepath << std::endl;
            return false;
        }
        
        std::unique_ptr<juce::AudioFormatReader> reader(audioFormatManager.createReaderFor(audioFile));
        if (!reader) {
            std::cerr << "Could not create reader for ambisonic IR: " << filepath << std::endl;
            return false;
        }
        
        if (reader->numChannels != 4) {
            std::cerr << "Expected 4-channel B-format, got " << reader->numChannels << " channels" << std::endl;
            return false;
        }
        
        auto lengthInSamples = static_cast<int>(reader->lengthInSamples);
        
        // LIMIT IR LENGTH BASED ON T-DESIGN TO PREVENT RESOURCE OVERLOAD
        int maxIRLength = getMaxIRLength();
        if (lengthInSamples > maxIRLength) {
            lengthInSamples = maxIRLength;
            std::cout << "Limiting IR length to " << maxIRLength << " samples for " 
                      << environmentNames[env] << " due to T-design " << nVirtualSpeakers << std::endl;
        }
        
        bFormatIRs[env].setSize(4, lengthInSamples);
        bFormatIRs[env].clear();
        
        reader->read(&bFormatIRs[env], 0, lengthInSamples, 0, true, true);
        
        std::cout << "Loaded B-format IR: " << environmentNames[env] 
                  << " (" << lengthInSamples << " samples)" << std::endl;
        
        return true;
    }
    
    void decodeBFormatToVirtualSpeakers(ReverbEnvironment env) {
        if (bFormatIRs.find(env) == bFormatIRs.end()) {
            std::cerr << "B-format IR not loaded for environment: " << environmentNames[env] << std::endl;
            return;
        }
        
        const auto& bFormatIR = bFormatIRs[env];
        int irLength = bFormatIR.getNumSamples();
        
        // Initialize virtual speaker IR storage
        virtualSpeakerIRs[env].clear();
        virtualSpeakerIRs[env].resize(nVirtualSpeakers);
        
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            virtualSpeakerIRs[env][ls].setSize(1, irLength);
            virtualSpeakerIRs[env][ls].clear();
            
            // Decode B-format to this virtual speaker using decoding matrix
            for (int sample = 0; sample < irLength; sample++) {
                float decodedSample = 0.0f;
                
                for (int ch = 0; ch < nSH && ch < 4; ch++) {  // Use min(nSH, 4) for 1st order
                    float bFormatSample = bFormatIR.getSample(ch, sample);
                    decodedSample += dec_mat[ls * nSH + ch] * bFormatSample;
                }
                
                virtualSpeakerIRs[env][ls].setSample(0, sample, decodedSample);
            }
        }
        
        std::cout << "Decoded " << environmentNames[env] << " to " << nVirtualSpeakers << " virtual speakers" << std::endl;
    }
    
    void setupReverbConvolution() {
        // Setup temp buffers
        reverbTempBuffers.resize(nVirtualSpeakers);
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            reverbTempBuffers[ls].setSize(1, maxBlockSize);
        }
        
        // Setup convolution processors
        currentReverbProcessors.clear();
        currentReverbProcessors.resize(nVirtualSpeakers);
        
        juce::dsp::ProcessSpec spec;
        spec.sampleRate = sampleRate;
        spec.maximumBlockSize = static_cast<juce::uint32>(maxBlockSize);
        spec.numChannels = 1;
        
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            currentReverbProcessors[ls] = std::make_unique<juce::dsp::Convolution>();
            currentReverbProcessors[ls]->prepare(spec);
        }
        
        // Load IRs for current environment
        updateConvolutionIRs();
        
        convolutionSetup = true;
        std::cout << "Reverb convolution processors setup complete" << std::endl;
    }
    
    void updateConvolutionIRs() {
        if (currentEnvironment == FREE_FIELD || !convolutionSetup) {
            return;
        }
        
        if (virtualSpeakerIRs.find(currentEnvironment) == virtualSpeakerIRs.end()) {
            std::cerr << "No virtual speaker IRs available for current environment" << std::endl;
            return;
        }
        
        const auto& currentVSIRs = virtualSpeakerIRs[currentEnvironment];
        
        for (int ls = 0; ls < nVirtualSpeakers && ls < currentVSIRs.size(); ls++) {
            if (currentVSIRs[ls].getNumSamples() > 0) {
                // Create a copy of the IR for JUCE convolution
                juce::AudioBuffer<float> irCopy(1, currentVSIRs[ls].getNumSamples());
                irCopy.copyFrom(0, 0, currentVSIRs[ls], 0, 0, currentVSIRs[ls].getNumSamples());
                
                // Load IR into JUCE convolution processor with partitioned convolution
                currentReverbProcessors[ls]->loadImpulseResponse(
                    std::move(irCopy),
                    sampleRate,
                    juce::dsp::Convolution::Stereo::no,
                    juce::dsp::Convolution::Trim::no,
                    juce::dsp::Convolution::Normalise::no
                );
            }
        }
        
        std::cout << "Updated convolution IRs for " << environmentNames[currentEnvironment] << std::endl;
    }
    
    void loadAllAmbisonicIRs() {
        for (auto& pair : environmentFilePaths) {
            ReverbEnvironment env = pair.first;
            const std::string& filepath = pair.second;
            
            if (loadBFormatIR(env, filepath)) {
                decodeBFormatToVirtualSpeakers(env);
            }
        }
        
        isInitialized = true;
    }
    
    void setEnvironment(ReverbEnvironment env) {
        if (env == currentEnvironment) return;
        
        currentEnvironment = env;
        updateConvolutionIRs();
        
        std::cout << "Switched to environment: " << environmentNames[env] << std::endl;
    }
    
    ReverbEnvironment getCurrentEnvironment() const {
        return currentEnvironment;
    }
    
    std::string getCurrentEnvironmentName() const {
        return environmentNames.at(currentEnvironment);
    }
    
    std::vector<std::string> getAllEnvironmentNames() const {
        std::vector<std::string> names;
        for (int i = 0; i <= CHAMBER; i++) {
            ReverbEnvironment env = static_cast<ReverbEnvironment>(i);
            names.push_back(environmentNames.at(env));
        }
        return names;
    }
    
    bool isCurrentlyFreeField() const {
        return currentEnvironment == FREE_FIELD;
    }
    
    // Apply JUCE partitioned convolution to virtual speaker signals
    void processReverbConvolution(std::vector<juce::AudioBuffer<float>>& virtualSpeakerBuffers, int numSamples) {
        if (currentEnvironment == FREE_FIELD || !isInitialized || !convolutionSetup) {
            return; // No reverb processing for free-field
        }
        
        // Get energy compensation and volume normalization
        float energyComp = getEnergyCompensationGain();
        float volumeNorm = getVolumeNormalizationGain();
        
        // Process each virtual speaker signal through its reverb convolution
        for (int ls = 0; ls < nVirtualSpeakers && ls < virtualSpeakerBuffers.size(); ls++) {
            if (ls < currentReverbProcessors.size() && currentReverbProcessors[ls]) {
                
                // Copy input to temp buffer
                reverbTempBuffers[ls].clear();
                reverbTempBuffers[ls].copyFrom(0, 0, virtualSpeakerBuffers[ls], 0, 0, numSamples);
                
                // Apply JUCE partitioned convolution
                juce::dsp::AudioBlock<float> audioBlock(
                    reverbTempBuffers[ls].getArrayOfWritePointers(), 
                    1, 
                    0, 
                    static_cast<size_t>(numSamples)
                );
                
                currentReverbProcessors[ls]->process(juce::dsp::ProcessContextReplacing<float>(audioBlock));
                
                // Apply both energy compensation and volume normalization
                float dryGain = 0.3f * energyComp * volumeNorm;
                float wetGain = 0.7f * energyComp * volumeNorm;
                
                for (int n = 0; n < numSamples; n++) {
                    float drySignal = virtualSpeakerBuffers[ls].getSample(0, n);
                    float wetSignal = reverbTempBuffers[ls].getSample(0, n);
                    virtualSpeakerBuffers[ls].setSample(0, n, dryGain * drySignal + wetGain * wetSignal);
                }
            }
        }
    }
    
    void updateTDesign(SpeakerLayout::TDesignType newTDesign) {
        if (newTDesign != currentTDesign) {
            currentTDesign = newTDesign;
            
            // Update speaker count for new T-design
            delete[] vls_dirs_deg;
            delete[] dec_mat;
            
            setupVirtualLoudspeakers();
            createDecodingMatrix();
            
            // RELOAD IRs with new length limits
            std::map<ReverbEnvironment, juce::AudioBuffer<float>> oldIRs = std::move(bFormatIRs);
            bFormatIRs.clear();
            
            for (auto& pair : environmentFilePaths) {
                ReverbEnvironment env = pair.first;
                const std::string& filepath = pair.second;
                if (loadBFormatIR(env, filepath)) {
                    decodeBFormatToVirtualSpeakers(env);
                }
            }
            
            setupReverbConvolution();
            
            std::cout << "Updated reverb processor to " << SpeakerLayout::getTDesignName(newTDesign) 
                      << " with IR length limits" << std::endl;
        }
    }
};