#include <iostream>
#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <algorithm>

// AlloLib includes
#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Spherical.hpp"
#include "al/scene/al_DynamicScene.hpp"
#include "al/ui/al_ControlGUI.hpp"
#include "al/ui/al_PickableManager.hpp"
#include "al/io/al_File.hpp"

// SAF includes
#include "saf.h"
#include "saf_hrir.h"
#include "saf_hoa.h"
#include "saf_utilities.h"
#include "saf_sofa_reader.h"  // SOFA reader for custom HRTF loading

// Asset loading
#include "al_ext/assets3d/al_Asset.hpp"

// JUCE includes
#include "juce_core/juce_core.h"
#include "juce_audio_basics/juce_audio_basics.h"
#include "juce_dsp/juce_dsp.h"
#include "juce_audio_formats/juce_audio_formats.h"

// Native File Dialog Extended
#include <nfd.h>

// Our modular components
#include "audio/speakerLayout.hpp"
#include "audio/ambisonicEncoder.hpp"
#include "audio/ambisonicBinauralDecoder.hpp"
#include "audio/AmbisonicReverbProcessor.hpp"
#include "headTrackingRotation.hpp"

using namespace al;

// ===================
// SOFA File Data Structure (defined in ambisonicBinauralDecoder.hpp)
// ===================

// ===================
// SOFA File Loading Function
// ===================
bool loadAndProcessSOFAFile(const std::string& filepath, HRIRData& hrirData) {
    std::cout << "Loading SOFA file: " << filepath << std::endl;
    
    #ifdef SAF_ENABLE_SOFA_READER_MODULE
    
    SAF_SOFA_ERROR_CODES error;
    saf_sofa_container sofa;
    
    char* filepath_cstr = const_cast<char*>(filepath.c_str());
    error = saf_sofa_open(&sofa, filepath_cstr, SAF_SOFA_READER_OPTION_DEFAULT);
    
    if (error != SAF_SOFA_OK) {
        std::cerr << "Error: Could not open SOFA file. Error code: " << error << std::endl;
        return false;
    }
    
    if (sofa.nReceivers != 2) {
        std::cerr << "Error: SOFA file must have 2 receivers (binaural), found: " 
                  << sofa.nReceivers << std::endl;
        saf_sofa_close(&sofa);
        return false;
    }
    
    std::cout << "SOFA file opened successfully:" << std::endl;
    std::cout << "  Sources: " << sofa.nSources << std::endl;
    std::cout << "  IR length: " << sofa.DataLengthIR << std::endl;
    std::cout << "  Sample rate: " << sofa.DataSamplingRate << " Hz" << std::endl;
    
    hrirData.hrir_fs = (int)sofa.DataSamplingRate;
    hrirData.hrir_len = sofa.DataLengthIR;
    hrirData.N_hrir_dirs = sofa.nSources;
    
    size_t hrir_data_size = hrirData.N_hrir_dirs * 2 * hrirData.hrir_len * sizeof(float);
    hrirData.hrirs = (float*)malloc(hrir_data_size);
    memcpy(hrirData.hrirs, sofa.DataIR, hrir_data_size);
    
    hrirData.hrir_dirs_deg = (float*)malloc(hrirData.N_hrir_dirs * 2 * sizeof(float));
    for (int i = 0; i < hrirData.N_hrir_dirs; i++) {
        hrirData.hrir_dirs_deg[i * 2] = sofa.SourcePosition[i * 3];
        hrirData.hrir_dirs_deg[i * 2 + 1] = sofa.SourcePosition[i * 3 + 1];
    }
    
    for (int i = 0; i < hrirData.N_hrir_dirs; i++) {
        float azi = hrirData.hrir_dirs_deg[i * 2];
        if (azi > 180.0f) {
            hrirData.hrir_dirs_deg[i * 2] = azi - 360.0f;
        }
    }
    
    std::cout << "Estimating ITDs..." << std::endl;
    hrirData.itds_s = (float*)malloc(hrirData.N_hrir_dirs * sizeof(float));
    estimateITDs(hrirData.hrirs, hrirData.N_hrir_dirs, 
                 hrirData.hrir_len, hrirData.hrir_fs, hrirData.itds_s);
    
    saf_sofa_close(&sofa);
    
    std::cout << "SOFA file loaded successfully!" << std::endl;
    return true;
    
    #else
    std::cerr << "Error: SAF SOFA reader module not enabled!" << std::endl;
    return false;
    #endif
}

// ===================
// Simplified Parameter Smoother
// ===================
class ParameterSmoother {
private:
    float currentValue;
    float targetValue;
    float smoothingCoeff;
    bool isSmoothing;
    
public:
    ParameterSmoother(float initialValue = 0.0f, float smoothingTimeMs = 10.0f, float sampleRate = 44100.0f) 
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

Vec3f sphericalToCartesian(float azimuthDeg, float elevationDeg, float radius) {
    float azimuth_rad = azimuthDeg * M_PI / 180.0f;
    float elevation_rad = elevationDeg * M_PI / 180.0f;
    
    float x = radius * cos(elevation_rad) * sin(azimuth_rad);
    float y = radius * sin(elevation_rad);
    float z = radius * cos(elevation_rad) * (-cos(azimuth_rad));
    
    return Vec3f(x, y, z);
}

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
    
    elevationDeg = asin(y / radius) * 180.0f / M_PI;
    azimuthDeg = atan2(x, -z) * 180.0f / M_PI;
    
    while (azimuthDeg > 180.0f) azimuthDeg -= 360.0f;
    while (azimuthDeg < -180.0f) azimuthDeg += 360.0f;
}

// ===================
// Audio Source Class
// ===================
class AudioSource {
public:
    int sourceId;
    std::string filename;
    Vec3f position;
    float azimuth = 0.0f;
    float elevation = 0.0f;
    float gain = 1.0f;
    
    juce::AudioBuffer<float> audioBuffer;
    int currentSamplePosition = 0;
    bool isLooping = true;
    bool isActive = true;
    bool isMuted = false;
    bool isSoloed = false;
    
    ParameterSmoother gainSmoother;
    
    Color color;
    bool isSelected = false;
    
    AudioSource(int id, const std::string& file, float sampleRate) 
        : sourceId(id), filename(file),
          gainSmoother(1.0f, 15.0f, sampleRate) {
        
        generateUniqueColor();
        setPosition(al::rnd::uniform(-180.0f, 180.0f), al::rnd::uniform(-45.0f, 45.0f));
    }
    
    void generateUniqueColor() {
        float hue = (sourceId * 137.508f) / 360.0f;
        hue = hue - floor(hue);
        
        float saturation = 0.7f + 0.3f * (sourceId % 3) / 3.0f;
        float value = 0.8f + 0.2f * ((sourceId + 1) % 2);

        float r, g, b;
        float h = hue * 360.0f;
        int i = int(h / 60.0f) % 6;
        float f = (h / 60.0f) - i;
        float p = value * (1.0f - saturation);
        float q = value * (1.0f - f * saturation);
        float t = value * (1.0f - (1.0f - f) * saturation);
        switch (i) {
            case 0: r = value; g = t; b = p; break;
            case 1: r = q; g = value; b = p; break;
            case 2: r = p; g = value; b = t; break;
            case 3: r = p; g = q; b = value; break;
            case 4: r = t; g = p; b = value; break;
            case 5: default: r = value; g = p; b = q; break;
        }
        color = Color(r, g, b, 1.0f);
    }
    
    bool loadAudioFile(const std::string& filepath, double targetSampleRate) {
        juce::AudioFormatManager formatManager;
        formatManager.registerBasicFormats();
        
        juce::File audioFile(filepath);
        if (!audioFile.exists()) {
            std::cerr << "Audio file does not exist: " << filepath << std::endl;
            return false;
        }
        
        std::unique_ptr<juce::AudioFormatReader> reader(formatManager.createReaderFor(audioFile));
        if (!reader) {
            std::cerr << "Could not create reader for: " << filepath << std::endl;
            return false;
        }
        
        auto lengthInSamples = static_cast<int>(reader->lengthInSamples);
        auto sampleRateRatio = targetSampleRate / reader->sampleRate;
        auto targetLength = static_cast<int>(lengthInSamples * sampleRateRatio);
        
        audioBuffer.setSize(1, targetLength);
        audioBuffer.clear();
        
        if (reader->numChannels == 1) {
            if (reader->sampleRate == targetSampleRate) {
                reader->read(&audioBuffer, 0, lengthInSamples, 0, true, false);
            } else {
                juce::AudioBuffer<float> tempBuffer(1, lengthInSamples);
                reader->read(&tempBuffer, 0, lengthInSamples, 0, true, false);
                
                for (int i = 0; i < targetLength; ++i) {
                    float sourceIndex = i / sampleRateRatio;
                    int index1 = static_cast<int>(sourceIndex);
                    int index2 = std::min(index1 + 1, lengthInSamples - 1);
                    float fraction = sourceIndex - index1;
                    
                    float sample1 = tempBuffer.getSample(0, index1);
                    float sample2 = tempBuffer.getSample(0, index2);
                    float interpolatedSample = sample1 + fraction * (sample2 - sample1);
                    
                    audioBuffer.setSample(0, i, interpolatedSample);
                }
            }
        } else {
            juce::AudioBuffer<float> tempBuffer(reader->numChannels, lengthInSamples);
            reader->read(&tempBuffer, 0, lengthInSamples, 0, true, true);
            
            for (int i = 0; i < targetLength; ++i) {
                float sourceIndex = i / sampleRateRatio;
                int index1 = static_cast<int>(sourceIndex);
                int index2 = std::min(index1 + 1, lengthInSamples - 1);
                float fraction = sourceIndex - index1;
                
                float monoSample = 0.0f;
                for (int ch = 0; ch < reader->numChannels; ++ch) {
                    float sample1 = tempBuffer.getSample(ch, index1);
                    float sample2 = tempBuffer.getSample(ch, index2);
                    float interpolatedSample = sample1 + fraction * (sample2 - sample1);
                    monoSample += interpolatedSample;
                }
                monoSample /= reader->numChannels;
                
                audioBuffer.setSample(0, i, monoSample);
            }
        }
        
        filename = audioFile.getFileNameWithoutExtension().toStdString();
        currentSamplePosition = 0;
        
        std::cout << "Loaded audio file: " << filename 
                  << " (" << audioBuffer.getNumSamples() << " samples at " 
                  << targetSampleRate << " Hz)" << std::endl;
        
        return true;
    }
    
    void setPosition(float azimuthDeg, float elevationDeg) {
        azimuth = azimuthDeg;
        elevation = elevationDeg;
        position = sphericalToCartesian(azimuthDeg, elevationDeg, 1.0f);
    }

    void setGain(float newGain) {
        gain = newGain;
        gainSmoother.setTarget(newGain);
    }

    float getNextSample() {
        if (!isActive || audioBuffer.getNumSamples() == 0) {
            return 0.0f;
        }
        
        float sample = audioBuffer.getSample(0, currentSamplePosition);
        sample *= gainSmoother.getNextValue();
        
        currentSamplePosition++;
        if (currentSamplePosition >= audioBuffer.getNumSamples()) {
            isActive = false;
            return 0.0f;
        }
        
        return sample;
    }

    void reset() {
        currentSamplePosition = 0;
        isActive = true;
        gainSmoother.reset(gain);
    }

    void updateSmoothingParameters(float sampleRate) {
        gainSmoother.setSmoothingTime(15.0f, sampleRate);
    }

    Vec3f getPosition() {
        return position;
    }
};

// ===================
// Multi-Source Audio Manager
// ===================
class MultiSourceManager {
private:
    std::vector<std::unique_ptr<AudioSource>> sources;
    int nextSourceId = 0;
    double sampleRate;
    
public:
    bool isPlaying = false;
    bool isPaused = false;
    bool sessionLooping = true;
    
    MultiSourceManager(double sr) : sampleRate(sr) {}
    
    ~MultiSourceManager() {
        sources.clear();
    }
    
    int addAudioFile(const std::string& filepath) {
        auto source = std::make_unique<AudioSource>(nextSourceId++, filepath, sampleRate);
        
        if (!source->loadAudioFile(filepath, sampleRate)) {
            std::cerr << "Failed to load audio file: " << filepath << std::endl;
            return -1;
        }
        
        source->updateSmoothingParameters(sampleRate);
        int sourceId = source->sourceId;
        sources.push_back(std::move(source));
        
        std::cout << "Added audio source " << sourceId << ": " << filepath << std::endl;
        return sourceId;
    }
    
    std::vector<int> addMultipleAudioFiles(const std::vector<std::string>& filepaths) {
        std::vector<int> sourceIds;
        
        for (const auto& filepath : filepaths) {
            int id = addAudioFile(filepath);
            if (id >= 0) {
                sourceIds.push_back(id);
            }
        }
        
        std::cout << "Added " << sourceIds.size() << " audio sources" << std::endl;
        return sourceIds;
    }

    
    void removeSource(int sourceId) {
        sources.erase(
            std::remove_if(sources.begin(), sources.end(),
                [sourceId](const std::unique_ptr<AudioSource>& source) {
                    return source->sourceId == sourceId;
                }),
            sources.end()
        );
    }
    
    void play() {
        isPlaying = true;
        isPaused = false;
        
        for (auto& source : sources) {
            if (!source->isActive && source->audioBuffer.getNumSamples() > 0) {
                source->isActive = true;
            }
        }
        
        std::cout << "Playing " << sources.size() << " audio sources" << std::endl;
    }
    
    void pause() {
        isPaused = true;
        std::cout << "Paused playback" << std::endl;
    }
    
    void stop() {
        isPlaying = false;
        isPaused = false;
        for (auto& source : sources) {
            source->currentSamplePosition = 0;
            source->isActive = true;
        }
        std::cout << "Stopped playback" << std::endl;
    }

    // In MultiSourceManager class, add:
int getLongestDuration() const {
    int maxDuration = 0;
    for (const auto& source : sources) {
        int duration = source->audioBuffer.getNumSamples();
        if (duration > maxDuration) {
            maxDuration = duration;
        }
    }
    return maxDuration;
}

int getCurrentPlaybackPosition() const {
    // Return the position of the first source (or you could track a global position)
    if (!sources.empty() && sources[0]->audioBuffer.getNumSamples() > 0) {
        return sources[0]->currentSamplePosition;
    }
    return 0;
}

void seekToPosition(int samplePosition) {
    for (auto& source : sources) {
        if (samplePosition < source->audioBuffer.getNumSamples()) {
            source->currentSamplePosition = samplePosition;
            source->isActive = true;
        } else {
            source->isActive = false;
        }
    }
}
    
    void setSourcePosition(int sourceId, float azimuth, float elevation) {
        for (auto& source : sources) {
            if (source->sourceId == sourceId) {
                source->setPosition(azimuth, elevation);
                break;
            }
        }
    }
    
    void setSourceGain(int sourceId, float gain) {
        for (auto& source : sources) {
            if (source->sourceId == sourceId) {
                source->setGain(gain);
                break;
            }
        }
    }
    
    AudioSource* getSource(int sourceId) {
        for (auto& source : sources) {
            if (source->sourceId == sourceId) {
                return source.get();
            }
        }
        return nullptr;
    }
    
    const std::vector<std::unique_ptr<AudioSource>>& getSources() const {
        return sources;
    }
    
    bool hasSoloedSources() const {
        for (const auto& source : sources) {
            if (source->isSoloed) return true;
        }
        return false;
    }
    
    void checkAndRestartSession() {
        if (!sessionLooping) return;
        
        bool allFinished = true;
        for (auto& source : sources) {
            if (source->isActive && source->audioBuffer.getNumSamples() > 0) {
                allFinished = false;
                break;
            }
        }
        
        if (allFinished && isPlaying && !isPaused) {
            for (auto& source : sources) {
                source->reset();
            }
        }
    }
    
    void processAudio(AmbisonicEncoder* encoder, int numSamples) {
        if (!encoder) return;
        
        encoder->clearSignals();
        
        std::vector<float> sourceBuffer(numSamples);
        int activeSources = 0;
        bool anySoloed = hasSoloedSources();
        
        for (auto& source : sources) {
            if (!source->isActive || source->audioBuffer.getNumSamples() == 0) continue;
            
            if (source->isMuted || (anySoloed && !source->isSoloed)) continue;
            
            activeSources++;
            if (activeSources > 8) break;
            
            for (int i = 0; i < numSamples; i++) {
                sourceBuffer[i] = source->getNextSample();
            }
            
            Vec3f pos = source->getPosition();
            float az, el, radius;
            cartesianToSpherical(pos, az, el, radius);
            
            encoder->encodeAndAccumulate(sourceBuffer.data(), numSamples, -az, el);
        }
        
        checkAndRestartSession();
    }
    
    bool getIsPlaying() const { return isPlaying && !isPaused; }
    bool getIsPaused() const { return isPaused; }
    size_t getSourceCount() const { return sources.size(); }
    
    void updateSmoothingParameters(double newSampleRate) {
        sampleRate = newSampleRate;
        for (auto& source : sources) {
            source->updateSmoothingParameters(sampleRate);
        }
    }
};

// ===================
// Pickable Classes
// ===================
class SelectablePickable : public PickableBB {
public:
    bool selected = false;
    int sourceId = -1;
    Color sourceColor = Color(0.2, 0.8, 0.4);

    SelectablePickable(int id = -1) : sourceId(id) {}

    bool onEvent(PickEvent e, Hit h) override {
        bool handled = PickableBB::onEvent(e, h);
        if (e.type == Pick && h.hit) {
            selected = true;
        }
        return handled;
    }
    
    void setSourceColor(const Color& color) {
        sourceColor = color;
    }
};

// ===================
// Main Application Class
// ===================
struct MyApp : public App {
    DynamicScene scene;
    PickableManager pickableManager;
    ControlGUI gui;

    Scene* headScene = nullptr;
    std::vector<Mesh> headMeshes;
    Vec3f headSceneMin, headSceneMax, headSceneCenter;
    
    SpeakerLayout::TDesignType currentTDesign = SpeakerLayout::T4;
    
    VAOMesh sphereMesh;
    std::vector<Vec3f> virtualSpeakerPositions;

    Scene* speakerScene = nullptr;
    std::vector<Mesh> speakerMeshes;
    Vec3f speakerSceneMin, speakerSceneMax, speakerSceneCenter;
    
    std::unique_ptr<MultiSourceManager> sourceManager;
    std::vector<std::unique_ptr<SelectablePickable>> sourcePickables;

    std::unique_ptr<AmbisonicReverbProcessor> reverbProcessor;
    
    // Head tracking and scene rotation
    std::unique_ptr<HeadTrackingRotation> headTracker;
    
    // Visual head rotation state
    float visualHeadYaw = 0.0f;
    float visualHeadPitch = 0.0f;
    float visualHeadRoll = 0.0f;
    
    int selectedSourceId = -1;
    bool pickablesUpdatingParameters = false;
    Pose fixedListenerPose{Vec3f(0, 0, 0)};
    float visualRadius = 3.0f;
    
    AmbisonicEncoder* ambiEncoder = nullptr;
    AmbisonicBinauralDecoder* ambiDecoder = nullptr;
    
    float* leftOutputBuffer = nullptr;
    float* rightOutputBuffer = nullptr;
    int bufferSize = 0;
    
    juce::AudioFormatManager audioFormatManager;

    std::unique_ptr<juce::dsp::Convolution> headphoneEQ_L;
    std::unique_ptr<juce::dsp::Convolution> headphoneEQ_R;
    int currentHeadphoneEQIndex = 0;
    
    // SOFA file loading
    HRIRData customHRIRData;
    std::string sofaFilePath = "";
    bool sofaFileLoaded = false;
    bool useDefaultHRTF = true;
    
    double lastParameterUpdateTime = 0.0;
    const double parameterUpdateInterval = 0.005;
    
    double getCurrentTime() {
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

    void enforceSourceLimit() {
        const int MAX_SOURCES = 20;
        
        if (sourceManager->getSourceCount() > MAX_SOURCES) {
            std::cout << "Warning: Too many sources loaded (" << sourceManager->getSourceCount() 
                      << "). Performance may be affected. Consider using fewer sources." << std::endl;
        }
    }

    void loadHeadModel() {
        std::string fileName = "../assets/models/head.obj";
        
        headScene = Scene::import(fileName);
        if (!headScene) {
            std::cout << "Error: Could not load head model from " << fileName << std::endl;
            return;
        }
        
        headScene->getBounds(headSceneMin, headSceneMax);
        headSceneCenter = (headSceneMin + headSceneMax) / 2.0f;
        
        headMeshes.resize(headScene->meshes());
        for (int i = 0; i < headScene->meshes(); i++) {
            headScene->mesh(i, headMeshes[i]);
        }
        
        std::cout << "Successfully loaded head model with " << headMeshes.size() << " meshes" << std::endl;
    }

    void loadSpeakerModel() {
        std::string fileName = "../assets/models/speaker.obj";
        
        speakerScene = Scene::import(fileName);
        if (!speakerScene) {
            std::cout << "Error: Could not load speaker model from " << fileName << std::endl;
            return;
        }
        
        speakerScene->getBounds(speakerSceneMin, speakerSceneMax);
        speakerSceneCenter = (speakerSceneMin + speakerSceneMax) / 2.0f;
        
        speakerMeshes.resize(speakerScene->meshes());
        for (int i = 0; i < speakerScene->meshes(); i++) {
            speakerScene->mesh(i, speakerMeshes[i]);
        }
        
        std::cout << "Successfully loaded speaker model" << std::endl;
    }

    void createWireframeSphere(Mesh& mesh, float radius, int numLongitudes = 12, int numLatitudes = 8) {
        mesh.reset();
        mesh.primitive(Mesh::LINES);

        for (int i = 0; i < numLongitudes; ++i) {
            float theta = 2.0f * M_PI * i / numLongitudes;

            for (int j = 0; j < numLatitudes; ++j) {
                float phi1 = M_PI * (j    ) / numLatitudes;
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

        for (int j = 1; j < numLatitudes; ++j) {
            float phi = M_PI * j / numLatitudes;
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
        virtualSpeakerPositions = SpeakerLayout::getTDesignPositions(currentTDesign, visualRadius);
        std::cout << "Virtual speakers setup: " << SpeakerLayout::getTDesignName(currentTDesign) 
                  << " with " << virtualSpeakerPositions.size() << " speakers" << std::endl;
    }

    // NEW: Open native file dialog and load selected files
// NEW: Open native file dialog and load selected files
// NEW: Open native file dialog and load selected files
// NEW: Open native file dialog and load selected files (with multiple selection support)
void openFileDialogAndLoadAudio() {
    const nfdpathset_t *outPaths = nullptr;
    nfdfilteritem_t filterItem[1] = { { "Audio Files", "wav,mp3,flac,ogg,aif,aiff,m4a" } };
    nfdresult_t result = NFD_OpenDialogMultiple(&outPaths, filterItem, 1, nullptr);
    
    if (result == NFD_OKAY) {
        nfdpathsetsize_t numPaths;
        NFD_PathSet_GetCount(outPaths, &numPaths);
        
        std::vector<std::string> selectedFiles;
        std::cout << "User selected " << numPaths << " file(s)" << std::endl;
        
        for (nfdpathsetsize_t i = 0; i < numPaths; i++) {
            nfdchar_t *path = nullptr;
            NFD_PathSet_GetPath(outPaths, i, &path);
            if (path) {
                selectedFiles.push_back(std::string(path));
                std::cout << "  File " << i << ": " << path << std::endl;
                NFD_PathSet_FreePath(path);
            }
        }
        
        // Load all selected files
        if (!selectedFiles.empty()) {
            int loadedCount = 0;
            for (const auto& filePath : selectedFiles) {
                int sourceId = sourceManager->addAudioFile(filePath);
                if (sourceId >= 0) {
                    createPickableForSource(sourceId);
                    loadedCount++;
                }
            }
            std::cout << "Successfully loaded " << loadedCount << " out of " 
                      << selectedFiles.size() << " file(s)" << std::endl;
        }
        
        NFD_PathSet_Free(outPaths);
        
    } else if (result == NFD_CANCEL) {
        std::cout << "User cancelled file selection" << std::endl;
    } else {
        std::cerr << "Error opening file dialog: " << NFD_GetError() << std::endl;
    }
}

// SOFA file loading
void openFileDialogAndLoadSOFA() {
    nfdchar_t *outPath = nullptr;
    nfdfilteritem_t filterItem[1] = { {"SOFA Files", "sofa"} };
    nfdresult_t result = NFD_OpenDialog(&outPath, filterItem, 1, nullptr);
    
    if (result == NFD_OKAY) {
        sofaFilePath = std::string(outPath);
        std::cout << "Selected SOFA file: " << sofaFilePath << std::endl;
        
        if (loadAndProcessSOFAFile(sofaFilePath, customHRIRData)) {
            sofaFileLoaded = true;
            useDefaultHRTF = false;  // Uncheck the "Use Default HRTF set" box
            std::cout << "SOFA file loaded successfully!" << std::endl;
            
            // Apply custom HRIRs to decoder
            if (ambiDecoder) {
                ambiDecoder->loadCustomHRIRs(customHRIRData);
            }
        } else {
            std::cout << "Failed to load SOFA file" << std::endl;
            sofaFileLoaded = false;
            sofaFilePath = "";
        }
        
        NFD_FreePath(outPath);
    } else if (result == NFD_CANCEL) {
        std::cout << "User cancelled SOFA file selection" << std::endl;
    } else {
        std::cout << "Error: " << NFD_GetError() << std::endl;
    }
}
    
    void createPickableForSource(int sourceId) {
        AudioSource* source = sourceManager->getSource(sourceId);
        if (!source) return;
        
        static bool sphereMeshCreated = false;
        static VAOMesh sharedSphereMesh;
        
        if (!sphereMeshCreated) {
            addSphere(sharedSphereMesh, 0.15);
            sharedSphereMesh.primitive(Mesh::TRIANGLES);
            sharedSphereMesh.update();
            sphereMeshCreated = true;
        }
        
        auto pickable = std::make_unique<SelectablePickable>(sourceId);
        pickable->set(sharedSphereMesh);
        pickable->setSourceColor(source->color);
        pickable->pose = Pose(source->position * visualRadius);
        
        pickableManager << pickable.get();
        sourcePickables.push_back(std::move(pickable));
        
        std::cout << "Created pickable for source " << sourceId << std::endl;
    }
    
    void removeSource(int sourceId) {
        sourceManager->removeSource(sourceId);
        
        sourcePickables.erase(
            std::remove_if(sourcePickables.begin(), sourcePickables.end(),
                [sourceId](const std::unique_ptr<SelectablePickable>& pickable) {
                    return pickable->sourceId == sourceId;
                }),
            sourcePickables.end()
        );
        
        std::cout << "Removed source " << sourceId << std::endl;
    }

    void loadHeadphoneEQ(int eqIndex) {
        if (eqIndex == 0) {
            currentHeadphoneEQIndex = 0;
            std::cout << "Headphone EQ: OFF" << std::endl;
            return;
        }
        
        const char* headphoneEQFilenames[] = {
            "",
            "AKG-K141MK2", "AKG-K240DF", "AKG-K240MK2", "AKG-K271MK2",
            "AKG-K271STUDIO", "AKG-K601", "AKG-K701", "AKG-K702",
            "AKG-K1000-Closed", "AKG-K1000-Open", "AudioTechnica-ATH-M50",
            "Beyerdynamic-DT250", "Beyerdynamic-DT770PRO-250Ohms",
            "Beyerdynamic-DT880", "Beyerdynamic-DT990PRO", "Presonus-HD7",
            "Sennheiser-HD430", "Sennheiser-HD480", "Sennheiser-HD560ovationII",
            "Sennheiser-HD565ovation", "Sennheiser-HD600", "Sennheiser-HD650",
            "SHURE-SRH940"
        };
        
        std::string filename = std::string(headphoneEQFilenames[eqIndex]) + ".wav";
        std::string filepath = "../assets/Headphone_EQ/" + filename;
        
        juce::File eqFile(filepath);
        if (!eqFile.exists()) {
            std::cerr << "Headphone EQ file not found: " << filepath << std::endl;
            return;
        }
        
        std::unique_ptr<juce::AudioFormatReader> reader(audioFormatManager.createReaderFor(eqFile));
        if (!reader || reader->numChannels != 2) {
            std::cerr << "Invalid headphone EQ file" << std::endl;
            return;
        }
        
        int irLength = static_cast<int>(reader->lengthInSamples);
        
        juce::AudioBuffer<float> stereoIR(2, irLength);
        reader->read(&stereoIR, 0, irLength, 0, true, true);
        
        juce::AudioBuffer<float> irLeft(1, irLength);
        juce::AudioBuffer<float> irRight(1, irLength);
        irLeft.copyFrom(0, 0, stereoIR, 0, 0, irLength);
        irRight.copyFrom(0, 0, stereoIR, 1, 0, irLength);
        
        headphoneEQ_L->loadImpulseResponse(
            std::move(irLeft), 
            reader->sampleRate,
            juce::dsp::Convolution::Stereo::no,
            juce::dsp::Convolution::Trim::no,
            juce::dsp::Convolution::Normalise::no
        );
        
        headphoneEQ_R->loadImpulseResponse(
            std::move(irRight), 
            reader->sampleRate,
            juce::dsp::Convolution::Stereo::no,
            juce::dsp::Convolution::Trim::no,
            juce::dsp::Convolution::Normalise::no
        );
        
        currentHeadphoneEQIndex = eqIndex;
        std::cout << "Loaded headphone EQ: " << headphoneEQFilenames[eqIndex] << std::endl;
    }
    
    void onCreate() override {
        double sampleRate = audioIO().framesPerSecond();
        int frameSize = audioIO().framesPerBuffer();
        
        audioFormatManager.registerBasicFormats();
        
        // Initialize NFD
        NFD_Init();
        
        sourceManager = std::make_unique<MultiSourceManager>(sampleRate);
        
        // Initialize head tracking
        headTracker = std::make_unique<HeadTrackingRotation>(
            SpeakerLayout::getRecommendedOrder(currentTDesign), 
            9000  // OSC port
        );
        headTracker->initOSC();
        
        ambiEncoder = new AmbisonicEncoder(1, frameSize, sampleRate);
        ambiDecoder = new AmbisonicBinauralDecoder(1, sampleRate, frameSize, currentTDesign);
        reverbProcessor = std::make_unique<AmbisonicReverbProcessor>(1, sampleRate, frameSize, currentTDesign);
        
        headphoneEQ_L = std::make_unique<juce::dsp::Convolution>();
        headphoneEQ_R = std::make_unique<juce::dsp::Convolution>();

        juce::dsp::ProcessSpec eqSpec;
        eqSpec.sampleRate = sampleRate;
        eqSpec.maximumBlockSize = static_cast<juce::uint32>(frameSize);
        eqSpec.numChannels = 1;

        headphoneEQ_L->prepare(eqSpec);
        headphoneEQ_R->prepare(eqSpec);

        std::cout << "Headphone EQ processors initialized" << std::endl;
        
        createWireframeSphere(sphereMesh, visualRadius, 24, 16);
        sphereMesh.primitive(Mesh::LINES);
        sphereMesh.update();

        setupVirtualSpeakerPositions();
        loadHeadModel();
        loadSpeakerModel();
        
        imguiInit();
        
        gui.init(5, 5, false);
        gui.setTitle("Multi-Source Ambisonics Control");
        
        bufferSize = audioIO().framesPerBuffer();
        leftOutputBuffer = new float[bufferSize];
        rightOutputBuffer = new float[bufferSize];
       
        nav().pos(0, 0, 8);
        nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0));
       
        std::cout << "Multi-Source Ambisonics-to-Binaural Renderer:" << std::endl;
        std::cout << "  1. Click 'Load Audio Files' to browse for audio files" << std::endl;
        std::cout << "  2. Click and drag sound sources to move them" << std::endl;
        std::cout << "  3. Use play/pause controls for synchronized playback" << std::endl;
        std::cout << "  4. Select different T-design layouts for quality/performance balance" << std::endl;
        std::cout << "  5. Choose acoustic environments for ambisonic reverb" << std::endl;
    }

    void updateAmbisonicOrderSafely(SpeakerLayout::TDesignType newTDesign) {
        audioIO().stop();

        if (newTDesign == currentTDesign) {
            audioIO().start();
            return;
        }

        std::cout << "Safely updating T-design to: " << SpeakerLayout::getTDesignName(newTDesign) << std::endl;

        double sampleRate = audioIO().framesPerSecond();
        int frameSize = audioIO().framesPerBuffer();
        int newOrder = SpeakerLayout::getRecommendedOrder(newTDesign);

        try {
            delete ambiEncoder;
            delete ambiDecoder;

            currentTDesign = newTDesign;
            ambiEncoder = new AmbisonicEncoder(newOrder, frameSize, sampleRate);
            ambiDecoder = new AmbisonicBinauralDecoder(newOrder, sampleRate, frameSize, currentTDesign);

            reverbProcessor->updateTDesign(newTDesign);
            
            // UPDATE HEAD TRACKER MAX ORDER
            bool wasEnabled = headTracker->isEnabled();
            int oscPort = headTracker->getOSCPort();
            headTracker = std::make_unique<HeadTrackingRotation>(newOrder, oscPort);
            headTracker->initOSC();
            headTracker->setEnabled(wasEnabled);
            
            sourceManager->updateSmoothingParameters(sampleRate);
            setupVirtualSpeakerPositions();
            
            std::cout << "Successfully updated to " << SpeakerLayout::getTDesignName(newTDesign) 
                      << " (order " << newOrder << ")" << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "Error updating T-design: " << e.what() << std::endl;
        }

        audioIO().start();
    }
   
    Vec3f constrainToSphere(const Vec3f& point) {
        float radius = point.mag();
        if (radius < 0.0001f)
            return Vec3f(0, 0, -visualRadius);
        return point.normalized() * visualRadius;
    }
   
    void onAnimate(double dt) override {
        navControl().active(!gui.usingInput());
       
        if (!gui.usingInput()) {
            pickablesUpdatingParameters = true;
           
            for (auto& pickable : sourcePickables) {
                Vec3f pos = pickable->pose.get().pos();
                Vec3f constrained = constrainToSphere(pos);
                pickable->pose.set(Pose(constrained));
               
                float az, el, radius;
                cartesianToSpherical(constrained, az, el, radius);
               
                AudioSource* source = sourceManager->getSource(pickable->sourceId);
                if (source) {
                    source->setPosition(az, el);
                }
            }
           
            pickablesUpdatingParameters = false;
        }
    }

    void onDraw(Graphics &g) override {
        g.clear(0);
        gl::depthTesting(true);
        
        g.lineWidth(2.0);
        Mesh axes;
        axes.primitive(Mesh::LINES);
        
        g.color(1, 0, 0);
        axes.vertex(0, 0, 0);
        axes.vertex(1, 0, 0);
        
        g.color(0, 1, 0);
        axes.vertex(0, 0, 0);
        axes.vertex(0, 1, 0);
        
        g.color(0, 0, 1);
        axes.vertex(0, 0, 0);
        axes.vertex(0, 0, 1);
        
        g.draw(axes);
        
        if (headScene && !headMeshes.empty()) {
            g.pushMatrix();
            g.translate(0, 0, 0);
            
            // GET CURRENT HEAD ORIENTATION
            headTracker->getOrientation(visualHeadYaw, visualHeadPitch, visualHeadRoll);
            
            // APPLY HEAD ROTATIONS (in correct order: Yaw -> Pitch -> Roll)
            g.rotate(visualHeadYaw, 0, 1, 0);      // Yaw around Y-axis
            g.rotate(visualHeadPitch, 1, 0, 0);    // Pitch around X-axis  
            g.rotate(visualHeadRoll, 0, 0, 1);     // Roll around Z-axis
            
            g.rotate(180, 0, 1, 0);  // Base rotation to face forward
            
            float headScale = 0.5f;
            float maxDimension = std::max({
                headSceneMax.x - headSceneMin.x,
                headSceneMax.y - headSceneMin.y, 
                headSceneMax.z - headSceneMin.z
            });
            if (maxDimension > 0) {
                headScale = 1.5f / maxDimension;
            }
            
            g.scale(headScale);
            g.translate(-headSceneCenter);
            g.color(0.3, 0.25, 0.2, 1.0);
            g.lighting(true);
            
            for (const auto& mesh : headMeshes) {
                g.draw(mesh);
            }
            
            g.popMatrix();
        }
        
        g.depthMask(false);
        g.color(1.0, 1.0, 1.0, 1.0);
        g.lineWidth(3.5);
        g.polygonMode(GL_LINE);
        g.draw(sphereMesh);
        g.depthMask(true);
        
        g.polygonMode(GL_FILL);
        for (int i = 0; i < virtualSpeakerPositions.size(); i++) {
            Vec3f speakerPos = virtualSpeakerPositions[i];
            
            g.pushMatrix();
            g.translate(speakerPos);
            
            Vec3f eye = speakerPos;
            Vec3f center = Vec3f(0, 0, 0);
            Vec3f up = Vec3f(0, 1, 0);
            
            Vec3f forward = (center - eye).normalize();
            Vec3f right = cross(forward, up).normalize();
            up = cross(right, forward).normalize();
            
            float matrix[16] = {
                right.x,   right.y,   right.z,   0,
                up.x,      up.y,      up.z,      0,
                -forward.x, -forward.y, -forward.z, 0,
                0,         0,         0,         1
            };
            
            g.multModelMatrix(Matrix4f(matrix));
            g.rotate(270, 0, 1, 0);
            g.rotate(90, 1, 0, 0);

            if (speakerScene && !speakerMeshes.empty()) {
                float speakerScale = 0.15f;
                float maxDimension = std::max({
                    speakerSceneMax.x - speakerSceneMin.x,
                    speakerSceneMax.y - speakerSceneMin.y, 
                    speakerSceneMax.z - speakerSceneMin.z
                });
                if (maxDimension > 0) {
                    speakerScale = 0.4f / maxDimension;
                }
                
                g.scale(speakerScale);
                g.translate(-speakerSceneCenter);
                g.color(0.9, 0.9, 0.9, 1.0);
                g.lighting(true);
                
                for (const auto& mesh : speakerMeshes) {
                    g.draw(mesh);
                }
            } else {
                g.color(1.0, 0.6, 0.2, 0.8);
                Mesh sphereMesh;
                addSphere(sphereMesh, 0.1);
                g.draw(sphereMesh);
            }
            
            g.popMatrix();
        }
        
        g.polygonMode(GL_FILL);
        g.lighting(false);
        for (auto& pickable : sourcePickables) {
            AudioSource* source = sourceManager->getSource(pickable->sourceId);
            if (source) {
                float brightR = std::min(1.0f, source->color.r * 1.5f + 0.3f);
                float brightG = std::min(1.0f, source->color.g * 1.5f + 0.1f);
                float brightB = std::min(1.0f, source->color.b * 1.5f + 0.1f);
                
                if (source->color.r > source->color.g && source->color.r > source->color.b) {
                    brightR = std::min(1.0f, brightR + 0.2f);
                } else if (source->color.g > source->color.r && source->color.g > source->color.b) {
                    brightG = std::min(1.0f, brightG + 0.2f);
                } else if (source->color.b > source->color.r && source->color.b > source->color.g) {
                    brightB = std::min(1.0f, brightB + 0.2f);
                }
                
                g.color(brightR, brightG, brightB, 1.0f);
                
                if (pickable->selected || pickable->sourceId == selectedSourceId) {
                    g.lineWidth(3.0);
                    g.color(1.0, 1.0, 1.0, 1.0);
                    g.polygonMode(GL_LINE);
                    
                    pickable->draw(g, [&](Pickable &p) {
                        auto &b = dynamic_cast<PickableBB &>(p);
                        b.drawMesh(g);
                    });
                    
                    g.polygonMode(GL_FILL);
                    g.color(brightR, brightG, brightB, 1.0f);
                }
                
                pickable->draw(g, [&](Pickable &p) {
                    auto &b = dynamic_cast<PickableBB &>(p);
                    b.drawMesh(g);
                });
                
                pickable->selected = false;
            }
        }
        g.lighting(true);
        
        imguiBeginFrame();
        ImGui::Begin("Multi-Source Audio Control");

        // MODIFIED: Simplified UI with file browser button
        ImGui::Text("Audio File Loading:");
        
        if (ImGui::Button("Load Audio Files", ImVec2(200, 30))) {
            openFileDialogAndLoadAudio();
        }
        
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(Click to browse for files)");
        
        ImGui::Separator();
        
        if (sourceManager->getSourceCount() > 10) {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Warning: %zu sources loaded", sourceManager->getSourceCount());
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Performance may be affected with many sources");
        }
        
        ImGui::Text("Playback Control:");
        ImGui::BeginGroup();
        {
            bool isPlaying = sourceManager->getIsPlaying();
            bool isPaused = sourceManager->getIsPaused();
            
            if (!isPlaying && !isPaused) {
                if (ImGui::Button("Play")) {
                    sourceManager->play();
                }
            } else if (isPlaying) {
                if (ImGui::Button("Pause")) {
                    sourceManager->pause();
                }
            } else if (isPaused) {
                if (ImGui::Button("Resume")) {
                    sourceManager->play();
                }
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Stop")) {
                sourceManager->stop();
            }
            
            ImGui::SameLine();
            bool sessionLoop = sourceManager->sessionLooping;
            if (ImGui::Checkbox("Loop", &sessionLoop)) {
                sourceManager->sessionLooping = sessionLoop;
            }
            
            ImGui::SameLine();
            if (isPlaying) {
                ImGui::TextColored(ImVec4(0, 1, 0, 1), "PLAYING");
            } else if (isPaused) {
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "PAUSED");
            } else {
                ImGui::TextColored(ImVec4(0.5, 0.5, 0.5, 1), "STOPPED");
            }
        }
        ImGui::EndGroup();

// Progress Bar Section
if (sourceManager->getSourceCount() > 0) {
    int longestDuration = sourceManager->getLongestDuration();
    
    if (longestDuration > 0) {
        int currentPos = sourceManager->getCurrentPlaybackPosition();
        float currentPosFloat = static_cast<float>(currentPos);
        
        // Convert samples to time
        float sampleRate = audioIO().framesPerSecond();
        float totalSeconds = longestDuration / sampleRate;
        float currentSeconds = currentPos / sampleRate;
        
        // Format time as MM:SS
        int totalMinutes = static_cast<int>(totalSeconds) / 60;
        int totalSecondsRemainder = static_cast<int>(totalSeconds) % 60;
        int currentMinutes = static_cast<int>(currentSeconds) / 60;
        int currentSecondsRemainder = static_cast<int>(currentSeconds) % 60;
        
        // Display time labels
        ImGui::Text("%02d:%02d / %02d:%02d", 
                    currentMinutes, currentSecondsRemainder,
                    totalMinutes, totalSecondsRemainder);
        
        // Progress slider (seekable)
        ImGui::PushItemWidth(-1);  // Full width
        if (ImGui::SliderFloat("##progress", &currentPosFloat, 0.0f, 
                               static_cast<float>(longestDuration), "")) {
            // User is seeking - update all sources
            sourceManager->seekToPosition(static_cast<int>(currentPosFloat));
        }
        ImGui::PopItemWidth();
    }
}

ImGui::Separator();
        
        ImGui::Separator();
        
        ImGui::Text("Speaker Layout:");
        int currentTDesignInt = static_cast<int>(currentTDesign);
        auto allNames = SpeakerLayout::getAllTDesignNames();
        std::vector<const char*> namesCStr;
        for (const auto& name : allNames) {
            namesCStr.push_back(name.c_str());
        }
        
        if (ImGui::Combo("##speakerlayout", &currentTDesignInt, namesCStr.data(), namesCStr.size())) {
            SpeakerLayout::TDesignType newTDesign = static_cast<SpeakerLayout::TDesignType>(currentTDesignInt);
            updateAmbisonicOrderSafely(newTDesign);
        }
        
        ImGui::Separator();
        
        ImGui::Text("Acoustic Space:");
        int currentEnvInt = static_cast<int>(reverbProcessor->getCurrentEnvironment());
        auto allEnvNames = reverbProcessor->getAllEnvironmentNames();
        std::vector<const char*> envNamesCStr;
        for (const auto& name : allEnvNames) {
            envNamesCStr.push_back(name.c_str());
        }
        
        if (ImGui::Combo("##environment", &currentEnvInt, envNamesCStr.data(), envNamesCStr.size())) {
            AmbisonicReverbProcessor::ReverbEnvironment newEnv = 
                static_cast<AmbisonicReverbProcessor::ReverbEnvironment>(currentEnvInt);
            reverbProcessor->setEnvironment(newEnv);
        }

        ImGui::Separator();

        ImGui::Text("Headphone Equalization:");
        static int currentHeadphoneEQ = 0;
        static const char* headphoneEQOptions[] = {
            "OFF",
            "AKG-K141MK2",
            "AKG-K240DF", 
            "AKG-K240MK2",
            "AKG-K271MK2",
            "AKG-K271STUDIO",
            "AKG-K601",
            "AKG-K701",
            "AKG-K702",
            "AKG-K1000-Closed", 
            "AKG-K1000-Open",
            "AudioTechnica-ATH-M50",
            "Beyerdynamic-DT250",
            "Beyerdynamic-DT770PRO-250Ohms",
            "Beyerdynamic-DT880",
            "Beyerdynamic-DT990PRO",
            "Presonus-HD7",
            "Sennheiser-HD430",
            "Sennheiser-HD480",
            "Sennheiser-HD560ovationII", 
            "Sennheiser-HD565ovation",
            "Sennheiser-HD600",
            "Sennheiser-HD650",
            "SHURE-SRH940"
        };
        static const int numHeadphoneEQOptions = sizeof(headphoneEQOptions) / sizeof(headphoneEQOptions[0]);

        if (ImGui::Combo("##headphoneeq", &currentHeadphoneEQ, headphoneEQOptions, numHeadphoneEQOptions)) {
            loadHeadphoneEQ(currentHeadphoneEQ);
        }

        ImGui::Separator();

        // HEAD TRACKING GUI SECTION
        ImGui::Text("Head Tracking:");

        bool htEnabled = headTracker->isEnabled();
        if (ImGui::Checkbox("Enable Head Tracking", &htEnabled)) {
            headTracker->setEnabled(htEnabled);
        }
        
        // Only show connection status if connected AND received messages
        if (headTracker->isOSCConnected() && headTracker->hasReceivedMessages()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0, 1, 0, 1), "OSC Connected");
        }
        
        static int oscPort = 9000;
        ImGui::Text("OSC Port:");
        ImGui::SameLine();
        ImGui::PushItemWidth(80);
        if (ImGui::InputInt("##oscport", &oscPort)) {
            if (oscPort >= 1024 && oscPort <= 65535) {
                headTracker->setOSCPort(oscPort);
            }
        }
        ImGui::PopItemWidth();
        
        ImGui::Text("Manual Control:");
        
        static float yawValue = 0.0f;
        static float pitchValue = 0.0f;
        static float rollValue = 0.0f;
        
        // Get current values from head tracker
        headTracker->getOrientation(yawValue, pitchValue, rollValue);
        
        ImGui::Text("Yaw");
        ImGui::SameLine(100);
        ImGui::Text("Pitch");
        ImGui::SameLine(200);
        ImGui::Text("Roll");
        
        ImGui::PushItemWidth(80);
        if (ImGui::SliderFloat("##yaw", &yawValue, -180.0f, 180.0f, "%.0f°")) {
            headTracker->setOrientation(yawValue, pitchValue, rollValue);
        }
        ImGui::PopItemWidth();
        
        ImGui::SameLine(100);
        ImGui::PushItemWidth(80);
        if (ImGui::SliderFloat("##pitch", &pitchValue, -90.0f, 90.0f, "%.0f°")) {
            headTracker->setOrientation(yawValue, pitchValue, rollValue);
        }
        ImGui::PopItemWidth();
        
        ImGui::SameLine(200);
        ImGui::PushItemWidth(80);
        if (ImGui::SliderFloat("##roll", &rollValue, -180.0f, 180.0f, "%.0f°")) {
            headTracker->setOrientation(yawValue, pitchValue, rollValue);
        }
        ImGui::PopItemWidth();
        
        // Flip buttons
        bool flipYaw = headTracker->getFlipYaw();
        bool flipPitch = headTracker->getFlipPitch();
        bool flipRoll = headTracker->getFlipRoll();

        if (ImGui::Checkbox("+/-##yaw", &flipYaw)) {
            headTracker->setFlipYaw(flipYaw);
        }
        ImGui::SameLine(100);
        if (ImGui::Checkbox("+/-##pitch", &flipPitch)) {
            headTracker->setFlipPitch(flipPitch);
        }
        ImGui::SameLine(200);
        if (ImGui::Checkbox("+/-##roll", &flipRoll)) {
            headTracker->setFlipRoll(flipRoll);
        }
        
        ImGui::Separator();

        ImGui::Text("HRTF Dataset:");

        if (ImGui::Checkbox("Use Default HRTF set", &useDefaultHRTF)) {
            if (useDefaultHRTF && sofaFileLoaded) {
                // Switch back to default HRTFs
                if (ambiDecoder) {
                    ambiDecoder->useDefaultHRTFs();
                }
                sofaFileLoaded = false;
                sofaFilePath = "";
                std::cout << "Switched back to default HRTFs" << std::endl;
            }
        }

        ImGui::Text("Custom SOFA File:");

        ImGui::PushItemWidth(250);
        ImGui::Text("%s", sofaFileLoaded ? sofaFilePath.c_str() : "(No file loaded)");
        ImGui::PopItemWidth();

        ImGui::SameLine();
        if (ImGui::Button("Browse...")) {
            openFileDialogAndLoadSOFA();
        }
        
        if (sofaFileLoaded) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Loaded");
            ImGui::Text("  HRIRs: %d directions", customHRIRData.N_hrir_dirs);
        }

        ImGui::Separator();
        ImGui::Text("Loaded Sources (%zu):", sourceManager->getSourceCount());
        
        const auto& sources = sourceManager->getSources();
        for (const auto& source : sources) {
            ImGui::PushID(source->sourceId);
            
            ImVec4 color(source->color.r, source->color.g, source->color.b, source->color.a);
            ImGui::ColorButton("##color", color, ImGuiColorEditFlags_NoTooltip, ImVec2(20, 20));
            
            ImGui::SameLine();
            
            bool isSelected = (selectedSourceId == source->sourceId);
            if (ImGui::Selectable(source->filename.c_str(), isSelected)) {
                selectedSourceId = isSelected ? -1 : source->sourceId;
            }
            
            if (isSelected) {
                ImGui::Indent();
                
                float az = source->azimuth;
                float el = source->elevation;
                
                if (ImGui::SliderFloat("Azimuth", &az, -180.0f, 180.0f)) {
                    source->setPosition(az, el);
                    
                    for (auto& pickable : sourcePickables) {
                        if (pickable->sourceId == source->sourceId) {
                            Vec3f newPos = sphericalToCartesian(az, el, visualRadius);
                            pickable->pose = Pose(newPos);
                            break;
                        }
                    }
                }
                
                if (ImGui::SliderFloat("Elevation", &el, -90.0f, 90.0f)) {
                    source->setPosition(az, el);
                    
                    for (auto& pickable : sourcePickables) {
                        if (pickable->sourceId == source->sourceId) {
                            Vec3f newPos = sphericalToCartesian(az, el, visualRadius);
                            pickable->pose = Pose(newPos);
                            break;
                        }
                    }
                }
                
                float gain = source->gain;
                if (ImGui::SliderFloat("Gain", &gain, 0.0f, 2.0f)) {
                    sourceManager->setSourceGain(source->sourceId, gain);
                }
                
                bool muted = source->isMuted;
                if (ImGui::Checkbox("M", &muted)) {
                    source->isMuted = muted;
                }
                
                ImGui::SameLine();
                bool soloed = source->isSoloed;
                if (ImGui::Checkbox("S", &soloed)) {
                    source->isSoloed = soloed;
                }
                
                if (ImGui::Button("Remove Source")) {
                    removeSource(source->sourceId);
                    selectedSourceId = -1;
                }
                
                ImGui::Unindent();
            }
            
            ImGui::PopID();
        }
        
        ImGui::Separator();
        
        ImGui::Text("System Info:");
        ImGui::Text("Current Layout: %s", SpeakerLayout::getTDesignName(currentTDesign).c_str());
        ImGui::Text("Virtual Speakers: %d", SpeakerLayout::getSpeakerCount(currentTDesign));
        ImGui::Text("Ambisonic Order: %d", SpeakerLayout::getRecommendedOrder(currentTDesign));
        ImGui::Text("Current Environment: %s", reverbProcessor->getCurrentEnvironmentName().c_str());
        ImGui::Text("Sample Rate: %.0f Hz", audioIO().framesPerSecond());
        ImGui::Text("Buffer Size: %d samples", audioIO().framesPerBuffer());
        
        // Head tracking status
        if (headTracker->isEnabled()) {
            float dispYaw, dispPitch, dispRoll;
            headTracker->getOrientation(dispYaw, dispPitch, dispRoll);
            ImGui::Text("Head: Yaw=%.1f° Pitch=%.1f° Roll=%.1f°", dispYaw, dispPitch, dispRoll);
        }
        
        ImGui::End();
        
        imguiEndFrame();
        imguiDraw();
    }

    void onSound(AudioIOData &io) override {
        Pose& listenerPose = scene.listenerPose();
        listenerPose = fixedListenerPose;
        
        int nFrames = io.framesPerBuffer();
        
        if (nFrames > bufferSize) {
            delete[] leftOutputBuffer;
            delete[] rightOutputBuffer;
            
            bufferSize = nFrames;
            leftOutputBuffer = new float[bufferSize];
            rightOutputBuffer = new float[bufferSize];
        }
        
        memset(leftOutputBuffer, 0, nFrames * sizeof(float));
        memset(rightOutputBuffer, 0, nFrames * sizeof(float));
        
        if (ambiEncoder && ambiDecoder && sourceManager && reverbProcessor) {
            bool shouldPlay = sourceManager->isPlaying && !sourceManager->isPaused;
            
            if (shouldPlay) {
                sourceManager->processAudio(ambiEncoder, nFrames);
                
                // APPLY HEAD TRACKING ROTATION TO AMBISONIC SOUNDFIELD
                if (headTracker && headTracker->isEnabled()) {
                    int currentOrder = SpeakerLayout::getRecommendedOrder(currentTDesign);
                    headTracker->rotateAmbisonics(ambiEncoder->ambiSignals, nFrames, currentOrder);
                }
                
                ambiDecoder->decodeWithReverb(ambiEncoder->ambiSignals, leftOutputBuffer, rightOutputBuffer, nFrames, reverbProcessor.get());
                
                if (currentHeadphoneEQIndex > 0 && headphoneEQ_L && headphoneEQ_R) {
                    juce::dsp::AudioBlock<float> leftBlock(&leftOutputBuffer, 1, 0, static_cast<size_t>(nFrames));
                    juce::dsp::AudioBlock<float> rightBlock(&rightOutputBuffer, 1, 0, static_cast<size_t>(nFrames));
                    
                    headphoneEQ_L->process(juce::dsp::ProcessContextReplacing<float>(leftBlock));
                    headphoneEQ_R->process(juce::dsp::ProcessContextReplacing<float>(rightBlock));
                }
            } else {
                ambiEncoder->clearSignals();
            }
        }
        
        for (int i = 0; i < nFrames; i++) {
            io.out(0, i) = leftOutputBuffer[i];
            io.out(1, i) = rightOutputBuffer[i];
        }
    }

    void onExit() override {
        NFD_Quit();
        delete ambiEncoder;
        delete ambiDecoder;
        delete[] leftOutputBuffer;
        delete[] rightOutputBuffer;
    }

    bool onKeyDown(const Keyboard &k) override {
        if (k.key() == ' ') {
            if (sourceManager->getIsPlaying()) {
                sourceManager->pause();
            } else {
                sourceManager->play();
            }
            return true;
        } else if (k.key() == 's') {
            sourceManager->stop();
            return true;
        }
        return false;
    }
   
    bool onMouseMove(const Mouse &m) override {
        if (gui.usingInput()) return true;
        pickableManager.onMouseMove(graphics(), m, width(), height());
        return true;
    }
   
    bool onMouseDown(const Mouse &m) override {
        if (gui.usingInput()) return true;
        pickableManager.onMouseDown(graphics(), m, width(), height());
       
        for (auto& pickable : sourcePickables) {
            if (pickable->selected) {
                selectedSourceId = pickable->sourceId;
                break;
            }
        }
       
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
    app.dimensions(1200, 800);
    app.title("Ambisonics-to-Binaural Renderer");
    app.configureAudio(44100, 256, 2, 0);  
    app.start();
    return 0;
}