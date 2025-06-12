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

// Asset loading
#include "al_ext/assets3d/al_Asset.hpp"

// JUCE includes - ONLY the modules we need, no GUI components
#include "juce_core/juce_core.h"
#include "juce_audio_basics/juce_audio_basics.h"
#include "juce_dsp/juce_dsp.h"
#include "juce_audio_formats/juce_audio_formats.h"

// Our modular components
#include "audio/speakerLayout.hpp"
#include "audio/ambisonicEncoder.hpp"
#include "audio/ambisonicBinauralDecoder.hpp"
#include "audio/AmbisonicReverbProcessor.hpp"

using namespace al;

// ===================
// Simplified Parameter Smoother - Only for Audio Parameters
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


// Helper: Convert Spherical to Cartesian
Vec3f sphericalToCartesian(float azimuthDeg, float elevationDeg, float radius) {
    float azimuth_rad = azimuthDeg * M_PI / 180.0f;
    float elevation_rad = elevationDeg * M_PI / 180.0f;
    
    // Visual positioning: 0° = front of head, positive = right side
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
    
    // Don't negate here - keep the visual azimuth as-is
    azimuthDeg = atan2(x, -z) * 180.0f / M_PI;
    
    // Normalize to [-180, +180] range
    while (azimuthDeg > 180.0f) azimuthDeg -= 360.0f;
    while (azimuthDeg < -180.0f) azimuthDeg += 360.0f;
}

// ===================
// Audio Source Class - FIXED for Resume Functionality
// ===================
class AudioSource {
public:
    int sourceId;
    std::string filename;
    Vec3f position;
    float azimuth = 0.0f;
    float elevation = 0.0f;
    float gain = 1.0f;
    
    // Audio data
    juce::AudioBuffer<float> audioBuffer;
    int currentSamplePosition = 0;
    bool isLooping = true;
    bool isActive = true;
    bool isMuted = false;
    bool isSoloed = false;
    
    // ONLY gain smoothing - no position smoothing for responsiveness
    ParameterSmoother gainSmoother;
    
    // Visual properties
    Color color;
    bool isSelected = false;
    
    AudioSource(int id, const std::string& file, float sampleRate) 
        : sourceId(id), filename(file),
          gainSmoother(1.0f, 15.0f, sampleRate) {  // Shorter smoothing for responsiveness
        
        generateUniqueColor();
        setPosition(al::rnd::uniform(-180.0f, 180.0f), al::rnd::uniform(-45.0f, 45.0f));
    }
    
    void generateUniqueColor() {
        float hue = (sourceId * 137.508f) / 360.0f;
        hue = hue - floor(hue);
        
        float saturation = 0.7f + 0.3f * (sourceId % 3) / 3.0f;
        float value = 0.8f + 0.2f * ((sourceId + 1) % 2);

        // HSV to RGB conversion
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
        // IMMEDIATE position update - no smoothing for real-time response
        azimuth = azimuthDeg;
        elevation = elevationDeg;
        position = sphericalToCartesian(azimuthDeg, elevationDeg, 1.0f);
    }

    void setGain(float newGain) {
        gain = newGain;  // Set immediately for UI responsiveness
        gainSmoother.setTarget(newGain);  // But smooth the audio to prevent clicks
    }

    float getNextSample() {
        if (!isActive || audioBuffer.getNumSamples() == 0) {
            return 0.0f;
        }
        
        float sample = audioBuffer.getSample(0, currentSamplePosition);
        sample *= gainSmoother.getNextValue();  // Apply smoothed gain
        
        currentSamplePosition++;
        if (currentSamplePosition >= audioBuffer.getNumSamples()) {
            // Individual files don't loop anymore - they just stop
            isActive = false;
            return 0.0f;
        }
        
        return sample;
    }

    void reset() {
        currentSamplePosition = 0;
        isActive = true;
        // CRITICAL FIX: Don't reset gain to zero! Keep current gain value for resume
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
// Multi-Source Audio Manager - FIXED State Management
// ===================
class MultiSourceManager {
private:
    std::vector<std::unique_ptr<AudioSource>> sources;
    int nextSourceId = 0;
    double sampleRate;
    
public:  // Make these public so onSound can access them directly
    bool isPlaying = false;
    bool isPaused = false;
    bool sessionLooping = true;  // Default to true for continuous playback
    
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
        
        // Make sure all sources are active when resuming
        for (auto& source : sources) {
            if (!source->isActive && source->audioBuffer.getNumSamples() > 0) {
                source->isActive = true;
            }
        }
        
        std::cout << "Playing " << sources.size() << " audio sources" << std::endl;
    }
    
    void pause() {
        isPaused = true;
        // Don't reset gain smoothers to zero - just pause the state!
        std::cout << "Paused playback" << std::endl;
    }
    
    void stop() {
        isPlaying = false;
        isPaused = false;
        // Reset playback positions but preserve gain settings
        for (auto& source : sources) {
            source->currentSamplePosition = 0;  // Reset playback position
            source->isActive = true;             // Reactivate source
            // Keep gain settings intact for proper resume
        }
        std::cout << "Stopped playback" << std::endl;
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
        
        // Check if all sources have finished
        bool allFinished = true;
        for (auto& source : sources) {
            if (source->isActive && source->audioBuffer.getNumSamples() > 0) {
                allFinished = false;
                break;
            }
        }
        
        // If all finished and session looping is on, restart all
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
            
            // Skip if muted, or if other sources are soloed and this one isn't
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
    
    double lastParameterUpdateTime = 0.0;
    const double parameterUpdateInterval = 0.005;  // Faster updates - 5ms
    
    double getCurrentTime() {
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

    void enforceSourceLimit() {
        const int MAX_SOURCES = 20; // Reasonable limit
        
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

    void loadSingleAudioFileFromPath(const std::string& filepath) {
        if (!filepath.empty()) {
            int sourceId = sourceManager->addAudioFile(filepath);
            if (sourceId >= 0) {
                createPickableForSource(sourceId);
                std::cout << "Successfully loaded single file: " << filepath << std::endl;
            }
        }
    }
    
    void loadMultipleAudioFilesFromPaths(const std::string& pathsString) {
        if (pathsString.empty()) return;
        
        std::vector<std::string> filepaths;
        std::stringstream ss(pathsString);
        std::string filepath;
        
        while (std::getline(ss, filepath, '\n')) {
            filepath.erase(0, filepath.find_first_not_of(" \t\r"));
            filepath.erase(filepath.find_last_not_of(" \t\r") + 1);
            
            if (!filepath.empty()) {
                filepaths.push_back(filepath);
            }
        }
        
        if (!filepaths.empty()) {
            std::cout << "Loading " << filepaths.size() << " files..." << std::endl;
            
            // Load files one by one with progress feedback
            int loadedCount = 0;
            for (const auto& path : filepaths) {
                int sourceId = sourceManager->addAudioFile(path);
                if (sourceId >= 0) {
                    createPickableForSource(sourceId);
                    loadedCount++;
                    
                    // Progress feedback every 5 files
                    if (loadedCount % 5 == 0) {
                        std::cout << "Loaded " << loadedCount << "/" << filepaths.size() << " files..." << std::endl;
                    }
                }
            }
            
            std::cout << "Successfully loaded " << loadedCount << " out of " << filepaths.size() << " files" << std::endl;
        }
    }
    
    void createPickableForSource(int sourceId) {
        AudioSource* source = sourceManager->getSource(sourceId);
        if (!source) return;
        
        // REUSE a single sphere mesh instead of creating new ones
        static bool sphereMeshCreated = false;
        static VAOMesh sharedSphereMesh;
        
        if (!sphereMeshCreated) {
            addSphere(sharedSphereMesh, 0.15);
            sharedSphereMesh.primitive(Mesh::TRIANGLES);
            sharedSphereMesh.update();
            sphereMeshCreated = true;
        }
        
        auto pickable = std::make_unique<SelectablePickable>(sourceId);
        pickable->set(sharedSphereMesh);  // Use shared mesh
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

    void onCreate() override {
        double sampleRate = audioIO().framesPerSecond();
        int frameSize = audioIO().framesPerBuffer();
        
        audioFormatManager.registerBasicFormats();
        
        sourceManager = std::make_unique<MultiSourceManager>(sampleRate);
        
        ambiEncoder = new AmbisonicEncoder(1, frameSize, sampleRate);
        ambiDecoder = new AmbisonicBinauralDecoder(1, sampleRate, frameSize, currentTDesign);
        reverbProcessor = std::make_unique<AmbisonicReverbProcessor>(1, sampleRate, frameSize, currentTDesign);
        
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
       std::cout << "  1. Enter file paths in the GUI text fields" << std::endl;
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

        // Update reverb processor with new T-design
        reverbProcessor->updateTDesign(newTDesign);
        
        // Reset normalizer for new configuration

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
               
               // CRITICAL FIX: Always update source position regardless of timing
               // This ensures that dragging any pickable immediately updates the corresponding source
               AudioSource* source = sourceManager->getSource(pickable->sourceId);
               if (source) {
                   // Update the source position directly
                   source->setPosition(az, el);
               }
           }
           
           pickablesUpdatingParameters = false;
       }
   }

   void onDraw(Graphics &g) override {
       g.clear(0);
       gl::depthTesting(true);
       
       // Draw coordinate axes
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
       
       // Draw head model
       if (headScene && !headMeshes.empty()) {
           g.pushMatrix();
           g.translate(0, 0, 0);
           g.rotate(180, 0, 1, 0);
           
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
       
       // Draw wireframe sphere
       g.depthMask(false);
       g.color(1.0, 1.0, 1.0, 1.0);
       g.lineWidth(2.5);
       g.polygonMode(GL_LINE);
       g.draw(sphereMesh);
       g.depthMask(true);
       
       // Draw virtual speakers
       g.polygonMode(GL_FILL);
       for (int i = 0; i < virtualSpeakerPositions.size(); i++) {
           Vec3f speakerPos = virtualSpeakerPositions[i];
           
           g.pushMatrix();
           g.translate(speakerPos);
           
           // Calculate orientation to face center
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
               // Fallback: simple sphere if speaker model fails to load
               g.color(1.0, 0.6, 0.2, 0.8);
               Mesh sphereMesh;
               addSphere(sphereMesh, 0.1);
               g.draw(sphereMesh);
           }
           
           g.popMatrix();
       }
       
       // Draw audio sources with increased brightness
// Draw audio sources with emissive bright colors (no lighting)
g.polygonMode(GL_FILL);
g.lighting(false);  // Disable lighting for balls
for (auto& pickable : sourcePickables) {
    AudioSource* source = sourceManager->getSource(pickable->sourceId);
    if (source) {
        // Boost saturation and brightness while preserving color identity
        float brightR = std::min(1.0f, source->color.r * 1.5f + 0.3f);
        float brightG = std::min(1.0f, source->color.g * 1.5f + 0.1f);  // Less green boost
        float brightB = std::min(1.0f, source->color.b * 1.5f + 0.1f);  // Less blue boost
        
        // Further enhance the dominant color channel
        if (source->color.r > source->color.g && source->color.r > source->color.b) {
            // Red is dominant - boost red more
            brightR = std::min(1.0f, brightR + 0.2f);
        } else if (source->color.g > source->color.r && source->color.g > source->color.b) {
            // Green is dominant - boost green more
            brightG = std::min(1.0f, brightG + 0.2f);
        } else if (source->color.b > source->color.r && source->color.b > source->color.g) {
            // Blue is dominant - boost blue more
            brightB = std::min(1.0f, brightB + 0.2f);
        }
        
        g.color(brightR, brightG, brightB, 1.0f);  // Full alpha
        
        if (pickable->selected || pickable->sourceId == selectedSourceId) {
            g.lineWidth(3.0);
            g.color(1.0, 1.0, 1.0, 1.0);  // Pure white for selection
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
g.lighting(true);  // Re-enable lighting for other objects
       
       // Draw GUI
       imguiBeginFrame();
       gui.draw(g);
       
       ImGui::Begin("Multi-Source Audio Control");
       
       ImGui::Text("Audio File Loading:");
       
       // Single file loading
       ImGui::Text("Single File Path:");
       static char filePathBuffer[512] = "";
       if (ImGui::InputText("##singlefile", filePathBuffer, sizeof(filePathBuffer), ImGuiInputTextFlags_EnterReturnsTrue)) {
           std::string filepath(filePathBuffer);
           if (!filepath.empty()) {
               loadSingleAudioFileFromPath(filepath);
               memset(filePathBuffer, 0, sizeof(filePathBuffer));
           }
       }
       ImGui::SameLine();
       if (ImGui::Button("Load Single File")) {
           std::string filepath(filePathBuffer);
           if (!filepath.empty()) {
               loadSingleAudioFileFromPath(filepath);
               memset(filePathBuffer, 0, sizeof(filePathBuffer));
           }
       }
       
       // Multiple files loading
       ImGui::Text("Multiple File Paths (one per line):");
       static char multipleFilePathsBuffer[2048] = "";
       if (ImGui::InputTextMultiline("##multiplefiles", multipleFilePathsBuffer, sizeof(multipleFilePathsBuffer), ImVec2(400, 100), ImGuiInputTextFlags_EnterReturnsTrue)) {
           std::string pathsString(multipleFilePathsBuffer);
           if (!pathsString.empty()) {
               loadMultipleAudioFilesFromPaths(pathsString);
               memset(multipleFilePathsBuffer, 0, sizeof(multipleFilePathsBuffer));
           }
       }
       if (ImGui::Button("Load Multiple Files")) {
           std::string pathsString(multipleFilePathsBuffer);
           if (!pathsString.empty()) {
               loadMultipleAudioFilesFromPaths(pathsString);
               memset(multipleFilePathsBuffer, 0, sizeof(multipleFilePathsBuffer));
           }
       }
       
       ImGui::Separator();
       
       // Show current source count and performance warning
       if (sourceManager->getSourceCount() > 10) {
           ImGui::TextColored(ImVec4(1, 1, 0, 1), "Warning: %zu sources loaded", sourceManager->getSourceCount());
           ImGui::TextColored(ImVec4(1, 1, 0, 1), "Performance may be affected with many sources");
       }
       
       // Playback controls
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
       
       ImGui::Separator();
       
       // T-design selection
       ImGui::Text("Speaker Layout:");
       int currentTDesignInt = static_cast<int>(currentTDesign);
       auto allNames = SpeakerLayout::getAllTDesignNames();
       std::vector<const char*> namesCStr;
       for (const auto& name : allNames) {
           namesCStr.push_back(name.c_str());
       }
       
       if (ImGui::Combo("T-design Layout", &currentTDesignInt, namesCStr.data(), namesCStr.size())) {
           SpeakerLayout::TDesignType newTDesign = static_cast<SpeakerLayout::TDesignType>(currentTDesignInt);
           updateAmbisonicOrderSafely(newTDesign);
       }
       
       ImGui::Separator();
       
       // Acoustic Environment selection
       ImGui::Text("Acoustic Environment:");
       int currentEnvInt = static_cast<int>(reverbProcessor->getCurrentEnvironment());
       auto allEnvNames = reverbProcessor->getAllEnvironmentNames();
       std::vector<const char*> envNamesCStr;
       for (const auto& name : allEnvNames) {
           envNamesCStr.push_back(name.c_str());
       }
       
       if (ImGui::Combo("Environment", &currentEnvInt, envNamesCStr.data(), envNamesCStr.size())) {
           AmbisonicReverbProcessor::ReverbEnvironment newEnv = 
               static_cast<AmbisonicReverbProcessor::ReverbEnvironment>(currentEnvInt);
           reverbProcessor->setEnvironment(newEnv);
       }
       
       ImGui::Separator();
       
       // Source management
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
               
               // FIXED: Get current position directly from source and ensure bidirectional sync
               float az = source->azimuth;
               float el = source->elevation;
               
               if (ImGui::SliderFloat("Azimuth", &az, -180.0f, 180.0f)) {
                   // Update source position directly
                   source->setPosition(az, el);
                   
                   // Update pickable position to match slider
                   for (auto& pickable : sourcePickables) {
                       if (pickable->sourceId == source->sourceId) {
                           Vec3f newPos = sphericalToCartesian(az, el, visualRadius);
                           pickable->pose = Pose(newPos);
                           break;
                       }
                   }
               }
               
               if (ImGui::SliderFloat("Elevation", &el, -90.0f, 90.0f)) {
                   // Update source position directly
                   source->setPosition(az, el);
                   
                   // Update pickable position to match slider
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
       
       // System info
       ImGui::Text("System Info:");
       ImGui::Text("Current Layout: %s", SpeakerLayout::getTDesignName(currentTDesign).c_str());
       ImGui::Text("Virtual Speakers: %d", SpeakerLayout::getSpeakerCount(currentTDesign));
       ImGui::Text("Ambisonic Order: %d", SpeakerLayout::getRecommendedOrder(currentTDesign));
       ImGui::Text("Current Environment: %s", reverbProcessor->getCurrentEnvironmentName().c_str());
       ImGui::Text("Sample Rate: %.0f Hz", audioIO().framesPerSecond());
       ImGui::Text("Buffer Size: %d samples", audioIO().framesPerBuffer());
       
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
    
    // ALWAYS clear output buffers first
    memset(leftOutputBuffer, 0, nFrames * sizeof(float));
    memset(rightOutputBuffer, 0, nFrames * sizeof(float));
    
    if (ambiEncoder && ambiDecoder && sourceManager && reverbProcessor) {  // Removed outputNormalizer
        bool shouldPlay = sourceManager->isPlaying && !sourceManager->isPaused;
        
        if (shouldPlay) {
            sourceManager->processAudio(ambiEncoder, nFrames);
            ambiDecoder->decodeWithReverb(ambiEncoder->ambiSignals, leftOutputBuffer, rightOutputBuffer, nFrames, reverbProcessor.get());
            // Removed normalization line
        } else {
            ambiEncoder->clearSignals();
        }
    }
    
    // Output (will be clean with energy compensation)
    for (int i = 0; i < nFrames; i++) {
        io.out(0, i) = leftOutputBuffer[i];
        io.out(1, i) = rightOutputBuffer[i];
    }
}

   void onExit() override {
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