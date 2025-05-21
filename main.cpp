#include <iostream>
#include <cmath>
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

using namespace al;

// ===================
// Helper: Convert Spherical to Cartesian using AlloSphere convention
// ===================
Vec3f sphericalToCartesian(float azimuthDeg, float elevationDeg, float radius) {
  // Convert azimuth and elevation from degrees to radians
  float azimuth_rad = azimuthDeg * M_PI / 180.0f;
  float elevation_rad = elevationDeg * M_PI / 180.0f;
  
  // Calculate the Cartesian coordinates using AlloSphere convention
  float x = radius * cos(elevation_rad) * sin(azimuth_rad);
  float y = radius * sin(elevation_rad);
  float z = -radius * cos(elevation_rad) * cos(azimuth_rad);  // Right-handed system flip
  
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
  
  elevationDeg = asin(y / radius) * 180.0f / M_PI;
  azimuthDeg = atan2(x, -z) * 180.0f / M_PI;  // Note the negative z for AlloSphere convention
}

// ===================
// Ambisonic Encoder Class
// ===================
class AmbisonicEncoder {
public:
    int order;                    // Ambisonic order
    int nSH;                      // Number of spherical harmonic signals
    float* Y;                     // SH weights for current direction
    float** ambiSignals;          // Ambisonic signals
    int frameSize;                // Frame size

    AmbisonicEncoder(int orderVal = 1, int frameSizeVal = 256) : 
        order(orderVal),
        frameSize(frameSizeVal) {
        
        // Calculate number of spherical harmonic channels
        nSH = (order + 1) * (order + 1);
        
        // Allocate memory for SH weights
        Y = new float[nSH];
        
        // Allocate memory for ambisonic signals
        ambiSignals = new float*[nSH];
        for (int i = 0; i < nSH; i++) {
            ambiSignals[i] = new float[frameSize];
            memset(ambiSignals[i], 0, frameSize * sizeof(float));
        }
        
        std::cout << "AmbisonicEncoder initialized with order: " << order 
                  << ", nSH: " << nSH << std::endl;
    }
    
    ~AmbisonicEncoder() {
        delete[] Y;
        for (int i = 0; i < nSH; i++) {
            delete[] ambiSignals[i];
        }
        delete[] ambiSignals;
    }
    
    // Update spherical harmonic weights for a given direction
    void updateDirection(float azimuthDeg, float elevationDeg) {
        float dirs_deg[2] = {azimuthDeg, elevationDeg};
        
        // Compute real spherical harmonics for the given direction
        // Using SAF's function to calculate RSH coefficients up to specified order
        getRSH(order, dirs_deg, 1, Y);
        
        std::cout << "Updated SH coefficients for direction: (" 
                  << azimuthDeg << ", " << elevationDeg << ")" << std::endl;
        
        // Debug: Print first few coefficients
        std::cout << "  Y[0]: " << Y[0] << ", Y[1]: " << Y[1];
        if (nSH > 2) std::cout << ", Y[2]: " << Y[2];
        if (nSH > 3) std::cout << ", Y[3]: " << Y[3];
        std::cout << std::endl;
    }
    
    // Encode a mono input signal to ambisonic signals
    void encode(const float* inputSignal, int numSamples) {
        // Clear previous content
        for (int i = 0; i < nSH; i++) {
            memset(ambiSignals[i], 0, frameSize * sizeof(float));
        }
        
        // Encode: multiply input by SH weights for each channel
        for (int ch = 0; ch < nSH; ch++) {
            for (int i = 0; i < numSamples; i++) {
                ambiSignals[ch][i] = inputSignal[i] * Y[ch];
            }
        }
    }
    
    // Get the ambisonic signal for a specific SH channel
    float* getAmbisonicChannel(int channel) {
        if (channel < 0 || channel >= nSH) {
            std::cerr << "Error: Channel index out of range" << std::endl;
            return nullptr;
        }
        return ambiSignals[channel];
    }
    
    // Convenience method to get order
    int getOrder() const { return order; }
    
    // Convenience method to get number of channels
    int getNumChannels() const { return nSH; }
};

// ===================
// Ambisonic Binaural Decoder Class
// ===================
class AmbisonicBinauralDecoder {
public:
    // Default HRIR data from SAF
    const float* hrirs;
    const float* hrir_dirs_deg;
    int N_hrir_dirs;
    int hrir_len;
    int fs;
    
    // Ambisonic order and related variables
    int order;
    int nSH;
    
    // Virtual loudspeaker setup for binaural decoding
    float* vls_dirs_deg;  // Virtual loudspeaker directions [azi elev] in degrees
    int nVirtualSpeakers; // Number of virtual loudspeakers
    
    // Decoding matrices
    float* dec_mat;       // Basic decoding matrix
    float* vls_gains;     // Gains for virtual loudspeakers
    
    // Convolution buffers and states
    float** hrtf_filters; // HRTF filters organized by virtual speaker
    float** conv_states_l; // Convolution states for left ear
    float** conv_states_r; // Convolution states for right ear
    
    AmbisonicBinauralDecoder(int orderVal = 1, int sampleRate = 44100) : 
        order(orderVal) {
        
        // Calculate number of spherical harmonic channels
        nSH = (order + 1) * (order + 1);
        
        // Get default HRIR data from SAF
        hrirs = &__default_hrirs[0][0][0];
        hrir_dirs_deg = &__default_hrir_dirs_deg[0][0];
        N_hrir_dirs = __default_N_hrir_dirs;
        hrir_len = __default_hrir_len;
        fs = __default_hrir_fs;
        
        // Set up virtual loudspeaker configuration
        setupVirtualLoudspeakers();
        
        // Create basic decoding matrix
        createDecodingMatrix();
        
        // Set up HRTF filters and convolution states
        setupHRTFFilters();
        
        std::cout << "AmbisonicBinauralDecoder initialized with order: " << order 
                  << ", nSH: " << nSH 
                  << ", Virtual speakers: " << nVirtualSpeakers << std::endl;
    }
    
    ~AmbisonicBinauralDecoder() {
        delete[] vls_dirs_deg;
        delete[] dec_mat;
        delete[] vls_gains;
        
        for (int i = 0; i < nVirtualSpeakers; i++) {
            delete[] hrtf_filters[i*2];
            delete[] hrtf_filters[i*2+1];
            delete[] conv_states_l[i];
            delete[] conv_states_r[i];
        }
        delete[] hrtf_filters;
        delete[] conv_states_l;
        delete[] conv_states_r;
    }
    
    // Set up virtual loudspeaker configuration based on ambisonic order
    void setupVirtualLoudspeakers() {
        // Determine number of virtual loudspeakers based on order
        // A good rule of thumb is (order+1)^2 or more
        nVirtualSpeakers = (order + 1) * (order + 1);
        
        // We'll use a t-design for virtual speaker layout
        // For simplicity, let's use a predefined t-design from SAF
        vls_dirs_deg = new float[nVirtualSpeakers * 2]; // [azi, elev] pairs
        
        // Select appropriate t-design based on order
        switch (order) {
            case 1:
                // 4 points for 1st order (minimum required)
                for (int i = 0; i < 4; i++) {
                    vls_dirs_deg[i*2] = __Tdesign_degree_2_dirs_deg[i][0];
                    vls_dirs_deg[i*2+1] = __Tdesign_degree_2_dirs_deg[i][1];
                }
                nVirtualSpeakers = 4;
                break;
            case 2:
                // 9 points for 2nd order (spherical covering design)
                for (int i = 0; i < 9; i++) {
                    vls_dirs_deg[i*2] = __SphCovering_9_dirs_deg[i][0];
                    vls_dirs_deg[i*2+1] = __SphCovering_9_dirs_deg[i][1];
                }
                nVirtualSpeakers = 9;
                break;
            case 3:
                // 16 points for 3rd order
                for (int i = 0; i < 16; i++) {
                    vls_dirs_deg[i*2] = __SphCovering_16_dirs_deg[i][0];
                    vls_dirs_deg[i*2+1] = __SphCovering_16_dirs_deg[i][1];
                }
                nVirtualSpeakers = 16;
                break;
            default:
                // Fallback to t-design for higher orders
                for (int i = 0; i < nVirtualSpeakers; i++) {
                    if (i < 36) {
                        vls_dirs_deg[i*2] = __Tdesign_degree_8_dirs_deg[i][0];
                        vls_dirs_deg[i*2+1] = __Tdesign_degree_8_dirs_deg[i][1];
                    } else {
                        // Fill remaining slots with default points
                        vls_dirs_deg[i*2] = 0.0f;
                        vls_dirs_deg[i*2+1] = 0.0f;
                    }
                }
                break;
        }
        
        std::cout << "Set up " << nVirtualSpeakers << " virtual loudspeakers for order " 
                  << order << " ambisonics" << std::endl;
    }
    
    // Create basic ambisonic decoding matrix
    void createDecodingMatrix() {
        // Allocate memory for decoding matrix: nVirtualSpeakers × nSH
        dec_mat = new float[nVirtualSpeakers * nSH];
        
        // Evaluate spherical harmonics at each loudspeaker position
        float* Y_ls = new float[nSH];
        
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            // Get spherical harmonics for this loudspeaker direction
            float ls_dir[2] = {vls_dirs_deg[ls*2], vls_dirs_deg[ls*2+1]};
            getRSH(order, ls_dir, 1, Y_ls);
            
            // Decoder matrix is the transpose of the spherical harmonics matrix
            for (int n = 0; n < nSH; n++) {
                dec_mat[ls*nSH + n] = Y_ls[n];
            }
        }
        
        delete[] Y_ls;
        
        // Allocate memory for virtual loudspeaker gains
        vls_gains = new float[nVirtualSpeakers];
        
        // Initialize with uniform gains (can be modified for different decoder types)
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            vls_gains[ls] = 1.0f / sqrt((float)nVirtualSpeakers);
        }
    }
    
    // Set up HRTF filters and convolution states
    void setupHRTFFilters() {
        // Allocate memory for HRTF filters for each virtual loudspeaker (left and right)
        hrtf_filters = new float*[nVirtualSpeakers * 2]; // left and right channels
        
        // Allocate memory for convolution states
        conv_states_l = new float*[nVirtualSpeakers];
        conv_states_r = new float*[nVirtualSpeakers];
        
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            // For each virtual loudspeaker, find the nearest HRIR
            int nearest_idx = findNearestHRIR(vls_dirs_deg[ls*2], vls_dirs_deg[ls*2+1]);
            
            // Get pointers to the left and right HRIRs for this direction
            const float* hrir_left = &hrirs[nearest_idx*2*hrir_len];
            const float* hrir_right = &hrirs[nearest_idx*2*hrir_len + hrir_len];
            
            // Copy HRIRs to our filter array
            hrtf_filters[ls*2] = new float[hrir_len];
            hrtf_filters[ls*2+1] = new float[hrir_len];
            
            memcpy(hrtf_filters[ls*2], hrir_left, hrir_len * sizeof(float));
            memcpy(hrtf_filters[ls*2+1], hrir_right, hrir_len * sizeof(float));
            
            // Initialize convolution states
            conv_states_l[ls] = new float[hrir_len];
            conv_states_r[ls] = new float[hrir_len];
            
            memset(conv_states_l[ls], 0, hrir_len * sizeof(float));
            memset(conv_states_r[ls], 0, hrir_len * sizeof(float));
        }
    }
    
    // Find the nearest HRIR direction for a given azimuth/elevation
    int findNearestHRIR(float azimuth_deg, float elevation_deg) {
        // Convert to the same convention as the stored directions
        float lookup_azi = azimuth_deg;
        if (lookup_azi < 0.0f) lookup_azi += 360.0f;  // Convert to 0-360
        float lookup_elev = elevation_deg;
        
        // Find the closest HRIR
        float min_dist = 1000000.0f;
        int nearest_idx = 0;
        
        for (int i = 0; i < N_hrir_dirs; i++) {
            float dir_azi = hrir_dirs_deg[i*2];
            float dir_elev = hrir_dirs_deg[i*2 + 1];
            
            // Compute angular distance
            float azi_diff = fabs(lookup_azi - dir_azi);
            if (azi_diff > 180.0f) azi_diff = 360.0f - azi_diff; // Wrap around
            float elev_diff = fabs(lookup_elev - dir_elev);
            
            // Simple Euclidean distance on the sphere (not perfect but works)
            float dist = sqrt(azi_diff*azi_diff + elev_diff*elev_diff);
            
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
    }
    
    // Decode ambisonic signals to binaural
    void decode(float** ambiSignals, float* leftOut, float* rightOut, int numSamples) {
        // Temporary buffer for virtual loudspeaker signals
        float* vls_signals = new float[nVirtualSpeakers * numSamples];
        memset(vls_signals, 0, nVirtualSpeakers * numSamples * sizeof(float));
        
        // 1. Decode ambisonic signals to virtual loudspeakers
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            for (int sh = 0; sh < nSH; sh++) {
                for (int n = 0; n < numSamples; n++) {
                    vls_signals[ls * numSamples + n] += dec_mat[ls*nSH + sh] * ambiSignals[sh][n];
                }
            }
            
            // Apply loudspeaker gain
            for (int n = 0; n < numSamples; n++) {
                vls_signals[ls * numSamples + n] *= vls_gains[ls];
            }
        }
        
        // 2. Apply HRTFs to each virtual loudspeaker signal
        // First, clear output buffers
        memset(leftOut, 0, numSamples * sizeof(float));
        memset(rightOut, 0, numSamples * sizeof(float));
        
        // Simple FIR filter implementation for each virtual loudspeaker
        for (int ls = 0; ls < nVirtualSpeakers; ls++) {
            // Get pointers to the HRTF filters for this loudspeaker
            float* hrtf_left = hrtf_filters[ls*2];
            float* hrtf_right = hrtf_filters[ls*2+1];
            
            // Get pointers to the convolution states
            float* state_l = conv_states_l[ls];
            float* state_r = conv_states_r[ls];
            
            // Process each sample
            for (int n = 0; n < numSamples; n++) {
                float sample = vls_signals[ls * numSamples + n];
                
                // Shift the state buffer (simple delay line implementation)
                for (int i = hrir_len - 1; i > 0; i--) {
                    state_l[i] = state_l[i-1];
                    state_r[i] = state_r[i-1];
                }
                
                // Add new sample to the beginning of the state buffer
                state_l[0] = sample;
                state_r[0] = sample;
                
                // Perform convolution
                float left_sample = 0.0f;
                float right_sample = 0.0f;
                
                for (int i = 0; i < hrir_len; i++) {
                    left_sample += state_l[i] * hrtf_left[i];
                    right_sample += state_r[i] * hrtf_right[i];
                }
                
                // Add to output
                leftOut[n] += left_sample;
                rightOut[n] += right_sample;
            }
        }
        
        delete[] vls_signals;
    }
};

// ===================
// Sine Tone Agent with Ambisonics Processing
// ===================
class SineAgent : public PositionedVoice {
public:
    float phase = 0.0f;
    float freq = 440.0f;
    float amplitude = 0.2f;
    float gain = 1.0f;
    
    // Current source direction
    float azimuth = 0.0f;
    float elevation = 30.0f;
    
    void onProcess(AudioIOData &io) override {
        int numFrames = io.framesPerBuffer();
        float phaseInc = 2.0f * M_PI * freq / io.framesPerSecond();
        
        // Generate sine wave
        for (int i = 0; i < numFrames; i++) {
            io.out(0, i) += amplitude * gain * sin(phase);
            io.out(1, i) += amplitude * gain * sin(phase);
            phase += phaseInc;
            if (phase > 2.0f * M_PI) 
                phase -= 2.0f * M_PI;
        }
    }
    
    void set(float azimuthDeg, float elevationDeg, float frequency, float gainVal) {
        // Convert spherical to Cartesian (unit sphere for pure direction)
        Vec3f position = sphericalToCartesian(azimuthDeg, elevationDeg, 1.0f);
        setPose(Pose(position));
        
        // Store current direction for binaural rendering
        azimuth = azimuthDeg;
        elevation = elevationDeg;
        freq = frequency;
        gain = gainVal;
    }
    
    void onTriggerOn() override {
        phase = 0.0f;
    }
};

// ===================
// Enhanced Pickable that tracks selection state
// ===================
class SelectablePickable : public PickableBB {
public:
    bool selected = false;

    bool onEvent(PickEvent e, Hit h) override {
        bool handled = PickableBB::onEvent(e, h);
        
        // Set selected on pick
        if (e.type == Pick && h.hit) {
            selected = true;
        }
        
        return handled;
    }
};

// ==========================
// Main App
// ==========================
struct MyApp : public App {
    DynamicScene scene;
    PickableManager pickableManager;
    
    // GUI controls
    ControlGUI gui;
    
    // Parameters for sine wave
    Parameter azimuthParam{"Azimuth", "", 0.0, "", -180.0, 180.0};
    Parameter elevationParam{"Elevation", "", 30.0, "", -90.0, 90.0};
    Parameter gainParam{"Gain", "", 1.0, "", 0.0, 2.0};
    Parameter freqParam{"Frequency", "Hz", 440.0, "", 20.0, 2000.0};
    
    // Additional parameter for ambisonic order
    Parameter orderParam{"Ambisonic Order", "", 1.0, "", 1.0, 3.0};
    
    // Meshes
    VAOMesh sourceMesh;
    VAOMesh sphereMesh;  // Reference sphere for Ambisonics
    
    // Sound source
    SineAgent* sineAgent = nullptr;
    
    // Flag to prevent feedback loop between parameter changes and pickable updates
    bool pickablesUpdatingParameters = false;
    
    // Fixed listener pose at origin
    Pose fixedListenerPose{Vec3f(0, 0, 0)};
    
    // Fixed radius for visualization
    float visualRadius = 3.0f;
    
    // Selectable pickable object
    SelectablePickable* pickable = nullptr;
    
    // Ambisonic encoder and decoder
    AmbisonicEncoder* ambiEncoder = nullptr;
    AmbisonicBinauralDecoder* ambiDecoder = nullptr;
    
    // Input and output buffers
    float* inputBuffer = nullptr;
    float* leftOutputBuffer = nullptr;
    float* rightOutputBuffer = nullptr;
    int bufferSize = 0;

    void onCreate() override {
        // Create ambisonic encoder and decoder (1st order initially)
        ambiEncoder = new AmbisonicEncoder(1);
        ambiDecoder = new AmbisonicBinauralDecoder(1);
        
        // Create source mesh for visualization
        addSphere(sourceMesh, 0.3);
        sourceMesh.primitive(Mesh::LINE_STRIP);
        sourceMesh.update();
        
        // Create reference sphere mesh
        addSphere(sphereMesh, visualRadius, 64, 64);
        sphereMesh.primitive(Mesh::LINES);
        sphereMesh.update();
        
        // Set up GUI
        imguiInit();
        
        gui.init(5, 5, false);
        gui.setTitle("Ambisonics Binaural Control");
        
        // Register parameter callbacks for GUI updates
        auto updateCallback = [this](float value) {
            if (!pickablesUpdatingParameters) {
                updateSourcePosition();
            }
        };
        
        azimuthParam.registerChangeCallback(updateCallback);
        elevationParam.registerChangeCallback(updateCallback);
        gainParam.registerChangeCallback(updateCallback);
        freqParam.registerChangeCallback(updateCallback);
        
        // Add callback for order change
        orderParam.registerChangeCallback([this](float value) {
            int newOrder = (int)value;
            updateAmbisonicOrder(newOrder);
        });
        
        // Create pickable object
        pickable = new SelectablePickable();
        pickable->set(sourceMesh);
        updateSourcePosition(); // Initialize position
        pickableManager << pickable;
        
        // Create and initialize sound agent
        sineAgent = scene.getVoice<SineAgent>();
        if (sineAgent) {
            sineAgent->set(azimuthParam.get(), elevationParam.get(), freqParam.get(), gainParam.get());
            scene.triggerOn(sineAgent);
        }
        
        // Allocate audio buffers based on the framesize
        bufferSize = audioIO().framesPerBuffer();
        inputBuffer = new float[bufferSize];
        leftOutputBuffer = new float[bufferSize];
        rightOutputBuffer = new float[bufferSize];
        
        // Prepare the scene for audio rendering
        scene.prepare(audioIO());
        
        // Set up camera - movable but doesn't affect audio
        nav().pos(0, 0, 10);
        
        std::cout << "Ambisonics-to-Binaural Renderer:" << std::endl;
        std::cout << "  1. Click and drag the sound source to move it on the sphere" << std::endl;
        std::cout << "  2. Use the GUI to adjust parameters" << std::endl;
        std::cout << "  3. Press SPACE to reset to default position" << std::endl;
        std::cout << "  4. Using SAF for Ambisonics encoding and binaural decoding" << std::endl;
    }
    
    // Update ambisonic order
    void updateAmbisonicOrder(int newOrder) {
        if (newOrder < 1 || newOrder > 3) {
            std::cerr << "Invalid order: " << newOrder << ". Must be between 1 and 3." << std::endl;
            return;
        }
        
        std::cout << "Updating ambisonic order to: " << newOrder << std::endl;
        
        // Delete and recreate encoder and decoder with new order
        delete ambiEncoder;
        delete ambiDecoder;
        
        ambiEncoder = new AmbisonicEncoder(newOrder, bufferSize);
        ambiDecoder = new AmbisonicBinauralDecoder(newOrder);
        
        // Update source position to recompute SH weights
        updateSourcePosition();
    }
    
    // Update pickable position based on current parameter values
    void updateSourcePosition() {
        if (sineAgent) {
            sineAgent->set(azimuthParam.get(), elevationParam.get(), freqParam.get(), gainParam.get());
        }
        
        if (pickable) {
            // For visualization, place the object on the sphere
            Vec3f position = sphericalToCartesian(azimuthParam.get(), elevationParam.get(), visualRadius);
            pickable->pose = Pose(position);
        }
        
        // Update ambisonic encoder with new direction
        if (ambiEncoder) {
            ambiEncoder->updateDirection(azimuthParam.get(), elevationParam.get());
        }
    }
    
    // Constrain a point to lie on the sphere
    Vec3f constrainToSphere(const Vec3f& point) {
        float radius = point.mag();
        if (radius < 0.0001f)
            return Vec3f(0, 0, -visualRadius);
        return point.normalized() * visualRadius;
    }
    
    void onAnimate(double dt) override {
        // Disable navControl when GUI is in use
        navControl().active(!gui.usingInput());
        
        // Update parameters if the object was moved by mouse interaction
        if (pickable && !gui.usingInput()) {
            pickablesUpdatingParameters = true;
            
            // Constrain to sphere
            Vec3f pos = pickable->pose.get().pos();
            Vec3f constrained = constrainToSphere(pos);
            pickable->pose.set(Pose(constrained));
            
            // Update parameters from position
            float az, el, radius;
            cartesianToSpherical(constrained, az, el, radius);
            
            azimuthParam.set(az);
            elevationParam.set(el);
            
            // Update audio source
            if (sineAgent) {
                sineAgent->set(az, el, freqParam.get(), gainParam.get());
            }
            
            // Update ambisonic encoder direction
            if (ambiEncoder) {
                ambiEncoder->updateDirection(az, el);
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
        
        // Draw reference sphere for Ambisonics
        g.color(0.3, 0.3, 0.3, 0.4);
        g.polygonMode(GL_LINE); // Use OpenGL constant for wireframe mode
        g.draw(sphereMesh);
        
        // Draw sound source
        if (pickable) {
            g.color(0.2, 0.8, 0.4); // Green for the sound source
            
            pickable->draw(g, [&](Pickable &p) {
                auto &b = dynamic_cast<PickableBB &>(p);
                b.drawMesh(g);
            });
            
            // Draw line from origin to source
            g.lineWidth(1.0);
            g.color(0.5, 0.5, 0.5, 0.3);
            Mesh line;
            line.primitive(Mesh::LINES);
            line.vertex(0, 0, 0);
            line.vertex(pickable->pose.get().pos());
            g.draw(line);
            
            // Get spherical coordinates for visualization helpers
            float az, el, radius;
            cartesianToSpherical(pickable->pose.get().pos(), az, el, radius);
            
            // Draw azimuth circle at the current elevation
            g.color(1.0, 1.0, 0.0, 0.5); // Yellow for azimuth
            g.lineWidth(2.0);
            Mesh azimuthCircle;
            azimuthCircle.primitive(Mesh::LINE_STRIP);
            
            float elRad = el * M_PI / 180.0f;
            float elRadius = visualRadius * cos(elRad);
            
            for (int i = 0; i <= 72; i++) {
                float angle = i * 5.0f * M_PI / 180.0f;
                float x = elRadius * sin(angle);
                float y = visualRadius * sin(elRad);
                float z = -elRadius * cos(angle);
                azimuthCircle.vertex(x, y, z);
            }
            g.draw(azimuthCircle);
            
            // Draw elevation arc
            g.color(1.0, 0.0, 1.0, 0.5); // Magenta for elevation
            Mesh elevationArc;
            elevationArc.primitive(Mesh::LINE_STRIP);
            
            float azRad = az * M_PI / 180.0f;
            for (int i = -18; i <= 18; i++) {
                float angle = i * 5.0f * M_PI / 180.0f;
                float x = visualRadius * cos(angle) * sin(azRad);
                float y = visualRadius * sin(angle);
                float z = -visualRadius * cos(angle) * cos(azRad);
                elevationArc.vertex(x, y, z);
            }
            g.draw(elevationArc);
        }
        
        // Draw GUI
        imguiBeginFrame();
        
        // Draw controls
        gui.draw(g);
        
        // Create custom ImGui controls for parameters
        ImGui::Begin("Source Controls");
        
        // Azimuth slider
        float azValue = azimuthParam.get();
        if (ImGui::SliderFloat("Azimuth", &azValue, -180.0f, 180.0f)) {
            azimuthParam.set(azValue);
            updateSourcePosition();
        }
        
        // Elevation slider
        float elValue = elevationParam.get();
        if (ImGui::SliderFloat("Elevation", &elValue, -90.0f, 90.0f)) {
            elevationParam.set(elValue);
            updateSourcePosition();
        }
        
        // Gain slider
        float gainValue = gainParam.get();
        if (ImGui::SliderFloat("Gain", &gainValue, 0.0f, 2.0f)) {
            gainParam.set(gainValue);
            updateSourcePosition();
        }
        
        // Frequency slider
        float freqValue = freqParam.get();
        if (ImGui::SliderFloat("Frequency", &freqValue, 20.0f, 2000.0f)) {
            freqParam.set(freqValue);
            updateSourcePosition();
        }
        
        // Ambisonic order slider
        float orderValue = orderParam.get();
        if (ImGui::SliderFloat("Ambisonic Order", &orderValue, 1.0f, 3.0f)) {
            orderValue = round(orderValue); // Round to nearest integer
            if (orderValue != orderParam.get()) {
                orderParam.set(orderValue);
                // Order change callback will take care of the rest
            }
        }
        
        ImGui::End();
        imguiEndFrame();
        imguiDraw();
    }

    void onSound(AudioIOData &io) override {
        // Pass fixed listener pose to scene (using proper reference)
        Pose& listenerPose = scene.listenerPose();
        listenerPose = fixedListenerPose;
        
        // Get buffer size
        int nFrames = io.framesPerBuffer();
        
        // Make sure our buffers are big enough
        if (nFrames > bufferSize) {
            delete[] inputBuffer;
            delete[] leftOutputBuffer;
            delete[] rightOutputBuffer;
            
            bufferSize = nFrames;
            inputBuffer = new float[bufferSize];
            leftOutputBuffer = new float[bufferSize];
            rightOutputBuffer = new float[bufferSize];
        }
        
        // Clear output buffers
        memset(leftOutputBuffer, 0, nFrames * sizeof(float));
        memset(rightOutputBuffer, 0, nFrames * sizeof(float));
        
        // Generate sine wave input
        if (sineAgent) {
            float phase = sineAgent->phase;
            float phaseInc = 2.0f * M_PI * sineAgent->freq / io.framesPerSecond();
            float amp = sineAgent->amplitude * sineAgent->gain;
            
            for (int i = 0; i < nFrames; i++) {
                inputBuffer[i] = amp * sin(phase);
                phase += phaseInc;
                if (phase > 2.0f * M_PI) 
                    phase -= 2.0f * M_PI;
            }
            
            // Save phase for next buffer
            sineAgent->phase = phase;
        }
        
        // Process through Ambisonics pipeline
        if (ambiEncoder && ambiDecoder) {
            // 1. Encode mono input to Ambisonics representation
            ambiEncoder->encode(inputBuffer, nFrames);
            
            // 2. Decode Ambisonics to binaural
            ambiDecoder->decode(ambiEncoder->ambiSignals, leftOutputBuffer, rightOutputBuffer, nFrames);
        }
        
        // Copy to output buffers
        for (int i = 0; i < nFrames; i++) {
            io.out(0, i) = leftOutputBuffer[i];
            io.out(1, i) = rightOutputBuffer[i];
        }
    }

    void onExit() override {
        // Clean up
        delete ambiEncoder;
        delete ambiDecoder;
        delete[] inputBuffer;
        delete[] leftOutputBuffer;
        delete[] rightOutputBuffer;
    }

    bool onKeyDown(const Keyboard &k) override {
        if (k.key() == ' ') {
            // Reset to default
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
  app.title("AlloLib Ambisonics-to-Binaural Renderer");
  app.configureAudio(44100, 256, 2, 0);  
  app.start();
  return 0;
}