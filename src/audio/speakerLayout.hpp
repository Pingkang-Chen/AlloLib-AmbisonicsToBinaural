#pragma once

#include "al/math/al_Vec.hpp"
#include <vector>
#include <string>
#include <cmath>

using namespace al;

/**
 * @brief T-design and custom speaker layout configurations for ambisonic decoding
 */
class SpeakerLayout {
public:
    enum TDesignType {
        MONO = 0,
        STEREO,
        QUAD,
        SURROUND_5_1,
        SURROUND_7_1,
        SURROUND_5_1_4,
        SURROUND_7_1_4,
        T4,
        T12,
        T24,
        T36,
        T48,
        ALLOSPHERE
    };
    
    /**
     * @brief Get speaker positions for a given layout
     */
    static std::vector<Vec3f> getTDesignPositions(TDesignType type, float radius = 1.0f) {
        std::vector<Vec3f> positions;
        
        const float (*coords)[2] = nullptr;
        int count = getSpeakerCount(type);
        
        switch (type) {
            case MONO:           coords = MONO_POSITIONS; break;
            case STEREO:         coords = STEREO_POSITIONS; break;
            case QUAD:           coords = QUAD_POSITIONS; break;
            case SURROUND_5_1:   coords = SURROUND_5_1_POSITIONS; break;
            case SURROUND_7_1:   coords = SURROUND_7_1_POSITIONS; break;
            case SURROUND_5_1_4: coords = SURROUND_5_1_4_POSITIONS; break;
            case SURROUND_7_1_4: coords = SURROUND_7_1_4_POSITIONS; break;
            case T4:             coords = T4_POSITIONS; break;
            case T12:            coords = T12_POSITIONS; break;
            case T24:            coords = T24_POSITIONS; break;
            case T36:            coords = T36_POSITIONS; break;
            case T48:            coords = T48_POSITIONS; break;
            case ALLOSPHERE:     coords = ALLOSPHERE_POSITIONS; break;
        }
        
        if (coords) {
            positions.reserve(count);
            for (int i = 0; i < count; i++) {
                Vec3f pos = sphericalToCartesian(coords[i][0], coords[i][1], radius);
                positions.push_back(pos);
            }
        }
        
        return positions;
    }
    
    /**
     * @brief Get spherical coordinates for a layout
     */
    static std::vector<std::pair<float, float>> getTDesignSphericalCoords(TDesignType type) {
        std::vector<std::pair<float, float>> coords;
        
        const float (*positions)[2] = nullptr;
        int count = getSpeakerCount(type);
        
        switch (type) {
            case MONO:           positions = MONO_POSITIONS; break;
            case STEREO:         positions = STEREO_POSITIONS; break;
            case QUAD:           positions = QUAD_POSITIONS; break;
            case SURROUND_5_1:   positions = SURROUND_5_1_POSITIONS; break;
            case SURROUND_7_1:   positions = SURROUND_7_1_POSITIONS; break;
            case SURROUND_5_1_4: positions = SURROUND_5_1_4_POSITIONS; break;
            case SURROUND_7_1_4: positions = SURROUND_7_1_4_POSITIONS; break;
            case T4:             positions = T4_POSITIONS; break;
            case T12:            positions = T12_POSITIONS; break;
            case T24:            positions = T24_POSITIONS; break;
            case T36:            positions = T36_POSITIONS; break;
            case T48:            positions = T48_POSITIONS; break;
            case ALLOSPHERE:     positions = ALLOSPHERE_POSITIONS; break;
        }
        
        if (positions) {
            coords.reserve(count);
            for (int i = 0; i < count; i++) {
                coords.emplace_back(positions[i][0], positions[i][1]);
            }
        }
        
        return coords;
    }
    
    /**
     * @brief Get recommended ambisonic order for a layout
     */
    static int getRecommendedOrder(TDesignType type) {
        return T_DESIGN_ORDERS[static_cast<int>(type)];
    }
    
    /**
     * @brief Get the number of speakers for a layout
     */
    static int getSpeakerCount(TDesignType type) {
        return T_DESIGN_COUNTS[static_cast<int>(type)];
    }
    
    /**
     * @brief Get human-readable name for a layout
     */
    static std::string getTDesignName(TDesignType type) {
        return std::string(T_DESIGN_NAMES[static_cast<int>(type)]);
    }
    
    /**
     * @brief Get all available layout options as strings
     */
    static std::vector<std::string> getAllTDesignNames() {
        std::vector<std::string> names;
        for (int i = 0; i < 13; i++) {
            names.emplace_back(T_DESIGN_NAMES[i]);
        }
        return names;
    }
    
private:
// Helper function for coordinate conversion
static Vec3f sphericalToCartesian(float azimuthDeg, float elevationDeg, float radius) {
    float azimuth_rad = azimuthDeg * M_PI / 180.0f;
    float elevation_rad = elevationDeg * M_PI / 180.0f;
    
    float x = radius * cos(elevation_rad) * sin(azimuth_rad);
    float y = radius * sin(elevation_rad);
    float z = radius * cos(elevation_rad) * (-cos(azimuth_rad));  // Note the negative sign
    
    return Vec3f(x, y, z);
}
    
    // Static arrays containing the actual speaker positions [azimuth, elevation]
    
    static constexpr float MONO_POSITIONS[][2] = {
        {0.0f, 0.0f}
    };
    
    static constexpr float STEREO_POSITIONS[][2] = {
        {30.0f, 0.0f}, {-30.0f, 0.0f}
    };
    
    static constexpr float QUAD_POSITIONS[][2] = {
        {45.0f, 0.0f}, {-45.0f, 0.0f}, {135.0f, 0.0f}, {-135.0f, 0.0f}
    };
    
    static constexpr float SURROUND_5_1_POSITIONS[][2] = {
        {30.0f, 0.0f}, {-30.0f, 0.0f}, {0.0f, 0.0f}, 
        {110.0f, 0.0f}, {-110.0f, 0.0f}
    };
    
    static constexpr float SURROUND_7_1_POSITIONS[][2] = {
        {30.0f, 0.0f}, {-30.0f, 0.0f}, {0.0f, 0.0f},
        {90.0f, 0.0f}, {-90.0f, 0.0f}, {135.0f, 0.0f}, {-135.0f, 0.0f}
    };
    
    static constexpr float SURROUND_5_1_4_POSITIONS[][2] = {
        {30.0f, 0.0f}, {-30.0f, 0.0f}, {0.0f, 0.0f}, 
        {110.0f, 0.0f}, {-110.0f, 0.0f},
        {30.0f, 45.0f}, {-30.0f, 45.0f}, {110.0f, 45.0f}, {-110.0f, 45.0f}
    };
    
    static constexpr float SURROUND_7_1_4_POSITIONS[][2] = {
        {30.0f, 0.0f}, {-30.0f, 0.0f}, {0.0f, 0.0f},
        {90.0f, 0.0f}, {-90.0f, 0.0f}, {135.0f, 0.0f}, {-135.0f, 0.0f},
        {30.0f, 45.0f}, {-30.0f, 45.0f}, {110.0f, 45.0f}, {-110.0f, 45.0f}
    };
    
    static constexpr float T4_POSITIONS[][2] = {
        {45.000f, 35.264f}, {-45.000f, -35.264f},
        {135.000f, -35.264f}, {-135.000f, 35.264f}
    };
    
    static constexpr float T12_POSITIONS[][2] = {
        {0.0f, -31.717f}, {-58.283f, 0.0f}, {-90.0f, 58.283f},
        {0.0f, 31.717f}, {-121.717f, 0.0f}, {90.0f, -58.283f},
        {180.0f, -31.717f}, {121.717f, 0.0f}, {90.0f, 58.283f},
        {180.0f, 31.717f}, {58.283f, 0.0f}, {-90.0f, -58.283f}
    };
    
    static constexpr float T24_POSITIONS[][2] = {
        {26.001f, 15.464f}, {-26.001f, -15.464f}, {17.109f, -24.994f}, {-17.109f, 24.994f},
        {153.999f, -15.464f}, {-153.999f, 15.464f}, {162.891f, 24.994f}, {-162.891f, -24.994f},
        {72.891f, 24.994f}, {107.109f, -24.994f}, {116.001f, 15.464f}, {63.999f, -15.464f},
        {-107.109f, 24.994f}, {-72.891f, -24.994f}, {-63.999f, 15.464f}, {-116.001f, -15.464f},
        {32.254f, 60.025f}, {-147.746f, 60.025f}, {-57.746f, 60.025f}, {122.254f, 60.025f},
        {-32.254f, -60.025f}, {147.746f, -60.025f}, {57.746f, -60.025f}, {-122.254f, -60.025f}
    };
    
    static constexpr float T36_POSITIONS[][2] = {
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
    
    static constexpr float T48_POSITIONS[][2] = {
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
    
    // UCSB Allosphere speaker layout - 54 speakers
    static constexpr float ALLOSPHERE_POSITIONS[][2] = {
        {-77.660913f, 41.000000f}, {-45.088015f, 41.000000f}, {-14.797289f, 41.000000f},
        {14.797289f, 41.000000f}, {45.088015f, 41.000000f}, {77.660913f, 41.000000f},
        {102.339087f, 41.000000f}, {134.911985f, 41.000000f}, {165.202711f, 41.000000f},
        {-165.202711f, 41.000000f}, {-134.911985f, 41.000000f}, {-102.339087f, 41.000000f},
        {-77.660913f, 0.000000f}, {-65.647587f, 0.000000f}, {-54.081600f, 0.000000f},
        {-42.869831f, 0.000000f}, {-31.928167f, 0.000000f}, {-21.181024f, 0.000000f},
        {-10.559657f, 0.000000f}, {0.000000f, 0.000000f}, {10.559657f, 0.000000f},
        {21.181024f, 0.000000f}, {31.928167f, 0.000000f}, {42.869831f, 0.000000f},
        {54.081600f, 0.000000f}, {65.647587f, 0.000000f}, {77.660913f, 0.000000f},
        {102.339087f, 0.000000f}, {114.352413f, 0.000000f}, {125.918400f, 0.000000f},
        {137.130169f, 0.000000f}, {148.071833f, 0.000000f}, {158.818976f, 0.000000f},
        {169.440343f, 0.000000f}, {180.000000f, 0.000000f}, {-169.440343f, 0.000000f},
        {-158.818976f, 0.000000f}, {-148.071833f, 0.000000f}, {-137.130169f, 0.000000f},
        {-125.918400f, 0.000000f}, {-114.352413f, 0.000000f}, {-102.339087f, 0.000000f},
        {-77.660913f, -32.500000f}, {-45.088015f, -32.500000f}, {-14.797289f, -32.500000f},
        {14.797289f, -32.500000f}, {45.088015f, -32.500000f}, {77.660913f, -32.500000f},
        {102.339087f, -32.500000f}, {134.911985f, -32.500000f}, {165.202711f, -32.500000f},
        {-165.202711f, -32.500000f}, {-134.911985f, -32.500000f}, {-102.339087f, -32.500000f}
    };
    
    static constexpr int T_DESIGN_COUNTS[] = {1, 2, 4, 5, 7, 9, 11, 4, 12, 24, 36, 48, 54};
    static constexpr int T_DESIGN_ORDERS[] = {0, 1, 1, 2, 2, 3, 3, 1, 3, 5, 6, 7, 7};
    static constexpr const char* T_DESIGN_NAMES[] = {
        "Mono", "Stereo", "4.0", "5.1", "7.1", "5.1.4", "7.1.4",
        "T-design (4)", "T-design (12)", "T-design (24)", "T-design (36)", "T-design (48)",
        "Allosphere (54)"
    };
};