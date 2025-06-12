#pragma once

#include "al/math/al_Vec.hpp"
#include <vector>
#include <string>
#include <cmath>

using namespace al;

/**
 * @brief T-design speaker layout configurations for ambisonic decoding
 */
class SpeakerLayout {
public:
    enum TDesignType {
        T4 = 0,   // T-design with 4 speakers
        T12 = 1,  // T-design with 12 speakers  
        T24 = 2,  // T-design with 24 speakers
        T36 = 3,  // T-design with 36 speakers
        T48 = 4   // T-design with 48 speakers
    };
    
    /**
     * @brief Get speaker positions for a given T-design layout
     */
    static std::vector<Vec3f> getTDesignPositions(TDesignType type, float radius = 1.0f) {
        std::vector<Vec3f> positions;
        
        const float (*coords)[2] = nullptr;
        int count = getSpeakerCount(type);
        
        switch (type) {
            case T4:  coords = T4_POSITIONS;  break;
            case T12: coords = T12_POSITIONS; break;
            case T24: coords = T24_POSITIONS; break;
            case T36: coords = T36_POSITIONS; break;
            case T48: coords = T48_POSITIONS; break;
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
     * @brief Get spherical coordinates for a T-design layout
     */
    static std::vector<std::pair<float, float>> getTDesignSphericalCoords(TDesignType type) {
        std::vector<std::pair<float, float>> coords;
        
        const float (*positions)[2] = nullptr;
        int count = getSpeakerCount(type);
        
        switch (type) {
            case T4:  positions = T4_POSITIONS;  break;
            case T12: positions = T12_POSITIONS; break;
            case T24: positions = T24_POSITIONS; break;
            case T36: positions = T36_POSITIONS; break;
            case T48: positions = T48_POSITIONS; break;
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
     * @brief Get recommended ambisonic order for a T-design layout
     */
    static int getRecommendedOrder(TDesignType type) {
        return T_DESIGN_ORDERS[static_cast<int>(type)];
    }
    
    /**
     * @brief Get the number of speakers for a T-design layout
     */
    static int getSpeakerCount(TDesignType type) {
        return T_DESIGN_COUNTS[static_cast<int>(type)];
    }
    
    /**
     * @brief Get human-readable name for a T-design layout
     */
    static std::string getTDesignName(TDesignType type) {
        return std::string(T_DESIGN_NAMES[static_cast<int>(type)]);
    }
    
    /**
     * @brief Get all available T-design options as strings
     */
    static std::vector<std::string> getAllTDesignNames() {
        std::vector<std::string> names;
        for (int i = 0; i < 5; i++) {
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
        float z = radius * cos(elevation_rad) * cos(azimuth_rad);
        
        return Vec3f(x, y, z);
    }
    
    // Static arrays containing the actual speaker positions
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
    
    static constexpr int T_DESIGN_COUNTS[] = {4, 12, 24, 36, 48};
    static constexpr int T_DESIGN_ORDERS[] = {1, 3, 5, 6, 7};
    static constexpr const char* T_DESIGN_NAMES[] = {
        "T-design (4)", "T-design (12)", "T-design (24)", "T-design (36)", "T-design (48)"
    };
};