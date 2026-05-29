#include "ptcamera/settings.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <exception>
#include <sstream>
#include <string>

namespace ptcamera {

namespace {

std::string trim(const std::string& value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) { return std::isspace(c); });
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::string stripQuotes(std::string value) {
    value = trim(value);
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

bool parseBool(std::string value, bool fallback) {
    value = stripQuotes(value);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "true" || value == "yes" || value == "1") {
        return true;
    }
    if (value == "false" || value == "no" || value == "0") {
        return false;
    }
    return fallback;
}

void loadByteTrackConfig(TrackerSettings& settings) {
    namespace fs = std::filesystem;

    fs::path configPath = settings.trackerConfigPath.empty()
                              ? fs::path(PTCAMERA_PROJECT_ROOT) / "bytetrack_drone.yaml"
                              : fs::path(settings.trackerConfigPath);
    if (!fs::exists(configPath)) {
        return;
    }
    settings.trackerConfigPath = configPath.string();

    std::ifstream input(configPath);
    std::string line;
    while (std::getline(input, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        const std::string key = trim(line.substr(0, colon));
        const std::string value = stripQuotes(line.substr(colon + 1));
        if (key.empty() || value.empty()) {
            continue;
        }

        try {
            if (key == "tracker_type") {
                settings.trackerBackend = value;
            } else if (key == "track_high_thresh") {
                settings.trackHighThreshold = std::stof(value);
            } else if (key == "track_low_thresh") {
                settings.trackLowThreshold = std::stof(value);
            } else if (key == "new_track_thresh") {
                settings.newTrackThreshold = std::stof(value);
            } else if (key == "track_buffer") {
                settings.trackBuffer = std::stoi(value);
            } else if (key == "match_thresh") {
                settings.trackMatchThreshold = std::stof(value);
            } else if (key == "fuse_score") {
                settings.fuseScore = parseBool(value, settings.fuseScore);
            }
        } catch (const std::exception&) {
            // Keep defaults if a local YAML value is malformed.
        }
    }
}

}  // namespace

std::string defaultModelPath() {
    return (std::filesystem::path(PTCAMERA_PROJECT_ROOT) / "models" / "drone_yolov8x" /
            "best.onnx")
        .string();
}

TrackerSettings defaultSettings() {
    TrackerSettings settings;
    settings.modelPath = defaultModelPath();
    loadByteTrackConfig(settings);
    return settings;
}

}  // namespace ptcamera
