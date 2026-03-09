#pragma once

#include <string>
#include <unordered_map>

namespace avs {

struct TopicInfo {
    std::string sensor_topic;
    std::string sensor_type;
    std::string folder_name;
};

using TopicMap = std::unordered_map<std::string, TopicInfo>;

TopicMap LoadTopicMap(const std::string& yaml_path);

// Returns folder_name for a given topic, or empty string if not found
std::string GetTopicFolder(const TopicMap& map,
                           const std::string& topic);

// Returns folder_name for a given topic, or empty string if not found
std::string GetTopicType(const TopicMap& map,
                           const std::string& topic);

} // namespace avs
