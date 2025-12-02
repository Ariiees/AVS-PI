#include "avs/topic_map.h"
#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace avs {

TopicMap LoadTopicMap(const std::string& yaml_path) {
    TopicMap topic_map;

    YAML::Node config = YAML::LoadFile(yaml_path);
    
    for (auto it = config.begin(); it != config.end(); ++it) {
        const YAML::Node key_node   = it->first;
        const YAML::Node value_node = it->second;

        TopicInfo topic;
        topic.sensor_topic = key_node.as<std::string>();
        topic.sensor_type  = value_node["sensor_type"].as<std::string>();
        topic.folder_name  = value_node["folder_name"].as<std::string>();

        topic_map[topic.sensor_topic] = topic;
    }

    return topic_map;
}

std::string GetTopicFolder(const TopicMap& map,
                           const std::string& topic) {
    auto it = map.find(topic);
    if (it == map.end()) return "";
    return it->second.folder_name;
}

std::string GetTopicType(const TopicMap& map,
                         const std::string& topic) {
    auto it = map.find(topic);
    if (it == map.end()) return "";
    return it->second.sensor_type;
}

}  // namespace avs
