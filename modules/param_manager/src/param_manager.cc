
#include "param_manager.hpp"

#include <exception>

ParamManager &ParamManager::instance() {
  static ParamManager instance;
  return instance;
}

void ParamManager::SetConfigPath(const std::filesystem::path &config_path) {
  // 验证config_path是否存在且为目录
  if (!std::filesystem::exists(config_path) || !std::filesystem::is_directory(config_path)) {
    throw std::runtime_error("Config path does not exist or is not a directory.");
  }
  config_path_ = config_path;
}

void ParamManager::Update() {
  // 检查config_path_是否已设置
  if (config_path_.empty()) {
    throw std::runtime_error("Config path is not set. Please call SetConfigPath first.");
  }
  // 遍历config目录下的所有toml文件
  for (const auto &entry : std::filesystem::directory_iterator(config_path_)) {
    if (entry.is_regular_file() && entry.path().extension() == ".toml") {
      try {
        const auto config = toml::parse(entry.path().string());
        config_map_[entry.path().stem().string()] = config;  // 解析后的toml对象存到unordered_map中，key为文件名
      } catch (const std::exception &e) {
        throw std::runtime_error("Error parsing TOML file: " + std::string(e.what()));
      }
    }
  }
}

const toml::value &ParamManager::operator[](const std::string &key) const {
  auto it = config_map_.find(key);
  if (it != config_map_.end()) {
    return it->second;
  } else {
    throw std::runtime_error("Key not found in configuration: " + key);
  }
};