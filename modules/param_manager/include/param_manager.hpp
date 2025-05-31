
#ifndef PARAM_MANAGER_HPP
#define PARAM_MANAGER_HPP

#include <filesystem>
#include <unordered_map>

#include <toml.hpp>

/**
 * @brief 单例类，管理程序运行中的所有参数，参数从config文件夹下的toml文件中读取。
 */
class ParamManager {
 public:
  /**
   * @brief   获取ParamManager的单例实例
   * @return  ParamManager& 单例实例
   */
  static ParamManager &instance();

  /**
   * @brief   设置配置文件夹路径
   * @param   config_path 配置文件夹路径
   * @throws  std::runtime_error 如果config_path不存在或不是目录
   */
  void SetConfigPath(const std::filesystem::path &config_path);

  /**
   * @brief   重新遍历config目录下的所有toml文件，并解析它们
   * @throws  std::runtime_error 如果config_path_未设置或解析失败
   * @throws  std::runtime_error 如果解析的toml文件有错误
   */
  void Update();

  /**
   * @brief   获取指定配置文件的所有配置项
   * @param   key   配置文件名（不带扩展名）
   * @return  const toml::value&
   */
  const toml::value &operator[](const std::string &key) const;

  ParamManager(const ParamManager &) = delete;
  ParamManager &operator=(const ParamManager &) = delete;

 private:
  ParamManager() = default;
  ~ParamManager() = default;

  std::filesystem::path config_path_{};                        ///< config文件夹路径
  std::unordered_map<std::string, toml::value> config_map_{};  ///< 存储解析后的配置
};

#endif  // PARAM_MANAGER_HPP