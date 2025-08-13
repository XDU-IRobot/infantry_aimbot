/**
 * @file  ros_node.hpp
 * @brief 一个rclcpp::Node单例，程序通过这个节点使用ROS2相关的功能
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ros/publisher_pool.hpp"

namespace ia {
namespace ros {

class NodeSingleton {
 public:
  static NodeSingleton &GetInstance() {
    static NodeSingleton instance;
    return instance;
  }

  rclcpp::Node::SharedPtr node() { return node_; }

  // TODO: 不好用，推断不了类型，后面再改进
  template <typename MessageType>
  void Publish(const std::string &topic_name, typename MessageType::SharedPtr &&message) {
    static PublisherPool<MessageType> ppool{node_};
    ppool.Publish(topic_name, std::forward<decltype(message)>(message));
  }

 private:
  NodeSingleton()
      : node_(rclcpp::Node::make_shared(
            "infantry_aimbot_node",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).allow_undeclared_parameters(
                true))),
        executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    executor_->add_node(node_);
    executor_thread_ = std::thread([&] { executor_->spin(); });
  }
  ~NodeSingleton() {
    executor_->cancel();
    executor_->remove_node(node_);
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    node_.reset();
    executor_.reset();
  }

  NodeSingleton(const NodeSingleton &) = delete;
  NodeSingleton &operator=(const NodeSingleton &) = delete;
  NodeSingleton(NodeSingleton &&) = delete;
  NodeSingleton &operator=(NodeSingleton &&) = delete;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
};

}  // namespace ros
}  // namespace ia