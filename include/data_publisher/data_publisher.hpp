#pragma once

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace simulation {

class IDataPublisher : public rclcpp::Node {
 public:
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual void Pause() = 0;
  virtual void Restart() = 0;

 protected:
  IDataPublisher(std::string node_name) : rclcpp::Node(node_name) {}
};

enum class DatasetType : std::uint8_t { EUROC = 0, TUM };

std::unique_ptr<IDataPublisher> CreateDataPublisher(DatasetType type,
                                                    std::string path);

}  // namespace simulation