#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace simulation {

class IDataPublisher {
 public:
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual void Pause() = 0;
  virtual void Restart() = 0;
  virtual void SetPath(std::string path) = 0;
  virtual std::shared_ptr<rclcpp::Node> GetNode() = 0;
};

enum class DatasetType : std::uint8_t { EUROC = 0, TUM = 1 };

std::unique_ptr<IDataPublisher> CreateDataPublisher(DatasetType type);

}  // namespace simulation