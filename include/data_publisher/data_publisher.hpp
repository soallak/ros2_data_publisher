#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace simulation {

/**
 * @brief Data Publisher Interface
 */
class IDataPublisher {
 public:
  /**
   * @brief Initialize and start publishing data
   */
  virtual void Start() = 0;

  /**
   * @brief Stop publishing data and reset the internal state. After a stop a
   * start is necessary to start publishing again.
   */
  virtual void Stop() = 0;

  /**
   * @brief Pauses data publishing
   */
  virtual void Pause() = 0;

  /**
   * @brief Similar to Stop then Start
   */
  virtual void Restart() = 0;

  /**
   * @brief Set the dataset path
   *
   * @param path directory path to the dataset
   */
  virtual void SetPath(std::string path) = 0;

  /**
   * @brief Return the underlying node publishing the data
   *
   * @return std::shared_ptr<rclcpp::Node> the underlying node
   */
  virtual std::shared_ptr<rclcpp::Node> GetNode() = 0;
};

enum class DatasetType : std::uint8_t { EUROC = 0, TUM = 1 };

/**
 * @brief Create a Data Publisher
 *
 * @param type Type of the dataset
 * @return std::unique_ptr<IDataPublisher> a pointer IDataPublisher
 */
std::unique_ptr<IDataPublisher> CreateDataPublisher(DatasetType type);

}  // namespace simulation
