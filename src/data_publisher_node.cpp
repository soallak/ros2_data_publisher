#include <data_publisher/data_publisher.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 1) {
    throw std::runtime_error("Usage: data_publisher_node path");
  }

  const std::string path(argv[1]);
  std::shared_ptr<simulation::IDataPublisher> data_publisher =
      simulation::CreateDataPublisher(simulation::DatasetType::EUROC, path);

  data_publisher->Start();
  rclcpp::spin(data_publisher);
  rclcpp::shutdown();
}