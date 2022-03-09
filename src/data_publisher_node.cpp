#include <data_publisher/data_publisher.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<simulation::IDataPublisher> data_publisher =
      simulation::CreateDataPublisher(simulation::DatasetType::EUROC);

  data_publisher->Start();
  rclcpp::spin(data_publisher->GetNode());
}