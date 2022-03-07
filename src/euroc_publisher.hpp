#pragma once

#include <atomic>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <chrono>
#include <data_publisher/data_publisher.hpp>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace simulation {

class EurocPublisher : public IDataPublisher,
                       public std::enable_shared_from_this<EurocPublisher> {
 public:
  EurocPublisher();
  virtual ~EurocPublisher();

  /**
  This is need to make this a component
  */
  explicit EurocPublisher(rclcpp::NodeOptions const& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface();

  void Start() override;
  void Stop() override;
  void Pause() override;
  void Restart() override;
  void SetPath(std::string path) override;
  std::shared_ptr<rclcpp::Node> GetNode() override;

 private:
  void Init();
  void LoadImages();
  void Publish();

 private:
  using ImagePublisher = image_transport::Publisher;
  using WallTimerPtr = rclcpp::WallTimer<std::function<void()>>::SharedPtr;

  struct Image {
    cv::Mat data;
    std::int64_t timestamp;
    boost::filesystem::path file;
  };

 private:
  std::string path_;
  std::atomic<bool> paused_;
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::ImageTransport it_;
  ImagePublisher pub_left_;
  ImagePublisher pub_right_;
  WallTimerPtr pub_timer_;
  WallTimerPtr load_img_timer_;

  std::mutex img_q_mtx_;
  std::queue<Image> left_img_q_;
  std::queue<Image> right_img_q_;

  std::vector<boost::filesystem::path> left_files_;
  std::vector<boost::filesystem::path> right_files_;
  unsigned int left_files_idx_{0};
  unsigned int right_files_idx_{0};

  static constexpr unsigned int q_size_ = 10;  /// Max amount of images loaded
  static constexpr unsigned int period_ms_ = 50;  /// Publishing period
};

}  // namespace simulation