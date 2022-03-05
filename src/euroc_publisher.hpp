#pragma once

#include <atomic>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <chrono>
#include <data_publisher/data_publisher.hpp>
#include <functional>
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
  EurocPublisher(std::string path);
  virtual ~EurocPublisher();
  void Start() override;
  void Stop() override;
  void Pause() override;
  void Restart() override;

 private:
  void Init();
  void LoadImages();
  void Publish();

 private:
  using ImagePublisherPtr =
      std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>;
  using WallTimerPtr = rclcpp::WallTimer<std::function<void()>>::SharedPtr;

  struct Image {
    cv::Mat data;
    std::int64_t timestamp;
    boost::filesystem::path file;
  };

 private:
  std::string path_;
  ImagePublisherPtr pub_left_;
  ImagePublisherPtr pub_right_;
  WallTimerPtr pub_timer_;
  WallTimerPtr load_img_timer_;

  std::atomic<bool> paused_;
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