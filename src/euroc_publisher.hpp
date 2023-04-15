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
  bool IsValidPath(std::string path);
  void LoadImages();
  void Publish();

 private:
  using CameraPublisher = image_transport::CameraPublisher;
  using WallTimerPtr = rclcpp::WallTimer<std::function<void()>>::SharedPtr;

  struct Image {
    cv::Mat data;
    std::int64_t timestamp;
    boost::filesystem::path file;
  };

 private:
  std::atomic<bool> paused_;
  std::atomic<bool> initialized_;
  std::shared_ptr<rclcpp::Node> node_;
  image_transport::ImageTransport it_;
  CameraPublisher pub_left_;
  CameraPublisher pub_right_;
  WallTimerPtr pub_timer_;
  WallTimerPtr load_img_timer_;

  std::mutex img_q_mtx_;
  std::queue<Image> left_img_q_;
  std::queue<Image> right_img_q_;

  std::vector<boost::filesystem::path> left_files_;
  std::vector<boost::filesystem::path> right_files_;
  unsigned int left_files_idx_{0};
  unsigned int right_files_idx_{0};

  static constexpr unsigned int q_size_ = 8;  /// Max amount of images loaded

  // ROS parameters
  std::string path_;
  std::string frame_id_;
  int64_t period_ms_ = 50;  /// Publishing period
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;

  // CameraInfo
  // https://gitlab.plc2.de/Ouassil/ros2-image-pipeline-acceleration/-/blob/main/resources/euroc_stereo.yaml
  // I won't make this complicated by reading these values from yaml. I hard
  // code them here

  const double fx_ = 435.2046959714599;
  const double fy_ = 435.2046959714599;
  const double cx_ = 367.4517211914062;
  const double cy_ = 252.2008514404297;

  const double k_left_[9] = {458.654, 0.0, 367.215, 0.0, 457.296,
                             248.375, 0.0, 0.0,     1.0};
  const double d_left_[5] = {-0.28340811, 0.07395907, 0.00019359,
                             1.76187114e-05, 0.0};
  const double r_left_[9] = {
      0.999966347530033,     -0.001422739138722922, 0.008079580483432283,
      0.001365741834644127,  0.9999741760894847,    0.007055629199258132,
      -0.008089410156878961, -0.007044357138835809, 0.9999424675829176};
  const double k_right_[9] = {457.587, 0.0, 379.999, 0.0, 456.134,
                              255.238, 0.0, 0.0,     1};
  const double d_right_[5] = {-0.28368365, 0.07451284, -0.00010473,
                              -3.555907e-05, 0.0};
  const double r_right_[9] = {
      0.9999633526194376,    -0.003625811871560086, 0.007755443660172947,
      0.003680398547259526,  0.9999684752771629,    -0.007035845251224894,
      -0.007729688520722713, 0.007064130529506649,  0.999945173484644};

  const double focal_x_baseline_ = 47.90639384423901;
  unsigned int width_ = 752;
  unsigned int height_ = 480;
};

}  // namespace simulation
