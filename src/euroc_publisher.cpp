#include "euroc_publisher.hpp"

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <array>
#include <boost/filesystem.hpp>
#include <boost/filesystem/directory.hpp>
#include <boost/filesystem/file_status.hpp>
#include <boost/filesystem/path.hpp>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <queue>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <string>
#include <vector>

#include "data_publisher/data_publisher.hpp"

namespace simulation {

EurocPublisher::EurocPublisher() : EurocPublisher(rclcpp::NodeOptions()) {}

EurocPublisher::~EurocPublisher() { Stop(); }

EurocPublisher::EurocPublisher(rclcpp::NodeOptions const& options)
    : paused_(true),
      initialized_(false),
      node_(std::make_shared<rclcpp::Node>("euroc", options)),
      it_(node_) {
  path_ = node_->declare_parameter("dataset_path", std::string(""));
  period_ms_ = node_->declare_parameter("period_ms", period_ms_);
  frame_id_ = node_->declare_parameter("frame_id", "camera_frame");
  // Autostart
  Start();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
EurocPublisher::get_node_base_interface() {
  if (node_) return node_->get_node_base_interface();
}

void EurocPublisher::Start() {
  if (!paused_) return;

  if (!initialized_) {
    Init();
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing dataset in " << path_);
  pub_left_ = it_.advertiseCamera("left/image_raw", q_size_);
  pub_right_ = it_.advertiseCamera("right/image_raw", q_size_);

  std::chrono::milliseconds period(period_ms_);
  std::function<void()> publish_callback =
      std::bind(&EurocPublisher::Publish, this);
  pub_timer_ = node_->create_wall_timer(period, publish_callback);

  std::function<void()> load_img_callback =
      std::bind(&EurocPublisher::LoadImages, this);
  load_img_timer_ = node_->create_wall_timer(period / 2, load_img_callback);
  paused_ = false;
}

void EurocPublisher::Stop() {
  paused_ = true;
  pub_timer_.reset();
  load_img_timer_.reset();
  pub_left_.shutdown();
  pub_right_.shutdown();
  std::lock_guard<std::mutex> lk(img_q_mtx_);
  left_files_.clear();
  right_files_.clear();
  left_img_q_ = std::queue<Image>();
  right_img_q_ = std::queue<Image>();
}

void EurocPublisher::Pause() { paused_ = true; }

void EurocPublisher::Restart() {
  Stop();
  Init();
  Start();
}

void EurocPublisher::SetPath(std::string path) {
  path_ = path;
  rclcpp::Parameter param("dataset_path", path_);
  node_->set_parameter(param);
  Restart();
}

std::shared_ptr<rclcpp::Node> EurocPublisher::GetNode() { return node_; }

void EurocPublisher::Init() {
  if (!IsValidPath(path_)) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Dataset path " << path_ << " is not valid");
    return;
  }

  namespace fs = boost::filesystem;
  auto leftdir = fs::path(path_ + "/cam0/data/");
  auto rightdir = fs::path(path_ + "/cam1/data/");

  auto read_image_files = [](fs::path dir) -> std::vector<fs::path> {
    auto dir_it = fs::directory_iterator(dir);
    std::vector<fs::path> files;
    for (auto it = dir_it; it != fs::directory_iterator(); ++it) {
      files.push_back(it->path());
    }
    return files;
  };

  left_files_ = read_image_files(leftdir);
  right_files_ = read_image_files(rightdir);

  auto compare = [](fs::path const& p1, fs::path const& p2) -> bool {
    auto timestamp1_str = p1.filename().stem().string();
    auto timestamp2_str = p2.filename().stem().string();

    auto timestamp1 = std::stol(timestamp1_str);
    auto timestamp2 = std::stol(timestamp2_str);

    return timestamp1 < timestamp2;
  };

  std::sort(left_files_.begin(), left_files_.end(), compare);
  std::sort(right_files_.begin(), right_files_.end(), compare);
  if (left_files_.size() != right_files_.size()) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "Amount of left images "
                           << left_files_.size()
                           << " is different than the amount of right images "
                           << right_files_.size());
  }
  left_files_idx_ = 0;
  right_files_idx_ = 0;
  initialized_ = true;
}

bool EurocPublisher::IsValidPath(std::string path) {
  namespace fs = boost::filesystem;
  fs::path root_path(path);
  fs::path cam0_path(path + "/cam0/data");
  fs::path cam1_path(path + "/cam1/data");

  return fs::is_directory(cam0_path) and fs::is_directory(cam1_path);
}

void EurocPublisher::LoadImages() {
  if (paused_ || (left_files_idx_ == left_files_.size() &&
                  right_files_idx_ == right_files_.size()))
    return;

  std::unique_lock<std::mutex> lk(img_q_mtx_);
  if (left_img_q_.size() < q_size_ / 2 && right_img_q_.size() < q_size_ / 2) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Loading images from disk");
    auto load_images = [&lk, q_size = q_size_](
                           std::queue<Image>& q,
                           std::vector<boost::filesystem::path> const& files,
                           unsigned int idx) -> int {
      int i = 0;
      while (q.size() <= q_size && (idx + i) < files.size()) {
        lk.unlock();
        // read images outside of lock
        Image img;
        img.file = files[idx + i++];
        img.data = cv::imread(img.file.string());
        img.timestamp = std::stol(img.file.filename().stem().string());
        // then push
        lk.lock();
        q.push(img);
      }
      lk.unlock();
      return i;
    };
    left_files_idx_ += load_images(left_img_q_, left_files_, left_files_idx_);
    lk.lock();
    right_files_idx_ +=
        load_images(right_img_q_, right_files_, right_files_idx_);

    if (left_files_idx_ == left_files_.size() &&
        right_files_idx_ == right_files_.size()) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "All images have been loaded");
    }
  }
}

void EurocPublisher::Publish() {
  if (paused_) return;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Publishing");
  auto front_and_pop = [](std::queue<Image>& q) -> Image {
    if (!q.empty()) {
      auto image = q.front();
      q.pop();
      return image;
    } else {
      return {cv::Mat(), 0, boost::filesystem::path()};
    }
  };

  Image left_img;
  Image right_img;
  {
    std::lock_guard<std::mutex> lk(img_q_mtx_);
    left_img = front_and_pop(left_img_q_);
    right_img = front_and_pop(right_img_q_);
  }

  auto publish = [this](Image& img, sensor_msgs::msg::CameraInfo& info,
                        CameraPublisher& pub) {
    if (!img.data.empty() && pub.getNumSubscribers()) {
      RCLCPP_DEBUG_STREAM(node_->get_logger(),
                          "Publishing topic: " << pub.getTopic() << "to "
                                               << pub.getNumSubscribers()
                                               << "subscribers");
      std_msgs::msg::Header header;
      header.frame_id = frame_id_;
      header.stamp = rclcpp::Time(img.timestamp);
      auto bridge_img = cv_bridge::CvImage(
          header, sensor_msgs::image_encodings::BGR8, img.data);

      info.header = header;

      pub.publish(*bridge_img.toImageMsg(), info);
    }
  };

  const double k[9] = {fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1};

  auto init_camera_info_msg = [width = width_, height = height_](
                                  double const raw_k[9], double const d[5],
                                  double const r[9]) {
    sensor_msgs::msg::CameraInfo info;
    info.width = width;
    info.height = height;
    info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    std::memcpy(info.k.data(), raw_k, 9 * sizeof(double));
    info.d.resize(5);
    std::memcpy(info.d.data(), d, 5 * sizeof(double));
    std::memcpy(info.r.data(), r, 9 * sizeof(double));

    std::memcpy(&info.p[0], raw_k, 3 * sizeof(double));
    info.p[3] = 0.;
    std::memcpy(&info.p[4], &raw_k[3], 3 * sizeof(double));
    info.p[7] = 0.;
    std::memcpy(&info.p[8], &raw_k[6], 3 * sizeof(double));
    info.p[11] = 0.;

    return info;
  };

  sensor_msgs::msg::CameraInfo left_info =
      init_camera_info_msg(k, d_left_, r_left_);
  sensor_msgs::msg::CameraInfo right_info =
      init_camera_info_msg(k, d_right_, r_right_);

  right_info.p[3] = -right_info.p[0] * focal_x_baseline_;

  publish(left_img, left_info, pub_left_);
  publish(right_img, right_info, pub_right_);
}

}  // namespace simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simulation::EurocPublisher)