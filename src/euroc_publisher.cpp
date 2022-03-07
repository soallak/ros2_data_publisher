#include "euroc_publisher.hpp"

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <array>
#include <boost/filesystem.hpp>
#include <boost/filesystem/directory.hpp>
#include <boost/filesystem/path.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <queue>
#include <sensor_msgs/image_encodings.hpp>
#include <string>
#include <vector>

#include "data_publisher/data_publisher.hpp"

namespace simulation {

EurocPublisher::EurocPublisher(std::string path)
    : path_(path),
      paused_(true),
      node_(std::make_shared<rclcpp::Node>("euroc")),
      it_(node_) {
  Init();
}

EurocPublisher::~EurocPublisher() { Stop(); }

void EurocPublisher::Start() {
  pub_left_ = it_.advertise("left/image_raw", q_size_);
  pub_right_ = it_.advertise("right/image_raw", q_size_);
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

std::shared_ptr<rclcpp::Node> EurocPublisher::GetNode() { return node_; }

void EurocPublisher::Init() {
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

  auto compare = [this](fs::path const& p1, fs::path const& p2) -> bool {
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

  auto publish = [this](Image& img, ImagePublisher& pub) {
    if (!img.data.empty() && pub.getNumSubscribers()) {
      RCLCPP_DEBUG_STREAM(node_->get_logger(),
                          "Publishing topic: " << pub.getTopic() << "to "
                                               << pub.getNumSubscribers()
                                               << "subscribers");
      std_msgs::msg::Header header;
      header.frame_id = "camera";
      header.stamp = rclcpp::Time(img.timestamp);
      auto bridge_img = cv_bridge::CvImage(
          header, sensor_msgs::image_encodings::BGR8, img.data);

      pub.publish(bridge_img.toImageMsg());
    }
  };

  publish(left_img, pub_left_);
  publish(right_img, pub_right_);
}

}  // namespace simulation