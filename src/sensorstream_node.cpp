#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <opencv2/imgcodecs.hpp>

#include "sensorstream_driver/banner.hpp"
#include "sensorstream_driver/protocol_handler.hpp"

namespace sensorstream {

// -------------------------
// Single-slot queue (keep-latest semantics)
// -------------------------

template <typename T>
class LatestQueue {
public:
  void push(T item) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      item_ = std::move(item);
    }
    cv_.notify_one();
  }

  std::optional<T> pop(const std::atomic<bool>& stop) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::milliseconds(100),
                 [&] { return item_.has_value() || stop.load(); });
    if (!item_.has_value()) return std::nullopt;
    std::optional<T> result = std::move(item_);
    item_.reset();
    return result;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> item_;
};

// -------------------------
// ROS 2 node
// -------------------------

class SensorStreamNode : public rclcpp::Node {
public:
  SensorStreamNode() : Node("sensorstream_driver") {
    declare_parameter("wifi_port", 5678);
    declare_parameter("usb_port",  2345);

    rclcpp::QoS camera_qos(rclcpp::KeepLast(5));
    camera_qos.best_effort();

    color_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "color_image/compressed", camera_qos);
    color_raw_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "color_image", camera_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "depth_image", rclcpp::QoS(10));
    imu_pub_   = create_publisher<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::QoS(10));
    gps_pub_   = create_publisher<sensor_msgs::msg::NavSatFix>(
      "gps/fix", rclcpp::QoS(10));
    pcl_pub_   = create_publisher<sensor_msgs::msg::PointCloud2>(
      "pointcloud", camera_qos);

    stop_.store(false);

    camera_worker_ = std::thread([this] { camera_loop();     });
    depth_worker_  = std::thread([this] { depth_loop();      });
    imu_worker_    = std::thread([this] { imu_loop();        });
    gps_worker_    = std::thread([this] { gps_loop();        });
    pcl_worker_    = std::thread([this] { pointcloud_loop(); });
  }

  ~SensorStreamNode() {
    stop_.store(true);
    if (camera_worker_.joinable()) camera_worker_.join();
    if (depth_worker_.joinable())  depth_worker_.join();
    if (imu_worker_.joinable())    imu_worker_.join();
    if (gps_worker_.joinable())    gps_worker_.join();
    if (pcl_worker_.joinable())    pcl_worker_.join();
  }

  int wifi_port() const { return get_parameter("wifi_port").as_int(); }
  int usb_port()  const { return get_parameter("usb_port").as_int();  }

  void enqueue_camera(CameraFrame frame)         { camera_q_.push(std::move(frame)); }
  void enqueue_depth(DepthFrame frame)           { depth_q_.push(std::move(frame));  }
  void enqueue_imu(sensor::IMUData imu)          { imu_q_.push(std::move(imu));      }
  void enqueue_gps(sensor::NavSatFix gps)        { gps_q_.push(std::move(gps));      }
  void enqueue_pointcloud(sensor::PointCloud2 pc){ pcl_q_.push(std::move(pc));       }

private:
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr color_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           color_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr             imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr       gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr     pcl_pub_;

  LatestQueue<CameraFrame>        camera_q_;
  LatestQueue<DepthFrame>         depth_q_;
  LatestQueue<sensor::IMUData>    imu_q_;
  LatestQueue<sensor::NavSatFix>  gps_q_;
  LatestQueue<sensor::PointCloud2> pcl_q_;

  std::atomic<bool> stop_;
  std::thread camera_worker_;
  std::thread depth_worker_;
  std::thread imu_worker_;
  std::thread gps_worker_;
  std::thread pcl_worker_;

  static builtin_interfaces::msg::Time ns_to_stamp(uint64_t ns) {
    builtin_interfaces::msg::Time t;
    t.sec    = static_cast<int32_t>(ns / 1'000'000'000ULL);
    t.nanosec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
    return t;
  }

  void camera_loop() {
    while (!stop_.load()) {
      auto item = camera_q_.pop(stop_);
      if (!item) continue;
      if (item->jpeg_data.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping camera frame with empty JPEG payload");
        continue;
      }

      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      if (item->timestamp_ns == 0) {
        msg->header.stamp = now();
      } else {
        msg->header.stamp = ns_to_stamp(item->timestamp_ns);
      }
      msg->header.frame_id = item->frame_id.empty() ? "color_image" : item->frame_id;
      msg->format          = "jpeg";
      msg->data            = std::move(item->jpeg_data);
      const size_t jpeg_bytes = msg->data.size();

      if (color_raw_pub_->get_subscription_count() > 0) {
        const cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1, msg->data.data());
        cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (decoded.empty()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Failed to decode JPEG camera frame for raw /color_image");
        } else {
          auto raw_msg = std::make_unique<sensor_msgs::msg::Image>();
          raw_msg->header = msg->header;
          raw_msg->height = static_cast<uint32_t>(decoded.rows);
          raw_msg->width = static_cast<uint32_t>(decoded.cols);
          raw_msg->encoding = "bgr8";
          raw_msg->is_bigendian = false;
          raw_msg->step = static_cast<uint32_t>(decoded.cols * decoded.elemSize());
          raw_msg->data.assign(decoded.datastart, decoded.dataend);
          color_raw_pub_->publish(std::move(raw_msg));
        }
      }

      color_compressed_pub_->publish(std::move(msg));

      static size_t published_count = 0;
      ++published_count;
      if (published_count <= 5 || published_count % 30 == 0) {
        RCLCPP_INFO(
          get_logger(), "Published camera frame #%zu: jpeg_bytes=%zu",
          published_count, jpeg_bytes);
      }
    }
  }

  void depth_loop() {
    while (!stop_.load()) {
      auto item = depth_q_.pop(stop_);
      if (!item || item->timestamp_ns == 0 || item->width == 0 || item->height == 0) continue;

      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp    = ns_to_stamp(item->timestamp_ns);
      msg->header.frame_id = item->frame_id;
      msg->width           = item->width;
      msg->height          = item->height;
      msg->encoding        = "16UC1";
      msg->is_bigendian    = false;
      msg->step            = item->width * sizeof(uint16_t);
      msg->data.resize(item->depth_data.size() * sizeof(uint16_t));
      std::memcpy(msg->data.data(), item->depth_data.data(), msg->data.size());
      depth_pub_->publish(std::move(msg));
    }
  }

  void imu_loop() {
    while (!stop_.load()) {
      auto item = imu_q_.pop(stop_);
      if (!item || item->timestamp() == 0) continue;

      auto msg = std::make_unique<sensor_msgs::msg::Imu>();
      msg->header.stamp    = ns_to_stamp(item->timestamp());
      msg->header.frame_id = item->frame_id();
      msg->orientation.x   = item->orientation().x();
      msg->orientation.y   = item->orientation().y();
      msg->orientation.z   = item->orientation().z();
      msg->orientation.w   = item->orientation().w();
      msg->angular_velocity.x    = item->gyro().x();
      msg->angular_velocity.y    = item->gyro().y();
      msg->angular_velocity.z    = item->gyro().z();
      msg->linear_acceleration.x = item->accel().x();
      msg->linear_acceleration.y = item->accel().y();
      msg->linear_acceleration.z = item->accel().z();
      imu_pub_->publish(std::move(msg));
    }
  }

  void gps_loop() {
    while (!stop_.load()) {
      auto item = gps_q_.pop(stop_);
      if (!item || item->timestamp() == 0) continue;

      auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
      msg->header.stamp    = ns_to_stamp(item->timestamp());
      msg->header.frame_id = item->frame_id();
      msg->status.status   = static_cast<int8_t>(item->status().status());
      msg->status.service  = static_cast<uint16_t>(item->status().service());
      msg->latitude        = item->latitude();
      msg->longitude       = item->longitude();
      msg->altitude        = item->altitude();
      const auto& cov = item->position_covariance();
      for (int i = 0, n = std::min(9, cov.size()); i < n; ++i) {
        msg->position_covariance[i] = cov[i];
      }
      msg->position_covariance_type =
        static_cast<uint8_t>(item->position_covariance_type());
      gps_pub_->publish(std::move(msg));
    }
  }

  void pointcloud_loop() {
    while (!stop_.load()) {
      auto item = pcl_q_.pop(stop_);
      if (!item || item->timestamp() == 0) continue;

      auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
      msg->header.stamp    = ns_to_stamp(item->timestamp());
      msg->header.frame_id = item->frame_id();
      msg->height          = item->height();
      msg->width           = item->width();
      msg->is_bigendian    = item->is_bigendian();
      msg->point_step      = item->point_step();
      msg->row_step        = item->row_step();
      msg->is_dense        = item->is_dense();

      const std::string& raw = item->data();
      msg->data.assign(reinterpret_cast<const uint8_t*>(raw.data()),
                       reinterpret_cast<const uint8_t*>(raw.data()) + raw.size());

      for (const auto& f : item->fields()) {
        sensor_msgs::msg::PointField pf;
        pf.name     = f.name();
        pf.offset   = f.offset();
        pf.datatype = static_cast<uint8_t>(f.datatype());
        pf.count    = f.count();
        msg->fields.push_back(std::move(pf));
      }

      pcl_pub_->publish(std::move(msg));
    }
  }
};

}  // namespace sensorstream


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<sensorstream::SensorStreamNode>();

  const int wifi_port = node->wifi_port();
  const int usb_port  = node->usb_port();

  sensorstream::print_banner(wifi_port, usb_port);

  // Capture node by value so the shared_ptr refcount keeps the node alive
  // even if stop_flag races with thread teardown.
  sensorstream::MessageCallbacks cb;
  cb.on_camera     = [node](sensorstream::CameraFrame f) { node->enqueue_camera(std::move(f));     };
  cb.on_depth      = [node](sensorstream::DepthFrame f)  { node->enqueue_depth(std::move(f));      };
  cb.on_imu        = [node](sensor::IMUData d)            { node->enqueue_imu(std::move(d));        };
  cb.on_gps        = [node](sensor::NavSatFix d)          { node->enqueue_gps(std::move(d));        };
  cb.on_pointcloud = [node](sensor::PointCloud2 d)        { node->enqueue_pointcloud(std::move(d)); };

  std::atomic<bool> stop_flag{false};

  std::thread wifi_thread([&cb, &stop_flag, wifi_port] {
    sensorstream::start_wifi_server(cb, wifi_port, stop_flag);
  });
  std::thread usb_thread([&cb, &stop_flag, usb_port] {
    sensorstream::start_usb_client(cb, usb_port, stop_flag);
  });

  rclcpp::spin(node);
  rclcpp::shutdown();

  stop_flag.store(true);
  if (wifi_thread.joinable()) wifi_thread.join();
  if (usb_thread.joinable())  usb_thread.join();

  return 0;
}
