#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "sensor.pb.h"

namespace sensorstream {

// Callback invoked for each decoded sensor message.
// The variant carries the specific message type.
struct CameraFrame {
  std::vector<uint8_t> jpeg_data;
  uint64_t timestamp_ns;
  uint32_t width;
  uint32_t height;
  std::string frame_id;
};

struct DepthFrame {
  std::vector<uint16_t> depth_data;  // row-major, mm units
  uint32_t width;
  uint32_t height;
  uint64_t timestamp_ns;
  std::string frame_id;
};

using OnCamera     = std::function<void(CameraFrame)>;
using OnDepth      = std::function<void(DepthFrame)>;
using OnImu        = std::function<void(sensor::IMUData)>;
using OnGps        = std::function<void(sensor::NavSatFix)>;
using OnPointCloud = std::function<void(sensor::PointCloud2)>;

struct MessageCallbacks {
  OnCamera     on_camera;
  OnDepth      on_depth;
  OnImu        on_imu;
  OnGps        on_gps;
  OnPointCloud on_pointcloud;
};

// -------------------------
// WiFi server
// -------------------------

// Starts a blocking TCP server on the given port.
// Spawns one thread per client connection.
// Call from a dedicated thread; loops until stop_flag is set.
void start_wifi_server(MessageCallbacks callbacks, int port,
                       const std::atomic<bool>& stop_flag);

// -------------------------
// USB client (PeerTalk via libusbmuxd)
// -------------------------

// Connects directly to the iPhone on the given port via usbmuxd with automatic
// device discovery and reconnection. No external iproxy process needed.
// Call from a dedicated thread; loops until stop_flag is set.
void start_usb_client(MessageCallbacks callbacks, int port,
                      const std::atomic<bool>& stop_flag);

// -------------------------
// Internal — chunk reassembly (used by WiFi handler)
// -------------------------

class ChunkReceiver {
public:
  explicit ChunkReceiver(uint32_t message_id);
  void add_chunk(uint32_t sequence, std::vector<uint8_t> data, bool is_last);
  bool is_complete() const;
  std::vector<uint8_t> reconstruct() const;

private:
  uint32_t message_id_;
  std::unordered_map<uint32_t, std::vector<uint8_t>> chunks_;
  bool last_received_{false};
};

}  // namespace sensorstream
