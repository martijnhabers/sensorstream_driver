#include "sensorstream_driver/protocol_handler.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <usbmuxd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <thread>

namespace sensorstream {

static rclcpp::Logger& logger() {
  static rclcpp::Logger inst = rclcpp::get_logger("sensorstream");
  return inst;
}

// -------------------------
// ChunkReceiver
// -------------------------

ChunkReceiver::ChunkReceiver(uint32_t message_id) : message_id_(message_id) {}

void ChunkReceiver::add_chunk(uint32_t sequence, std::vector<uint8_t> data, bool is_last) {
  if (chunks_.count(sequence)) {
    RCLCPP_WARN(logger(), "Duplicate chunk %u for msg %u, ignoring", sequence, message_id_);
    return;
  }
  chunks_.emplace(sequence, std::move(data));
  if (is_last) last_received_ = true;
}

bool ChunkReceiver::is_complete() const {
  if (!last_received_) return false;
  uint32_t max_seq = std::max_element(chunks_.begin(), chunks_.end(),
    [](const auto& a, const auto& b) { return a.first < b.first; })->first;
  for (uint32_t i = 0; i <= max_seq; ++i) {
    if (!chunks_.count(i)) return false;
  }
  return true;
}

std::vector<uint8_t> ChunkReceiver::reconstruct() const {
  uint32_t max_seq = std::max_element(chunks_.begin(), chunks_.end(),
    [](const auto& a, const auto& b) { return a.first < b.first; })->first;
  std::vector<uint8_t> result;
  for (uint32_t i = 0; i <= max_seq; ++i) {
    const auto& chunk = chunks_.at(i);
    result.insert(result.end(), chunk.begin(), chunk.end());
  }
  return result;
}

// -------------------------
// Helpers
// -------------------------

static void set_recv_timeout(int sock, int timeout_ms) {
  struct timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

static constexpr size_t RECV_BUF_SIZE = 65536;

static uint32_t read_be32(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}

static uint32_t read_le32(const uint8_t* p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

static uint32_t read_net32(const uint8_t* p) {
  uint32_t v;
  std::memcpy(&v, p, 4);
  return ntohl(v);
}

static void dispatch_protobuf(const uint8_t* data, size_t len,
                               const MessageCallbacks& cb,
                               const char* transport) {
  if (len > static_cast<size_t>(std::numeric_limits<int>::max())) {
    RCLCPP_ERROR(logger(), "%s: protobuf message too large (%zu bytes)", transport, len);
    return;
  }
  sensor::SensorMessage msg;
  if (!msg.ParseFromArray(data, static_cast<int>(len))) {
    RCLCPP_ERROR(logger(), "%s: failed to parse protobuf message", transport);
    return;
  }

  if (msg.has_camera()) {
    const auto& cam = msg.camera();
    CameraFrame frame;
    const std::string& raw = cam.image_data();
    frame.jpeg_data.assign(reinterpret_cast<const uint8_t*>(raw.data()),
                           reinterpret_cast<const uint8_t*>(raw.data()) + raw.size());
    frame.timestamp_ns = cam.timestamp();
    frame.width = cam.width();
    frame.height = cam.height();
    frame.frame_id = cam.frame_id();
    if (cb.on_camera) cb.on_camera(std::move(frame));
    static size_t camera_count = 0;
    ++camera_count;
    if (camera_count <= 5 || camera_count % 30 == 0 || raw.empty() || cam.timestamp() == 0) {
      RCLCPP_INFO(logger(),
                  "%s camera frame decoded #%zu: timestamp=%lu, encoding='%s', size=%ux%u, image_bytes=%zu",
                  transport, camera_count, cam.timestamp(), cam.encoding().c_str(),
                  cam.width(), cam.height(), raw.size());
    }

  } else if (msg.has_imu()) {
    if (cb.on_imu) cb.on_imu(msg.imu());

  } else if (msg.has_depth()) {
    const auto& d = msg.depth();
    const std::string& raw = d.depth_data();
    if (raw.size() % sizeof(uint16_t) != 0) {
      RCLCPP_ERROR(logger(), "%s: depth data size not a multiple of 2", transport);
      return;
    }
    DepthFrame frame;
    frame.width  = d.width();
    frame.height = d.height();
    frame.timestamp_ns = d.timestamp();
    frame.frame_id = d.frame_id();
    frame.depth_data.resize(raw.size() / sizeof(uint16_t));
    std::memcpy(frame.depth_data.data(), raw.data(), raw.size());
    if (cb.on_depth) cb.on_depth(std::move(frame));
    RCLCPP_DEBUG(logger(), "%s depth frame dispatched", transport);

  } else if (msg.has_gps()) {
    if (cb.on_gps) cb.on_gps(msg.gps());
    RCLCPP_DEBUG(logger(), "%s GPS fix dispatched", transport);

  } else if (msg.has_pointcloud()) {
    if (cb.on_pointcloud) cb.on_pointcloud(msg.pointcloud());
    RCLCPP_DEBUG(logger(), "%s pointcloud dispatched", transport);

  } else {
    RCLCPP_WARN(logger(), "%s: unknown protobuf message type", transport);
  }
}

static bool looks_like_sensor_message(const uint8_t* data, size_t len) {
  if (len > static_cast<size_t>(std::numeric_limits<int>::max())) return false;
  sensor::SensorMessage msg;
  return msg.ParseFromArray(data, static_cast<int>(len)) &&
         (msg.has_camera() || msg.has_imu() || msg.has_depth() ||
          msg.has_gps() || msg.has_pointcloud());
}

// -------------------------
// USB handler (PeerTalk)
// -------------------------

static constexpr uint32_t USB_FRAME_TYPE_DATA      = 200;
static constexpr uint32_t USB_FRAME_TYPE_HEARTBEAT = 201;
static constexpr size_t   PEERTALK_HEADER_SIZE     = 16;

// Returns true if at least one valid frame was received (real iPhone connected).
static bool handle_usb_connection(int sock, const MessageCallbacks& cb,
                                  const std::atomic<bool>& stop_flag) {
  set_recv_timeout(sock, 200);

  std::vector<uint8_t> buf;
  buf.reserve(RECV_BUF_SIZE * 4);
  size_t offset = 0;
  bool ever_received_data = false;

  while (!stop_flag.load()) {
    uint8_t tmp[RECV_BUF_SIZE];
    ssize_t n = recv(sock, tmp, sizeof(tmp), 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  // timeout, re-check stop flag
      RCLCPP_ERROR(logger(), "USB: recv error: %s", strerror(errno));
      break;
    }
    if (n == 0) break;  // orderly close

    if (!ever_received_data) {
      ever_received_data = true;
      RCLCPP_INFO(logger(), "USB: iPhone connected");
    }

    buf.insert(buf.end(), tmp, tmp + n);

    // Process all complete PeerTalk frames
    while (true) {
      size_t available = buf.size() - offset;
      if (available < PEERTALK_HEADER_SIZE) break;

      const uint8_t* h = buf.data() + offset;
      // [version:4][type:4][tag:4][payload_size:4]  — big-endian network order
      uint32_t frame_type   = read_net32(h + 4);
      uint32_t payload_size = read_net32(h + 12);

      if (available < PEERTALK_HEADER_SIZE + payload_size) break;

      const uint8_t* payload = h + PEERTALK_HEADER_SIZE;

      if (frame_type == USB_FRAME_TYPE_DATA) {
        // [length:4][protobuf:variable]
        if (payload_size >= 4) {
          uint32_t pb_len = read_net32(payload);
          if (payload_size >= 4 + pb_len) {
            dispatch_protobuf(payload + 4, pb_len, cb, "USB");
          }
        }
      } else if (frame_type == USB_FRAME_TYPE_HEARTBEAT) {
        // ignore
      } else {
        RCLCPP_WARN(logger(), "USB: unknown PeerTalk frame type %u", frame_type);
      }

      offset += PEERTALK_HEADER_SIZE + payload_size;
    }

    // Compact buffer when at least half is consumed data
    if (offset > 0 && offset >= buf.size() / 2) {
      buf.erase(buf.begin(), buf.begin() + static_cast<ptrdiff_t>(offset));
      offset = 0;
    }
  }

  if (ever_received_data) {
    RCLCPP_INFO(logger(), "USB: iPhone disconnected");
  }
  return ever_received_data;
}

void start_usb_client(MessageCallbacks callbacks, int port,
                      const std::atomic<bool>& stop_flag) {
  constexpr int RETRY_DELAY_S = 3;
  constexpr int POLL_DELAY_S  = 1;
  bool connect_fail_logged = false;

  while (!stop_flag.load()) {
    // Query usbmuxd for connected devices
    usbmuxd_device_info_t* devices = nullptr;
    int count = usbmuxd_get_device_list(&devices);

    if (count <= 0 || !devices) {
      if (devices) usbmuxd_device_list_free(&devices);
      connect_fail_logged = false;
      std::this_thread::sleep_for(std::chrono::seconds(POLL_DELAY_S));
      continue;
    }

    // Use the first USB-connected device
    uint32_t handle = devices[0].handle;
    if (!connect_fail_logged) {
      RCLCPP_INFO(logger(), "USB: found device \"%s\", connecting to port %d",
                  devices[0].udid, port);
    }
    usbmuxd_device_list_free(&devices);

    // Connect directly to the device port via usbmuxd — no iproxy needed
    int sock = usbmuxd_connect(handle, static_cast<uint16_t>(port));
    if (sock < 0) {
      if (!connect_fail_logged) {
        RCLCPP_WARN(logger(), "USB: usbmuxd_connect failed (device may be in use by another instance)");
        connect_fail_logged = true;
      }
      std::this_thread::sleep_for(std::chrono::seconds(RETRY_DELAY_S));
      continue;
    }

    connect_fail_logged = false;
    bool had_data = handle_usb_connection(sock, callbacks, stop_flag);
    usbmuxd_disconnect(sock);

    if (had_data) {
      RCLCPP_INFO(logger(), "USB: connection ended, retrying in %ds", RETRY_DELAY_S);
      std::this_thread::sleep_for(std::chrono::seconds(RETRY_DELAY_S));
    }
  }
}

// -------------------------
// WiFi handler (TCP chunked)
// -------------------------

static constexpr size_t WIFI_HEADER_SIZE = 13;  // [msg_id:4][seq:4][is_last:1][data_size:4]
static constexpr uint32_t MAX_WIFI_PAYLOAD_SIZE = 32 * 1024 * 1024;

static void handle_wifi_connection(int sock, const sockaddr_in& addr,
                                   const MessageCallbacks& cb,
                                   const std::atomic<bool>& stop_flag) {
  set_recv_timeout(sock, 200);

  char addr_str[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &addr.sin_addr, addr_str, sizeof(addr_str));
  RCLCPP_INFO(logger(), "WiFi connection established with %s:%d",
              addr_str, ntohs(addr.sin_port));

  std::vector<uint8_t> buf;
  buf.reserve(RECV_BUF_SIZE * 4);
  size_t offset = 0;

  // message_id -> ChunkReceiver. Stale entries (lost chunk, ID wrap) are evicted
  // when a new message_id arrives that collides with an incomplete entry.
  std::unordered_map<uint32_t, ChunkReceiver> receivers;
  size_t recv_count = 0;
  bool warned_about_framing = false;

  while (!stop_flag.load()) {
    uint8_t tmp[RECV_BUF_SIZE];
    ssize_t n = recv(sock, tmp, sizeof(tmp), 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  // timeout, re-check stop flag
      RCLCPP_ERROR(logger(), "WiFi: recv error from %s:%d: %s",
                   addr_str, ntohs(addr.sin_port), strerror(errno));
      break;
    }
    if (n == 0) {
      RCLCPP_INFO(logger(), "WiFi connection closed by %s:%d",
                  addr_str, ntohs(addr.sin_port));
      break;
    }

    ++recv_count;
    if (recv_count <= 5 || recv_count % 30 == 0) {
      RCLCPP_INFO(logger(), "WiFi: received packet #%zu from %s:%d: %zd bytes",
                  recv_count, addr_str, ntohs(addr.sin_port), n);
    }

    buf.insert(buf.end(), tmp, tmp + n);

    while (true) {
      size_t available = buf.size() - offset;
      if (available < 4) break;

      // Some senders use a simpler [protobuf_length:4][protobuf] TCP stream.
      // Support it when the chunked 13-byte header is not plausible.
      uint32_t pb_len_be = read_be32(buf.data() + offset);
      uint32_t pb_len_le = read_le32(buf.data() + offset);
      uint32_t pb_len = pb_len_be <= MAX_WIFI_PAYLOAD_SIZE ? pb_len_be : pb_len_le;
      if (pb_len >= 8 && pb_len <= MAX_WIFI_PAYLOAD_SIZE && available >= 4 + pb_len &&
          looks_like_sensor_message(buf.data() + offset + 4, pb_len)) {
        RCLCPP_INFO(logger(), "WiFi: dispatching length-prefixed protobuf: %u bytes", pb_len);
        dispatch_protobuf(buf.data() + offset + 4, pb_len, cb, "WiFi");
        offset += 4 + pb_len;
        continue;
      }

      if (available < WIFI_HEADER_SIZE) {
        if (pb_len <= MAX_WIFI_PAYLOAD_SIZE && available >= 4 + pb_len) {
          RCLCPP_INFO(logger(), "WiFi: dispatching length-prefixed protobuf: %u bytes", pb_len);
          dispatch_protobuf(buf.data() + offset + 4, pb_len, cb, "WiFi");
          offset += 4 + pb_len;
          continue;
        }
        break;
      }

      const uint8_t* h = buf.data() + offset;
      uint32_t message_id = read_be32(h);
      uint32_t sequence   = read_be32(h + 4);
      bool     is_last    = h[8] != 0;
      uint32_t data_size  = read_be32(h + 9);

      if (data_size > MAX_WIFI_PAYLOAD_SIZE) {
        const uint32_t le_data_size = read_le32(h + 9);
        if (le_data_size <= MAX_WIFI_PAYLOAD_SIZE) {
          message_id = read_le32(h);
          sequence = read_le32(h + 4);
          data_size = le_data_size;
          if (recv_count <= 5) {
            RCLCPP_INFO(logger(), "WiFi: using little-endian chunk header");
          }
        }
      }

      if (data_size > MAX_WIFI_PAYLOAD_SIZE) {
        uint32_t pb_len_be = read_be32(h);
        uint32_t pb_len_le = read_le32(h);
        uint32_t pb_len = pb_len_be <= MAX_WIFI_PAYLOAD_SIZE ? pb_len_be : pb_len_le;
        if (pb_len <= MAX_WIFI_PAYLOAD_SIZE && available >= 4 + pb_len) {
          RCLCPP_WARN(logger(),
                      "WiFi: chunk header looked invalid (data_size=%u), trying length-prefixed protobuf (%u bytes)",
                      data_size, pb_len);
          dispatch_protobuf(h + 4, pb_len, cb, "WiFi");
          offset += 4 + pb_len;
          continue;
        }

        if (!warned_about_framing) {
          warned_about_framing = true;
          RCLCPP_WARN(logger(),
                      "WiFi: invalid frame header: msg_id=%u seq=%u is_last=%d data_size=%u available=%zu. "
                      "This usually means the iOS sender framing does not match [msg_id:4][seq:4][is_last:1][data_size:4].",
                      message_id, sequence, is_last, data_size, available);
        }
        break;
      }

      if (available < WIFI_HEADER_SIZE + data_size) break;

      const uint8_t* payload = h + WIFI_HEADER_SIZE;
      std::vector<uint8_t> chunk(payload, payload + data_size);
      offset += WIFI_HEADER_SIZE + data_size;

      auto it = receivers.find(message_id);
      if (it == receivers.end()) {
        // If this message_id was seen before but never completed, it's a stale
        // entry that was already erased. A fresh entry is correct here.
        it = receivers.emplace(message_id, ChunkReceiver(message_id)).first;
      } else if (sequence == 0) {
        // sequence 0 always starts a new message — evict any stale incomplete entry
        RCLCPP_WARN(logger(), "WiFi: evicting stale incomplete receiver for msg_id=%u", message_id);
        it->second = ChunkReceiver(message_id);
      }
      it->second.add_chunk(sequence, std::move(chunk), is_last);

      if (it->second.is_complete()) {
        auto complete = it->second.reconstruct();
        receivers.erase(it);
        dispatch_protobuf(complete.data(), complete.size(), cb, "WiFi");
      }
    }

    // Compact buffer when at least half is consumed data
    if (offset > 0 && offset >= buf.size() / 2) {
      buf.erase(buf.begin(), buf.begin() + static_cast<ptrdiff_t>(offset));
      offset = 0;
    }
  }
}

void start_wifi_server(MessageCallbacks callbacks, int port,
                       const std::atomic<bool>& stop_flag) {
  int server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (server_sock < 0) throw std::runtime_error("WiFi: failed to create server socket");

  int opt = 1;
  setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  set_recv_timeout(server_sock, 200);  // so accept() wakes up to re-check stop_flag

  sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port        = htons(static_cast<uint16_t>(port));

  if (bind(server_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    throw std::runtime_error("WiFi: bind failed");
  if (listen(server_sock, 5) < 0)
    throw std::runtime_error("WiFi: listen failed");

  RCLCPP_INFO(logger(), "WiFi server listening on 0.0.0.0:%d", port);

  while (!stop_flag.load()) {
    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int client_sock = accept(server_sock,
                             reinterpret_cast<sockaddr*>(&client_addr), &client_len);
    if (client_sock < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) continue;  // timeout, re-check stop flag
      if (!stop_flag.load()) {
        RCLCPP_ERROR(logger(), "WiFi: accept failed: %s", strerror(errno));
      }
      continue;
    }

    // Spawn a thread per client; detach so it cleans itself up on disconnect.
    // stop_flag outlives all threads since it lives in main().
    std::thread([client_sock, client_addr, callbacks, &stop_flag]() {
      handle_wifi_connection(client_sock, client_addr, callbacks, stop_flag);
      close(client_sock);
    }).detach();
  }

  close(server_sock);
}

}  // namespace sensorstream
