#ifndef HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_DRIVER_H_
#define HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_DRIVER_H_

#include <ros/ros.h>

#include <chrono>
#include <mutex>

#include "hikvs_mv_im5005_ros/udp_message.h"
#include "hikvs_mv_im5005_ros/udp_socket.h"

namespace hikvs_mv {

constexpr double kFrequencySec = 1.0 / 50.0;
constexpr double kTimeOutSec = 1.0;

static float MicronMeter2MilliMeter(int32_t value) {
  return static_cast<float>(value / 1000.f);
}

class HikvsMvIm5005Driver {
  enum class DriverState : uint16_t {
    NORMAL = 0,
    ILLEGAL = 0xFFFE,      // 非法数据
    DISCONNECTED = 0xFFFF  // 掉线
  };

  struct HikvsMvIm5005Data {
    uint32_t time_stamp;
    uint64_t tag_number;
    float x_deviation;
    float y_deviation;
    float theta;

    HikvsMvIm5005Data()
        : time_stamp(0),
          tag_number(0),
          x_deviation(0.f),
          y_deviation(0.f),
          theta(0.f) {}
  };

 public:
  HikvsMvIm5005Driver();
  ~HikvsMvIm5005Driver() = default;

  bool Connect(std::string ip, int port);

  void Run();

  inline uint32_t time_stamp() const { return data_.time_stamp; }
  inline uint64_t tag_number() const { return data_.tag_number; }
  inline float x_deviation() const { return data_.x_deviation; }
  inline float y_deviation() const { return data_.y_deviation; }
  inline float theta() const { return data_.theta; }
  inline uint16_t GetErrorCode() {
    std::chrono::duration<double> time_cycle =
        std::chrono::system_clock::now() - previous_time_;
    if (time_cycle.count() >= kTimeOutSec) {
      driver_state_ = DriverState::DISCONNECTED;
    } else {
      driver_state_ = DriverState::NORMAL;
    }

    return static_cast<uint16_t>(driver_state_);
  }

 private:
  void MessageCallback(HikvsMvIm5005Message msg);

  std::shared_ptr<UdpSocket<HikvsMvIm5005Message>> socket_;

  std::mutex mutex_;

  std::chrono::system_clock::time_point previous_time_;

  DriverState driver_state_;
  HikvsMvIm5005Data data_;
};
}  // namespace hikvs_mv

#endif  // HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_DRIVER_H_