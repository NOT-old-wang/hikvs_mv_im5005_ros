#include "hikvs_mv_im5005_ros/hikvs_mv_im5005_driver.h"

namespace hikvs_mv {

HikvsMvIm5005Driver::HikvsMvIm5005Driver()
    : driver_state_(DriverState::DISCONNECTED) {}

bool HikvsMvIm5005Driver::Connect(std::string ip, int port) {
  ROS_INFO_STREAM("hikvs mv-im-5005 connecting");
  ROS_INFO_STREAM("ip: " << ip << ", port: " << port);
  if (ip.empty()) {
    ROS_ERROR("set ip error.");
    return false;
  }
  socket_ = std::make_shared<UdpSocket<HikvsMvIm5005Message>>(
      ip, static_cast<unsigned short>(port));
  return true;
}

void HikvsMvIm5005Driver::Run() {
  ROS_INFO_STREAM("hikvs mv-im-5005 run");
  socket_->Listen(std::bind(&HikvsMvIm5005Driver::MessageCallback, this,
                            std::placeholders::_1));
  previous_time_ = std::chrono::system_clock::now();
  socket_->Run();
}

void HikvsMvIm5005Driver::MessageCallback(HikvsMvIm5005Message msg) {
  std::unique_lock<std::mutex> lock(mutex_);
  previous_time_ = std::chrono::system_clock::now();

  if (!msg.Check()) {
    driver_state_ = DriverState::ILLEGAL;
    ROS_WARN("data check error.");
    return;
  }
  msg.Parse();

  data_.time_stamp = msg.time_stamp();
  data_.tag_number = static_cast<uint64_t>(std::atoi(msg.tag_number().c_str()));
  data_.x_deviation = MicronMeter2MilliMeter(msg.x_deviation());
  data_.y_deviation = MicronMeter2MilliMeter(msg.y_deviation());
  data_.theta = msg.theta() / 1000.f;

  ROS_DEBUG("algorithm_time_delay: %d", msg.algorithm_time_delay());
  ROS_DEBUG_STREAM("qr info: " << msg.qr_info());
  ROS_DEBUG_STREAM("tag_number: " << msg.tag_number());
}

}  // namespace hikvs_mv