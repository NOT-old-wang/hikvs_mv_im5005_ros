#ifndef HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_UDP_MESSAGE_H_
#define HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_UDP_MESSAGE_H_

#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace hikvs_mv {
// 300字节缓存, mv_im5005 实际一包 268 字节
constexpr std::size_t kMaxUdpBuffer = 300;
constexpr std::size_t kHikvsBufferSize = 268;

constexpr char kHeader[5] = {'H', 'K', 'V', 'S'};
constexpr std::size_t kHeaderSize = 4;
constexpr char kSymbol_1 = {'<'};
constexpr char kSymbol_2 = {'>'};
constexpr char kSymbol_3 = {'@'};
constexpr std::size_t kTimeStampOffset = 20;
constexpr std::size_t kAlgorithmTimeDelay = kTimeStampOffset + 4;
constexpr std::size_t kSymbol1Index = 36;
// constexpr std::size_t kSymbol2IndexLimit = 93;
constexpr std::size_t kXdeviationOffset = 248;
constexpr std::size_t kYdeviationOffset = kXdeviationOffset + 4;
constexpr std::size_t kThetadeviationOffset = kYdeviationOffset + 4;

uint32_t GetData(const uint8_t *buf) {
  uint32_t value = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
  return std::move(value);
}

class UdpMessage {
 public:
  UdpMessage() = default;
  virtual ~UdpMessage() = default;

  inline virtual unsigned char *Data() { return buffer_; }

  inline virtual std::size_t ReadBufferSize() { return kMaxUdpBuffer; }

  inline virtual void Clear() { std::memset(buffer_, 0, kMaxUdpBuffer); }

 protected:
  unsigned char buffer_[kMaxUdpBuffer];
};

class HikvsMvIm5005Message : public UdpMessage {
 public:
  HikvsMvIm5005Message()
      : localizer_success_(false),
        time_stamp_(0),
        algorithm_time_delay_(0),
        x_deviation_(0),
        y_deviation_(0),
        theta_(0) {}
  ~HikvsMvIm5005Message() = default;

  bool Check() {
    char header[5] = "";
    std::memcpy(&header, buffer_, kHeaderSize);
    if (std::strcmp(header, kHeader) != 0) {
      return false;
    }
    return true;
  }

  void Parse() {
    time_stamp_ = GetData(&buffer_[kTimeStampOffset]);
    algorithm_time_delay_ = GetData(&buffer_[kAlgorithmTimeDelay]);

    if (kSymbol_1 == buffer_[kSymbol1Index]) {
      auto i = kSymbol1Index;
      for (i; buffer_[i] != kSymbol_2; i++) {
        qr_info_.push_back(buffer_[i]);
        // TODO, 数据包中无'>'
      }
      qr_info_.push_back(buffer_[i]);

      auto size = qr_info_.find(kSymbol_3);
      if (std::string::npos != size) {
        tag_number_ = qr_info_.substr(1, size - 1);
      }
    }

    x_deviation_ = GetData(&buffer_[kXdeviationOffset]);
    y_deviation_ = GetData(&buffer_[kYdeviationOffset]);
    theta_ = GetData(&buffer_[kThetadeviationOffset]);
  }

  inline uint32_t time_stamp() const { return time_stamp_; }
  inline uint32_t algorithm_time_delay() {
    // ms
    return algorithm_time_delay_;
  }
  inline std::string qr_info() const { return qr_info_; }
  inline std::string tag_number() const { return tag_number_; }
  inline int32_t x_deviation() const {
    return static_cast<int32_t>(x_deviation_);
  }
  inline int32_t y_deviation() const {
    return static_cast<int32_t>(y_deviation_);
  }
  inline int32_t theta() const { return static_cast<int32_t>(theta_); }

  virtual void Clear() {
    time_stamp_ = 0;
    algorithm_time_delay_ = 0;
    qr_info_.clear();
    x_deviation_ = 0;
    y_deviation_ = 0;
    theta_ = 0;
    std::memset(buffer_, 0, kMaxUdpBuffer);
  }

 private:
  bool localizer_success_;

  uint32_t time_stamp_;
  uint32_t algorithm_time_delay_;

  std::string qr_info_;
  std::string tag_number_;
  uint32_t x_deviation_;
  uint32_t y_deviation_;
  uint32_t theta_;
};

}  // namespace hikvs_mv

#endif  // HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_UDP_MESSAGE_H_
