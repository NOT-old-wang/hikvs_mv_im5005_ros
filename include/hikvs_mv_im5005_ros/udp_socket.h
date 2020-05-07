#ifndef HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_UDP_SOCKET_H_
#define HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_UDP_SOCKET_H_

#include <boost/asio.hpp>
#include <deque>
#include <functional>
#include <string>
#include <thread>
#include <utility>

namespace hikvs_mv {

using boost::asio::ip::address_v4;
using boost::asio::ip::udp;

template <typename MessageType>
class UdpSocket {
 public:
  explicit UdpSocket(std::string ip, unsigned short port)
      : local_host_(udp::v4(), port),
        remote_host_(address_v4::from_string(ip), port) {
    socket_ = std::make_shared<udp::socket>(service_, local_host_);
  }

  ~UdpSocket() {
    service_.post([this]() { socket_->close(); });
    service_.stop();
  }

  void Run() {
    runner_ = std::make_shared<std::thread>([this]() {
      try {
        service_.run();
      } catch (boost::system::system_error &e) {
        std::cout << e.what() << std::endl;
      }
    });
  }

  void Close() {
    service_.post([this]() { socket_->close(); });
  }

  void Listen(std::function<void(MessageType)> handler) { DoRead(handler); }

 protected:
  void DoRead(std::function<void(MessageType)> handler) {
    receive_message_.Clear();
    socket_->async_receive_from(
        boost::asio::buffer(receive_message_.Data(),
                            receive_message_.ReadBufferSize()),
        local_host_,
        [this, handler](boost::system::error_code ec, std::size_t length) {
          if (!ec) {
            if (length <= 0) {
              std::cout << "receive data length = 0." << std::endl;
            } else {
              // ROS_DEBUG_STREAM("udp async read length: " << length);
              handler(receive_message_);
            }
          } else {
            // ROS_ERROR_STREAM(ec.message());
            std::cout << ec.message() << std::endl;
          }

          DoRead(handler);
        });
  }

 private:
  MessageType receive_message_;

  boost::asio::io_service service_;
  std::shared_ptr<udp::socket> socket_;
  std::shared_ptr<std::thread> runner_;

  udp::endpoint local_host_;
  udp::endpoint remote_host_;
};

}  // namespace hikvs_mv

#endif  // HIKVS_MV_IM_5005_INCLUDE_HIKVS_MV_IM_5005_INCLUDE_UDP_SOCKET_H_
