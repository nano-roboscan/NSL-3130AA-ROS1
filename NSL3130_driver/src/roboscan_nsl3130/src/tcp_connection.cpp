#include <ros/ros.h>
#include <iostream>
#include "tcp_connection.hpp"

using boost::asio::ip::tcp;

namespace nanosys {
bool TcpConnection::reConnect = false;
bool TcpConnection::timerStart = false;

typedef std::vector<uint8_t> Packet;

TcpConnection::TcpConnection(boost::asio::io_service& ioService)
  : resolver(ioService), socket(ioService), state(STATE_DISCONNECTED) {
}

TcpConnection::~TcpConnection() {
  try {
    disconnect();
  } catch (boost::system::system_error e) {
    std::cerr << e.what() << std::endl;
  }
}

void TcpConnection::sendCommand(const std::vector<uint8_t>& data) {

  if (!isConnected()) return;

  uint32_t data_len = data.size();
  size_t buf_size = MARKER_SIZE + sizeof(data_len) + data_len + MARKER_SIZE;

  std::ostringstream os;
  os << START_MARKER;
  os << static_cast<uint8_t>((data_len >> 24) & 0xff);
  os << static_cast<uint8_t>((data_len >> 16) & 0xff);
  os << static_cast<uint8_t>((data_len >>  8) & 0xff);
  os << static_cast<uint8_t>((data_len >>  0) & 0xff);
  for (int i = 0; i < data_len; ++i) {
    os << static_cast<uint8_t>(data[i]);
  }
  os << END_MARKER;

  boost::system::error_code error;
  socket.write_some(boost::asio::buffer(os.str(), os.tellp()), error);

  if (error) {
    throw boost::system::system_error(error);
  }
  waitAck();
}

void TcpConnection::connect(const std::string& ipAddress) {
  if (isConnected()) return;
  
  updateState(STATE_CONNECTING);
  tcp::resolver::query query(ipAddress, PORT);
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
  tcp::resolver::iterator end;

  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && endpoint_iterator != end) {
    socket.close();
    socket.connect(*endpoint_iterator++, error);
  }
  if (error) {
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      std::cout << "reconnect" << std::endl;
      connect(ipAddress);
      reconnectCount++;

      if (reconnectCount >= 5) {
          throw boost::system::system_error(error);
      }
  } else {
      std::cout << "connect IP : " << ipAddress << std::endl;
      reconnectCount = 0;
  }
  updateState(STATE_CONNECTED);
}


void TcpConnection::disconnect() {
  if (isDisconnected()) return;
  
  updateState(STATE_CLOSING);
  boost::system::error_code error;
  socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
  if (error) {
    revertState();
    throw boost::system::system_error(error);
  }
  updateState(STATE_DISCONNECTED);
}


void TcpConnection::waitAck() {
    Packet buf(ACK_BUF_SIZE);
    boost::system::error_code error;

    this->updateState(STATE_WAIT_ACK);
    
    boost::asio::steady_timer timer(socket.get_io_service());
    timer.expires_from_now(std::chrono::milliseconds(300));

    timer.async_wait([this](const boost::system::error_code& ec) {
        if (!ec) {
            TcpConnection::timerStart = true;
            disconnect();

            std::string newipAddress;
            if (!ros::param::get("camera/set_ip", newipAddress)) {
                std::cout << newipAddress << std::endl;
            }
            connect(newipAddress);

            if(isConnected())
                TcpConnection::reConnect = true;
        }
    });
    
    size_t len = socket.read_some(boost::asio::buffer(buf), error);
    timer.cancel();

    this->revertState();
}


void TcpConnection::updateState(State state_) const {
  previousState = state;
  state = state_;
}

void TcpConnection::revertState() const {
  state = previousState;
}

bool TcpConnection::isConnected() const {
  return state == STATE_CONNECTED;
}

bool TcpConnection::isDisconnected() const {
  return state == STATE_DISCONNECTED;
}


} // end namespace nanosys
