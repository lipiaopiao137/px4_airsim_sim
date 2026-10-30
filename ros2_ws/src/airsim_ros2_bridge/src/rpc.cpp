#include "airsim_ros2_bridge/rpc.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace airsim_bridge
{

namespace
{

std::string errno_text(const char * prefix)
{
  return std::string(prefix) + ": " + std::strerror(errno);
}

std::string error_text(const msgpack::Value & error)
{
  if (error.type() == msgpack::Value::Type::String ||
    error.type() == msgpack::Value::Type::Bin)
  {
    return error.as_string();
  }
  if (error.type() == msgpack::Value::Type::Nil) {
    return "unknown error";
  }
  try {
    return std::to_string(error.as_double());
  } catch (const std::exception &) {
    return "unprintable AirSim error";
  }
}

}  // namespace

AirSimRpcClient::AirSimRpcClient(const std::string & ip, int port, double timeout_s)
: timeout_s_(timeout_s)
{
  connect_socket(ip, port);
}

AirSimRpcClient::~AirSimRpcClient()
{
  close();
}

void AirSimRpcClient::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  buffer_.clear();
}

void AirSimRpcClient::connect_socket(const std::string & ip, int port)
{
  const std::string service = std::to_string(port);
  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo * head = nullptr;
  const int rc = ::getaddrinfo(ip.c_str(), service.c_str(), &hints, &head);
  if (rc != 0) {
    throw AirSimRpcError(
            "cannot resolve " + ip + ":" + service + ": " + ::gai_strerror(rc));
  }

  const int timeout_ms = static_cast<int>(timeout_s_ * 1000.0);
  std::string last_error = "no usable address";

  for (struct addrinfo * addr = head; addr != nullptr; addr = addr->ai_next) {
    int fd = ::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if (fd < 0) {
      last_error = errno_text("socket");
      continue;
    }

    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    bool connected = false;
    if (::connect(fd, addr->ai_addr, addr->ai_addrlen) == 0) {
      connected = true;
    } else if (errno == EINPROGRESS) {
      struct pollfd poll_fd;
      poll_fd.fd = fd;
      poll_fd.events = POLLOUT;
      poll_fd.revents = 0;
      int ready;
      do {
        ready = ::poll(&poll_fd, 1, timeout_ms);
      } while (ready < 0 && errno == EINTR);
      if (ready > 0) {
        int socket_error = 0;
        socklen_t length = sizeof(socket_error);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &length) == 0 &&
          socket_error == 0)
        {
          connected = true;
        } else {
          errno = socket_error;
          last_error = errno_text("connect");
        }
      } else if (ready == 0) {
        last_error = "connect timed out";
      } else {
        last_error = errno_text("poll");
      }
    } else {
      last_error = errno_text("connect");
    }

    if (!connected) {
      ::close(fd);
      continue;
    }

    ::fcntl(fd, F_SETFL, flags);

    struct timeval tv;
    tv.tv_sec = static_cast<long>(timeout_s_);
    tv.tv_usec = static_cast<suseconds_t>(
      (timeout_s_ - static_cast<double>(tv.tv_sec)) * 1e6);
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    const int one = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    fd_ = fd;
    break;
  }

  ::freeaddrinfo(head);

  if (fd_ < 0) {
    throw AirSimRpcError("cannot connect to " + ip + ":" + service + ": " + last_error);
  }
}

void AirSimRpcClient::send_all(const std::string & data)
{
  size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t written = ::send(
      fd_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
    if (written > 0) {
      sent += static_cast<size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    throw AirSimRpcError(errno_text("send failed"));
  }
}

std::string AirSimRpcClient::recv_some()
{
  std::vector<char> chunk(1 << 20);
  for (;;) {
    const ssize_t received = ::recv(fd_, chunk.data(), chunk.size(), 0);
    if (received > 0) {
      return std::string(chunk.data(), static_cast<size_t>(received));
    }
    if (received == 0) {
      return std::string();
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      throw AirSimRpcError("timed out waiting for AirSim");
    }
    throw AirSimRpcError(errno_text("recv failed"));
  }
}

msgpack::Value AirSimRpcClient::call(
  const std::string & method, std::vector<msgpack::Value> args)
{
  if (fd_ < 0) {
    throw AirSimRpcError("AirSim RPC connection is closed");
  }

  const uint32_t msg_id = ++msg_id_;

  msgpack::Value request = msgpack::Value::make_array();
  msgpack::Value arg_array = msgpack::Value::make_array();
  for (auto & arg : args) {
    arg_array.push(std::move(arg));
  }
  request.push(msgpack::Value::make_uint(0));
  request.push(msgpack::Value::make_uint(msg_id));
  request.push(msgpack::Value::make_string(method));
  request.push(std::move(arg_array));

  msgpack::Writer writer;
  writer.value(request);
  send_all(writer.buffer());

  for (;;) {
    size_t consumed = 0;
    while (true) {
      msgpack::Value message;
      bool complete;
      try {
        complete = msgpack::Decoder(buffer_).next(message, consumed);
      } catch (const msgpack::Error & error) {
        throw AirSimRpcError(std::string("malformed msgpack response: ") + error.what());
      }
      if (!complete) {
        break;
      }
      buffer_.erase(0, consumed);

      if (message.type() != msgpack::Value::Type::Array || message.size() < 3) {
        continue;
      }
      if (message.at(0).as_int() != 1 || message.at(1).as_int() != msg_id) {
        continue;
      }
      const msgpack::Value & error = message.at(2);
      if (!error.is_nil()) {
        throw AirSimRpcError(error_text(error));
      }
      return message.size() > 3 ? message.at(3) : msgpack::Value();
    }

    const std::string chunk = recv_some();
    if (chunk.empty()) {
      throw AirSimRpcError("AirSim closed the RPC connection");
    }
    buffer_ += chunk;
  }
}

}  // namespace airsim_bridge
