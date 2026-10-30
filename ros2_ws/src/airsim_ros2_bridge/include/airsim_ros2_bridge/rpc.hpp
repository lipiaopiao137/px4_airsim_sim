// Minimal msgpack-rpc client for AirSim.
//
// The protocol is trivial, so this module implements just enough of it:
//
//     request  = [0, msg_id, method, args]
//     response = [1, msg_id, error, result]
//
// Responses are decoded with binary payloads left intact (PNG images, float
// point clouds).

#ifndef AIRSIM_ROS2_BRIDGE__RPC_HPP_
#define AIRSIM_ROS2_BRIDGE__RPC_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "airsim_ros2_bridge/msgpack.hpp"

namespace airsim_bridge
{

class AirSimRpcError : public std::runtime_error
{
public:
  explicit AirSimRpcError(const std::string & what)
  : std::runtime_error(what) {}
};

class AirSimRpcClient
{
public:
  AirSimRpcClient(const std::string & ip, int port, double timeout_s = 10.0);
  ~AirSimRpcClient();

  AirSimRpcClient(const AirSimRpcClient &) = delete;
  AirSimRpcClient & operator=(const AirSimRpcClient &) = delete;

  void close();

  msgpack::Value call(
    const std::string & method, std::vector<msgpack::Value> args = {});

private:
  void connect_socket(const std::string & ip, int port);
  void send_all(const std::string & data);
  std::string recv_some();

  int fd_ = -1;
  double timeout_s_;
  uint32_t msg_id_ = 0;
  std::string buffer_;
};

}  // namespace airsim_bridge

#endif  // AIRSIM_ROS2_BRIDGE__RPC_HPP_
