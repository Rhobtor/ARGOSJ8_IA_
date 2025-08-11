//////////////////////////////////////
// placeholder for FollowZED.hpp
/////////////////////////////////////////////////////////

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <thread>
#include <functional>
#include <string>
#include <mutex>

// POSIX sockets (Linux)
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class TcpClient {
public:
  using LineCallback = std::function<void(const std::string&)>;
  TcpClient() = default;
  ~TcpClient() { close(); }

  bool connect(const std::string& host, int port);
  void close();
  bool send_line(const std::string& line);
  void set_on_line(LineCallback cb) { on_line_ = std::move(cb); }
  bool is_connected() const { return connected_; }

private:
  void reader_loop();
  int sock_ = -1;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::thread reader_;
  LineCallback on_line_;
  std::mutex send_mtx_;
};

class FollowZEDNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit FollowZEDNode(const std::string& name);

  // Lifecycle
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&) override;

private:
  // Params
  std::string jetson_host_;
  int jetson_port_ = 5000;
  std::string cmd_start_ = "START_FOLLOW\n";
  std::string cmd_stop_  = "STOP_FOLLOW\n";
  std::string cmd_ping_  = "PING\n";
  bool relay_video_ = true;
  std::string video_in_topic_ = "/zed/person_follow/image_raw";
  std::string video_out_topic_ = "/follow_zed/image_for_gui";
  double ping_period_s_ = 2.0;

  // TCP
  TcpClient client_;
  rclcpp::TimerBase::SharedPtr ping_timer_;

  // Video relay (opcional)
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Callbacks
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_tcp_line(const std::string& line);
  void start_tcp();
  void stop_tcp();
};



//////////////////////////////////////////////////////////////////////////////////