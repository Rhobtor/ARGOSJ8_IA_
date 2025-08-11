/////////////////////////////////////////
/// Placeholder for FollowZED.Cpp
//////////////////////////////////////////////


#include "ctl_mission/FollowZED.hpp"
#include <chrono>
using namespace std::chrono_literals;

/* ---------------- TCP CLIENT ---------------- */

bool TcpClient::connect(const std::string& host, int port) {
  close();
  sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock_ < 0) return false;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
    ::close(sock_);
    sock_ = -1;
    return false;
  }
  if (::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(sock_);
    sock_ = -1;
    return false;
  }
  running_ = true;
  connected_ = true;
  reader_ = std::thread(&TcpClient::reader_loop, this);
  return true;
}

void TcpClient::close() {
  running_ = false;
  if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    sock_ = -1;
  }
  if (reader_.joinable()) reader_.join();
  connected_ = false;
}

bool TcpClient::send_line(const std::string& line) {
  std::lock_guard<std::mutex> lk(send_mtx_);
  if (sock_ < 0) return false;
  const char* data = line.c_str();
  size_t left = line.size();
  while (left > 0) {
    ssize_t n = ::send(sock_, data, left, 0);
    if (n <= 0) return false;
    data += n;
    left -= static_cast<size_t>(n);
  }
  return true;
}

void TcpClient::reader_loop() {
  std::string buf;
  buf.reserve(4096);
  char tmp[1024];
  while (running_) {
    ssize_t n = ::recv(sock_, tmp, sizeof(tmp), 0);
    if (n <= 0) break;
    buf.append(tmp, tmp + n);
    // linhas por '\n'
    size_t pos;
    while ((pos = buf.find('\n')) != std::string::npos) {
      std::string line = buf.substr(0, pos);
      if (on_line_) on_line_(line);
      buf.erase(0, pos + 1);
    }
  }
  connected_ = false;
  running_ = false;
}

/* -------------- FOLLOW ZED NODE ------------- */

FollowZEDNode::FollowZEDNode(const std::string& name)
: rclcpp_lifecycle::LifecycleNode(name) {
  // Declara parámetros con defaults
  this->declare_parameter<std::string>("jetson.host", "192.168.1.50");
  this->declare_parameter<int>("jetson.port", 5000);
  this->declare_parameter<std::string>("commands.start", "START_FOLLOW\n");
  this->declare_parameter<std::string>("commands.stop",  "STOP_FOLLOW\n");
  this->declare_parameter<std::string>("commands.ping",  "PING\n");
  this->declare_parameter<bool>("relay_video", true);
  this->declare_parameter<std::string>("video_in_topic", "/zed/person_follow/image_raw");
  this->declare_parameter<std::string>("video_out_topic","/follow_zed/image_for_gui");
  this->declare_parameter<double>("ping_period_s", 2.0);
}

auto FollowZEDNode::on_configure(const rclcpp_lifecycle::State&)
-> rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn {
  this->get_parameter("jetson.host", jetson_host_);
  this->get_parameter("jetson.port", jetson_port_);
  this->get_parameter("commands.start", cmd_start_);
  this->get_parameter("commands.stop",  cmd_stop_);
  this->get_parameter("commands.ping",  cmd_ping_);
  this->get_parameter("relay_video", relay_video_);
  this->get_parameter("video_in_topic", video_in_topic_);
  this->get_parameter("video_out_topic", video_out_topic_);
  this->get_parameter("ping_period_s", ping_period_s_);

  RCLCPP_INFO(get_logger(), "FollowZED configured. Jetson %s:%d, relay_video=%s",
              jetson_host_.c_str(), jetson_port_, relay_video_ ? "true" : "false");

  // Publisher (lifecycle) para la GUI si hacemos relay
  if (relay_video_) {
    img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(video_out_topic_, rclcpp::SensorDataQoS());
    // Sub al tópico que publica la Jetson (si está en mismo dominio DDS aparecerá)
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      video_in_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FollowZEDNode::on_image, this, std::placeholders::_1));
  }
  // Callback de líneas TCP desde Jetson
  client_.set_on_line([this](const std::string& line){ this->on_tcp_line(line); });

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

auto FollowZEDNode::on_activate(const rclcpp_lifecycle::State&)
-> rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn {
  RCLCPP_INFO(get_logger(), "Activating FollowZED: connecting TCP + START");
  if (relay_video_ && img_pub_) img_pub_->on_activate();

  start_tcp();
  if (client_.is_connected()) {
    client_.send_line(cmd_start_);
  }

  // Timer de keepalive/ping
  ping_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(ping_period_s_),
    [this](){
      if (client_.is_connected() && !cmd_ping_.empty()) {
        client_.send_line(cmd_ping_);
      }
    });

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

auto FollowZEDNode::on_deactivate(const rclcpp_lifecycle::State&)
-> rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn {
  RCLCPP_INFO(get_logger(), "Deactivating FollowZED: STOP + close TCP");
  if (ping_timer_) ping_timer_->cancel(), ping_timer_.reset();

  if (client_.is_connected()) {
    client_.send_line(cmd_stop_);
    // breve pausa para que se envíe
    std::this_thread::sleep_for(100ms);
  }
  stop_tcp();

  if (relay_video_ && img_pub_) img_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

auto FollowZEDNode::on_cleanup(const rclcpp_lifecycle::State&)
-> rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn {
  RCLCPP_INFO(get_logger(), "Cleanup FollowZED");
  stop_tcp();
  img_sub_.reset();
  img_pub_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

/* ----------- helpers & callbacks ---------- */

void FollowZEDNode::start_tcp() {
  if (!client_.is_connected()) {
    if (!client_.connect(jetson_host_, jetson_port_)) {
      RCLCPP_ERROR(get_logger(), "TCP connect failed to %s:%d", jetson_host_.c_str(), jetson_port_);
    } else {
      RCLCPP_INFO(get_logger(), "TCP connected to %s:%d", jetson_host_.c_str(), jetson_port_);
    }
  }
}

void FollowZEDNode::stop_tcp() { client_.close(); }

void FollowZEDNode::on_image(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (relay_video_ && img_pub_ && img_pub_->is_activated()) {
    img_pub_->publish(*msg); // relay 1:1 para la GUI
  }
}

void FollowZEDNode::on_tcp_line(const std::string& line) {
  // Aquí parseas eventos/telemetría de la Jetson si envías JSON/CSV por TCP
  // Ejemplos:
  //  "STATUS:OK"
  //  "TARGET:x,y,conf"
  RCLCPP_DEBUG(get_logger(), "[Jetson] %s", line.c_str());
  // TODO: publicar tópicos/diagnósticos si te interesa
}
// Placeholder para el final del archivo
//////////////////////////////////////////////////////////////////////////////////