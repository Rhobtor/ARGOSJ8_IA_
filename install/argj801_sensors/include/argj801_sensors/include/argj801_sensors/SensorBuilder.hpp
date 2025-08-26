#ifndef SENSORBUILDER_H
#define SENSORBUILDER_H
#include <memory>
#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "argj801_sensors_msgs/msg/motor_telemetry.hpp"
#include "argj801_sensors_msgs/msg/odometer.hpp"
#include "argj801_sensors/Sensor.hpp"

namespace Builder {
	class SensorBuilder
	{
	public:
    virtual void buildPreviousConfig(std::string url) = 0;
    virtual void buildVelocine(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> publisher, std::string frame_id) = 0;
    virtual void buildSick(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> publisher, std::string frame_id) = 0;
    virtual void buildTwist(std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> publisher, std::string frame_id) = 0;
    virtual void buildOdometer(std::shared_ptr<rclcpp::Publisher<argj801_sensors_msgs::msg::Odometer>> publisher, std::string frame_id) = 0;
    virtual void buildMotorTelemetryLeft(std::shared_ptr<rclcpp::Publisher<argj801_sensors_msgs::msg::MotorTelemetry>> publisher, std::string frame_id) = 0;
    virtual void buildMotorTelemetryRigth(std::shared_ptr<rclcpp::Publisher<argj801_sensors_msgs::msg::MotorTelemetry>> publisher, std::string frame_id) = 0;
    virtual void buildCamera(rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher, std::string frame_id,int compresion_ratio, bool resize) = 0;
    virtual std::shared_ptr<Sensor> getSensor() = 0;
	};

  class NotAvailableOptionException  : public std::exception {
    private:
      std::string error;
    public:
      NotAvailableOptionException(const std::string msg) : error(msg) {}

      const char* what() const noexcept override {
          return error.c_str();
      }
  };
}
#endif // SENSORBUILDER_H