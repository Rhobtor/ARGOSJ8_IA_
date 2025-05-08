#ifndef SENSOR_HPP
#define SENSOR_HPP
#include "data/Composite.hpp"
#include <map>
#include <vector>
#include <string>
class Sensor
{
public:
  virtual std::string getID() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void getFastData(std::shared_ptr<Visitor::Visitor> visitor) = 0;
  virtual void getSlowData(std::shared_ptr<Visitor::Visitor> visitor) = 0;

};

class SensorException : public std::exception {
  private:
    std::string error;
  public:
    SensorException(const std::string msg) : error(msg) {}

    const char* what() const noexcept override {
        return error.c_str();
    }
};
#endif // IMU_HPP