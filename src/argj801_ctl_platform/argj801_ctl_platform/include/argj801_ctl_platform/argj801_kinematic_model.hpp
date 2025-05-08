#ifndef ARGO_J8_KINEMATICMODEL_H
#define ARGO_J8_KINEMATICMODEL_H
#include <iostream>
#include <cmath>
class Argo_J8_KinematicModel {
public:
    Argo_J8_KinematicModel(double effective_radius, double xICR, double throttle_to_percent,
                             double steer_to_percent, double steer_acc, double throttle_acc, double desired_freq);

    ~Argo_J8_KinematicModel();

    // Method to update the robot's position based on the kinematic model
    void update(double speed, double rotation);

    // Method to get the calculated throttle and steering values
    double getThrottle() const;
    double getSteering() const;

private:
    double effective_radius_;
    double xICR_;
    double throttle_to_percent_;
    double steer_to_percent_;

    double throttle_;
    double steering_;
    double throttle_acc;
    double steer_acc;
    double desired_freq;
    double last_throttle_;
    double last_steering_;

    void calculate(double speed, double rotation);
};

#endif // ARGO_J8_KINEMATICMODEL_H
