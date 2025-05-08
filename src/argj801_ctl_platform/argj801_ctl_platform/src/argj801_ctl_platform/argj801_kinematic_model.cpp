#include "argj801_ctl_platform/argj801_kinematic_model.hpp"

Argo_J8_KinematicModel::Argo_J8_KinematicModel(double effective_radius, double xICR, double throttle_to_percent,
                             double steer_to_percent, double steer_acc, double throttle_acc, double desired_freq)
    : effective_radius_(effective_radius), xICR_(xICR), throttle_to_percent_(throttle_to_percent), steer_to_percent_(steer_to_percent), 
    throttle_(0), steering_(0), steer_acc(steer_acc), throttle_acc(throttle_acc), desired_freq(desired_freq), last_throttle_(0), last_steering_(0) {
}

Argo_J8_KinematicModel::~Argo_J8_KinematicModel() {
}

void Argo_J8_KinematicModel::update(double speed, double rotation) {
    calculate(speed, rotation);
}

double Argo_J8_KinematicModel::getThrottle() const {
    return throttle_;
}

double Argo_J8_KinematicModel::getSteering() const {
    return steering_;
}

void Argo_J8_KinematicModel::calculate(double speed, double rotation) {
    // Calculate wheel velocities
    double w_right_wheel = (speed + rotation * xICR_) / effective_radius_;
    double w_left_wheel = (speed - rotation * xICR_) / effective_radius_;

    // Calculate throttle and steering speed based on wheel speeds
    double throttle_speed_ = (w_right_wheel + w_left_wheel) / 2;
    double steering_speed_ = (w_right_wheel - w_left_wheel) / 2;

    // Time step for the control loop
    double t_step = 1 / 50.0;  // 50 Hz control loop

    // Calculate the requested acceleration for throttle and steering
    double requested_throttle_acc = (throttle_speed_ - last_throttle_) / t_step;
    double requested_steer_acc = (steering_speed_ - last_steering_) / t_step;

    // Check if throttle is increasing in absolute value
    if (fabs(throttle_speed_) > fabs(last_throttle_) && fabs(requested_throttle_acc) > throttle_acc) {
        // Throttle is increasing in absolute value and exceeds the allowed acceleration
        double throttle_change = std::copysign(throttle_acc * t_step, throttle_speed_ - last_throttle_);
        throttle_speed_ = last_throttle_ + throttle_change;
        std::cout << "Throttle limited to: " << throttle_ << std::endl;
    } 

    // Check if steering is increasing in absolute value
    if (fabs(steering_speed_) > fabs(last_steering_) && fabs(requested_steer_acc) > steer_acc) {
        // Steering is increasing in absolute value and exceeds the allowed acceleration
        double steering_change = std::copysign(steer_acc * t_step, steering_speed_ - last_steering_);
        steering_speed_ = last_steering_ + steering_change;
        std::cout << "Steering limited to: " << steering_ << std::endl;
    } 

    // Update the last throttle and steering values for the next iteration
    last_throttle_ = throttle_speed_;
    last_steering_ = steering_speed_;
    throttle_ = throttle_speed_ * throttle_to_percent_;
    steering_ = steering_speed_ * steer_to_percent_;

    // Debug output
    std::cout << "Throttle: " << throttle_ << std::endl;
    std::cout << "Steering: " << steering_ << std::endl;
    std::cout << "Requested Throttle Acc: " << requested_throttle_acc << std::endl;
    std::cout << "Requested Steering Acc: " << requested_steer_acc << std::endl;
}



