#include "carla_shenlan_pid_controller/pid_controller.h"

#include <assert.h>
#include <iostream>
namespace shenlan {
namespace control {

PIDController::PIDController(const double kp, const double ki, const double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    std::cout << "Kp: " << kp_ << ", Ki: " << ki_ << ", Kd: " << kd_ << std::endl;
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
    assert(dt > 0 && "dt must be positive!!!");   

    // 比例
    proportional_part = kp_ * error;
    // 微分
    if (!first_hit_){
        first_hit_ = true;
        derivative_part = 0.0;
    }
    else{
        derivative_part = kd_ * (error - previous_error_) / dt;
    } 
    // 积分
    integral_ += error * dt;
    integral_part = ki_ * integral_;
    
    // current_output = (proportional_part + integral_part + derivative_part);
    std::cout << "Proportional_part: " << proportional_part << ", Integral_part: " << integral_part << ", Derivative_part: " << derivative_part << std::endl;
    current_output = proportional_part + derivative_part;

    previous_error_ = error;
    previous_output_ = current_output;

    return current_output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double previous_error_ = 0.0;
    double previous_output_ = 0.0;
    double integral_ = 0.0;
    bool first_hit_ = false;
                
    double proportional_part = 0;
    double integral_part = 0;
    double derivative_part = 0;
    double current_output = 0;
}

}    // namespace control
}    // namespace shenlan
