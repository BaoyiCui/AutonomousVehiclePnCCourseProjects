#include <math.h>

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "carla_shenlan_stanley_pid_controller/pid_controller.h"
#include "carla_shenlan_stanley_pid_controller/stanely_controller.h"

using namespace std;

namespace shenlan {
namespace control {

shenlan::control::PIDController e_theta_pid_controller(1.0, 0.0, 0.4);    // PID控制器中的微分环节相当于阻尼，加在航向误差引起的前轮转角上

double atan2_to_PI(const double atan2) { return atan2 * M_PI / 180; }

double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) {
    const double dx = point.x - x;
    const double dy = point.y - y;
    return dx * dx + dy * dy;
}

void StanleyController::LoadControlConf() {
    k_y_ = 0.5;
    k_s_ = 0.5;
}

// /** to-do **/ 计算需要的控制命令, 实现对应的stanley模型,并将获得的控制命令传递给汽车
// 提示，在该函数中你需要调用计算误差
// 控制器中，前轮转角的命令是弧度单位，发送给carla的横向控制指令，范围是 -1~1
void StanleyController::ComputeControlCmd(const VehicleState &vehicle_state, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {
    trajectory_points_ = planning_published_trajectory.trajectory_points;
    // find the closest point on the reference trajectory
    TrajectoryPoint nearest_pt = QueryNearestPointByPosition(vehicle_state.x, vehicle_state.y);
    // theta_ref_在QueryNearestPointByPosition中已经更新了

    // get lateral error and heading error
    double e_y = 0.0;
    double e_theta = 0.0;

    ComputeLateralErrors(vehicle_state.x - nearest_pt.x, vehicle_state.y - nearest_pt.y, vehicle_state.heading, e_y, e_theta);

    double e_theta_pd = e_theta_pid_controller.Control(e_theta, 0.01);
    cmd.steer_target = e_theta_pd + atan2(k_y_ * e_y, vehicle_state.velocity + k_s_);

    // 输出限幅
    if (cmd.steer_target > 1.0) {
        cmd.steer_target = 1.0;
    } else if (cmd.steer_target < -1.0) {
        cmd.steer_target = -1.0;
    }

}

// /** to-do **/ 计算需要的误差，包括横向误差，朝向误差，误差计算函数没有传入主车速度，因此返回的位置误差就是误差，不是根据误差计算得到的前轮转角
void StanleyController::ComputeLateralErrors(const double x, const double y, const double theta, double &e_y, double &e_theta) {
    // x = ref.x - curr.x
    // y = ref.y - curr.y
    // 车头方向的单位矢量 (cos(theta), sin(theta))
    // 横向误差以车辆朝向为参考，左正右负
    e_y = cos(theta) * y - sin(theta) * x;

    e_theta = theta - theta_ref_;
    if (e_theta <= -M_PI) {
        e_theta += 2 * M_PI;
    } else if (e_theta >= M_PI) {
        e_theta -= 2 * M_PI;
    }
    std::cout << "theta: " << theta << " theta_ref_: " << theta_ref_ << std::endl;
    std::cout << "e_theta: " << e_theta << std::endl;
}

// 返回参考路径上和车辆当前位置距离最近的点，返回的是点结构体
TrajectoryPoint StanleyController::QueryNearestPointByPosition(const double x, const double y) {
    std::cout << trajectory_points_.size() << std::endl;
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);    // 得到当前位置距离参考路径的第一个点的距离

    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    // cout << " index_min: " << index_min << endl;
    // cout << "tarjectory.heading: " << trajectory_points_[index_min].heading << endl;
    theta_ref_ = trajectory_points_[index_min].heading;    // 获得距离车辆当前位置最近的路径点的航向角

    return trajectory_points_[index_min];
}
}    // namespace control
}    // namespace shenlan