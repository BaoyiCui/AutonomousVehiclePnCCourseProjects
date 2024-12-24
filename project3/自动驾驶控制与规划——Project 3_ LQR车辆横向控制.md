@[TOC](目录)

# 零、任务介绍
1. 补全`src/ros-bridge/carla_shenlan_projects/carla_shenlan_lqr_pid_controller/src/lqr_controller.cpp`中的`TODO`部分，实现基于LQR的车辆横向控制。
# 一、系统建模
## 1.1 连续模型
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/16af849ebe794b3f85ba0a481b8261a7.png)

选取系统状态变量为$x = [e_{cg}, \dot e_{cg}, e_\theta, \dot e_\theta]^\top$，控制输入为$u = \delta$。系统参考轨迹每个点可以给出位置、偏航角、速度、加速度、曲率，这些量可以表示为沿轨迹行驶路程$s$的函数。车辆的横向误差动力学可以用如下的线性模型描述
$$
\dot x = Ax + B_1\delta + B_2 r_{des}
$$
其中
$$
A = \left[\begin{matrix}
0 & 1 & 0 & 0\\
0 & -\frac{(c_f + c_r)}{mv} & \frac{c_f + c_r}{m} & \frac{l_r c_r - l_f c_f}{mv}\\
0 & 0 & 0 & 1\\
0 & \frac{l_r c_r - l_f c_f}{I_z v} & \frac{l_r c_r - l_f c_f}{I_z} & -\frac{(l_f^2 c_f + l_r^2 c_r)}{I_z v}
\end{matrix}\right]
$$

$$
B_1 = \left[\begin{matrix}
0\\
c_f / m\\
0\\
l_f c_f /I_z
\end{matrix}\right]
$$

$$
B_2 = \left[
\begin{matrix}
0\\
(l_r c_r - l_f c_f)/(mv) - v\\
0\\
-(l_f^2 c_f + l_r^2 c_r)/(I_z v)
\end{matrix}
\right]
$$
这里实际上是假设在一小段时间内车辆纵向速度$v$保持不变，然后在工作点线性化，参考轨迹的偏航角速度$r_{des}$可以由参考点速度$v_{des}$和参考点曲率$\kappa_{des}$计算
$$
r_{des} = v_{des}  \kappa_{des}
$$
## 1.2 离散化
实际实现时，需要将系统离散化，离散化可以采取前向欧拉法和零阶保持器精确离散化等方法。采样周期为$T$，前向欧拉法得到的离散系统矩阵和输入矩阵为
$$
\begin{aligned}
A_d &= I + TA
\\
B_{d1} &= TB_1
\\
B_{d2} &= TB_2
\end{aligned}
$$
为了达到更好的准确性和稳定性，可以使用零阶保持器进行精确离散化，精确离散化的计算公式如下：
$$
\begin{aligned}
A_d &= e^{AT}\\
B_{d1} &= (\int_0^T e^{A\tau} d\tau)B_1
\\
B_{d2} &= (\int_0^T e^{A\tau} d\tau)B_2
\end{aligned}
$$
矩阵指数可以用如下的拉普拉斯逆变换得到
$$
e^{At} = \mathcal L^{-1}[(sI - A)^{-1}]
$$
本次project中的参数如下
|参数名称	| 符号 | 数值 |
|--|--| --|
| 前轮侧偏刚度（左右轮之和） | $c_f$ | 155494.663 |
| 后轮侧偏刚度（左右轮之和） | $c_r$ | 155494.663 |
| 前轮到质心的距离 | $l_f$ | 1.426 |
| 后轮到质心的距离 | $l_r$ | 1.426 |
| 整车质量 | $m$ | 1845.0 |
| Z轴转动惯量 | $I_z$ | 3751.76 |
| 控制周期 | $t_s$ | 0.01 |

使用mathematica计算矩阵拉普拉斯反变换
```mathematica
ts = 0.01;
cf = 155494.663;
cr = 155494.663;
mfl = 1845.0/4;
mfr = 1845.0/4;
mrl = 1845.0/4;
mrr = 1845.0/4;
mf = mfl + mfr;
mr = mrl + mrr;
m = mf + mr
wheelbase = 2.852;
lf = wheelbase * (1.0 - mf/m)
lr = wheelbase * (1.0 - mr /m)

iz = lf*lf*mf + lr*lr*mr

A = {
  {0, 1, 0, 0},
  {0, -(cf + cr)/(m*v), (cf + cr)/m, (lr*cr - lf*cf)/(m*v)},
  {0, 0, 0, 1},
  {0, (lr*cr - lf*cf)/(iz * v), (lr*cr - lf*cf)/
    iz, -(lf^2*cf + lr^2*cr)/(iz * v)}
  }
  InverseLaplaceTransform[Inverse[s*IdentityMatrix[4] - A], s, 0.01]
```
计算得到的矩阵$A_d$如下
$$
A_d = \left(
\begin{array}{cccc}
1.0 & v \left(0.00593268 - 0.00593268 e^{-1.68558/v}\right) & v \left(\left(0.00593268 e^{-1.68558/v}-0.00593268\right) v + 0.01\right) & 168.558 \left(e^{-1.68558/v} \left(4.176213162156246 \times 10^{-7} v + 3.519668608485906 \times 10^{-7}\right) - 4.176213162156246 \times 10^{-7} v + 3.519668608485956 \times 10^{-7}\right) v^2 \\
0. & 1. e^{-1.68558/v} & v \left(1. - 1. e^{-1.68558/v}\right) & 168.558 \left(e^{-1.68558/v} \left(-0.0000351967 v - 0.0000593268\right) + 0.0000351967 v\right) v \\
0. & 0. & 1. & e^{-1.68558/v} \left(-0.00593268 v - 3.3723378595376876 \times 10^{-18}\right) + 0.00593268 v \\
0. & 0. & 0. & \frac{e^{-1.68558/v} \left(1. v + 2.842170943040401 \times 10^{-16}\right)}{v}
\end{array}
\right)
$$
通过观察可以得到，矩阵$A$不可逆，因此计算$B_d$时采用幂级数展开求$e^{A\tau}$的积分，即
$$
\begin{aligned}
B_{d1} &= \int_0^T e^{A\tau} d\tau B_1
\\
&= \int_0^T(I + A\tau + \frac{(A\tau)^2}{2!} + \frac{(A\tau)^3}{3!} + \cdots)d\tau B_1
\\
& = (TI + \frac{T^2}{2}A + \frac{T^3}{6}A^2 + \cdots) B_1
\\
&\approx TB_1
\end{aligned}
$$
类似地有$B_{d2} = TB_2$。
# 二、算法
## 2.1 离散时间LQR
离散时间LQR的最优控制律为
$$
u^*(k) = - K x(k)
$$
其中反馈增益$K$为
$$
K = (R + B_d^\top P B_d)^{-1} B_d^\top P A_d
$$
其中正定矩阵P可以使用动态规划算法得出，迭代形式如下
$$
P_{t-1} = A_d^\top P_{t} A_d - A_d^\top P_{t} B_d (R + B_d^\top P_{t} B_d)^{-1}B_d^\top P_{t} A_d + Q
$$
此处的推导细节参考[基础算法 - LQR - 离散时间有限边界](https://zgh551.github.io/2020/02/20/%E5%9F%BA%E7%A1%80%E7%AE%97%E6%B3%95-LQR-%E7%A6%BB%E6%95%A3%E6%97%B6%E9%97%B4%E6%9C%89%E9%99%90%E8%BE%B9%E7%95%8C/)
## 2.2 前馈控制
采用上述状态反馈控制律后，系统的模型可以改写成
$$
\dot x = (A - B_1 K) x + B_2 r_{des}
$$
闭环系统$A - B_1K$是渐进稳定的，但是由于$B_2 r_{des}$的存在，无法保证系统状态量收敛至0。因此需要在控制律中加入前馈项
$$
\delta = -Kx + \delta_{ff}
$$
前馈项的设计参考[自动驾驶控制算法 —— 横向LQR控制+前馈控制](https://zhuanlan.zhihu.com/p/521695558)
$$
\begin{aligned}
\delta_{ff} = L \kappa + K_v a_y - k_3 \kappa (l_r - \frac{l_f mv_x^2}{2c_{\alpha r} L})
\end{aligned}
$$
其中
$$
K_v = \frac{l_r m}{2 c_{\alpha f} (l_f + l_r)} - \frac{l_f m}{2 c_{\alpha r} (l_f + l_r)}
$$

# 三、代码实现
计算控制量
```cpp 
bool LqrController::ComputeControlCommand(const VehicleState &localization, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {
    // 规划轨迹
    trajectory_points_ = planning_published_trajectory.trajectory_points;
    /*
    A matrix (Gear Drive)
    [0.0,                               1.0,                            0.0,                                               0.0;
     0.0,            (-(c_f + c_r) / m) / v,                (c_f + c_r) / m,                   (l_r * c_r - l_f * c_f) / m / v;
     0.0,                               0.0,                            0.0,                                               1.0;
     0.0,   ((lr * cr - lf * cf) / i_z) / v,   (l_f * c_f - l_r * c_r) / i_z,   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    // TODO 01 配置状态矩阵A
    double v = max(localization.velocity, minimum_speed_protection_);    // 避免除0
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    // TODO 02 动力矩阵B
    matrix_bd_ = matrix_b_ * ts_;
    // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
    // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;
    // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading Error Rate]

    // TODO 03 计算横向误差并且更新状态向量x
    UpdateState(localization);

    // TODO 04 更新状态矩阵A并将状态矩阵A离散化
    UpdateMatrix(localization);

    // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
    // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;

    // TODO 05 Solve Lqr Problem
    SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_, lqr_max_iteration_, &matrix_k_);

    // TODO 06 计算feedback, 根据反馈对计算状态变量（误差状态）的时候的符号的理解：K里面的值实际运算中全部为正值，steer = -K *
    // state，按照代码中采用的横向误差的计算方式，横向误差为正值的时候（state中的第一项为正），参考点位于车辆左侧，车辆应该向左转以减小误差，而根据试验，仿真器中，给正值的时候，车辆向右转，给负值的时候，车辆向左转。
    //   feedback = - K * state
    //   Convert vehicle steer angle from rad to degree and then to steer degrees
    //   then to 100% ratio
    std::cout << "matrix_k_: " << matrix_k_ << std::endl;
    cout << "matrix_k * matrix_state: " << (matrix_k_ * matrix_state_) << endl;
    double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0);

    // TODO 07 计算前馈控制，计算横向转角的反馈量
    double steer_angle_feedforward = 0.0;
    steer_angle_feedforward = ComputeFeedForward(localization, ref_curv_);
    double steer_angle = steer_angle_feedback - 0.9 * steer_angle_feedforward;
    cout << "steer_angle_feedforward: " << steer_angle_feedforward << endl;
    // 限制前轮最大转角，这里定义前轮最大转角位于 [-20度～20度]
    if (steer_angle >= atan2_to_PI(20.0)) {
        steer_angle = atan2_to_PI(20.0);
    } else if (steer_angle <= -atan2_to_PI(20.0)) {
        steer_angle = -atan2_to_PI(20.0);
    }
    // Set the steer commands

    cmd.steer_target = steer_angle;

    return true;
}
```

计算横向误差
```cpp
// TODO 03 计算误差
void LqrController::ComputeLateralErrors(const double x, const double y, const double theta, const double linear_v, const double angular_v, const double linear_a, LateralControlErrorPtr &lat_con_err) {
    // 轨迹上最近的点
    TrajectoryPoint target_point = QueryNearestPointByPosition(x, y);

    double dx = target_point.x - x;
    double dy = target_point.y - y;
    double theta_p = target_point.heading;

    double e_cg = cos(theta_p) * dy - sin(theta_p) * dx;
    double e_theta = NormalizeAngle(theta_p - theta);
    double diff_e_cg = linear_v * sin(e_theta);
    double diff_e_theta = angular_v - target_point.kappa * target_point.v;

    lat_con_err->heading_error = e_theta;
    lat_con_err->lateral_error = e_cg;
    lat_con_err->heading_error_rate = diff_e_theta;
    lat_con_err->lateral_error_rate = diff_e_cg;
}
```
更新状态矩阵并离散化
```cpp
// TODO 04 更新状态矩阵A并将状态矩阵A离散化
void LqrController::UpdateMatrix(const VehicleState &vehicle_state) {
    double v = max(vehicle_state.velocity, minimum_speed_protection_);
    // 更新A矩阵
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
    // 更新A_d矩阵，前向欧拉法
    // matrix_ad_ = Matrix::Identity(basic_state_size_, basic_state_size_) + matrix_a_ * ts_;
    // 更新A_d矩阵，ZOH精确离散化
    // 计算指数项
    double exp_term = exp(-1.68558 / v);

    // 使用ZOH精确离散化
    matrix_ad_(0, 0) = 1.0;
    matrix_ad_(0, 1) = 0.00593268 * (1.0 - exp_term);
    matrix_ad_(0, 2) = 0.010000000000000142 + (-0.00593268 + 0.00593268 * exp_term) / v;
    matrix_ad_(0, 3) = 168.55790027100272 * (3.519668608485956e-7 + exp_term * (3.519668608485906e-7 + 4.176213162156246e-7 * v)) - 4.176213162156246e-7 * v;

    matrix_ad_(1, 0) = 0.0;
    matrix_ad_(1, 1) = 1.0 * exp_term;
    matrix_ad_(1, 2) = 168.55790027100272 * (exp_term * (-0.000059326795029614625 - 0.000035196686084859064 * v) + 0.000035196686084859064 * v);
    matrix_ad_(1, 3) = 0.0;

    matrix_ad_(2, 0) = 0.0;
    matrix_ad_(2, 1) = 0.0;
    matrix_ad_(2, 2) = 1.0;
    matrix_ad_(2, 3) = exp_term * (-0.00593268 * v - 3.3723378595376876e-18) + 0.00593268 * v;

    matrix_ad_(3, 0) = 0.0;
    matrix_ad_(3, 1) = 0.0;
    matrix_ad_(3, 2) = 0.0;
    matrix_ad_(3, 3) = exp_term * (1.0 * v + 2.842170943040401e-16) / v;
}
```
求解LQR问题
```cpp
// TODO 05:求解LQR方程
void LqrController::SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, const double tolerance, const uint max_num_iteration, Matrix *ptr_K) {
    // 防止矩阵的维数出错导致后续的运算失败
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
        std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
        return;
    }

    Matrix P = Q;
    Matrix P_new = Q;
    Matrix At = A.transpose();
    Matrix Bt = B.transpose();

    for (int i = 0; i < max_num_iteration; i++) {
        P_new = At * P * A - At * P * B * (R + Bt * P * B).inverse() * Bt * P * A + Q;
        double diff = fabs((P_new - P).maxCoeff());
        P = P_new;
        if (diff < tolerance) {
            cout << "diff: " << diff << endl;
            *ptr_K = (R + Bt * P * B).inverse() * Bt * P * A;
            return;
        }
    }
    cout << "exceed max iteration" << endl;
}
```
计算前馈控制量
```cpp 
// TODO 07 前馈控制，计算横向转角的反馈量
double LqrController::ComputeFeedForward(const VehicleState &localization, double ref_curvature) {
    if (isnan(ref_curvature)) {
        ref_curvature = 0;
    }
    const double K_v = lr_ * mass_ / (2 * cf_ * (lf_ + lr_));
    const double v = localization.velocity;
    double steer_angle_feedforward = ref_curvature * wheelbase_ 
    	+ K_v * v * v * ref_curvature 
    	- matrix_k_(0, 2) * ref_curvature * (lr_ - lf_ * mass_ * v * v / (2 * cr_ * wheelbase_));
    return steer_angle_feedforward;
}
```
# 四、效果展示
ZOH离散化（无前馈）
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/ef89f954f6bb4271a7f5b84c15bf07c7.png)
ZOH离散化（前馈）
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/5bdd6d6c10174ed3a86c2e82d5cfffad.png)
前向欧拉法离散化（前馈）
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/7ca6a988af424ad984c11fc16d32895f.png)
通过观察可以发现
1. 使用ZOH离散化后，转弯过后的横向振荡明显减少。
2. 当轨迹的偏航角速度不为0时（转弯），未加入前馈时出现较大的横向误差，加入前馈后横向误差显著减小。

[video(video-DEZx0s0E-1734746215217)(type-csdn)(url-https://live.csdn.net/v/embed/439868)(image-https://img-home.csdnimg.cn/images/20230724024159.png?origin_url=https%3A%2F%2Fv-blog.csdnimg.cn%2Fasset%2Fe51306e474a85cd01a35f500e7e4aef2%2Fcover%2FCover0.jpg&pos_id=img-Nv9UQLQP-1734746355185))(title-自动驾驶控制与规划——Project 3:LQR车辆横向控制)]



