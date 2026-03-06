#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <algorithm>
#include <vector>

#include "convert.hpp"
#include "mpc.hpp"
#include "pid.hpp"
#include "message/msg/ship_state.hpp"
#include "message/msg/control_objective.hpp"
#include "message/msg/controller_output.hpp"
#include "message/msg/navi_mode.hpp"

class ControllerNode : public rclcpp::Node
{
public:
    // 控制目标
    int stop = 0;         // 0正常，1刹车，2停车
    int control_mode = 0; // 0转速控制，1航速控制，2位置控制
    double v_d = 0, psi_d = 0;

    double delta_calibration = 0, limit = 30;

    // 船体状态
    double boat_v = 0, boat_angle = -90, yaw_rate = 0, boat_heading = 0;
    int n_now = 0, u_now = 0;
    Eigen::Vector2d x;

    // PID
    PIDController pid_heading;
    PIDController pid_RPM;
    PIDController pid_speed;

    // MPC
    MPCController mpc_heading;
    Eigen::Matrix2d A_d;
    Eigen::Vector2d B_d;

    // 控制周期，仿真倍速
    double control_T = 0.1;
    int t = 1000 * control_T, simulation_speed = 1;

    ControllerNode() : Node("controller_node")
    {
        // 创建话题发布者
        controller_output_publisher = this->create_publisher<message::msg::ControllerOutput>("controller_output", 1);

        // 创建话题订阅者
        simspeed_subscription_ = this->create_subscription<std_msgs::msg::Int32>("sim_speed", 1, std::bind(&ControllerNode::change_timer_period, this, std::placeholders::_1));
        shipstate_subscription_ = this->create_subscription<message::msg::ShipState>("shipstate_data", 1, std::bind(&ControllerNode::ship_state_data, this, std::placeholders::_1));
        control_objective_subscription_ = this->create_subscription<message::msg::ControlObjective>("control_objective", 1, std::bind(&ControllerNode::control_objective, this, std::placeholders::_1));
        identify_subscription_ = this->create_subscription<message::msg::NaviMode>("identify_completed_trigger", 1, std::bind(&ControllerNode::identify_completed, this, std::placeholders::_1));

        // 创建控制器定时器
        timer_controller_ = this->create_wall_timer(std::chrono::milliseconds(t), std::bind(&ControllerNode::controllers, this));

        // 声明参数->触发回调函数->初始化控制参数
        this->declare_parameter<std::vector<double>>("pid_heading_gains", std::vector<double>{0.5, 0, 1});
        this->declare_parameter<std::vector<double>>("pid_rpm_gains", std::vector<double>{0.15, 0.05, 0});
        this->declare_parameter<std::vector<double>>("pid_speed_gains", std::vector<double>{6, 2, 0});
        this->declare_parameter<std::vector<double>>("mpc_heading_gains", std::vector<double>{20, 10, 1, 0.1, 0.5}); // Np,Nu,q1,q2,R

        // MPC模型初始化
        A_d << 1, 1, 0, 1;
        B_d << 0.1, 0.2;
        mpc_heading.update_model(A_d, B_d);

        parameters_set(); // 读取辨识参数
    }

    ~ControllerNode()
    {
    }

    void parameters_set()
    {
        double identify_K, identify_T;
        double zeta = 0.707, w_n = 1, A, B, Kp, Kd;
        // 读取文件
        std::string filename = "identify_params.txt";
        std::ifstream file(filename);
        if (file.is_open())
        {
            // 尝试读取三个参数
            if (!(file >> identify_K >> identify_T >> delta_calibration))
            {
                RCLCPP_ERROR(this->get_logger(), "文件格式不正确或参数数量不足: %s", filename.c_str());
                delta_calibration = 0; // 初始化，防止delta_calibration = inf
                file.close();
                return;
            }

            // 打印文件路径
            std::filesystem::path full_path = std::filesystem::absolute(filename);
            RCLCPP_INFO(this->get_logger(), "读取参数文件: %s", full_path.string().c_str());
            file.close();

            // 根据模型参数调参
            A = pow(w_n, 2) * (1 / (zeta * w_n) + 2 * identify_T) / (2 * identify_K * 2);
            B = pow(w_n, 2) * (1 / (zeta * w_n) - 2 * identify_T) / (2 * identify_K * 2);
            Kp = A + B;
            Kd = -B;
            pid_heading.setGains(Kp, 0, Kd); // 更新PD控制器参数
            RCLCPP_INFO(this->get_logger(), "PD控制器(航向)参数: Kp = %f, Kd = %f", Kp, Kd);
            A_d << 1, identify_T * (1 - exp(-control_T / identify_T)),
                0, exp(-control_T / identify_T);
            B_d << identify_K * (control_T - identify_T * (1 - exp(-control_T / identify_T))),
                identify_K * (1 - exp(-control_T / identify_T));
            mpc_heading.update_model(A_d, B_d); // 更新MPC模型
            std::stringstream s_A_d;
            s_A_d << A_d;
            std::stringstream s_B_d;
            s_B_d << B_d;
            RCLCPP_INFO(this->get_logger(), "状态空间模型参数:\n A_d = %s\n B_d = %s", s_A_d.str().c_str(), s_B_d.str().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法读取文件: %s", filename.c_str());
        }
    }

private:
    // 设置参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_ =
        this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                bool should_update_gains = false;

                for (const auto &parameter : parameters)
                {
                    const std::string &name = parameter.get_name();

                    if (name == "pid_heading_gains" || name == "pid_rpm_gains" || name == "pid_speed_gains")
                    {
                        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
                        {
                            result.successful = false;
                            result.reason = "PID gain parameters must be double arrays.";
                            RCLCPP_WARN(this->get_logger(), "Rejected parameter set for '%s': %s", name.c_str(), result.reason.c_str());
                            return result;
                        }
                        auto gains = parameter.as_double_array();
                        if (gains.size() < 3)
                        {
                            result.successful = false;
                            result.reason = "PID gain array must contain at least 3 elements (Kp, Ki, Kd).";
                            RCLCPP_WARN(this->get_logger(), "Rejected parameter set for '%s': %s", name.c_str(), result.reason.c_str());
                            return result;
                        }
                        should_update_gains = true;
                    }
                    else if (name == "mpc_heading_gains")
                    {
                        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
                        {
                            result.successful = false;
                            result.reason = "MPC gain parameters must be double arrays.";
                            RCLCPP_WARN(this->get_logger(), "Rejected parameter set for '%s': %s", name.c_str(), result.reason.c_str());
                            return result;
                        }
                        auto gains = parameter.as_double_array();
                        if (gains.size() < 5)
                        {
                            result.successful = false;
                            result.reason = "MPC gain array must contain at least 5 elements (Np, Nu, q1, q2, R).";
                            RCLCPP_WARN(this->get_logger(), "Rejected parameter set for '%s': %s", name.c_str(), result.reason.c_str());
                            return result;
                        }
                        should_update_gains = true;
                    }
                }

                if (result.successful && should_update_gains)
                {
                    for (const auto &parameter : parameters)
                    {
                        const std::string &name = parameter.get_name();
                        if (name == "pid_heading_gains")
                        {
                            auto gains = parameter.as_double_array();
                            pid_heading.setGains(gains[0], gains[1], gains[2]);
                            RCLCPP_INFO(this->get_logger(), "Dynamically updated PID Heading Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f", gains[0], gains[1], gains[2]);
                        }
                        else if (name == "pid_rpm_gains")
                        {
                            auto gains = parameter.as_double_array();
                            pid_RPM.setGains(gains[0], gains[1], gains[2]);
                            RCLCPP_INFO(this->get_logger(), "Dynamically updated PID RPM Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f", gains[0], gains[1], gains[2]);
                        }
                        else if (name == "pid_speed_gains")
                        {
                            auto gains = parameter.as_double_array();
                            pid_speed.setGains(gains[0], gains[1], gains[2]);
                            RCLCPP_INFO(this->get_logger(), "Dynamically updated PID Speed Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f", gains[0], gains[1], gains[2]);
                        }
                        else if (name == "mpc_heading_gains")
                        {
                            auto gains = parameter.as_double_array();
                            int Np = static_cast<int>(gains[0]), Nu = static_cast<int>(gains[1]); // 预测时域Np不能小于控制时域Nu！
                            if (Np < Nu)
                            {
                                result.successful = false;
                                result.reason = "MPC gain parameters must be Np >= Nu.";
                                RCLCPP_WARN(this->get_logger(), "Rejected parameter set for '%s': %s", name.c_str(), result.reason.c_str());
                                return result;
                            }
                            Eigen::Matrix2d Q;
                            double q1 = gains[2], q2 = gains[3];
                            Q << q1, 0, 0, q2;
                            double R = gains[4];
                            mpc_heading.update_parameters(Np, Nu, Q, R);
                            RCLCPP_INFO(this->get_logger(), "Dynamically updated MPC Heading Gains: Np=%d, Nu=%d, q1=%.2f, q2=%.2f, R=%.2f", Np, Nu, q1, q2, R);
                        }
                    }
                }

                return result;
            });

    void change_timer_period(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int tmp = msg->data;
        if (tmp > 0)
        {
            if (tmp != simulation_speed)
            {
                simulation_speed = tmp;
                // 取消现有定时器
                timer_controller_->cancel();
                // 重新创建定时器，使用新的周期
                timer_controller_ = this->create_wall_timer(std::chrono::milliseconds(t / simulation_speed), std::bind(&ControllerNode::controllers, this));
            }
        }
    }

    void ship_state_data(const message::msg::ShipState::SharedPtr msg)
    {
        boat_angle = msg->angle;
        if (boat_angle > 180)
        {
            boat_angle -= 360;
        }
        boat_heading = msg->heading;
        if (boat_heading > 180)
        {
            boat_heading -= 360;
        }
        yaw_rate = msg->yaw_rate;
        boat_v = msg->speed;
        if (boat_v < 0.5)
        {
            // 船速小于0.5m/s时，用艏向代替航向
            boat_angle = boat_heading;
        }
        x << boat_angle,
            yaw_rate;
        n_now = msg->rpm;
        u_now = (msg->throttle_l + msg->throttle_r) / 2;
    }

    void control_objective(const message::msg::ControlObjective::SharedPtr msg)
    {
        stop = msg->stop;
        control_mode = msg->control_mode;
        v_d = msg->v_d;
        psi_d = msg->psi_d;
    }

    void identify_completed(const message::msg::NaviMode::SharedPtr msg)
    {
        if (msg->identify_completed)
        {
            parameters_set();
        }
    }

    double controller_heading(double psi_r)
    {
        // 角度相差超过180时，选择转弯幅度更小的方向
        if (psi_r - boat_angle < -180)
        {
            psi_r = 360 + psi_r;
        }
        if (psi_r - boat_angle > 180)
        {
            psi_r = -360 + psi_r;
        }

        // 计算控制量
        // double Delta = mpc_heading.computeControl(x, psi_r);
        double error = psi_r - boat_angle;
        double Delta = pid_heading.compute(error, control_T);
        // 限幅
        Delta = std::max(-limit, std::min(limit, Delta));

        return Delta;
    }

    int controller_RPM(int n_r)
    {
        int interval = 12, interval_count = interval / control_T; // 控制间隔
        static int u0 = 0, count = 0;
        static int n_r_last = 0;
        if (n_r_last != n_r) // 目标值变更
        {
            n_r_last = n_r;
            count = 0;
            pid_RPM.reset(); // 积分微分重置
        }

        if (count % interval_count == 0)
        {
            double error = n_r - n_now;
            if (abs(error) < 40 && abs(error) > 20)
            {
                pid_RPM.integral_sep(false);
            }
            else
            {
                pid_RPM.integral_sep(true); // 积分分离
            }
            // 计算控制量
            u0 = static_cast<int>(pid_RPM.compute(error, interval) + u_now);
            // 限幅
            u0 = std::max(1, std::min(450, u0));
        }

        count++;
        return u0;
    }

    int controller_speed(double v_r)
    {
        int interval = 10, interval_count = interval / control_T; // 控制间隔
        static int u0 = 0, count = 0;
        static double v_r_last = 0;
        if (v_r_last != v_r) // 目标值变更
        {
            v_r_last = v_r;
            count = 0;
            pid_speed.reset(); // 积分微分重置
        }

        if (count % interval_count == 0)
        {
            double error = v_r - boat_v;
            if (abs(error) < 0.4 && abs(error) > 0.2)
            {
                pid_speed.integral_sep(false);
            }
            else
            {
                pid_speed.integral_sep(true); // 积分分离
            }
            // 计算控制量
            u0 = static_cast<int>(pid_speed.compute(error, interval) + u_now);
            // 限幅
            u0 = std::max(1, std::min(450, u0));
        }

        count++;
        return u0;
    }

    void controllers()
    {
        int u = 0;
        double delta = 0;

        switch (stop)
        {
        case 2:
        {
            u = 0;
            delta = 0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "停车...");
            break;
        }
        case 1:
        {
            u = controller_speed(0);
            delta = 0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "刹车...");
            break;
        }
        case 0:
        default:
        {
            switch (control_mode)
            {
            case 3:
                break;
            case 2:
            {
                u = controller_speed(v_d);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "目标航速: %f | 当前航速: %f", v_d, boat_v);
                break;
            }
            case 1:
            {
                u = copysign(controller_RPM(abs(v_d)), v_d);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "目标转速: %d | 当前转速: %d", static_cast<int>(v_d), n_now);
                break;
            }
            case 0:
            default:
            {
                u = static_cast<int>(v_d);
                break;
            }
            }
            delta = controller_heading(psi_d);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "目标航向: %f | 当前航向: %f", psi_d, boat_angle);
            break;
        }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "油门控制指令: %d", u);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "舵角指令: %f + (%f)", delta, delta_calibration);

        // 状态预测
        Eigen::Vector2d x_predict = A_d * x + B_d * delta;

        // 舵角零位补偿
        delta += delta_calibration;
        delta = std::max(-limit, std::min(limit, delta));

        // 发布控制指令话题
        auto message_controller_output = message::msg::ControllerOutput();
        message_controller_output.u = u;
        message_controller_output.delta = delta;
        message_controller_output.angle_p = x_predict[0];
        message_controller_output.yaw_rate_p = x_predict[1];
        controller_output_publisher->publish(message_controller_output);
    }

    rclcpp::TimerBase::SharedPtr timer_controller_;

    rclcpp::Publisher<message::msg::ControllerOutput>::SharedPtr controller_output_publisher;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr simspeed_subscription_;
    rclcpp::Subscription<message::msg::ShipState>::SharedPtr shipstate_subscription_;
    rclcpp::Subscription<message::msg::ControlObjective>::SharedPtr control_objective_subscription_;
    rclcpp::Subscription<message::msg::NaviMode>::SharedPtr identify_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}