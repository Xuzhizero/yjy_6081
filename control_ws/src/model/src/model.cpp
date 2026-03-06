#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <cmath>
#include <memory>

#include "redis_utils.hpp"
#include "convert.hpp"

class ModelNode : public rclcpp::Node
{
public:
    double delta_offset = 3, delta = 0, delta1 = 0, delta2 = 0, limit = 30;
    // int u = 0, u1 = 0, Gear = 0, n = 0, n1 = 0, n_min = 450, n_max = 850, n_v0 = 0, N = n_min, N1 = n_min;
    int u = 0, u1 = 0, Gear = 0, n = 0, n1 = 0, n_min = 0, n_max = 450, n_v0 = 0, N = n_min, N1 = n_min; // 6081
    // double v_min = 2, v_max = 5; // 渔船
    // double v_min = 2, v_max = 12; // 槽道舟
    double v_min = 0, v_max = 11; // 6081
    // double K_turning = 0.13, T_turning = 7, K_speed = 0.0075, T_speed = 10, K_n = 4, T_n = 0.5; // 渔船
    // double K_turning = 0.75, T_turning = 3, K_speed = (v_max - v_min) / (n_max - n_min), T_speed = 3, K_n = 4, T_n = 0.5; // 槽道舟
    double K_turning = 0.75, T_turning = 3, K_speed = (v_max - v_min) / (n_max - n_min), T_speed = 3, K_n = 1, T_n = 0.5; // 6081
    double coefficient1, coefficient2, coefficient3, coefficient4, coefficient5, coefficient6, coefficient7;

    double boat_lat = 29.91715, boat_lon = 122.30931, boat_x, boat_y, boat_angle = -90, yaw_rate = 0, boat_v = 0, ve, vn;
    double boat_angle1 = boat_angle, boat_angle2 = boat_angle, boat_v1 = boat_v;
    double deg2rad = M_PI / 180.0, rad2deg = 180.0 / M_PI;
    ROUTE_POS now_pos;
    PLA_POINT now_point;
    std::unique_ptr<GPSLocalConvert> m_pGCT = std::make_unique<GPSLocalConvert>(); // 经纬度转换类
    double T_step = 0.1;
    int t_step = 1000 * T_step, simulation_speed = 1;

    ModelNode() : Node("model_node")
    {
        // 初始化Redis连接
        init_redis("127.0.0.1", 6379, 1000, 1000); // ip，端口，连接超时，命令超时(ms)

        // 参数给定
        n_v0 = (v_max * n_min - v_min * n_max) / (v_max - v_min);
        coefficient1 = 8 * T_turning / (4 * T_turning + 2 * T_step);
        coefficient2 = (2 * T_step - 4 * T_turning) / (4 * T_turning + 2 * T_step);
        coefficient3 = pow(T_step, 2) * K_turning / (4 * T_turning + 2 * T_step);
        coefficient4 = (2 * T_speed - T_step) / (2 * T_speed + T_step);
        coefficient5 = T_step * K_speed / (2 * T_speed + T_step);
        coefficient6 = (2 * T_n - T_step) / (2 * T_n + T_step);
        coefficient7 = T_step * K_n / (2 * T_n + T_step);

        // 初始化船体位置
        now_pos.lat = boat_lat;
        now_pos.lon = boat_lon;
        now_point = m_pGCT->BL2XY(now_pos);
        boat_x = now_point.x;
        boat_y = now_point.y;

        // 创建话题发布者
        simspeed_publisher_ = this->create_publisher<std_msgs::msg::Int32>("sim_speed", 1);

        // 创建模型定时器
        timer_model_ = this->create_wall_timer(std::chrono::milliseconds(t_step), std::bind(&ModelNode::model, this));
    }

    ~ModelNode()
    {
        // 关闭Redis连接
        close_redis();
    }

private:
    void update_data()
    {
        // 获取仿真倍速
        int tmp = 0;
        if (read_from_redis("Navi", "Sim_speed_scaler", tmp))
        {
            // 数值校验
            if (tmp > 0)
            {
                // 发布仿真倍速话题
                auto message_simspeed = std_msgs::msg::Int32();
                message_simspeed.data = tmp;
                simspeed_publisher_->publish(message_simspeed);

                // 防止多次重建
                if (tmp != simulation_speed)
                {
                    simulation_speed = tmp;
                    // 取消当前定时器
                    timer_model_->cancel();
                    // 重新创建定时器
                    timer_model_ = this->create_wall_timer(std::chrono::milliseconds(t_step / simulation_speed), std::bind(&ModelNode::model, this));
                }
            }
        }

        // 获取转速控制指令
        if (!read_from_redis("Navi", "TargetU", u))
        {
            u = 0;
        }

        // 根据转速控制指令判断挡位
        if (u == 0)
        {
            Gear = 0;
        }
        else
        {
            Gear = u / abs(u);
        }

        // 设置当前油门开度
        write_to_redis("Navi", "now_left_pos", u);
        write_to_redis("Navi", "now_right_pos", u);

        u = abs(u); // 转速模型u为正

        // 获取目标舵角
        if (!read_from_redis("Navi", "TargetDuo", delta))
        {
            delta = 0;
        }

        // 设置当前舵角
        write_to_redis("Navi", "NowDuo", delta);

        // 舵角偏置
        delta -= delta_offset;
    }

    void update_model()
    {
        write_to_redis("engine_parameters", "zhuan_su", N); // 设置转速
        write_to_redis("IMU", "Lon", now_pos.lon);          // 设置经度
        write_to_redis("IMU", "Lat", now_pos.lat);          // 设置纬度
        write_to_redis("IMU", "speed", boat_v);             // 设置速度
        write_to_redis("IMU", "heading", boat_angle);       // 设置艏向角
        write_to_redis("IMU", "angle", boat_angle);         // 设置航向角
        write_to_redis("IMU", "yaw_rate", yaw_rate);        // 设置角速度
    }

    void model()
    {
        // 更新数据
        update_data();

        // 船体转向模型，输入为舵角，输出为实际转角，限定至-180~180
        boat_angle = coefficient1 * boat_angle1 + coefficient2 * boat_angle2 + coefficient3 * (delta2 + 2 * delta1 + delta);
        if (boat_angle > 180)
        {
            boat_angle2 = boat_angle2 - 360;
            boat_angle1 = boat_angle1 - 360;
            boat_angle = boat_angle - 360;
        }
        if (boat_angle < -180)
        {
            boat_angle2 = boat_angle2 + 360;
            boat_angle1 = boat_angle1 + 360;
            boat_angle = boat_angle + 360;
        }
        // 更新状态
        delta2 = delta1;
        delta1 = delta;
        yaw_rate = (boat_angle - boat_angle1) / T_step;
        boat_angle2 = boat_angle1;
        boat_angle1 = boat_angle;

        if (Gear == 0)
        {
            N = n_v0;
        }
        if (Gear == -1)
        {
            N = 2 * n_v0 - N;
        }
        // 船体速度模型
        boat_v = coefficient4 * boat_v1 + coefficient5 * ((N - n_v0) + (N1 - n_v0));
        boat_v1 = boat_v;
        N1 = N;
        // 转速模型
        n = coefficient6 * n1 + coefficient7 * (u + u1);
        n1 = n;
        u1 = u;
        N = n + n_min;

        // 根据船速更新船体位置
        ve = boat_v * sin(boat_angle * deg2rad);
        vn = boat_v * cos(boat_angle * deg2rad);
        boat_x = boat_x + ve * T_step;
        boat_y = boat_y + vn * T_step;

        // 转换至经纬度
        now_point.x = boat_x;
        now_point.y = boat_y;
        now_pos = m_pGCT->XY2BL(now_point);

        // 更新模型
        update_model();
    }

    rclcpp::TimerBase::SharedPtr timer_model_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr simspeed_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}