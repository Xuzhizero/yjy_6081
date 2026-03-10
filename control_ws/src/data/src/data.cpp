#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <chrono>
#include <nlohmann/json.hpp>

#include "redis_utils.hpp"
#include "message/msg/ship_state.hpp"
#include "message/msg/navi_mode.hpp"
#include "message/msg/controller_output.hpp"

using json = nlohmann::json;

class DataNode : public rclcpp::Node
{
public:
    int RC_mode = 0;  // 遥控模式
    int State = 0;    // 手自动状态
    int identify = 0; // 自动辨识模式
    int u = 0;
    double delta = 0, delta_calibration = 0;

    int t = 100, simulation_speed = 1;

    DataNode() : Node("data_node")
    {
        // 初始化Redis连接
        init_redis("127.0.0.1", 6379, 1000, 1000); // ip，端口，连接超时，命令超时(ms)

        // 创建话题发布者
        navimode_publisher_ = this->create_publisher<message::msg::NaviMode>("navimode_data", 1);
        shipstate_publisher_ = this->create_publisher<message::msg::ShipState>("shipstate_data", 1);
        identify_publisher_ = this->create_publisher<message::msg::NaviMode>("identify_completed_trigger", 1);

        // 创建话题订阅者
        simspeed_subscription_ = this->create_subscription<std_msgs::msg::Int32>("sim_speed", 1, std::bind(&DataNode::change_timer_period, this, std::placeholders::_1));
        controller_output_subscription_ = this->create_subscription<message::msg::ControllerOutput>("controller_output", 1, std::bind(&DataNode::command_sender, this, std::placeholders::_1));
        berth_subscription_ = this->create_subscription<message::msg::NaviMode>("berth_path", 1, std::bind(&DataNode::berth_path_set, this, std::placeholders::_1));

        // 创建定时器，用于定期接收数据，发布话题
        timer_data_ = this->create_wall_timer(std::chrono::milliseconds(t), std::bind(&DataNode::get_data, this)); // 默认周期100ms
    }

    ~DataNode()
    {
        // 关闭Redis连接
        close_redis();
    }

    std::vector<std::string> split_by_comma(const std::string &str)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;

        while (std::getline(ss, token, ','))
        {
            tokens.push_back(token);
        }
        return tokens;
    }

private:
    void change_timer_period(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int tmp = msg->data;
        if (tmp > 0)
        {
            if (tmp != simulation_speed)
            {
                simulation_speed = tmp;
                // 取消现有定时器
                timer_data_->cancel();
                // 重新创建定时器，使用新的周期
                timer_data_ = this->create_wall_timer(std::chrono::milliseconds(t / simulation_speed), std::bind(&DataNode::get_data, this));
            }
        }
    }

    void command_sender(const message::msg::ControllerOutput::SharedPtr msg)
    {
        if (identify == 0) // 自动辨识模式下采用预设输入
        {
            // 获取控制指令
            u = msg->u;
            delta = msg->delta;
        }

        if (State && !RC_mode) // 自动模式且非遥控模式时，下发控制指令
        {
            // 设置舵角
            write_to_redis("Navi", "TargetDuo", delta);
            // write_to_redis("DP_Ctrl", "prop__angle_l", -delta);
            // write_to_redis("DP_Ctrl", "prop__angle_r", -delta);

            // 设置油门开度
            write_to_redis("Navi", "TargetU", u);
            // write_to_redis("Navi", "left_zhuansu", u);
            // write_to_redis("Navi", "right_zhuansu", u);
        }
    }

    void berth_path_set(const message::msg::NaviMode::SharedPtr msg)
    {
        // 设置靠泊路径
        write_to_redis("Navi", "GPath", msg->berth_path);
    }

    void identify_mode()
    {
        static int icount = 0;
        double run_time = static_cast<double>(icount) * t / 1000.0;
        double T_s = 20, T_t = 100; // 稳定直行20秒, 回转100秒
        double yaw_rate = 0;
        static std::vector<double> yaw_rate_arr;

        if (run_time < T_s)
        {
            if (icount == 0)
            {
                yaw_rate_arr.clear();
                RCLCPP_INFO(this->get_logger(), "自动辨识模式");
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "直行...");
            delta = delta_calibration; // 尽可能保证直行，以使初始角速度为稳态0
            u = 50;
            icount++;
        }
        else if (run_time < T_s + T_t)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "回转...");
            read_from_redis("IMU", "yaw_rate", yaw_rate);
            yaw_rate_arr.push_back(yaw_rate);
            delta = 20;
            u = 50;
            icount++;
        }
        else
        {
            double yaw_rate_ss = yaw_rate_arr.back();                                 // 输出稳态终值
            double D_yaw_rate = yaw_rate_ss - yaw_rate_arr.front();                   // 输出变化量
            double identify_K = D_yaw_rate / (delta - delta_calibration), identify_T; // 辨识参数
            double T_yaw_rate = D_yaw_rate * 0.632 + yaw_rate_arr.front();            // 1T处输出
            double min_d = abs(D_yaw_rate);
            for (size_t i = 0; i < yaw_rate_arr.size(); i++)
            {
                if (abs(yaw_rate_arr[i] - T_yaw_rate) < min_d)
                {
                    min_d = abs(yaw_rate_arr[i] - T_yaw_rate);
                    identify_T = static_cast<double>(i) * t / 1000.0;
                }
            }
            delta_calibration = delta - yaw_rate_ss / identify_K; // 舵角零位补偿
            RCLCPP_INFO(this->get_logger(), "辨识完成: K = %f, T = %f, delta_calibration = %f", identify_K, identify_T, delta_calibration);
            // 写入文件
            std::string filename = "identify_params.txt";
            std::ofstream file(filename, std::ios::trunc);
            if (file.is_open())
            {
                file << identify_K << "\n";
                file << identify_T << "\n";
                file << delta_calibration << "\n";
                file.close();

                // 获取当前工作目录并显示完整路径
                std::filesystem::path full_path = std::filesystem::absolute(filename);
                RCLCPP_INFO(this->get_logger(), "参数已写入文件: %s", full_path.string().c_str());

                // 发布辨识完成消息
                auto message_identify = message::msg::NaviMode();
                message_identify.identify_completed = true;
                identify_publisher_->publish(message_identify);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "无法写入文件: %s", filename.c_str());
            }

            // 辨识模式置0
            if (write_to_redis("Navi", "identify", 0))
            {
                icount = 0;
            }
        }
    }

    void process_path(std::string &path, int &control_mode)
    {
        // 获取控制模式
        read_from_redis("Navi", "speed_control_mode", control_mode);

        // 获取目标航速
        std::string target_speed = "0";
        if (control_mode == 0) // 挡位控制
        {
            int tmp = 0;
            read_from_redis("Navi", "TargetSpeed", tmp);
            switch (tmp)
            {
            case 1:
                target_speed = "80";
                break;
            case 2:
                target_speed = "180";
                break;
            case 3:
                target_speed = "280";
                break;
            case 4:
                target_speed = "310";
                break;
            case 5:
                target_speed = "360";
                break;
            case 6:
                target_speed = "450";
                break;
            default:
                target_speed = "0";
                break;
            }
        }
        else
        {
            read_from_redis("Navi", "TargetSpeed", target_speed);
        }

        // 获取路径
        if (!read_from_redis("Navi", "LPath", path) || path.empty() || path == "$LP,-666,0")
        {
            /////////////////////////////////////////////////////////
            // 6081专供，GP模式下仅支持挡位控制
            write_to_redis("Navi", "speed_control_mode", 0);
            /////////////////////////////////////////////////////////

            // 如果LP不存在或为空字符串，找GP
            if (!read_from_redis("Navi", "GPath", path) || path.empty())
            {
                // LP和GP均不存在
                path = "$GP," + target_speed + ",0";
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无目标路径！");
            }
            else
            {
                path.insert(4, target_speed + ","); // 4 是 "$GP," 后的第一个位置
            }
        }

        // 遥控或手动模式下，发送GP0
        if (RC_mode || !State)
        {
            path = "$GP,0,0";
        }
    }

    void process_autopilot(int &autopilot, double &heading_target)
    {
        std::string autopilot_data = "0";
        // 获取自动舵模式数据"开关,航向角"
        read_from_redis("Navi", "zidongduo", autopilot_data);
        std::vector<std::string> data = split_by_comma(autopilot_data);

        autopilot = std::stoi(data[0]);
        if (autopilot)
        {
            if (data.size() > 1)
            {
                heading_target = std::stod(data[1]);
            }
            else
            {
                autopilot = 0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无目标航向！");
            }
        }
    }

    void process_berth(int &berthing, double &berth_heading, double &berth_lon, double &berth_lat)
    {
        // 获取靠泊模式数据
        std::string berth_mode;
        if (read_from_redis("Dock_Mode", "Status", berth_mode))
        {
            if (berth_mode == "docking")
            {
                berthing = 1;
            }
            else if (berth_mode == "departing")
            {
                berthing = 2;
            }
            else
            {
                berthing = 0;
            }
        }

        if (berthing)
        {
            std::string jsonStr;
            if (read_from_redis("Dock_Mode", "berth", jsonStr))
            {
                auto j = json::parse(jsonStr);
                berth_heading = j["heading"];
                berth_lon = j["center"]["lng"];
                berth_lat = j["center"]["lat"];
            }
            else
            {
                berthing = 0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无泊位信息！");
            }
        }
    }

    void publish_navimode()
    {
        std::string path;
        int stop = 0, control_mode = 0, autopilot = 0, berthing = 0;
        double heading_target = 0, berth_heading = 0, berth_lon = 0, berth_lat = 0;

        read_from_redis("MotorCtrl", "Stop", stop);
        process_path(path, control_mode);
        process_autopilot(autopilot, heading_target);
        process_berth(berthing, berth_heading, berth_lon, berth_lat);

        // 发布航行模式消息
        auto message_navimode = message::msg::NaviMode();
        message_navimode.stop = stop;
        message_navimode.control_mode = control_mode;
        message_navimode.path = path;
        message_navimode.autopilot = autopilot;
        message_navimode.heading_target = heading_target;
        message_navimode.berthing = berthing;
        message_navimode.berth_heading = berth_heading;
        message_navimode.berth_lon = berth_lon;
        message_navimode.berth_lat = berth_lat;
        navimode_publisher_->publish(message_navimode);
    }

    void publish_shipstate()
    {
        double lon = 0, lat = 0, angle = 0, heading = 0, yaw_rate = 0, speed = 0;
        int rpm = 0, throttle_l = 0, throttle_r = 0;

        read_from_redis("IMU", "Lon", lon);                    // 获取经度
        read_from_redis("IMU", "Lat", lat);                    // 获取纬度
        read_from_redis("IMU", "angle", angle);                // 获取航向角
        read_from_redis("IMU", "heading", heading);            // 获取艏向
        read_from_redis("IMU", "yaw_rate", yaw_rate);          // 获取角速度
        read_from_redis("IMU", "speed", speed);                // 获取速度
        read_from_redis("engine_parameters", "zhuan_su", rpm); // 获取转速
        read_from_redis("Navi", "now_left_pos", throttle_l);   // 获取油门开度左
        read_from_redis("Navi", "now_right_pos", throttle_r);  // 获取油门开度右

        // 发布船体状态消息
        auto message_shipstate = message::msg::ShipState();
        message_shipstate.lon = lon;
        message_shipstate.lat = lat;
        message_shipstate.angle = angle;
        message_shipstate.heading = heading;
        message_shipstate.yaw_rate = yaw_rate;
        message_shipstate.speed = speed;
        message_shipstate.rpm = rpm;
        message_shipstate.throttle_l = throttle_l;
        message_shipstate.throttle_r = throttle_r;
        shipstate_publisher_->publish(message_shipstate);
    }

    void get_data()
    {
        // 获取手自动模式状态
        if (!read_from_redis("Navi", "State", State))
        {
            State = 0;
        }

        // 获取遥控模式状态
        if (!read_from_redis("Navi", "RC", RC_mode))
        {
            RC_mode = 0;
        }
        if (RC_mode)
        {
            double tmp = 0;
            // 远程遥控模式下监控Navi_s下的Target_Duo，判断是否失连
            if (read_from_redis("Navi_s", "TargetDuo", tmp))
            {
                // 连接正常
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "遥控中...");
            }
            else
            {
                // 连接失败
                RC_mode = 0;
                RCLCPP_INFO(this->get_logger(), "遥控模式连接失败！");

                // 远程遥控模式置0
                write_to_redis("Navi", "RC", 0);
            }
        }

        // 获取自动辨识模式状态
        if (!read_from_redis("Navi", "identify", identify))
        {
            identify = 0;
        }
        if (identify)
        {
            // 辨识模式
            identify_mode();
        }

        // 航行模式话题
        publish_navimode();

        // 船体状态话题
        publish_shipstate();
    }

    rclcpp::TimerBase::SharedPtr timer_data_;

    rclcpp::Publisher<message::msg::NaviMode>::SharedPtr navimode_publisher_;
    rclcpp::Publisher<message::msg::ShipState>::SharedPtr shipstate_publisher_;
    rclcpp::Publisher<message::msg::NaviMode>::SharedPtr identify_publisher_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr simspeed_subscription_;
    rclcpp::Subscription<message::msg::ControllerOutput>::SharedPtr controller_output_subscription_;
    rclcpp::Subscription<message::msg::NaviMode>::SharedPtr berth_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}