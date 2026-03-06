#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <utility>
#include <algorithm>
#include <memory>

#include "convert.hpp"
#include "message/msg/ship_state.hpp"
#include "message/msg/navi_mode.hpp"
#include "message/msg/control_objective.hpp"

class ModeNode : public rclcpp::Node
{
public:
    // 控制目标
    int stop = 0;         // 0正常，1刹车，2停车
    int control_mode = 0; // 0转速控制，1航速控制，2位置控制
    double v_d = 0, psi_d = 0;

    // 航行模式
    int auto_pilot = 0, berthing = 0, berth_start = 0;
    double heading_target = 0, berth_heading = 0;
    PLA_POINT now_point, D_point, C_point, B_point, A_point;
    ROUTE_POS D_pos, C_pos, B_pos, A_pos;
    std::vector<std::pair<double, double>> line;
    int num = 0, linenum = 0;
    double R0, r0, R1, r1, r2, r_final; // r0视线半径，r1全局接纳圆，r2局部接纳圆，r_final终点接纳圆

    // 船体状态
    double boat_lon, boat_lat, boat_x, boat_y, boat_angle = 0;
    double deg2rad = M_PI / 180.0, rad2deg = 180.0 / M_PI;
    std::unique_ptr<GPSLocalConvert> m_pGCT = std::make_unique<GPSLocalConvert>(); // 经纬度转换类

    // 控制周期
    double control_T = 0.1;
    int t = 1000 * control_T, simulation_speed = 1;

    ModeNode() : Node("mode_node")
    {
        // 创建话题发布者
        control_objective_publisher_ = this->create_publisher<message::msg::ControlObjective>("control_objective", 1);
        berth_path_publisher_ = this->create_publisher<message::msg::NaviMode>("berth_path", 1); // "$GP,4,lon,lat,lon,lat,lon,lat,lon,lat"

        // 创建话题订阅者
        simspeed_subscription_ = this->create_subscription<std_msgs::msg::Int32>("sim_speed", 1, std::bind(&ModeNode::change_timer_period, this, std::placeholders::_1));
        navimode_subscription_ = this->create_subscription<message::msg::NaviMode>("navimode_data", 1, std::bind(&ModeNode::navi_mode_data, this, std::placeholders::_1));
        shipstate_subscription_ = this->create_subscription<message::msg::ShipState>("shipstate_data", 1, std::bind(&ModeNode::ship_state_data, this, std::placeholders::_1));

        // 创建定时器，用于模式选择，发布控制目标话题
        timer_mode_ = this->create_wall_timer(std::chrono::milliseconds(t), std::bind(&ModeNode::modes, this));

        // 声明参数->触发回调函数->初始化LOS参数
        this->declare_parameter<std::vector<double>>("LOS_parameters", std::vector<double>{150, 60, 60, 60}); // r0,r1,r2,r_final
    }

    ~ModeNode()
    {
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

    double calculate_distance(PLA_POINT A, PLA_POINT B)
    {
        return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
    }

private:
    // 设置参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_ =
        this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                bool should_update_parameters = false;

                for (const auto &parameter : parameters)
                {
                    const std::string &name = parameter.get_name();

                    if (name == "LOS_parameters")
                    {
                        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
                        {
                            result.successful = false;
                            result.reason = "LOS parameters must be double arrays.";
                            RCLCPP_WARN(this->get_logger(), "Rejected parameter set for '%s': %s", name.c_str(), result.reason.c_str());
                            return result;
                        }
                        auto gains = parameter.as_double_array();
                        if (gains.size() < 4)
                        {
                            result.successful = false;
                            result.reason = "LOS gain array must contain at least 4 elements (r0, r1, r2, r_final).";
                            RCLCPP_WARN(this->get_logger(), "Rejected parameter set for '%s': %s", name.c_str(), result.reason.c_str());
                            return result;
                        }
                        should_update_parameters = true;
                    }
                }

                if (result.successful && should_update_parameters)
                {
                    for (const auto &parameter : parameters)
                    {
                        const std::string &name = parameter.get_name();
                        if (name == "LOS_parameters")
                        {
                            auto gains = parameter.as_double_array();
                            r0 = gains[0];
                            r1 = gains[1];
                            r2 = gains[2];
                            r_final = gains[3];
                            RCLCPP_INFO(this->get_logger(), "Dynamically updated LOS parameters: r0=%.2f, r1=%.2f, r2=%.2f, r_final=%.2f", gains[0], gains[1], gains[2], gains[3]);
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
                timer_mode_->cancel();
                // 重新创建定时器，使用新的周期
                timer_mode_ = this->create_wall_timer(std::chrono::milliseconds(t / simulation_speed), std::bind(&ModeNode::modes, this));
            }
        }
    }

    void process_path_data(std::string &path_data)
    {
        std::vector<std::string> path = split_by_comma(path_data);

        // 最小长度3"$GP,0,0"
        if (path.size() < 3)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "路径数据非法！");
            return;
        }
        // 判断"$GP,0,0"后的数量是否为偶数
        if ((path.size() - 3) % 2 != 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "经纬度坐标不成对！");
            return;
        }
        // 判断路径点数量是否符合
        if ((path.size() - 3) / 2 != static_cast<size_t>(std::stoi(path[2])))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "路径点数量不符！");
            return;
        }

        // 格式头
        if (path[0] == "$GP")
        {
            R1 = r1;
        }
        else if (path[0] == "$LP")
        {
            R1 = r2;
        }
        else
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "格式头未定义！");
            return;
        }

        // 目标航速
        v_d = std::stod(path[1]);

        static std::string path_last;
        std::string path_now;
        for (size_t i = 3; i < path.size(); i++)
        {
            path_now += path[i];
        }
        // 路径变更
        if (path_now != path_last)
        {
            line.clear();
            num = 0;

            ROUTE_POS path_pos;
            PLA_POINT path_point;
            // 坐标转换并存储
            for (size_t i = 3; i < path.size(); i += 2)
            {
                // 成对读取坐标
                path_pos.lon = std::stod(path[i]);
                path_pos.lat = std::stod(path[i + 1]);

                // 经纬度转换至平面直角坐标系
                path_point = m_pGCT->BL2XY(path_pos);

                // 存储至line
                line.emplace_back(path_point.x, path_point.y);
            }

            // std::unique移动相邻的重复元素到末尾
            auto new_end_it = std::unique(line.begin(), line.end());
            // 去除重复路径点
            line.erase(new_end_it, line.end());
            // 路径点数量
            linenum = line.size();

            path_last = path_now;
        }
    }

    void process_berth_data(double berth_lon, double berth_lat)
    {
        static std::string berth_data_last;
        std::string berth_data = std::to_string(berth_heading) + std::to_string(berth_lon) + std::to_string(berth_lat);

        if (berth_data != berth_data_last)
        {
            RCLCPP_INFO(this->get_logger(), "目标泊位变更...");
            berth_start = 0;

            auto calculate_point = [](PLA_POINT origin, double angle_deg, double distance) -> PLA_POINT
            {
                double angle_rad = angle_deg * M_PI / 180.0; // 顺时针
                PLA_POINT result;
                result.x = origin.x + distance * sin(angle_rad);
                result.y = origin.y + distance * cos(angle_rad);
                return result;
            };

            D_pos.lon = berth_lon;
            D_pos.lat = berth_lat;
            D_point = m_pGCT->BL2XY(D_pos);
            double angle_deg = berth_heading + 180;
            C_point = calculate_point(D_point, angle_deg, 3);
            double heading_D2now = atan2(boat_x - D_point.x, boat_y - D_point.y) * rad2deg;
            double heading_temp = heading_D2now - berth_heading; // -360~360
            if ((heading_temp > 0 && heading_temp < 180) || (heading_temp > -360 && heading_temp < -180))
            {
                B_point = calculate_point(C_point, angle_deg - 30, 100);
                A_point = calculate_point(B_point, angle_deg - 60, 200);
            }
            else if ((heading_temp > -180 && heading_temp < 0) || (heading_temp > 180 && heading_temp < 360))
            {
                B_point = calculate_point(C_point, angle_deg + 30, 100);
                A_point = calculate_point(B_point, angle_deg + 60, 200);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "当前位置无法靠泊！");
                berthing = 0;
                return;
            }
            D_pos = m_pGCT->XY2BL(D_point);
            C_pos = m_pGCT->XY2BL(C_point);
            B_pos = m_pGCT->XY2BL(B_point);
            A_pos = m_pGCT->XY2BL(A_point);

            // 发布靠泊路径话题
            auto message_berth_path = message::msg::NaviMode();
            std::ostringstream oss;
            oss << "$GP,4,"
                << A_pos.lon << "," << A_pos.lat << ","
                << B_pos.lon << "," << B_pos.lat << ","
                << C_pos.lon << "," << C_pos.lat << ","
                << D_pos.lon << "," << D_pos.lat;
            message_berth_path.berth_path = oss.str();
            berth_path_publisher_->publish(message_berth_path);

            berth_data_last = berth_data;
        }
    }

    void navi_mode_data(const message::msg::NaviMode::SharedPtr msg)
    {
        stop = msg->stop;
        control_mode = msg->control_mode;
        process_path_data(msg->path);
        auto_pilot = msg->autopilot;
        heading_target = msg->heading_target;
        if (heading_target > 180)
        {
            heading_target -= 360;
        }
        berthing = msg->berthing;
        if (berthing == 1)
        {
            berth_heading = msg->berth_heading;
            if (berth_heading > 180)
            {
                berth_heading -= 360;
            }
            process_berth_data(msg->berth_lon, msg->berth_lat);
        }
        else
        {
            berth_start = 0;
        }
    }

    void ship_state_data(const message::msg::ShipState::SharedPtr msg)
    {
        ROUTE_POS now_pos;
        now_pos.lon = msg->lon;
        now_pos.lat = msg->lat;
        now_point = m_pGCT->BL2XY(now_pos);
        boat_x = now_point.x;
        boat_y = now_point.y;
        boat_angle = msg->angle;
        if (boat_angle > 180)
        {
            boat_angle -= 360;
        }
    }

    int is_incircle(int k = 1)
    {
        if (k < 1)
        {
            k = 1; // 搜索窗口最小为1
        }
        int flag = 0;
        for (int i = num, j = 0; (i < linenum && j <= k); i++, j++)
        {
            double dis_point = calculate_distance(now_point, PLA_POINT(line[i].first, line[i].second));
            // 判断是否在接纳圆内
            if (dis_point < R1)
            {
                // 终点接纳圆半径更小
                if (i == linenum - 1 && dis_point > r_final)
                {
                    break;
                }
                num = i;
                flag = 1;
            }
        }
        return flag;
    }

    void path_segment(int k = 1)
    {
        if (k < 1)
        {
            k = 1; // 搜索窗口最小为1
        }
        double A, B, C, d, X_N, Y_N;
        double start_dis, new_dis, dis_final;
        for (int i = num, j = 0; (i < linenum - 1 && j <= k); i++, j++)
        {
            A = line[i + 1].second - line[i].second;
            B = line[i].first - line[i + 1].first;
            C = line[i + 1].first * line[i].second - line[i].first * line[i + 1].second;
            d = abs(A * boat_x + B * boat_y + C) / sqrt(pow(A, 2) + pow(B, 2));
            X_N = (-A * C - A * B * boat_y + B * B * boat_x) / (pow(A, 2) + pow(B, 2));
            Y_N = (-B * C + A * A * boat_y - A * B * boat_x) / (pow(A, 2) + pow(B, 2));

            // 判断当前位置在航线上的垂足是否在延长线上
            if (((line[i].first - X_N) * (line[i + 1].first - X_N) + (line[i].second - Y_N) * (line[i + 1].second - Y_N)) > 0)
            {
                new_dis = calculate_distance(now_point, PLA_POINT(line[i].first, line[i].second));
            }
            else
            {
                new_dis = d;
            }
            if (i == num)
            {
                start_dis = new_dis;
            }
            if (start_dis > new_dis)
            {
                start_dis = new_dis;
                num = i;
            }
        }
        // 判断是否已经越过终点
        if (num == linenum - 2)
        {
            dis_final = calculate_distance(now_point, PLA_POINT(line[num + 1].first, line[num + 1].second));
            if (((line[num].first - X_N) * (line[num + 1].first - X_N) + (line[num].second - Y_N) * (line[num + 1].second - Y_N)) > 0)
            {
                if (dis_final < new_dis)
                {
                    num++;
                }
            }
        }
    }

    void los_control()
    {
        double A, B, C, d, X_N, Y_N, theta, L, X_LOS, Y_LOS, psi_PATH;
        static double d_max = 0, d_sum = 0, d2_sum = 0, d_average = 0, d_count = 1, RMSE = 0, psi_PATH1 = 0;
        static int num1 = 0, steady_state = 0, Flag = 0;
        // 航段检测，更新num
        if (is_incircle() == 0)
        {
            path_segment();
        }
        if (calculate_distance(PLA_POINT(line[0].first, line[0].second), PLA_POINT(line[linenum - 1].first, line[linenum - 1].second)) < 100)
        {
            // 起点终点距离小于100米，跑圈
            num %= linenum - 1;
        }
        if (num < linenum - 1)
        {
            // LOS算法
            A = line[num + 1].second - line[num].second;
            B = line[num].first - line[num + 1].first;
            C = line[num + 1].first * line[num].second - line[num].first * line[num + 1].second;
            d = abs(A * boat_x + B * boat_y + C) / sqrt(pow(A, 2) + pow(B, 2));
            X_N = (-A * C - A * B * boat_y + B * B * boat_x) / (pow(A, 2) + pow(B, 2));
            Y_N = (-B * C + A * A * boat_y - A * B * boat_x) / (pow(A, 2) + pow(B, 2));
            theta = atan2(A, -B);
            if (1.1 * d > r0)
            {
                R0 = 1.1 * d; // arccos10/11 = 24.62
            }
            else
            {
                R0 = r0;
            }
            L = sqrt(pow(R0, 2) - pow(d, 2));
            X_LOS = X_N + L * cos(theta);
            Y_LOS = Y_N + L * sin(theta);
            psi_d = atan2(X_LOS - boat_x, Y_LOS - boat_y) * rad2deg; // LOS角

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前跟踪第%d航段", num + 1);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "垂直距离: %f m", d);

            // 计算误差
            psi_PATH = atan2(line[num + 1].first - line[num].first, line[num + 1].second - line[num].second) * rad2deg;
            if (abs(psi_PATH1 - psi_PATH) > 1) // 跟踪路径航段发生变更
            {
                d_average = d_sum / d_count;
                RMSE = sqrt(d2_sum / d_count);
                RCLCPP_INFO(this->get_logger(), "第%d航段, 平均误差: %f, 均方根误差: %f, 最大误差: %f", num1 + 1, d_average, RMSE, d_max);
                num1 = num;
                steady_state = 0;
                psi_PATH1 = psi_PATH;
            }
            if (abs(boat_angle - psi_PATH) < 10 && d < 10)
            {
                steady_state = 1; // 进入稳态
            }
            if (steady_state)
            {
                d_sum += d;
                d2_sum += d * d;
                d_count++;
                if (d > d_max)
                {
                    d_max = d;
                }
            }

            // 航行状态: 未完成
            Flag = 0;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "航行完成!");
            stop = 2;
            if (Flag == 0)
            {
                d_average = d_sum / d_count;
                RMSE = sqrt(d2_sum / d_count);
                RCLCPP_INFO(this->get_logger(), "第%d航段, 平均误差: %f, 均方根误差: %f, 最大误差: %f", num1 + 1, d_average, RMSE, d_max);
                d_sum = 0;
                d2_sum = 0;
                d_count = 0;
                d_max = 0;
                num1 = 0;
                steady_state = 0;

                // 航行状态: 完成
                Flag = 1;
            }
        }
    }

    void modes()
    {
        if (stop != 1)
        {
            if (berthing == 1) // 靠泊模式下控制模式自定义
            {
                if (!berth_start)
                {
                    if (calculate_distance(now_point, A_point) < 100)
                    {
                        berth_start = 1;
                    }
                    else
                    {
                        // 前往起始A点
                        control_mode = 0;
                        v_d = 50;
                        psi_d = atan2(A_point.x - boat_x, A_point.y - boat_y) * rad2deg;
                    }
                }
                else
                {
                    R1 = 5; // 靠泊接纳圆半径小
                    if (num < 2)
                    {
                        // 跟踪靠泊路径
                        control_mode = 2;
                        v_d = 1.2;
                        los_control();
                    }
                    else
                    {
                        control_mode = 2;
                        v_d = 0;
                        psi_d = berth_heading;
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "终点距离: %f", calculate_distance(now_point, D_point));
                    }
                }
            }
            else
            {
                if (v_d == 0)
                {
                    stop = 2;
                }
                else
                {
                    stop = 0;
                }

                if (auto_pilot) // 自动舵模式
                {
                    psi_d = heading_target;
                }
                else // 循迹模式
                {
                    if (linenum > 1)
                    {
                        los_control();
                    }
                    else
                    {
                        stop = 2; // 航线读取失败，停车
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "路径点过少，请重新设置航线...");
                    }
                }
            }
        }

        // 发布控制目标话题
        auto message_control_objective = message::msg::ControlObjective();
        message_control_objective.stop = stop;
        message_control_objective.control_mode = control_mode;
        message_control_objective.v_d = v_d;
        message_control_objective.psi_d = psi_d;
        control_objective_publisher_->publish(message_control_objective);
    }

    rclcpp::TimerBase::SharedPtr timer_mode_;

    rclcpp::Publisher<message::msg::ControlObjective>::SharedPtr control_objective_publisher_;
    rclcpp::Publisher<message::msg::NaviMode>::SharedPtr berth_path_publisher_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr simspeed_subscription_;
    rclcpp::Subscription<message::msg::NaviMode>::SharedPtr navimode_subscription_;
    rclcpp::Subscription<message::msg::ShipState>::SharedPtr shipstate_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}