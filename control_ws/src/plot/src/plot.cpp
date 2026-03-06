#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>

#include "redis_utils.hpp"
#include "matplotlibcpp.h"
#include "message/msg/control_objective.hpp"
#include "message/msg/controller_output.hpp"

namespace plt = matplotlibcpp;

class PlotterNode : public rclcpp::Node
{
public:
    // 绘图数据
    int control_mode = 0;
    double v_d = 0, psi_d = 0, angle_predict = 0, yaw_rate_predict = 0;
    std::vector<double> t_;

    // figure1
    std::vector<double> rudder_target_;
    std::vector<double> rudder_now_;
    std::vector<double> rudder_error_;
    std::unique_ptr<plt::Plot> plot_rudder_target_;
    std::unique_ptr<plt::Plot> plot_rudder_now_;
    std::unique_ptr<plt::Plot> plot_rudder_error_;

    // figure2
    std::vector<double> v_d_;
    std::vector<double> v_;
    std::vector<double> v_error_;
    std::unique_ptr<plt::Plot> plot_v_d_;
    std::unique_ptr<plt::Plot> plot_v_;
    std::unique_ptr<plt::Plot> plot_v_error_;

    // figure3
    std::vector<double> psi_d_;
    std::vector<double> angle_;
    std::vector<double> angle_predict_;
    std::vector<double> angle_error_;
    std::unique_ptr<plt::Plot> plot_psi_d_;
    std::unique_ptr<plt::Plot> plot_angle_;
    std::unique_ptr<plt::Plot> plot_angle_predict_;
    std::unique_ptr<plt::Plot> plot_angle_error_;

    // figure4
    std::vector<double> yaw_rate_;
    std::vector<double> yaw_rate_predict_;
    std::vector<double> predict_error_;
    std::unique_ptr<plt::Plot> plot_yaw_rate_;
    std::unique_ptr<plt::Plot> plot_yaw_rate_predict_;
    std::unique_ptr<plt::Plot> plot_predict_error_;

    // 绘图参数
    int xlim_min = 0, xlim_max = 100, scale_min_x = 10;
    long unsigned int x_length = xlim_max - xlim_min, v_size = x_length * 10;
    int ylim_min1 = -30, ylim_max1 = 30, scale_min1 = 5;
    int ylim_min2 = 0, ylim_max2 = 1000, scale_min2 = 20;
    int ylim_min3 = -180, ylim_max3 = 180, scale_min3 = 20;
    int ylim_min4 = -30, ylim_max4 = 30, scale_min4 = 5;
    bool first_draw_ = true;

    PlotterNode() : Node("plotter_node")
    {
        // 启动交互模式，允许实时更新
        plt::ion();

        // 初始化Redis连接
        init_redis("127.0.0.1", 6379, 1000, 1000); // ip，端口，连接超时，命令超时(ms)

        // 创建话题订阅者
        control_objective_subscription_ = this->create_subscription<message::msg::ControlObjective>("control_objective", 1, std::bind(&PlotterNode::control_objective, this, std::placeholders::_1));
        controller_output_subscription_ = this->create_subscription<message::msg::ControllerOutput>("controller_output", 1, std::bind(&PlotterNode::predicted_data, this, std::placeholders::_1));

        // 数据更新固定周期100ms
        timer_update_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlotterNode::update_data, this));
        // 绘图500ms
        timer_plot_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlotterNode::plot, this));
    }

    ~PlotterNode()
    {
        close_redis();
        plt::close();
    }

private:
    void control_objective(const message::msg::ControlObjective::SharedPtr msg)
    {
        control_mode = msg->control_mode;
        v_d = msg->v_d;
        psi_d = msg->psi_d;
    }

    void predicted_data(const message::msg::ControllerOutput::SharedPtr msg)
    {
        angle_predict = msg->angle_p;
        yaw_rate_predict = msg->yaw_rate_p;
    }

    void push_with_limit(std::vector<double> &vec, double value, size_t max_size)
    {
        if (vec.size() >= max_size)
        {
            vec.erase(vec.begin());
        }
        vec.push_back(value);
    }

    void update_series_from_redis(std::vector<double> &data, const std::string &redis_key, const std::string &redis_field, size_t max_size, double fallback_value = 0.0)
    {
        double tmp = fallback_value;
        if (!read_from_redis(redis_key, redis_field, tmp))
        {
            tmp = fallback_value; // 备用方案
        }
        push_with_limit(data, tmp, max_size);
    }

    void update_data()
    {
        static int count = 0;

        // 更新无需特殊处理的变量
        update_series_from_redis(rudder_target_, "Navi", "TargetDuo", v_size);
        update_series_from_redis(rudder_now_, "Navi", "NowDuo", v_size);
        push_with_limit(v_d_, v_d, v_size);
        push_with_limit(psi_d_, psi_d, v_size);
        update_series_from_redis(yaw_rate_, "IMU", "yaw_rate", v_size);
        push_with_limit(yaw_rate_predict_, yaw_rate_predict, v_size);

        // 更新需要特殊处理的变量
        if (control_mode == 0)
        {
            update_series_from_redis(v_, "Navi", "now_left_pos", v_size);
        }
        else if (control_mode == 1)
        {
            update_series_from_redis(v_, "engine_parameters", "zhuan_su", v_size);
        }
        else if (control_mode == 2)
        {
            update_series_from_redis(v_, "IMU", "speed", v_size);
        }

        update_series_from_redis(angle_, "IMU", "angle", v_size);
        if (!angle_.empty() && angle_.back() > 180.0)
        {
            angle_.back() -= 360.0;
        }

        push_with_limit(angle_predict_, angle_predict, v_size);
        if (!angle_predict_.empty() && angle_predict_.back() > 180.0)
        {
            angle_predict_.back() -= 360.0;
        }

        // 计算派生变量
        push_with_limit(rudder_error_, rudder_target_.back() - rudder_now_.back(), v_size);
        push_with_limit(v_error_, v_d_.back() - v_.back(), v_size);
        push_with_limit(angle_error_, psi_d_.back() - angle_.back(), v_size);
        push_with_limit(predict_error_, yaw_rate_.back() - yaw_rate_predict_.back(), v_size);

        // 更新时间轴
        push_with_limit(t_, count * 0.1, v_size);

        count++;
    }

    void plot()
    {
        // 坐标轴刻度更新
        int min = (int(t_.front() / scale_min_x) + 1) * scale_min_x, max = min + xlim_max - xlim_min - scale_min_x;
        if (std::fmod(t_.front(), scale_min_x) < 1e-2)
        {
            min = t_.front();
        }
        std::vector<double> xticks;
        for (int i = min; i <= max; i += scale_min_x)
        {
            xticks.push_back(i);
        }
        // figure1
        std::vector<double> yticks1;
        for (int i = ylim_min1; i <= ylim_max1; i += scale_min1)
        {
            yticks1.push_back(i);
        }
        // figure2
        std::vector<double> yticks2;
        if (control_mode == 0)
        {
            ylim_min2 = -100;
            ylim_max2 = 100;
            scale_min2 = 10;
        }
        else if (control_mode == 1)
        {
            ylim_min2 = 0;
            ylim_max2 = 1000;
            scale_min2 = 50;
        }
        else if (control_mode == 2)
        {
            ylim_min2 = 0;
            ylim_max2 = 20;
            scale_min2 = 1;
        }
        for (int i = ylim_min2; i <= ylim_max2; i += scale_min2)
        {
            yticks2.push_back(i);
        }
        // figure3
        std::vector<double> yticks3;
        for (int i = ylim_min3; i <= ylim_max3; i += scale_min3)
        {
            yticks3.push_back(i);
        }
        // figure4
        std::vector<double> yticks4;
        for (int i = ylim_min4; i <= ylim_max4; i += scale_min4)
        {
            yticks4.push_back(i);
        }

        // Plot类动态绘图
        if (first_draw_)
        {
            // // 期望舵角和实际舵角
            // plt::figure(1);
            // plot_rudder_target_ = std::make_unique<plt::Plot>("rudder_target", t_, rudder_target_, "k-");
            // plot_rudder_now_ = std::make_unique<plt::Plot>("rudder_now", t_, rudder_now_, "b-");
            // plot_rudder_error_ = std::make_unique<plt::Plot>("rudder_error", t_, rudder_error_, "g--");
            // plt::legend();
            // plt::xlabel("time(s)");
            // plt::ylabel("rudder(deg)");
            // plt::grid(true);

            // 期望航速和实际航速
            plt::figure(2);
            plot_v_d_ = std::make_unique<plt::Plot>("v_d", t_, v_d_, "k-");
            plot_v_ = std::make_unique<plt::Plot>("v", t_, v_, "b-");
            plot_v_error_ = std::make_unique<plt::Plot>("v_error", t_, v_error_, "g--");
            plt::legend();
            plt::xlabel("time(s)");
            if (control_mode == 0)
            {
                plt::ylabel("throttle(%)");
            }
            else if (control_mode == 1)
            {
                plt::ylabel("motor_speed(rpm)");
            }
            else if (control_mode == 2)
            {
                plt::ylabel("speed(m/s)");
            }
            plt::grid(true);

            // 期望角和航向角
            plt::figure(3);
            plot_psi_d_ = std::make_unique<plt::Plot>("psi_d", t_, psi_d_, "k-");
            plot_angle_ = std::make_unique<plt::Plot>("angle", t_, angle_, "b-");
            plot_angle_predict_ = std::make_unique<plt::Plot>("angle_predict", t_, angle_predict_, "c-");
            plot_angle_error_ = std::make_unique<plt::Plot>("angle_error", t_, angle_error_, "g--");
            plt::legend();
            plt::xlabel("time(s)");
            plt::ylabel("angle(deg)");
            plt::grid(true);

            // // 实际角速度和预测角速度
            // plt::figure(4);
            // plot_yaw_rate_ = std::make_unique<plt::Plot>("yaw_rate", t_, yaw_rate_, "b-");
            // plot_yaw_rate_predict_ = std::make_unique<plt::Plot>("yaw_rate_predict", t_, yaw_rate_predict_, "c-");
            // plot_predict_error_ = std::make_unique<plt::Plot>("predict_error", t_, predict_error_, "g--");
            // plt::legend();
            // plt::xlabel("time(s)");
            // plt::ylabel("yaw_rate(deg/s)");
            // plt::grid(true);

            first_draw_ = false;
        }
        else
        {
            // 更新数据
            // // figure1
            // plot_rudder_target_->update(t_, rudder_target_);
            // plot_rudder_now_->update(t_, rudder_now_);
            // plot_rudder_error_->update(t_, rudder_error_);

            // figure2
            plot_v_d_->update(t_, v_d_);
            plot_v_->update(t_, v_);
            plot_v_error_->update(t_, v_error_);

            // figure3
            plot_psi_d_->update(t_, psi_d_);
            plot_angle_->update(t_, angle_);
            plot_angle_predict_->update(t_, angle_predict_);
            plot_angle_error_->update(t_, angle_error_);

            // // figure4
            // plot_yaw_rate_->update(t_, yaw_rate_);
            // plot_yaw_rate_predict_->update(t_, yaw_rate_predict_);
            // plot_predict_error_->update(t_, predict_error_);
        }

        // 更新标题和坐标轴（滚动窗口）
        // plt::figure(1);
        // plt::title("error: " + std::to_string(rudder_error_.back()) + "deg");
        // plt::xlim(t_.front(), t_.front() + x_length);
        // plt::xticks(xticks);
        // plt::ylim(ylim_min1, ylim_max1);
        // plt::yticks(yticks1);

        plt::figure(2);
        if (control_mode == 0)
        {
            plt::title("error: " + std::to_string(v_error_.back()) + "%");
            plt::ylabel("throttle(%)");
        }
        else if (control_mode == 1)
        {
            plt::title("error: " + std::to_string(v_error_.back()) + "rpm");
            plt::ylabel("motor_speed(rpm)");
        }
        else if (control_mode == 2)
        {
            plt::title("error: " + std::to_string(v_error_.back()) + "m/s");
            plt::ylabel("speed(m/s)");
        }
        plt::xlim(t_.front(), t_.front() + x_length);
        plt::xticks(xticks);
        plt::ylim(ylim_min2, ylim_max2);
        plt::yticks(yticks2);

        plt::figure(3);
        plt::title("error: " + std::to_string(angle_error_.back()) + "deg");
        plt::xlim(t_.front(), t_.front() + x_length);
        plt::xticks(xticks);
        plt::ylim(ylim_min3, ylim_max3);
        plt::yticks(yticks3);

        // plt::figure(4);
        // plt::title("error: " + std::to_string(predict_error_.back()) + "deg/s");
        // plt::xlim(t_.front(), t_.front() + x_length);
        // plt::xticks(xticks);
        // plt::ylim(ylim_min4, ylim_max4);
        // plt::yticks(yticks4);

        // 强制刷新图形
        plt::draw();
        plt::pause(0.01);
    }

    rclcpp::TimerBase::SharedPtr timer_update_;
    rclcpp::TimerBase::SharedPtr timer_plot_;
    rclcpp::Subscription<message::msg::ControlObjective>::SharedPtr control_objective_subscription_;
    rclcpp::Subscription<message::msg::ControllerOutput>::SharedPtr controller_output_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlotterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}