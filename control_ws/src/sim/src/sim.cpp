#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdint>
#include <memory>
#include <sstream>
#include <iomanip>
#include <fcntl.h>

#include "redis_utils.hpp"

class SimNode : public rclcpp::Node
{
public:
    int primary_auto = 0;
    int rudder_auto = 0;

    std::string ip_primary = "192.168.0.98";
    int port_primary = 8500;

    std::string ip_rudder = "192.168.0.99";
    int port_rudder = 8500;

    int recv_port_primary = 5858, recv_port_rudder = 5859;

    double T = 0.1;
    int t = 1000 * T, simulation_speed = 1;

    SimNode() : Node("sim_node")
    {
        // 创建UDP套接字，发送
        udp_socket_primary_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_primary_ < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to create socket: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        // 设置目标地址
        memset(&target_addr_primary_, 0, sizeof(target_addr_primary_));
        target_addr_primary_.sin_family = AF_INET;
        target_addr_primary_.sin_port = htons(port_primary);
        if (inet_pton(AF_INET, ip_primary.c_str(), &target_addr_primary_.sin_addr) <= 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid IP address: %s", ip_primary.c_str());
            rclcpp::shutdown();
            return;
        }

        udp_socket_rudder_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_rudder_ < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to create socket: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        memset(&target_addr_rudder_, 0, sizeof(target_addr_rudder_));
        target_addr_rudder_.sin_family = AF_INET;
        target_addr_rudder_.sin_port = htons(port_rudder);
        if (inet_pton(AF_INET, ip_rudder.c_str(), &target_addr_rudder_.sin_addr) <= 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid IP address: %s", ip_rudder.c_str());
            rclcpp::shutdown();
            return;
        }

        // 创建UDP套接字，接收
        udp_recv_socket_primary_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_recv_socket_primary_ < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to create primary recv socket: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        memset(&recv_addr_primary_, 0, sizeof(recv_addr_primary_));
        recv_addr_primary_.sin_family = AF_INET;
        recv_addr_primary_.sin_addr.s_addr = INADDR_ANY;
        recv_addr_primary_.sin_port = htons(recv_port_primary);
        if (bind(udp_recv_socket_primary_, (struct sockaddr *)&recv_addr_primary_, sizeof(recv_addr_primary_)) < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to bind primary recv socket: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        fcntl(udp_recv_socket_primary_, F_SETFL, O_NONBLOCK);

        udp_recv_socket_rudder_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_recv_socket_rudder_ < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to create rudder recv socket: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        memset(&recv_addr_rudder_, 0, sizeof(recv_addr_rudder_));
        recv_addr_rudder_.sin_family = AF_INET;
        recv_addr_rudder_.sin_addr.s_addr = INADDR_ANY;
        recv_addr_rudder_.sin_port = htons(recv_port_rudder);
        if (bind(udp_recv_socket_rudder_, (struct sockaddr *)&recv_addr_rudder_, sizeof(recv_addr_rudder_)) < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to bind rudder recv socket: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        fcntl(udp_recv_socket_rudder_, F_SETFL, O_NONBLOCK);

        // 初始化Redis连接
        init_redis("127.0.0.1", 6379, 1000, 1000); // ip，端口，连接超时，命令超时(ms)

        // 创建话题订阅者
        simspeed_subscription_ = this->create_subscription<std_msgs::msg::Int32>("sim_speed", 1, std::bind(&SimNode::change_timer_period, this, std::placeholders::_1));

        // 创建模型定时器
        timer_sim_ = this->create_wall_timer(std::chrono::milliseconds(t), std::bind(&SimNode::sim_communication, this));
    }

    ~SimNode()
    {
        // 关闭Redis连接
        close_redis();

        // 关闭UDP连接
        close(udp_socket_primary_);
        close(udp_socket_rudder_);
        close(udp_recv_socket_primary_);
        close(udp_recv_socket_rudder_);
    }

    uint32_t control_command_primary(int u_l, int u_r)
    {
        uint32_t u_comb = (static_cast<uint32_t>(static_cast<int16_t>(u_l)) << 16) | static_cast<uint32_t>(static_cast<int16_t>(u_r));

        return htonl(u_comb); // 转换为网络字节序（大端）
    }

    uint32_t control_command_rudder(double delta)
    {
        float delta_f = static_cast<float>(delta);
        uint32_t delta_uint32;
        std::memcpy(&delta_uint32, &delta_f, sizeof(delta_f));

        return htonl(delta_uint32); // 转换为网络字节序（大端）
    }

    bool parse_primary_data(const uint8_t *data, size_t length, int &status, int16_t &val1, int16_t &val2)
    {
        // 状态1字节，数据4字节
        if (length < 5)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Primary data too short: %zu bytes", length);
            return false;
        }

        status = static_cast<int>(data[0]);

        uint16_t val1_be = (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[2]);
        uint16_t val2_be = (static_cast<uint16_t>(data[3]) << 8) | static_cast<uint16_t>(data[4]);
        val1 = static_cast<int16_t>(val1_be);
        val2 = static_cast<int16_t>(val2_be);

        return true;
    }

    bool parse_rudder_data(const uint8_t *data, size_t length, int &status, float &parsed_float)
    {
        // 状态1字节，数据4字节
        if (length < 5)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Rudder data too short: %zu bytes", length);
            return false;
        }

        status = static_cast<int>(data[0]);

        uint32_t float_be = (static_cast<uint32_t>(data[1]) << 24) |
                            (static_cast<uint32_t>(data[2]) << 16) |
                            (static_cast<uint32_t>(data[3]) << 8) |
                            static_cast<uint32_t>(data[4]);
        std::memcpy(&parsed_float, &float_be, sizeof(parsed_float));

        return true;
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
                timer_sim_->cancel();
                // 重新创建定时器，使用新的周期
                timer_sim_ = this->create_wall_timer(std::chrono::milliseconds(t / simulation_speed), std::bind(&SimNode::sim_communication, this));
            }
        }
    }

    void sim_communication()
    {
        int throttle_l = 0, throttle_r = 0;
        read_from_redis("Navi", "left_zhuansu", throttle_l);
        read_from_redis("Navi", "right_zhuansu", throttle_r);
        uint32_t throttle_hex = control_command_primary(throttle_l, throttle_r);

        // 将数据发送到目标地址
        int result_primary = sendto(udp_socket_primary_, &throttle_hex, sizeof(throttle_hex), 0, (struct sockaddr *)&target_addr_primary_, sizeof(target_addr_primary_));
        if (result_primary < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to send_data: %s", strerror(errno));
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sent value: %d | %d (as hex: %08X)", throttle_l, throttle_r, ntohl(throttle_hex));
        }

        double delta = 0;
        read_from_redis("Navi", "TargetDuo", delta);
        uint32_t delta_hex = control_command_rudder(delta);

        // 将数据发送到目标地址
        int result_rudder = sendto(udp_socket_rudder_, &delta_hex, sizeof(delta_hex), 0, (struct sockaddr *)&target_addr_rudder_, sizeof(target_addr_rudder_));
        if (result_rudder < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to send_data: %s", strerror(errno));
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sent value: %.6f (as hex: %08X)", delta, ntohl(delta_hex));
        }

        // 尝试接收来自 primary 的数据
        uint8_t buffer_primary[1024];
        socklen_t addr_len_primary = recv_addr_len_primary_;
        ssize_t bytes_received_primary = recvfrom(udp_recv_socket_primary_, buffer_primary, sizeof(buffer_primary) - 1, 0, (struct sockaddr *)&recv_addr_primary_, &addr_len_primary);
        if (bytes_received_primary > 0)
        {
            int16_t pos_l, pos_r;
            if (parse_primary_data(buffer_primary, static_cast<size_t>(bytes_received_primary), primary_auto, pos_l, pos_r))
            {
                if (!primary_auto)
                {
                    write_to_redis("RC", "left_zhuansu", pos_l);
                    write_to_redis("RC", "right_zhuansu", pos_r);
                }
                write_to_redis("Navi", "now_left_pos", pos_l);
                write_to_redis("Navi", "now_right_pos", pos_r);
                write_to_redis("RC", "now_left_pos", pos_l);
                write_to_redis("RC", "now_right_pos", pos_r);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not parse primary data: %zd bytes", bytes_received_primary);
            }
        }
        else if (bytes_received_primary < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            // 忽略 EAGAIN/EWOULDBLOCK，这是非阻塞模式下的正常情况
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error receiving from Primary: %s", strerror(errno));
        }
        if (primary_auto)
        {
            write_to_redis("RC", "left_zhuansu", throttle_l);
            write_to_redis("RC", "right_zhuansu", throttle_r);
        }

        // 尝试接收来自 rudder 的数据
        uint8_t buffer_rudder[1024];
        socklen_t addr_len_rudder = recv_addr_len_rudder_;
        ssize_t bytes_received_rudder = recvfrom(udp_recv_socket_rudder_, buffer_rudder, sizeof(buffer_rudder) - 1, 0, (struct sockaddr *)&recv_addr_rudder_, &addr_len_rudder);
        if (bytes_received_rudder > 0)
        {
            float now_rudder;
            if (parse_rudder_data(buffer_rudder, static_cast<size_t>(bytes_received_rudder), rudder_auto, now_rudder))
            {
                if (!rudder_auto)
                {
                    write_to_redis("RC", "TargetDuo", now_rudder);
                }
                write_to_redis("Navi", "NowDuo", now_rudder);
                write_to_redis("RC", "NowDuo", now_rudder);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not parse rudder data: %zd bytes", bytes_received_rudder);
            }
        }
        else if (bytes_received_rudder < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error receiving from Rudder: %s", strerror(errno));
        }
        if (rudder_auto)
        {
            write_to_redis("RC", "TargetDuo", delta);
        }
    }

    int udp_socket_primary_ = -1;
    struct sockaddr_in target_addr_primary_{};

    int udp_socket_rudder_ = -1;
    struct sockaddr_in target_addr_rudder_{};

    int udp_recv_socket_primary_ = -1;
    struct sockaddr_in recv_addr_primary_{};
    socklen_t recv_addr_len_primary_ = sizeof(recv_addr_primary_);

    int udp_recv_socket_rudder_ = -1;
    struct sockaddr_in recv_addr_rudder_{};
    socklen_t recv_addr_len_rudder_ = sizeof(recv_addr_rudder_);

    rclcpp::TimerBase::SharedPtr timer_sim_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr simspeed_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}