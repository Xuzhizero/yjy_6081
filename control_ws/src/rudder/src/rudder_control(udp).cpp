#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstdint>
#include <cerrno>
#include <cstdlib>

#include "redis_utils.hpp"
#include "pid.hpp"

class RudderNode : public rclcpp::Node
{
public:
    double t = 0.1;
    int t_ms = 1000 * t; // 定时器周期
    double limit_rudder = 30;
    int limit_motor = 10000; // 转速万分比
    // PID
    PIDController pid_motor{500, 0, 0};

    RudderNode() : Node("rudder_node")
    {
        // 创建UDP套接字，发送
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            rclcpp::shutdown();
        }

        // 设置目标地址
        std::string ip_udp = "193.0.1.88";
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(5307);
        target_addr_.sin_addr.s_addr = inet_addr(ip_udp.c_str());

        // 初始化Redis连接
        init_redis("127.0.0.1", 6379, 1000, 1000); // ip，端口，连接超时，命令超时(ms)

        // 创建定时器，用于定期接收数据，发布话题
        timer_ = this->create_wall_timer(std::chrono::milliseconds(t_ms), std::bind(&RudderNode::rudder_control, this)); // 默认周期100ms
    }

    ~RudderNode()
    {
        // 关闭Redis连接
        close_redis();

        // 关闭udp连接
        close(udp_socket_);
    }

    std::vector<unsigned char> hex_str_to_bytes(const std::string &hex_str)
    {
        std::vector<unsigned char> bytes;
        // 检查是否为空
        if (hex_str.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "hex_str_to_bytes: Input string is empty.");
            return bytes; // 返回空向量
        }
        // 检查字符串长度是否为偶数
        if (hex_str.length() % 2 != 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Hex string length is odd: %s", hex_str.c_str());
            return bytes; // 出错则返回空向量
        }

        // 逐两个字符解析为十六进制字节
        for (size_t i = 0; i < hex_str.length(); i += 2)
        {
            std::string byte_str = hex_str.substr(i, 2);
            char *end_ptr = nullptr;
            // 使用 strtoul 安全解析十六进制值，范围限定在 0x00-0xFF
            unsigned long byte_val = strtoul(byte_str.c_str(), &end_ptr, 16);
            // 校验解析结果：非法字符或超出1字节范围
            if (*end_ptr != '\0' || byte_val > 0xFF)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid hex character or out of range: %s", byte_str.c_str());
                bytes.clear(); // 出错则清空向量并返回
                return bytes;
            }
            bytes.push_back(static_cast<unsigned char>(byte_val));
        }
        return bytes;
    }

    std::string to8DigitHex(int n)
    {
        std::ostringstream oss;
        uint32_t val = static_cast<std::uint32_t>(static_cast<std::int32_t>(n));
        oss << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << val;

        return oss.str();
    }

private:
    void rudder_control()
    {
        int state = 0;
        double now_rudder = 0, target_rudder = 0;
        read_from_redis("Navi", "rudder_auto", state);                                  // 获取手自动状态
        read_from_redis("Navi", "NowDuo", now_rudder);                                  // 获取当前舵角
        read_from_redis("Navi", "TargetDuo", target_rudder);                            // 获取目标舵角
        target_rudder = std::max(-limit_rudder, std::min(target_rudder, limit_rudder)); // 限制目标舵角范围

        std::string control_str = "E000000000000000";
        if (state)
        {
            // pid控制
            double error = target_rudder - now_rudder; // 注意：电机正反转可能颠倒，需提前测试
            error = -error;
            int speed_motor = pid_motor.compute(error, t);
            speed_motor = std::max(-limit_motor, std::min(speed_motor, limit_motor));
            std::string speed_motor_Hex = to8DigitHex(speed_motor);
            control_str = "E0010000" + speed_motor_Hex;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前舵角: %.2f | 目标舵角: %.2f | 控制指令: %s", now_rudder, target_rudder, control_str.c_str());

        // 将控制指令字符串转换为字节数组
        std::vector<unsigned char> send_data = hex_str_to_bytes(control_str);
        if (send_data.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty send_data, skipping.");
            return;
        }

        // 将数据发送到目标地址
        int result = sendto(udp_socket_, send_data.data(), send_data.size(), 0, (struct sockaddr *)&target_addr_, sizeof(target_addr_));
        if (result < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to send_data: %s", strerror(errno));
        }
    }

    int udp_socket_;
    struct sockaddr_in target_addr_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RudderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}