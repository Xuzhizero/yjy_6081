#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "redis_utils.hpp"
#include "pid.hpp"

class RudderNode : public rclcpp::Node
{
public:
    // 串口参数
    const std::string SERIAL_PORT = "/dev/ttyS3";
    const int BAUD_RATE = 115200;
    int serial_fd = -1;
    bool serial_initialized_ = false;

    double t = 0.1;
    int t_ms = 1000 * t; // 定时器周期
    double limit_rudder = 20;
    int limit_motor = 1000; // 转速千分比
    // PID
    PIDController pid_motor{50, 0, 0};

    RudderNode() : Node("rudder_node")
    {
        // 初始化串口
        if (!init_serial())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to initialize serial port! Will retry in control loop.");
        }
        else
        {
            serial_initialized_ = true;
        }

        // 初始化Redis连接
        init_redis("127.0.0.1", 6379, 1000, 1000); // ip，端口，连接超时，命令超时(ms)

        // 创建定时器
        timer_ = this->create_wall_timer(std::chrono::milliseconds(t_ms), std::bind(&RudderNode::rudder_control, this));
    }

    ~RudderNode()
    {
        // 关闭Redis连接
        close_redis();

        // 关闭串口
        if (serial_fd >= 0)
        {
            close(serial_fd);
            serial_fd = -1;
            RCLCPP_INFO(this->get_logger(), "Serial port %s closed.", SERIAL_PORT.c_str());
        }
    }

    bool init_serial()
    {
        // 以读写、非阻塞模式打开串口，并且不让它成为控制终端
        serial_fd = open(SERIAL_PORT.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd < 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to open %s: %s", SERIAL_PORT.c_str(), strerror(errno));
            return false;
        }

        struct termios opt;
        // 获取串口当前配置
        if (tcgetattr(serial_fd, &opt) != 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to get attrs for %s: %s", SERIAL_PORT.c_str(), strerror(errno));
            close(serial_fd);
            serial_fd = -1;
            return false;
        }

        speed_t baud_rate_const;
        switch (BAUD_RATE)
        {
        case 115200:
            baud_rate_const = B115200;
            break;
        case 57600:
            baud_rate_const = B57600;
            break;
        case 38400:
            baud_rate_const = B38400;
            break;
        case 19200:
            baud_rate_const = B19200;
            break;
        case 9600:
            baud_rate_const = B9600;
            break;
        default:
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unsupported baud rate: %d", BAUD_RATE);
            close(serial_fd);
            serial_fd = -1;
            return false;
        }
        cfsetispeed(&opt, baud_rate_const);
        cfsetospeed(&opt, baud_rate_const);

        // 配置串口参数：8位数据位、无校验、1位停止位、禁用硬件流控
        opt.c_cflag &= ~CSIZE;         // 清除数据位设置掩码
        opt.c_cflag |= CS8;            // 设置为8位数据位
        opt.c_cflag &= ~PARENB;        // 无校验
        opt.c_cflag &= ~CSTOPB;        // 1位停止位
        opt.c_cflag &= ~CRTSCTS;       // 禁用RTS/CTS硬件流控
        opt.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略调制解调器控制线

        // 禁用软件流控(XON/XOFF)并配置为原始输入/输出模式
        opt.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控
        opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入模式(无行缓冲、回显等)
        opt.c_oflag &= ~OPOST;                          // 原始输出模式(无处理，如CR转CRLF)

        // 设置非阻塞读取的超时时间
        opt.c_cc[VTIME] = 1; // 字符间超时 = 0.1秒 (1 decisecond)
        opt.c_cc[VMIN] = 0;  // 最少读取字符数 = 0 (非阻塞)

        // 在应用新设置前，刷新输入和输出缓冲区
        tcflush(serial_fd, TCIOFLUSH);

        // 立即应用新的串口配置
        if (tcsetattr(serial_fd, TCSANOW, &opt) != 0)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to set attrs for %s: %s", SERIAL_PORT.c_str(), strerror(errno));
            close(serial_fd);
            serial_fd = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port initialized: %s @ %d", SERIAL_PORT.c_str(), BAUD_RATE);
        return true;
    }

    bool send_serial(unsigned char *data, int len)
    {
        // 检查串口是否已初始化
        if (serial_fd < 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Serial not initialized.");
            return false;
        }
        // 验证输入参数
        if (len <= 0 || data == nullptr)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid send data.");
            return false;
        }

        // 将数据写入串口
        int bytes_written = write(serial_fd, data, len);
        if (bytes_written != len)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Send failed! Exp: %d, Act: %d, Err: %s", len, bytes_written, strerror(errno));
            return false;
        }

        return true;
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

    std::string to_4_digit_hex(int n)
    {
        std::ostringstream oss;
        uint16_t val = static_cast<std::uint16_t>(static_cast<std::int16_t>(n));
        oss << std::uppercase << std::setfill('0') << std::setw(4) << std::hex << val;

        return oss.str();
    }

    std::string add_checksum(const std::string &hex4Digit)
    {
        const int prefix = 0xAD;

        if (hex4Digit.length() != 4)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error: Input hex string must be 4 characters long.");
            return "";
        }

        // 高8位
        int high = std::stoi(hex4Digit.substr(0, 2), nullptr, 16);
        // 低8位
        int low = std::stoi(hex4Digit.substr(2, 2), nullptr, 16);
        // 和检验
        int checksum = (prefix + high + low) & 0xFF;

        std::ostringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0')
           << std::setw(2) << prefix
           << hex4Digit
           << std::setw(2) << checksum;

        return ss.str();
    }

private:
    void rudder_control()
    {
        // 串口检查
        if (!serial_initialized_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Serial port is not initialized. Attempting to reinitialize %s...", SERIAL_PORT.c_str());
            if (init_serial())
            {
                serial_initialized_ = true;
            }
            return;
        }

        double now_rudder = 0, target_rudder = 0;
        read_from_redis("Navi", "NowDuo", now_rudder);                                  // 获取当前舵角
        read_from_redis("Navi", "TargetDuo", target_rudder);                            // 获取目标舵角
        target_rudder = std::max(-limit_rudder, std::min(target_rudder, limit_rudder)); // 限制目标舵角范围

        // pid控制
        double error = target_rudder - now_rudder; // 注意：电机正反转可能颠倒，需提前测试
        int speed_motor = pid_motor.compute(error, t);
        speed_motor = std::max(-limit_motor, std::min(speed_motor, limit_motor));
        std::string speed_motor_Hex = to_4_digit_hex(speed_motor);
        std::string control_str = add_checksum(speed_motor_Hex);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "当前舵角: %.2f | 目标舵角: %.2f | 控制指令: %s", now_rudder, target_rudder, control_str.c_str());

        // 将控制指令字符串转换为字节数组
        std::vector<unsigned char> send_data = hex_str_to_bytes(control_str);
        if (send_data.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty send_data, skipping.");
            return;
        }

        // 发送指令到串口
        if (!send_serial(send_data.data(), send_data.size()))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sending command '%s' via serial failed. Command may not have been executed.", control_str.c_str());
        }
    }

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