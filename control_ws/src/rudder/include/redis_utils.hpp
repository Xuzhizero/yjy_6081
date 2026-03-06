#ifndef REDIS_UTILS_HPP
#define REDIS_UTILS_HPP

#include <hiredis/hiredis.h>
#include <string>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <type_traits>
#include <stdexcept>

static redisContext *redis_ctx = nullptr;
static std::string redis_host = "127.0.0.1";
static int redis_port = 6379;
static int connect_timeout_ms = 1000; // 默认1秒连接超时
static int command_timeout_ms = 1000; // 默认1秒命令超时
static std::mutex redis_mutex;

// 全局logger和clock（用于节流）
inline rclcpp::Logger get_redis_logger()
{
    static auto logger = rclcpp::get_logger("redis_utils");
    return logger;
}
inline rclcpp::Clock &get_redis_clock()
{
    static rclcpp::Clock clock(RCL_ROS_TIME);
    return clock;
}

// 将毫秒转为timeval
static struct timeval ms_to_timeval(int ms)
{
    return {ms / 1000, (ms % 1000) * 1000};
}

// redis检查，自动重连
static bool ensure_redis_connected()
{
    std::lock_guard<std::mutex> lock(redis_mutex);

    if (redis_ctx && !redis_ctx->err)
    {
        return true;
    }

    // 释放旧连接
    if (redis_ctx)
    {
        redisFree(redis_ctx);
        redis_ctx = nullptr;
    }

    // 重新连接
    struct timeval connect_tv = ms_to_timeval(connect_timeout_ms);
    redis_ctx = redisConnectWithTimeout(redis_host.c_str(), redis_port, connect_tv);
    if (!redis_ctx || redis_ctx->err)
    {
        RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis connection failed: %s", redis_ctx ? redis_ctx->errstr : "unknown error");
        if (redis_ctx)
        {
            redisFree(redis_ctx);
            redis_ctx = nullptr;
        }
        return false;
    }

    // 设置命令超时
    struct timeval cmd_tv = ms_to_timeval(command_timeout_ms);
    redisSetTimeout(redis_ctx, cmd_tv);

    RCLCPP_INFO(get_redis_logger(), "Redis connected to %s:%d", redis_host.c_str(), redis_port);
    return true;
}

// 初始化
inline bool init_redis(
    const std::string &host = "127.0.0.1",
    int port = 6379,
    int connect_timeout_ms = 1000,
    int command_timeout_ms = 1000)
{
    std::lock_guard<std::mutex> lock(redis_mutex);
    redis_host = host;
    redis_port = port;
    connect_timeout_ms = connect_timeout_ms;
    command_timeout_ms = command_timeout_ms;

    if (redis_ctx)
    {
        redisFree(redis_ctx);
        redis_ctx = nullptr;
    }

    struct timeval connect_tv = ms_to_timeval(connect_timeout_ms);
    redis_ctx = redisConnectWithTimeout(host.c_str(), port, connect_tv);
    if (!redis_ctx || redis_ctx->err)
    {
        RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Failed to initialize Redis: %s", redis_ctx ? redis_ctx->errstr : "connection failed");
        if (redis_ctx)
        {
            redisFree(redis_ctx);
            redis_ctx = nullptr;
        }
        return false;
    }

    struct timeval cmd_tv = ms_to_timeval(command_timeout_ms);
    redisSetTimeout(redis_ctx, cmd_tv);

    RCLCPP_INFO(get_redis_logger(), "Redis initialized successfully at %s:%d", host.c_str(), port);
    return true;
}

// 读取字符串
inline bool read_string_from_redis(const std::string &key, const std::string &field, std::string &out_value)
{
    if (!ensure_redis_connected())
        return false;

    std::lock_guard<std::mutex> lock(redis_mutex);
    redisReply *reply = (redisReply *)redisCommand(redis_ctx, "HGET %s %s", key.c_str(), field.c_str());
    bool success = false;

    if (reply)
    {
        if (reply->type == REDIS_REPLY_STRING)
        {
            out_value.assign(reply->str, reply->len);
            success = true;
        }
        else
        {
            // RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HGET {%s}.{%s} returned unexpected type: %d", key.c_str(), field.c_str(), reply->type);
            success = false;
        }
        freeReplyObject(reply);
    }
    else
    {
        if (redis_ctx && redis_ctx->err)
        {
            RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HGET {%s}.{%s} error: %s", key.c_str(), field.c_str(), redis_ctx->errstr);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HGET {%s}.{%s} failed (null reply)", key.c_str(), field.c_str());
        }
        success = false;
        if (redis_ctx)
            redis_ctx->err = 1; // 触发下次重连
    }

    return success;
}

// 模板读取（支持int, double, string）
template <typename T>
inline bool read_from_redis(const std::string &key, const std::string &field, T &value)
{
    static_assert(std::is_same_v<T, int> || std::is_same_v<T, double> || std::is_same_v<T, std::string>,
                  "read_from_redis only supports T = int, double, std::string");

    std::string str_value;
    if (!read_string_from_redis(key, field, str_value))
    {
        return false;
    }

    try
    {
        if constexpr (std::is_same_v<T, std::string>)
        {
            value = str_value;
            return true;
        }
        else if constexpr (std::is_same_v<T, int>)
        {
            if (str_value.empty())
            {
                RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HGET {%s}.{%s}: Cannot convert empty string to int.", key.c_str(), field.c_str());
                return false;
            }
            value = std::stoi(str_value);
            return true;
        }
        else if constexpr (std::is_same_v<T, double>)
        {
            if (str_value.empty())
            {
                RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HGET {%s}.{%s}: Cannot convert empty string to double.", key.c_str(), field.c_str());
                return false;
            }
            value = std::stod(str_value);
            return true;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HGET {%s}.{%s}: Failed to convert string '%s' to type T: %s", key.c_str(), field.c_str(), str_value.c_str(), e.what());
        return false;
    }

    return false;
}

// 写入字符串
inline bool write_string_to_redis(const std::string &key, const std::string &field, const std::string &value)
{
    if (!ensure_redis_connected())
        return false;

    std::lock_guard<std::mutex> lock(redis_mutex);
    redisReply *reply = (redisReply *)redisCommand(redis_ctx, "HSET %s %s %s", key.c_str(), field.c_str(), value.c_str());
    bool success = false;

    if (reply)
    {
        if (reply->type == REDIS_REPLY_INTEGER)
        {
            success = true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HSET {%s}.{%s} returned unexpected type: %d", key.c_str(), field.c_str(), reply->type);
            success = false;
        }
        freeReplyObject(reply);
    }
    else
    {
        if (redis_ctx && redis_ctx->err)
        {
            RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HSET {%s}.{%s} error: %s", key.c_str(), field.c_str(), redis_ctx->errstr);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(get_redis_logger(), get_redis_clock(), 1000, "Redis HSET {%s}.{%s} failed (null reply)", key.c_str(), field.c_str());
        }
        success = false;
        if (redis_ctx)
            redis_ctx->err = 1;
    }

    return success;
}

// 模板写入（支持算术类型、string、const char*）
template <typename T>
inline bool write_to_redis(const std::string &key, const std::string &field, const T &value)
{
    static_assert(std::is_arithmetic_v<T> || std::is_same_v<std::decay_t<T>, std::string> || std::is_same_v<std::decay_t<T>, const char *>,
                  "write_to_redis only supports arithmetic types, std::string, and const char*");

    std::string str_value;
    if constexpr (std::is_arithmetic_v<T>)
    {
        str_value = std::to_string(value);
    }
    else if constexpr (std::is_same_v<std::decay_t<T>, std::string>)
    {
        str_value = value;
    }
    else if constexpr (std::is_same_v<std::decay_t<T>, const char *>)
    {
        str_value = std::string(value);
    }

    return write_string_to_redis(key, field, str_value);
}

inline void close_redis()
{
    std::lock_guard<std::mutex> lock(redis_mutex);

    if (redis_ctx)
    {
        redisFree(redis_ctx);
        redis_ctx = nullptr;
        RCLCPP_INFO(rclcpp::get_logger("redis_utils"), "Redis connection closed.");
    }
}

#endif // REDIS_UTILS_HPP