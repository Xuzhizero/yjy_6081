#pragma once
#include <algorithm>

class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0), beta(1) {}

    // 默认构造函数
    PIDController() : kp_(0.0), ki_(0.0), kd_(0.0), integral_(0.0), prev_error_(0.0), beta(1) {}

    void setGains(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void reset()
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

    void integral_sep(bool sep)
    {
        beta = !sep;
    }

    double compute(double error, double dt)
    {
        double error_ = error;
        integral_ += beta * error_ * dt;
        double derivative = (error_ - prev_error_) / dt;
        prev_error_ = error_;

        double output = kp_ * error_ + beta * ki_ * integral_ + kd_ * derivative;
        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double prev_error_;
    int beta;
};