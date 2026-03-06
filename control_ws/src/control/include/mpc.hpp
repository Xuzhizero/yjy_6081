#ifndef MPC_HPP
#define MPC_HPP

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

class MPCController
{
public:
    MPCController(const Eigen::Matrix2d &A_d,
                  const Eigen::Vector2d &B_d,
                  int prediction_horizon, // 预测时域
                  int control_horizon,    // 控制时域
                  const Eigen::Matrix2d &Q,
                  double R)
        : A_d_(A_d), B_d_(B_d), Np_(prediction_horizon), Nu_(control_horizon), Q_(Q), R_(R)
    {
        // 计算预测矩阵
        buildPredictionMatrices();
    }

    // 默认构造函数
    MPCController()
        : A_d_(Eigen::Matrix2d::Zero()),
          B_d_(Eigen::Vector2d::Zero()),
          Np_(1),
          Nu_(1),
          Q_(Eigen::Matrix2d::Identity()),
          R_(1.0)
    {
        // 计算预测矩阵
        buildPredictionMatrices();
    }

    void update_model(const Eigen::Matrix2d &A_d, const Eigen::Vector2d &B_d)
    {
        // 更新模型
        A_d_ = A_d;
        B_d_ = B_d;
        // 更新预测矩阵
        buildPredictionMatrices();
    }

    void update_parameters(int prediction_horizon, int control_horizon, const Eigen::Matrix2d &Q, double R)
    {
        // 更新参数
        Np_ = prediction_horizon;
        Nu_ = control_horizon;
        Q_ = Q;
        R_ = R;
        // 更新预测矩阵
        buildPredictionMatrices();
    }

    // 计算最优控制量
    double computeControl(const Eigen::Vector2d &x, double ref)
    {
        // 构建参考轨迹
        Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(2 * Np_);
        for (int i = 0; i < Np_; ++i)
        {
            X_ref(2 * i) = ref;
        }

        // 构建QP问题
        Eigen::MatrixXd H = S_u_.transpose() * Q_bar_ * S_u_ + R_bar_;
        Eigen::VectorXd f = (S_x_ * x - X_ref).transpose() * Q_bar_ * S_u_;

        // 求解QP问题
        Eigen::VectorXd U = solveQP(H, f);

        return U(0); // 仅应用第一步控制量
    }

private:
    // 构建预测矩阵
    void buildPredictionMatrices()
    {
        // 初始化矩阵
        S_x_ = Eigen::MatrixXd::Zero(2 * Np_, 2);
        S_u_ = Eigen::MatrixXd::Zero(2 * Np_, Nu_);
        Q_bar_ = Eigen::MatrixXd::Zero(2 * Np_, 2 * Np_);
        R_bar_ = Eigen::MatrixXd::Zero(Nu_, Nu_);

        // 预计算A_d的幂次
        std::vector<Eigen::Matrix2d> A_d_pows(Np_ + 1);
        A_d_pows[0] = Eigen::Matrix2d::Identity();
        for (int i = 1; i <= Np_; ++i)
        {
            A_d_pows[i] = A_d_ * A_d_pows[i - 1];
        }

        // 填充矩阵
        for (int i = 0; i < Np_; ++i)
        {
            // S_x矩阵
            S_x_.block(2 * i, 0, 2, 2) = A_d_pows[i + 1];

            // S_u矩阵(保持策略：当预测时域大于控制时域时，超出控制时域后保持最后一个控制输入)
            for (int j = 0; j < Nu_; ++j)
            {
                // 直接控制影响
                if (j <= i)
                {
                    S_u_.block(2 * i, j, 2, 1) = A_d_pows[i - j] * B_d_; // 完整矩阵
                }

                // 保持策略的累积效应
                if (i >= Nu_ && j == Nu_ - 1) // Np_ = Nu_时，不会进入该逻辑
                {
                    S_u_.block(2 * i, j, 2, 1) += A_d_pows[i - Nu_ + 1] * B_d_; // 覆盖超出控制时域后的最后一列
                }
            }

            // 权重矩阵
            Q_bar_.block(2 * i, 2 * i, 2, 2) = Q_;
            if (i < Nu_)
            {
                R_bar_(i, i) = R_;
            }
        }
    }

    // 简单QP求解器
    Eigen::VectorXd solveQP(const Eigen::MatrixXd &H, const Eigen::VectorXd &f)
    {
        // 使用Eigen的LDLT求解器
        return -H.ldlt().solve(f);
    }

    Eigen::Matrix2d A_d_;           // 离散状态矩阵
    Eigen::Vector2d B_d_;           // 离散输入矩阵
    int Np_;                        // 预测时域
    int Nu_;                        // 控制时域
    Eigen::Matrix2d Q_;             // 状态权重
    double R_;                      // 控制权重
    Eigen::MatrixXd S_x_, S_u_;     // 预测矩阵
    Eigen::MatrixXd Q_bar_, R_bar_; // 块对角权重矩阵
};

#endif // MPC_HPP