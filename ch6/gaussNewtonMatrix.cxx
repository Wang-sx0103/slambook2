#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "MatJoint.h"

// void MatJoint(Eigen::MatrixXd&, Eigen::VectorXd&);

int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;         // 真实参数值
    double ae = 1, be = 2.2, ce = 0.9;        // 估计参数值
    int N = 100;                                 // 数据点
    double w_sigma = 1.0;                        // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                 // OpenCV随机数产生器

    

    std::vector<double> x_data, y_data;      // 数据
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
    }

    /*开始Gauss-Newton迭代*/
    // 迭代次数
    int iterations = 100;
    double cost = 0, lastCost = 0;  // 本次迭代的cost和上一次迭代的cost

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (int iter = 0; iter < iterations; iter++)
    {
        // Hessian = J^T W^{-1} J in Gauss-Newton
        Eigen::Matrix3d Hi;
        Eigen::MatrixXd H_Joint = Eigen::MatrixXd::Zero(3*N, 3);
        // bias
        Eigen::Vector3d bi;
        Eigen::VectorXd b_Joint = Eigen::VectorXd::Zero(3*N);

        MatJoint jointH(N*3*3), jointB(N*3);

        cost = 0;

        /**
         * @brief /sum{J_{i}*J_{i}^{T}} {i from [1,100]}
         * 
         */
        for (int i = 0; i < N; i++)
        {
            double xi = x_data[i], yi = y_data[i];  // 第i个数据点
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Eigen::Vector3d J; // 雅可比矩阵
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);  // de/da
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);  // de/db
            J[2] = -exp(ae * xi * xi + be * xi + ce);  // de/dc

            Hi = inv_sigma * inv_sigma * J * J.transpose();
            bi = -inv_sigma * inv_sigma * error * J;
            // we should solve whole Jacobain Matrix, not one 
            jointH.Append(Hi);
            jointB.Append(bi);
            // std::cout << "\n output H:\n" << Hi;
            // std::cout << "\n output b:\n" << bi;
            // std::cout << "\n output H_Joint:\n" << H_Joint;
            // std::cout << "\n output b_Joint:\n" << b_Joint;

            cost += error * error;
        }

        jointH.Vec2Matrix(H_Joint);
        jointB.Vec2Matrix(b_Joint);

        // std::cout << "\n output H:\n" << H_Joint;
        // std::cout << "\n output b:\n" << b_Joint;

        // 求解线性方程 Hx=b
        // Eigen::Vector3d dx = H.ldlt().solve(b);
        
        // Eigen::Matrix3d A;
        // 求解线性方程 H^{T}*H*deltaX = H^{T}*b
        Eigen::Vector3d dx = (H_Joint.transpose()*H_Joint).ldlt().solve(H_Joint.transpose()*b_Joint);
        // 求解线性方程 deltaX = (H^{T}*H)^{-1}*H^{T}*b
        // Eigen::Vector3d dx = (H_Joint.transpose()*H_Joint).inverse()*H_Joint.transpose()*b_Joint;
        if (isnan(dx[0]))
        {
            std::cout << "result is nan!" << std::endl;
            break;
        }

        if (iter > 0 && cost >= lastCost)
        {
            std::cout << "iterations: " << iter << std::endl; 
            std::cout << "cost: " << cost << ">= last cost: " << lastCost << ", break." << std::endl;
            break;
        }

        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        lastCost = cost;

        std::cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
            "\t\testimated params: " << ae << "," << be << "," << ce << std::endl;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "solve time cost = " << time_used.count() << " seconds. " << std::endl;

    std::cout << "estimated abc = " << ae << ", " << be << ", " << ce << std::endl;
    return 0;
}

// void MatJoint(Eigen::MatrixXd& H, Eigen::VectorXd& b)
// {

// }
