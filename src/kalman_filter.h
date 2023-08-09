/*
 * @Author: jk_zeng@qq.com
 * @Date: 2023-08-03 09:21:31
 * @LastEditors: jk_zeng@qq.com
 * @LastEditTime: 2023-08-09 13:53:19
 * @FilePath: /sg-amr-control-code-phase2/src/general_lib/include/filter/kalman_filter.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${jk_zeng@qq.com}, All Rights Reserved. 
 */
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"


class KalmanFilterXd
{
public:
    KalmanFilterXd(unsigned int dimension_, 
                   double *Q_, 
                   double *R_)
                   :dimension(dimension_)
    {
        Init();

        Q.setZero(dimension, dimension);
        setDiagonalParameter(Q, Q_);
        R.setZero(dimension, dimension);
        setDiagonalParameter(R, R_);
    }
    
    KalmanFilterXd(unsigned int dimension_, 
                   double *Q_, 
                   double *R_, 
                   Eigen::MatrixXd B_)
                   :dimension(dimension_)
    {
        Init();

        B.resize(dimension, dimension);
        B = B_;
        Q.setZero(dimension, dimension);
        setDiagonalParameter(Q, Q_);
        R.setZero(dimension, dimension);
        setDiagonalParameter(R, R_);
    }

    Eigen::Matrix<double, Eigen::Dynamic, 1>& Filter(Eigen::Matrix<double, Eigen::Dynamic, 1>& z)
    {
        x = A * x;
        P = A * P * A.transpose() + Q;
        Eigen::MatrixXd temp = H * P * H.transpose() + R;
        K = P * H.transpose() * temp.inverse();
        x = x + K * (z - H * x);
        P = (I - K * H) * P;

        return x;
    }
    Eigen::Matrix<double, Eigen::Dynamic, 1>& Filter(Eigen::Matrix<double, Eigen::Dynamic, 1>& z, 
                                                     Eigen::Matrix<double, Eigen::Dynamic, 1>& u)
    {
        x = A * x + B * u;
        P = A * P * A.transpose() + Q;
        Eigen::MatrixXd temp = H * P * H.transpose() + R;
        K = P * H.transpose() * temp.inverse();
        x = x + K * (z - H * x);
        P = (I - K * H) * P;

        return x;
    }

    const unsigned char Dimension() const { return dimension; }

private:
    unsigned int dimension;             //维度

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;      //状态转换矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B;      //控制输入矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H;      //转换矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> I;      //单位矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q;      //过程噪声协方差
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R;      //测量噪声协方差
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;      //卡尔曼增益
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;      //协方差
    Eigen::Matrix<double, Eigen::Dynamic, 1> x;                   //系统状态
    Eigen::Matrix<double, Eigen::Dynamic, 1> u;                   //输入控制量

    void Init()
    {
        A.setIdentity(dimension, dimension);
        H.setIdentity(dimension, dimension);
        I.setIdentity(dimension, dimension);
        K.setIdentity(dimension, dimension);
        P.setIdentity(dimension, dimension);
        x.setZero(dimension, 1);
    }

    void setDiagonalParameter(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& matrix, 
                              double *array)
    {
        for (unsigned int i = 0; i < dimension; i++)
        {
            matrix(i, i) = array[i];
        }
    }
};


#endif // KALMAN_FILTER_H_