<!--
 * @Author: jk_zeng@qq.com
 * @Date: 2023-08-09 17:16:40
 * @LastEditors: jk_zeng@qq.com
 * @LastEditTime: 2023-08-10 11:27:04
 * @FilePath: /Kalman_Filter/README.md
 * @Description:
 * 
 * Copyright (c) 2023 by ${jk_zeng@qq.com}, All Rights Reserved. 
-->
# Kalman_Filter
### 软件介绍
     一个卡尔曼滤波器，适应任意维度、任意模型。
### 安装说明
1. 安装Eigen线性代数运算库；
2. 将头文件加入工程
3. 引用头文件。
### 软件说明
#### 问题描述
<p align="center">
x(k)=Ax(k-1)+Bu(k)+w(k)
</p>

     x(k)为k时刻系统的真实状态，只能估计，永远不能知道；
     u(k)为输入控制量；
     w(k)为符合高斯分布的过程噪声，均值为0，协方差为Q；
     A为状态转换矩阵，将k-1时刻的状态转化为k时刻状态；
     B为控制输入矩阵，将k-1时刻的输入转化为k时刻状态。

<p align="center">
z(k)=Hx(k)+y(k)
</p>

     z(k)为k时刻的观测值，即为传感器测量得到的值，确切地知道；
     y(k)为符合高斯分布的测量噪声，均值为0，协方差为R；
     H为转换矩阵，将状态转换在测量空间内。
     
#### 目标
**已经有了测量值z(k)，估计系统的真实状态x(k)。**
##### 预测：
<p align="center">
x(k|k-1)=Ax(k-1|k-1)+Bu(k)  
</p>
<p align="center">  
P(k|k-1)=AP(k-1|k-1) A^T+Q
</p>

     x(k|k-1)为根据k-1时刻的系统状态预测的k时刻系统状态；
     P(k|k-1)为根据k-1时刻的得到的k时刻协方差；
     A为状态转换矩阵；
     B为控制输入矩阵；
     Q为过程噪声协方差。
     
##### 滤波：
<p align="center">
K(k)=P(k|k-1) H^T [HP(k|k-1) H^T+R]^(-1)
</p>
<p align="center">
x(k|k)=x(k|k-1)+K(k)[z(k)-Hx(k|k-1)]
</p>
<p align="center">
P(k|k)=[I-K(k)H]P(k|k-1)
</p>

     K(k)为k时刻的卡尔曼增益；
     z(k)为k时刻的测量值；
     x(k|k)为滤波后的k时刻系统状态（也就是最终预测的x(k)），初始值可为任意数；
     P(k|k)为滤波后的k时刻的协方差，初始值可为不为0的任意数；
     H为转换矩阵；
     R为测量噪声协方差。
