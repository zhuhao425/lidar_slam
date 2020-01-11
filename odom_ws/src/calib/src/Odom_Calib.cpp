#include "../include/calib_odom/Odom_Calib.hpp"

//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3,9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len<INT_MAX)
    {
        //TODO: 构建超定方程组
        A(3*now_len,0) = Odom(0,0);
        A(3*now_len,1) = Odom(1,0);
        A(3*now_len,2) = Odom(2,0);
        A(3*now_len,3) = 0;
        A(3*now_len,4) = 0;
        A(3*now_len,5) = 0;
        A(3*now_len,6) = 0;
        A(3*now_len,7) = 0;
        A(3*now_len,8) = 0;
        A(3*now_len+1,0) = 0;
        A(3*now_len+1,1) = 0;
        A(3*now_len+1,2) = 0;
        A(3*now_len+1,3) = Odom(0,0);
        A(3*now_len+1,4) = Odom(1,0);
        A(3*now_len+1,5) = Odom(2,0);
        A(3*now_len+1,6) = 0;
        A(3*now_len+1,7) = 0;
        A(3*now_len+1,8) = 0;
        A(3*now_len+2,0) = 0;
        A(3*now_len+2,1) = 0;
        A(3*now_len+2,2) = 0;
        A(3*now_len+2,3) = 0;
        A(3*now_len+2,4) = 0;
        A(3*now_len+2,5) = 0;
        A(3*now_len+2,6) = Odom(0,0);
        A(3*now_len+2,7) = Odom(1,0);
        A(3*now_len+2,8) = Odom(2,0);

        b(3*now_len,0) = scan(0,0);
        b(3*now_len+1,0) =  scan(1,0);
        b(3*now_len+2,0) = scan(2,0);
        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //TODO:求解线性最小二乘
    Eigen::Matrix<double, 9, 1> x;

    x = (A.transpose() * A).inverse() * A.transpose() * b;

    for(int i = 0; i < 9; i ++)
        correct_matrix(i/3,i%3) = x(i,0);
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len==0&&now_len>=1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
