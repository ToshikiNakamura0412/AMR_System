#ifndef POSE_H
#define POSE_H

#include <cmath>


class Pose
{
public:
    Pose(){}; // デフォルトコンストラクタ
    Pose(const double x, const double y, const double yaw); // コンストラクタ
    Pose& operator =(Pose& pose); // 代入演算子
    void set(const double x, const double y, const double yaw); // setter

    // ノイズを含む移動
    void move(double length, double direction, double rotation, const double fw_noise, const double rot_noise);

private:
    void normalize_angle(); // 適切な角度(-M_PI ~ M_PI)に変更

    double x_;   // [m]
    double y_;   // [m]
    double yaw_; // [rad]
};

#endif
