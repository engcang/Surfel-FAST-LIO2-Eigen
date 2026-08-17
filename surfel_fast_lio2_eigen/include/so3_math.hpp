#ifndef SO3_MATH_HPP
#define SO3_MATH_HPP

#include <cmath>
#include <Eigen/Core>

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

template<typename T>
Eigen::Matrix<T, 3, 3> skewSymMat(const Eigen::Matrix<T, 3, 1> &_v)
{
    Eigen::Matrix<T, 3, 3> skew_sym_mat;
    skew_sym_mat << 0.0, -_v[2], _v[1], _v[2], 0.0, -_v[0], -_v[1], _v[0], 0.0;
    return skew_sym_mat;
}

template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> exp(const Eigen::Matrix<T, 3, 1> &_ang_vel, const Ts &_dt)
{
    T ang_vel_norm = _ang_vel.norm();
    Eigen::Matrix<T, 3, 3> eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = _ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> k;

        k << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * _dt;

        /// Roderigous Tranformation
        return eye3 + std::sin(r_ang) * k + (1.0 - std::cos(r_ang)) * k * k;
    }
    else
    {
        return eye3;
    }
}

#endif
