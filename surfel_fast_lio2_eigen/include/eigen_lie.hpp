#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lie
{

    using Matrix3d = Eigen::Matrix3d;
    using Quaterniond = Eigen::Quaterniond;
    using Vector3d = Eigen::Vector3d;

    inline Matrix3d hat(const Vector3d &_vector)
    {
        Matrix3d result;
        result << 0.0, -_vector.z(), _vector.y(),
            _vector.z(), 0.0, -_vector.x(),
            -_vector.y(), _vector.x(), 0.0;
        return result;
    }

    inline Matrix3d rotationMatrixExp(const Vector3d &_angular_velocity,
                                      const double _dt)
    {
        const double angular_speed = _angular_velocity.norm();
        const Matrix3d identity = Matrix3d::Identity();
        if (angular_speed <= 1.0e-7)
        {
            return identity;
        }

        const Vector3d rotation_axis = _angular_velocity / angular_speed;
        const Matrix3d rotation_axis_hat = hat(rotation_axis);
        const double rotation_angle = angular_speed * _dt;
        return identity + std::sin(rotation_angle) * rotation_axis_hat +
               (1.0 - std::cos(rotation_angle)) * rotation_axis_hat * rotation_axis_hat;
    }

    inline Quaterniond exp(const Vector3d &_rotation_vector)
    {
        const double scale = 0.5;
        const double scaled_squared_angle = scale * scale * _rotation_vector.squaredNorm();
        const double taylor_bound = std::sqrt(std::sqrt(std::numeric_limits<double>::epsilon()));
        double real;
        double sinc;
        if (scaled_squared_angle >= taylor_bound)
        {
            const double scaled_angle = std::sqrt(scaled_squared_angle);
            real = std::cos(scaled_angle);
            sinc = std::sin(scaled_angle) / scaled_angle;
        }
        else
        {
            static constexpr double inverse[] = {1.0 / 3.0, 1.0 / 4.0, 1.0 / 5.0, 1.0 / 6.0, 1.0 / 7.0, 1.0 / 8.0, 1.0 / 9.0};
            real = 1.0;
            sinc = 1.0;
            double term = -0.5 * scaled_squared_angle;
            for (int index = 0; index < 3; ++index)
            {
                real += term;
                term *= inverse[2 * index];
                sinc += term;
                term *= -inverse[2 * index + 1] * scaled_squared_angle;
            }
        }

        const double imaginary_scale = sinc * scale;

        return Quaterniond(real,
                           imaginary_scale * _rotation_vector.x(),
                           imaginary_scale * _rotation_vector.y(),
                           imaginary_scale * _rotation_vector.z());
    }

    inline Vector3d log(const Quaterniond &_quaternion)
    {
        double imaginary_norm = _quaternion.vec().norm();
        if (imaginary_norm < 1.0e-11)
        {
            imaginary_norm = 1.0e-11;
        }
        const double scale = 2.0 / imaginary_norm * std::atan(imaginary_norm / _quaternion.w());
        return scale * _quaternion.vec();
    }

    inline Matrix3d leftJacobian(const Vector3d &_rotation_vector)
    {
        const double squared_angle = _rotation_vector[0] * _rotation_vector[0] +
                                     _rotation_vector[1] * _rotation_vector[1] +
                                     _rotation_vector[2] * _rotation_vector[2];
        const double angle = std::sqrt(squared_angle);
        if (angle < 1.0e-11)
        {
            return Matrix3d::Identity();
        }

        const Matrix3d rotation_hat = hat(_rotation_vector);
        return Matrix3d::Identity() + ((1.0 - std::cos(angle)) / squared_angle) * rotation_hat + ((1.0 - std::sin(angle) / angle) / squared_angle) * rotation_hat * rotation_hat;
    }

} // namespace lie
