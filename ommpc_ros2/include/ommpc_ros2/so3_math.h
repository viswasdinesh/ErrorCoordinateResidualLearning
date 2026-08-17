#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>

class SO3 {
public:
    static Eigen::Matrix3d hat(const Eigen::Vector3d& w) {
        Eigen::Matrix3d w_hat;
        w_hat << 0, -w(2), w(1),
                 w(2), 0, -w(0),
                 -w(1), w(0), 0;
        return w_hat;
    }

    static Eigen::Vector3d vee(const Eigen::Matrix3d& w_hat) {
        return Eigen::Vector3d(w_hat(2,1), w_hat(0,2), w_hat(1,0));
    }

    static Eigen::Matrix3d exp(const Eigen::Vector3d& w) {
        double theta = w.norm();
        
        if (theta < 1e-10) {
            Eigen::Matrix3d w_hat = hat(w);
            return Eigen::Matrix3d::Identity() + w_hat;
        }
        
        Eigen::Vector3d axis = w / theta;
        Eigen::Matrix3d axis_hat = hat(axis);
        
        return Eigen::Matrix3d::Identity() 
             + sin(theta) * axis_hat
             + (1 - cos(theta)) * axis_hat * axis_hat;
    }

    static Eigen::Vector3d log(const Eigen::Matrix3d& R) {
        // Clamp before acos: for a near-identity R (common at hover), the
        // trace computed in floating point can drift a few ULPs past 3.0,
        // pushing this argument fractionally outside [-1, 1] -- acos() of
        // an out-of-domain value is NaN (IEEE 754), which then propagates
        // through every downstream use of this rotation. Standard fix (same
        // one Sophus/Eigen-based SO3 implementations use).
        double cos_theta = std::clamp((R.trace() - 1) / 2, -1.0, 1.0);
        double theta = acos(cos_theta);

        if (theta < 1e-10) {
            return vee(R - Eigen::Matrix3d::Identity());
        }
        
        return vee((theta / (2 * sin(theta))) * (R - R.transpose()));
    }

    static Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& w) {
        double theta = w.norm();
        
        if (theta < 1e-10) {
            Eigen::Matrix3d w_hat = hat(w);
            return Eigen::Matrix3d::Identity() 
                 + 0.5 * w_hat 
                 + (1.0/6.0) * w_hat * w_hat;
        }
        
        Eigen::Vector3d a = w / theta;
        Eigen::Matrix3d a_hat = hat(a);
        
        Eigen::Matrix3d J = Eigen::Matrix3d::Identity()
                          + ((1 - cos(theta)) / theta) * a_hat
                          + (1 - sin(theta) / theta) * a_hat * a_hat;
        
        return J;
    }
};
