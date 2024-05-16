#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include "Eigen/Dense"
#include "cppTypes.h"
#include "math/orientation_tools.h"

class Biped
{
public:
    Biped()
    {
        setBiped();
    }
    void setBiped()
    {

        mass = 63.6;
        waistwidth = 0.310 / 2;

        leg_offset_x = 0.0;
        leg_offset_y = waistwidth; //
        leg_offset_z = -0.23243;

        thighLinkLength = 0.460; //
        calfLinkLength = 0.37;
        toeLinkLength = 0.098;
    }
    double toeLinkLength;
    double thighLinkLength;
    double calfLinkLength;
    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;
    double mass;
    double waistwidth;
    template <typename T>
    float sign(T num)
    {
        if (num > 0)
            return 1;
        else if (num < 0)
            return -1;
        else
            return 0;
    }

    void ComputeIK(Vec6<double> &q, Vec3<double> &p, Mat33<double> R=Mat33<double>::Identity())
    {
        using namespace ori;
        Vec3<double> r_toe;
        r_toe << 0, 0, -toeLinkLength;
        Vec3<double> r = p - R * r_toe; // BrB5=BrBE-BR5 * 5r5E

        r = R.transpose() * (-r);

        double C = r.norm();
        double Clknee = (C * C - pow(thighLinkLength, 2) - pow(calfLinkLength, 2)) / (2 * thighLinkLength * calfLinkLength);
        std::cout<<"CLknee\n"<<Clknee<<'\n';
        if (Clknee >= 1)
            q(3) = 0;
        else if (Clknee <= -1)
            q(3) = M_PI;
        else
            q(3) = acos(Clknee);

        q(5) = atan2(r(1), r(2));
        if (q(5) > 0.5 * M_PI)
            q(5) -= M_PI;
        else if (q(5) < -0.5 * M_PI)
            q(5) += M_PI;

        double alfa = asin(thighLinkLength / C * sin(q(3)));
        q(4) = -atan2(r(0), sign(r(2)) * sqrt((pow(r(1), 2)) + pow(r(2), 2))) - alfa;

        Mat33<double> R1 = R * coordinateRotation(CoordinateAxis::X, q(5)).transpose() * coordinateRotation(CoordinateAxis::Y, q(4)).transpose() *
                           coordinateRotation(CoordinateAxis::Y, q(3)).transpose();

        q(0) = atan2(-R1(0, 1), R1(1, 1));
        q(1) = atan2(R1(2, 1), -R1(0, 1) * sin(q[0]) + R1(1, 1) * cos(q[0]));
        q(2) = atan2(-R1(2, 0), R1(2, 2));
      //  std::cout<<"逆运动解\n"<<q<<'\n';
    }
};

#endif
