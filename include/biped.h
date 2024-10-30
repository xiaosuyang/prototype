#ifndef PROJECT_BIPED_H
#define PROJECT_BIPED_H

#include "Eigen/Dense"
#include "utility/cppTypes.h"
#include "math/orientation_tools.h"
#include "math/MathUtilities.h"

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

        axis[0] = Vec3<double>(0, 0, 1);
        axis[1] = Vec3<double>(1, 0, 0);
        axis[2] = Vec3<double>(0, 1, 0);
        axis[3] = Vec3<double>(0, 1, 0);
        axis[4] = Vec3<double>(0, 1, 0);
        axis[5] = Vec3<double>(1, 0, 0);

        axisname[0] = ori::CoordinateAxis::Z;
        axisname[1] = ori::CoordinateAxis::X;
        axisname[2] = ori::CoordinateAxis::Y;
        axisname[3] = ori::CoordinateAxis::Y;
        axisname[4] = ori::CoordinateAxis::Y;
        axisname[5] = ori::CoordinateAxis::X;
    }
    double toeLinkLength;
    double thighLinkLength;
    double calfLinkLength;
    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;
    double mass;
    double waistwidth;
    Vec3<double> axis[6];
    ori::CoordinateAxis axisname[6];
    // std::array<double,6> Initialq{0,0,-0.338,0.7616,-0.425,0};
    // double Initialq[6]={0,0,-0.338,0.7616,-0.425,0};
    double Initialq[6] = {0, 0, -0.245154, 0.551675, -0.30652, 0};

    const float J0gangW = 1.32e-4;
    const float J0gangY = 0.69e-4;
    const float J1gangW = 1.32e-4;
    const float J1gangY = 0.69e-4;
    const float J2gangW = 2.54e-4;
    const float J2gangY = 1.00e-4;
    const float J3gangW = 3.8e-4;
    const float J3gangY = 2.03e-4;
    const float J4gangW = 3.8e-4;
    const float J4gangY = 1.29e-4;
    const float J5gangW = 1.54e-4;
    const float J5gangY = 0.75e-4;

    float jointdeg0[12]={0};

    const float Dp = 1.5e-6; // 立方米每转

    float flowrate;
    uint32_t pumpvelFF = 0;
    float sampletime = 0.01;
    float timer;
    float rlacc;
    float rlspeed;
    bool AngleInit(float &rj2, float &rj3, float &rj4, float &lj2, float &lj3, float &lj4);

    // const std::vector<Point> points = {
    //     {-1, 0},
    //     {-0.45, 0.1},
    //     {-0.333, 0.2},
    //     {-0.233, 0.3},
    //     {-0.138, 0.4},
    //     {-0.1, 0.5},
    //     {0, 0.59},
    //     {0.1, 0.5},
    //     {0.155, 0.4},
    //     {0.195, 0.3},
    //     {0.263, 0.2},
    //     {0.358, 0.1},
    //     {0.78, 0},

    // };

    float RJ2convert(float DEG)
    {

        float DEG0 = 10, OA = 27, AB = 50, e = 16.5;
        // DEG*=-1;
        DEG += DEG0;
        DEG0 = deg2rad(DEG0);
        DEG = deg2rad(DEG);
        float L0 = OA * sin(DEG0) - AB * cos(asin((OA * cos(DEG0) - e) / AB));
        float L = OA * sin(DEG) - AB * cos(asin((OA * cos(DEG) - e) / AB));
        return L - L0;
    }
    float LJ2convert(float DEG)
    {
        return RJ2convert(DEG);
    }

    float RJ3Convert(float DEG)
    {
        float DEG0 = 130;
        DEG0 = deg2rad(DEG0);
        DEG = deg2rad(DEG);

        float OA = 222.4, OB = 30;

        float AB0 = sqrt(OA * OA + OB * OB - 2 * OA * OB * cos(DEG0));
        float AB = sqrt(OA * OA + OB * OB - 2 * OA * OB * cos(DEG0 - DEG));

        return AB - AB0;
    }

    float LJ3Convert(float DEG)
    {
        return RJ3Convert(DEG);
    }

    void RJ4RJ5convert(float DEG_4, float DEG_5, float &L4, float &L5);

    void LJ4LJ5convert(float DEG_4, float DEG_5, float &L4, float &L5);

    void RJ0RJ1convert(float DEG_O, float DEG_1, float &L0, float &L1);

    void LJ0LJ1convert(float DEG_O, float DEG_1, float &L0, float &L1);

    void ComputeIK(Vec6<double> &q, Vec3<double> &p, Mat33<double> R = Mat33<double>::Identity())
    {
        using namespace ori;
        Vec3<double> r_toe;
        r_toe << 0, 0, -toeLinkLength;
        Vec3<double> r = p - R * r_toe; // BrB5=BrBE-BR5 * 5r5E

        r = R.transpose() * (-r);

        double C = r.norm();
        double Clknee = (C * C - pow(thighLinkLength, 2) - pow(calfLinkLength, 2)) / (2 * thighLinkLength * calfLinkLength);
        // std::cout<<"CLknee\n"<<Clknee<<'\n';
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

    void IKinbodyframe(Biped &_biped, Vec6<double> &q, Vec3<double> *p, int leg, Mat33<double> R = Mat33<double>::Identity())
    {
        double side = -1.0; // 1 for Left legs; -1 for right legs; leg0:rightleg;
        if (leg == 1)
        {
            side = 1.0;
        }
        Vec3<double> pbiped;
        pbiped << _biped.leg_offset_x, side * _biped.leg_offset_y, _biped.leg_offset_z; // trt0
        Vec3<double> r;
        r = *p - pbiped;
        // std::cout << "IKinbodyframe,r:\n"
        //           << r << '\n';
        ComputeIK(q, r, R);
    }

    Vec3<double> getHip2Location(int leg)
    {
        assert(leg >= 0 && leg < 2);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 1)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 0)
        {
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        return pHip;
    };

    void computepumpvel(float jointdeg[]);

    void computepumpvel(float jointv1[], float jointv2[], float jointv1_5[]);

    // 线性插值函数
    // double linear_interpolation(double x)
    // {
    // int i;

    // for (i = 0; i < points.size() - 1; ++i)
    // {
    //     if (x >= points[i].x && x <= points[i + 1].x)
    //     {
    //     double x1 = points[i].x;
    //     double y1 = points[i].y;
    //     double x2 = points[i + 1].x;
    //     double y2 = points[i + 1].y;
    //     return (y1 + (x - x1) * (y2 - y1) / (x2 - x1))*0.6;
    //     }
    // }
    // return 0;
    // }
};

#endif
