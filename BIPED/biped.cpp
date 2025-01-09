#include "biped.h"
#include "math/orientation_tools.h"



void Biped::RJ0RJ1convert(float DEG_0, float DEG_1, float &L0, float &L1)
{
    DEG_0 = deg2rad(DEG_0);
    DEG_1 = deg2rad(DEG_1);

    Vec3<float> B_O2P1{-65, 190, 17};
    Vec3<float> F1_O2t1{-65, 54, -30};

    Vec3<float> y0 = B_O2P1 - F1_O2t1;
    Vec3<float> y = B_O2P1 - ori::coordinateRotation(ori::CoordinateAxis::Z, DEG_0) * ori::coordinateRotation(ori::CoordinateAxis::X, DEG_1) * F1_O2t1;

    L0 = (y.norm() - y0.norm());

    Vec3<float> B_O2P1_1{65, 190, 17};
    Vec3<float> F1_O2t1_1{65, 54, -30};

    y0 = B_O2P1_1 - F1_O2t1_1;
    y = B_O2P1_1 - ori::coordinateRotation(ori::CoordinateAxis::Z, DEG_0) * ori::coordinateRotation(ori::CoordinateAxis::X, DEG_1) * F1_O2t1_1;

    L1 = (y.norm() - y0.norm());
}

void Biped::LJ0LJ1convert(float DEG_0, float DEG_1, float &L0, float &L1)
{
    DEG_0 = deg2rad(DEG_0);
    DEG_1 = deg2rad(DEG_1);

    Vec3<float> B_O2P1{-85, -190, 17};
    Vec3<float> F1_O2t1{-85, -54, -30};

    Vec3<float> y0 = B_O2P1 - F1_O2t1;
    Vec3<float> y = B_O2P1 - ori::coordinateRotation(ori::CoordinateAxis::Z, DEG_0) * ori::coordinateRotation(ori::CoordinateAxis::X, DEG_1) * F1_O2t1;

    L0 = (y.norm() - y0.norm());

    Vec3<float> B_O2P1_1{85, -190, 17};
    Vec3<float> F1_O2t1_1{85, -54, -30};

    y0 = B_O2P1_1 - F1_O2t1_1;
    y = B_O2P1_1 - ori::coordinateRotation(ori::CoordinateAxis::Z, DEG_0) * ori::coordinateRotation(ori::CoordinateAxis::X, DEG_1) * F1_O2t1_1;

    L1 = (y.norm() - y0.norm());
}

void Biped::RJ4RJ5convert(float DEG_4, float DEG_5, float &L4, float &L5)
{
    float DEG_40 = 55.29;
    DEG_40 = deg2rad(DEG_40);
    DEG_4 = deg2rad(DEG_4);
    DEG_5 = deg2rad(DEG_5);

    float OA4 = 25, OB4 = 188.64;

    float AB0 = sqrt(OA4 * OA4 + OB4 * OB4 - 2 * OA4 * OB4 * cos(DEG_40));
    float AB = sqrt(OA4 * OA4 + OB4 * OB4 - 2 * OA4 * OB4 * cos(DEG_40 - DEG_4));
    L4 = AB - AB0;

    Vec3<float> W_OA(0, -45, 150);
    Vec3<float> OB_5(0, -45, 0);

    Vec3<float> W_OB = ori::coordinateRotation(ori::CoordinateAxis::Y, DEG_4) * ori::coordinateRotation(ori::CoordinateAxis::X, DEG_5) * OB_5;
    // std::cout<<"W_OB:\n"<<W_OB<<'\n';
    Vec3<float> AB_initial(0, 0, 150);
    Vec3<float> W_AB = W_OA - W_OB;
    L5 = W_AB.norm() - AB_initial.norm();
}

void Biped::LJ4LJ5convert(float DEG_4, float DEG_5, float &L4, float &L5)
{

    float DEG_40 = 55.29;
    DEG_40 = deg2rad(DEG_40);
    DEG_4 = deg2rad(DEG_4);
    DEG_5 = deg2rad(DEG_5);

    float OA4 = 25, OB4 = 188.64;

    float AB0 = sqrt(OA4 * OA4 + OB4 * OB4 - 2 * OA4 * OB4 * cos(DEG_40));
    float AB = sqrt(OA4 * OA4 + OB4 * OB4 - 2 * OA4 * OB4 * cos(DEG_40 - DEG_4));
    L4 = AB - AB0;

    Vec3<float> W_OA(0, 45, 150);
    Vec3<float> OB_5(0, 45, 0);

    Vec3<float> W_OB = ori::coordinateRotation(ori::CoordinateAxis::Y, DEG_4) * ori::coordinateRotation(ori::CoordinateAxis::X, DEG_5) * OB_5;
    // std::cout<<"W_OB:\n"<<W_OB<<'\n';
    Vec3<float> AB_initial(0, 0, 150);
    Vec3<float> W_AB = W_OA - W_OB;
    L5 = W_AB.norm() - AB_initial.norm();
}

bool Biped::AngleInit(float &rj1,float &rj2, float &rj3, float &rj4,float &rj5 ,
float &lj1,float &lj2, float &lj3, float &lj4,float &lj5)
{
    if (timer > 2)
    {
        // rj1=1.11;
        // rj5=-1.11;

        
        rj2 = lj2 = rad2deg(Initialq[2]);
        rj3 = lj3 = rad2deg(Initialq[3]);
        rj4 = lj4 = rad2deg(Initialq[4]);
        return true;
    }
   // lj1=rj1=timer/2*1.11;
    rj2 = lj2 = timer / 2 * rad2deg(Initialq[2]);
    rj3 = lj3 = timer / 2 * rad2deg(Initialq[3]);
    rj4 = lj4 = timer / 2 * rad2deg(Initialq[4]);
    //rj5=lj5=timer/2*(-1.11);
    timer += sampletime;
    return false;
}

bool Biped::zmpAngleInit(float &rj1,float &rj2, float &rj3, float &rj4,float &rj5 ,
float &lj1,float &lj2, float &lj3, float &lj4,float &lj5)
{
    if (timer > 2)
    {
    rj1=rad2deg(zmpInitialq[1]);
    lj1=rad2deg(zmpInitialq[1+6]);
    rj5=rad2deg(zmpInitialq[5]);
    lj5=rad2deg(zmpInitialq[5+6]);
    rj2 = lj2 = rad2deg(zmpInitialq[2]);
    rj3 = lj3 = rad2deg(zmpInitialq[3]);
    rj4 = lj4 = rad2deg(zmpInitialq[4]);
        return true;
    }
   // lj1=rj1=timer/2*1.11;
//    std::cout<<"ZMP角度初始值:\n";
//    for(int i=0;i<6;i++)
//    {
//     std::cout<<zmpInitialq[i]<<'\n';
//    }

    rj1=timer/2*rad2deg(zmpInitialq[1]);
    lj1=timer/2*rad2deg(zmpInitialq[1+6]);
    rj5=timer/2*rad2deg(zmpInitialq[5]);
    lj5=timer/2*rad2deg(zmpInitialq[5+6]);
    rj2 = lj2 = timer / 2 * rad2deg(zmpInitialq[2]);
    rj3 = lj3 = timer / 2 * rad2deg(zmpInitialq[3]);
    rj4 = lj4 = timer / 2 * rad2deg(zmpInitialq[4]);
    //rj5=lj5=timer/2*(-1.11);
    timer += sampletime;
    return false;

}

void Biped::computepumpvel(float jointdeg1[])
{
    float RJ0v1 = jointdeg0[0];
    float RJ0v2 = jointdeg1[0];
    float RJ1v1 = jointdeg0[1];
    float RJ1v2 = jointdeg1[1];
    float RJ2v1 = jointdeg0[2];
    float RJ2v2 = jointdeg1[2];
    float RJ3v1 = jointdeg0[3];
    float RJ3v2 = jointdeg1[3];
    float RJ4v1 = jointdeg0[4];
    float RJ4v2 = jointdeg1[4];
    float RJ5v1 = jointdeg0[5];
    float RJ5v2 = jointdeg1[5];

    float RL0v1, RL0v2, RL1v1, RL1v2;
    RJ0RJ1convert(RJ0v1, RJ1v1, RL0v1, RL1v1);
    RJ0RJ1convert(RJ0v2, RJ1v2, RL0v2, RL1v2);
    float RL2v1 = RJ2convert(RJ2v1);
    float RL2v2 = RJ2convert(RJ2v2);
    float RL3v1 = RJ3Convert(RJ3v1);
    float RL3v2 = RJ3Convert(RJ3v2);
    float RL4v1, RL4v2, RL5v1, RL5v2;
    RJ4RJ5convert(RJ4v1, RJ5v1, RL4v1, RL5v1);
    RJ4RJ5convert(RJ4v2, RJ5v2, RL4v2, RL5v2);
    
    RL0v2 *= 1e-3;
    RL0v1*=1e-3;
    RL1v1 *= 1e-3;
    RL1v2 *= 1e-3;
    RL2v1 *= 1e-3;
    RL2v2 *= 1e-3;
    RL3v1 *= 1e-3;
    RL3v2 *= 1e-3;
    RL4v1 *= 1e-3;
    RL4v2 *= 1e-3;
    RL5v1 *= 1e-3;
    RL5v2 *= 1e-3;

    flowrate = 0;

     float RL0speed = numderivative(RL0v2, RL0v1, sampletime*0.5);
    float RL1speed = numderivative(RL1v2, RL1v1, sampletime*0.5);
    float RL2speed = numderivative(RL2v2, RL2v1, sampletime*0.5);
    float RL3speed = numderivative(RL3v2, RL3v1, sampletime*0.5);
    float RL4speed = numderivative(RL4v2, RL4v1, sampletime*0.5);
    float RL5speed = numderivative(RL5v2, RL5v1, sampletime*0.5);
    
//   std::cout<<"rlspeed"<<RL0speed<<'\n';
//     std::cout<<"rlspeed"<<RL1speed<<'\n';
//     std::cout<<"rlspeed"<<RL2speed<<'\n';
//     std::cout<<"rlspeed"<<RL3speed<<'\n';
//     std::cout<<"rlspeed"<<RL4speed<<'\n';

// std::cout<<"rlspeed"<<RL5speed<<'\n';


    // float RL3acc=secondaryderivative(RL3v2,RL3v1_5,RL3v1,sampletime);

    flowrate += std::abs(RL0speed > 0 ? RL0speed * J0gangW : RL0speed * J0gangY);
    flowrate += std::abs(RL1speed > 0 ? RL1speed * J1gangW : RL1speed * J1gangY);
    flowrate += std::abs(RL2speed > 0 ? RL2speed * J2gangW : RL2speed * J2gangY);
    flowrate += std::abs(RL3speed > 0 ? RL3speed * J3gangW : RL3speed * J3gangY);
    flowrate += std::abs(RL4speed > 0 ? RL4speed * J4gangW : RL4speed * J4gangY);
    flowrate += std::abs(RL5speed > 0 ? RL5speed * J5gangW : RL5speed * J5gangY);

    //     float minflowrate=2e-6;

    //    if(RL3acc>0.01||RL3acc<-0.01)
    //     flowrate=std::max(minflowrate,flowrate);

    float pumpvel = flowrate / Dp;

    pumpvel *= 60;

    if (pumpvel > 65536)
        pumpvel = 65536;

    pumpvelFF = pumpvel;

    for(int i=0;i<12;i++)
    {
        jointdeg0[i]=jointdeg1[i];
    }

    // rlacc=RL3acc;
    rlspeed = RL3speed;

}

void Biped::computepumpvel(float jointv1[], float jointv2[], float jointv1_5[])
{
    float RJ0v1 = jointv1[0];
    float RJ0v2 = jointv2[0];
    float RJ1v1 = jointv1[1];
    float RJ1v2 = jointv2[1];
    float RJ2v1 = jointv1[2];
    float RJ2v2 = jointv2[2];
    float RJ3v1 = jointv1[3];
    float RJ3v2 = jointv2[3];
    float RJ4v1 = jointv1[4];
    float RJ4v2 = jointv2[4];
    float RJ5v1 = jointv1[5];
    float RJ5v2 = jointv2[5];

    float RL0v1, RL0v2, RL1v1, RL1v2;
    RJ0RJ1convert(RJ0v1, RJ1v1, RL0v1, RL1v1);
    RJ0RJ1convert(RJ0v2, RJ1v2, RL0v2, RL1v2);
    float RL2v1 = RJ2convert(RJ2v1);
    float RL2v2 = RJ2convert(RJ2v2);
    float RL3v1 = RJ3Convert(RJ3v1);
    float RL3v2 = RJ3Convert(RJ3v2);
    float RL4v1, RL4v2, RL5v1, RL5v2;
    RJ4RJ5convert(RJ4v1, RJ5v1, RL4v1, RL5v1);
    RJ4RJ5convert(RJ4v2, RJ5v2, RL4v2, RL5v2);

    RL0v1 *= 1e-3;
    RL0v2 *= 1e-3;
    RL1v1 *= 1e-3;
    RL1v2 *= 1e-3;
    RL2v1 *= 1e-3;
    RL2v2 *= 1e-3;
    RL3v1 *= 1e-3;
    RL3v2 *= 1e-3;
    RL4v1 *= 1e-3;
    RL4v2 *= 1e-3;
    RL5v1 *= 1e-3;
    RL5v2 *= 1e-3;

    flowrate = 0;

    float RL0speed = numderivative(RL0v2, RL0v1, sampletime);
    float RL1speed = numderivative(RL1v2, RL1v1, sampletime);
    float RL2speed = numderivative(RL2v2, RL2v1, sampletime);
    float RL3speed = numderivative(RL3v2, RL3v1, sampletime);
    float RL4speed = numderivative(RL4v2, RL4v1, sampletime);
    float RL5speed = numderivative(RL5v2, RL5v1, sampletime);

    // float RL3acc=secondaryderivative(RL3v2,RL3v1_5,RL3v1,sampletime);

    flowrate += std::abs(RL0speed > 0 ? RL0speed * J0gangW : RL0speed * J0gangY);
    flowrate += std::abs(RL1speed > 0 ? RL1speed * J1gangW : RL1speed * J1gangY);
    flowrate += std::abs(RL2speed > 0 ? RL2speed * J2gangW : RL2speed * J2gangY);
    flowrate += std::abs(RL3speed > 0 ? RL3speed * J3gangW : RL3speed * J3gangY);
    flowrate += std::abs(RL4speed > 0 ? RL4speed * J4gangW : RL4speed * J4gangY);
    flowrate += std::abs(RL5speed > 0 ? RL5speed * J5gangW : RL5speed * J5gangY);

    //     float minflowrate=2e-6;

    //    if(RL3acc>0.01||RL3acc<-0.01)
    //     flowrate=std::max(minflowrate,flowrate);

    float pumpvel = flowrate / Dp;

    pumpvel *= 60;

    if (pumpvel > 65536)
        pumpvel = 65536;

    pumpvelFF = pumpvel;

    // rlacc=RL3acc;
    rlspeed = RL3speed;
}